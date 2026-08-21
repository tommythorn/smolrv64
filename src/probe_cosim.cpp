// Cosim glue for the sharded-OoO probe core, verilated under tb_vl.v with
// -DPROBE_COSIM. backend_top emits one probe_retire() DPI call per committed
// instruction (and per trap) in program order; here we lockstep those against
// the simmerv golden model (C ABI in ~/simmerv) and abort on the first
// divergence. Mirrors src/sim_main.cpp's comparison/arming machinery; the one
// structural difference is that the probe reports no architectural next_pc, so
// we BUFFER one retire and fill its next_pc with the following retire's pc
// (exact for in-order commit, traps included).
//
// Init is lazy (on the first probe_retire): we read the same +hex flat byte
// image tb_vl.v loads, install it in simmerv at +reset_pc (default 0x80000000),
// zero registers and set the PC. No main() here -- verilator --binary owns it.

#include "verilated.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "simmerv_cosim.h"

namespace {

constexpr uint64_t AXI_BASE = 0x80000000ULL;
#ifndef COSIM_MEM_SIZE_LG2
#define COSIM_MEM_SIZE_LG2 27
#endif
constexpr size_t MEM_BYTES = 1ULL << COSIM_MEM_SIZE_LG2;

SimmervCtx* g_ctx   = nullptr;
bool        g_inited = false;
uint64_t    g_seqno = 0;

struct RingEntry { SimmervRetire dut; SimmervRetire ref; bool valid; };
constexpr size_t RING_N = 65536;   // widened for +tracepc window dumps (was 320)
RingEntry g_ring[RING_N] = {};
size_t    g_ring_idx = 0;

// buffered previous retire (so next_pc = next retire's pc)
bool          g_have_prev = false;
SimmervRetire g_prev{};
uint64_t      g_prev_mtimecmp = ~0ULL;
bool          g_prev_seip = false;

// +tracepc=<hexVA>: dump the DUT-vs-REF ring window the FIRST time this PC retires.
uint64_t      g_tracepc   = 0;
bool          g_tp_parsed = false;
bool          g_traced    = false;

const char* plusarg(const char* key) {
    // verilated --binary parsed the args; fetch "+key=...". Match "key=" (not just the
    // prefix "key") so e.g. plusarg("dtb")/("initrd") don't also match +dtb_off=/+initrd_off=
    // -- commandArgsPlusMatch is a prefix match, and those share the dtb/initrd prefix.
    char k[64]; std::snprintf(k, sizeof k, "%s=", key);
    const char* m = Verilated::commandArgsPlusMatch(k);
    if (!m || !m[0]) return nullptr;
    const char* eq = std::strchr(m, '=');
    return eq ? eq + 1 : nullptr;
}

// Load a raw binary file into simmerv at phys addr (BASE + off).
bool load_bin_at(const char* path, uint64_t off) {
    std::ifstream in(path, std::ios::binary);
    if (!in) { std::fprintf(stderr, "cosim: cannot open %s\n", path); return false; }
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
    if (off + buf.size() > MEM_BYTES) {
        // Print the baked-in bound: "overflows" with off < the intended DDR size means
        // the binary was compiled with a smaller MEM_LG2 (stale build).
        std::fprintf(stderr, "cosim: %s overflows DDR (off=%llx size=%zu MEM_BYTES=%llx)\n",
                     path, (unsigned long long)off, buf.size(), (unsigned long long)MEM_BYTES);
        return false;
    }
    if (simmerv_write_memory(g_ctx, AXI_BASE + off, buf.data(), buf.size()) != 0) {
        std::fprintf(stderr, "cosim: simmerv_write_memory(%s) failed\n", path); return false;
    }
    std::fprintf(stderr, "cosim: loaded %zu bytes @ DDR+%llx (%s)\n",
                 buf.size(), (unsigned long long)off, path);
    return true;
}

bool load_flat_hex(const char* path) {
    std::ifstream in(path);
    if (!in) { std::fprintf(stderr, "cosim: cannot open %s\n", path); return false; }
    std::vector<uint8_t> ram(MEM_BYTES, 0);
    std::string tok;
    size_t i = 0;
    while (in >> tok) {
        if (tok.empty() || tok[0] == '@' || tok[0] == '/') {
            if (!tok.empty() && tok[0] == '@') i = std::stoull(tok.substr(1), nullptr, 16);
            continue;
        }
        if (i >= ram.size()) { std::fprintf(stderr, "cosim: image overflow\n"); return false; }
        ram[i++] = (uint8_t)std::stoul(tok, nullptr, 16);
    }
    if (simmerv_write_memory(g_ctx, AXI_BASE, ram.data(), ram.size()) != 0) {
        std::fprintf(stderr, "cosim: simmerv_write_memory failed\n"); return false;
    }
    return true;
}

void cosim_init() {
    g_inited = true;
    setvbuf(stdout, nullptr, _IONBF, 0);   // so $display debug survives abort()
    // Announce the size this OBJECT FILE was compiled with, not what a shell variable
    // said.  MEM_LG2 reaches here as a -CFLAGS -D, so a stale probe_cosim.o silently gives
    // the reference model a different amount of RAM than the RTL -- on 2026-08-20 that was
    // 512 MiB vs 2 GiB, and every GB5 run "diverged" on a load to a PA that was real memory
    // to the DUT and past the end of the world here.  The runner's log line printed the
    // shell's MEM_LG2 and therefore confirmed nothing.  This one cannot lie.
    std::fprintf(stderr, "cosim: reference RAM = %llu MiB at 0x%llx (COSIM_MEM_SIZE_LG2=%d, compiled in)\n",
                 (unsigned long long)(MEM_BYTES >> 20), (unsigned long long)AXI_BASE,
                 (int)COSIM_MEM_SIZE_LG2);
    g_ctx = simmerv_create(MEM_BYTES);
    if (!g_ctx) { std::fprintf(stderr, "cosim: simmerv_create failed\n"); std::abort(); }
    simmerv_zero_registers(g_ctx);
    uint64_t reset_pc = AXI_BASE;
    const char* fw = plusarg("fw");
    if (fw) {
        // Linux mode: fw_payload@DDR+0, DTB/initrd at workload offsets (default linux;
        // overridden per workload via +dtb_off=/+initrd_off= to match the RTL harness), a1=DTB.
        // NB: plusarg() returns a pointer into a SHARED static buffer that the next plusarg()
        // call overwrites -- consume each result (load the file / convert the number) BEFORE
        // the next plusarg() call; never hold two plusarg pointers across a call.
        if (!load_bin_at(fw, 0)) std::abort();
        const char* doff = plusarg("dtb_off");
        const uint64_t OFF_DTB    = doff ? std::strtoull(doff, nullptr, 16) : 0x2000000ULL;
        const char* ioff = plusarg("initrd_off");
        const uint64_t OFF_INITRD = ioff ? std::strtoull(ioff, nullptr, 16) : 0x762b000ULL;
        const char* dtb = plusarg("dtb");
        if (!dtb) { std::fprintf(stderr, "cosim: need +dtb with +fw\n"); std::abort(); }
        if (!load_bin_at(dtb, OFF_DTB)) std::abort();
        if (const char* ird = plusarg("initrd")) if (!load_bin_at(ird, OFF_INITRD)) std::abort();
        const char* a1 = plusarg("a1");
        simmerv_write_register(g_ctx, 11, a1 ? std::strtoull(a1, nullptr, 16) : (AXI_BASE + OFF_DTB));
    } else {
        const char* hex = plusarg("hex");
        if (!hex) { std::fprintf(stderr, "cosim: need +hex or +fw\n"); std::abort(); }
        if (!load_flat_hex(hex)) std::abort();
        const char* rpc = plusarg("reset_pc");
        if (rpc) reset_pc = std::strtoull(rpc, nullptr, 16);
    }
    simmerv_set_pc(g_ctx, reset_pc);
    simmerv_set_mtime(g_ctx, 0);
    std::fprintf(stderr, "cosim: simmerv ready (reset_pc=%016llx%s)\n",
                 (unsigned long long)reset_pc, fw ? ", linux" : "");
}

// (memory effect is printed by dump_retire below)
void dump_retire(const char* label, const SimmervRetire& r) {
    std::fprintf(stderr,
        "  %s seq=%llu pc=%016llx npc=%016llx insn=%08x prv=%u trap=%u "
        "rd=(k%u,x%u)=%016llx cause=%016llx tval=%016llx mepc=%016llx mem=%s@%012llx\n",
        label, (unsigned long long)r.seqno, (unsigned long long)r.pc,
        (unsigned long long)r.next_pc, r.insn, r.prv, r.trapped, r.rd_kind, r.rd_idx,
        (unsigned long long)r.rd_val, (unsigned long long)r.trap_cause,
        (unsigned long long)r.trap_tval, (unsigned long long)r.mepc,
        r.mem_kind == 1 ? "ld" : r.mem_kind == 2 ? "st" : "--",
        (unsigned long long)r.mem_pa);
}

[[noreturn]] void mismatch_abort(const SimmervRetire& dut, const SimmervRetire& ref) {
    std::fprintf(stderr, "\n*** cosim MISMATCH at retire #%llu ***\n\n",
                 (unsigned long long)g_seqno);
    std::fprintf(stderr, "Recent history (DUT vs REF):\n");
    for (size_t i = 0; i < RING_N; i++) {
        size_t idx = (g_ring_idx + i) % RING_N;
        if (!g_ring[idx].valid) continue;
        dump_retire("DUT", g_ring[idx].dut);
        dump_retire("REF", g_ring[idx].ref);
    }
    std::fprintf(stderr, "Diverging retire:\n");
    dump_retire("DUT", dut);
    dump_retire("REF", ref);
    // Reference-side GPRs at the divergence: registers matched through the
    // previous retire, so these recover operand values for the diverging op --
    // e.g. a load's base register pins the exact poisoned memory address for a
    // follow-up write-watchpoint run.
    static const char* rn[32] = {
        "x0","ra","sp","gp","tp","t0","t1","t2","s0","s1","a0","a1","a2","a3",
        "a4","a5","a6","a7","s2","s3","s4","s5","s6","s7","s8","s9","s10","s11",
        "t3","t4","t5","t6"};
    std::fprintf(stderr, "REF GPRs at divergence:\n");
    for (int i = 0; i < 32; i += 4)
        std::fprintf(stderr, "  %-3s=%016llx %-3s=%016llx %-3s=%016llx %-3s=%016llx\n",
            rn[i],   (unsigned long long)simmerv_read_register(g_ctx, i),
            rn[i+1], (unsigned long long)simmerv_read_register(g_ctx, i+1),
            rn[i+2], (unsigned long long)simmerv_read_register(g_ctx, i+2),
            rn[i+3], (unsigned long long)simmerv_read_register(g_ctx, i+3));
    std::fflush(stderr);
    std::abort();
}

uint32_t canon_insn(uint32_t insn) {
    return (insn & 0x3) == 0x3 ? insn : (insn & 0xffffu);
}

int csr_read_to_override(uint32_t insn) {
    if ((insn & 0x7f) != 0x73) return -1;
    const uint32_t f3 = (insn >> 12) & 0x7;
    if (f3 == 0 || f3 == 4) return -1;
    const uint32_t csrno = (insn >> 20) & 0xfff;
    switch (csrno) {
        case 0xC00: case 0xC01: case 0xC02:
        case 0xB00: case 0xB02:
        case 0xF11: case 0xF12: case 0xF13:
        // PMP cfg/addr: no enforcement modeled; the DUT stores them verbatim while
        // simmerv's CSR fast-path ignores writes -> let the DUT's read value win.
        case 0x3A0: case 0x3B0: return (int)csrno;
        default:
            if ((csrno >= 0xB03 && csrno <= 0xB1F) ||
                (csrno >= 0xC03 && csrno <= 0xC1F) ||
                (csrno >= 0x323 && csrno <= 0x33F)) return (int)csrno;
            return -1;
    }
}

// Step simmerv for one buffered DUT retire P and compare.
void step_compare(const SimmervRetire& dut, uint64_t mtimecmp, bool seip) {
    simmerv_set_mtime(g_ctx, dut.mtime);
    const unsigned long long MTIP_CAUSE = 0x8000000000000007ULL;
    simmerv_set_mtimecmp(g_ctx, (dut.trapped && dut.trap_cause == MTIP_CAUSE) ? mtimecmp : ~0ULL);
    // Full interrupt DUT-follow: simmerv takes EXACTLY the interrupt the DUT took this
    // retire (cause MSB set), or none. Replaces the per-type mtip/stip/seip armed gates.
    simmerv_set_forced_interrupt(g_ctx, (dut.trapped && (dut.trap_cause >> 63)) ? dut.trap_cause : 0ULL);
    simmerv_set_seip(g_ctx, seip);
    simmerv_set_plic_ip(g_ctx, 10, seip);
    if (!dut.trapped && dut.rd_kind != 0) {
        const int oc = csr_read_to_override(dut.insn);
        if (oc >= 0) simmerv_arm_csr_read(g_ctx, (uint16_t)oc, dut.rd_val);
        simmerv_arm_load_value(g_ctx, dut.rd_val);
    }
    SimmervRetire ref{};
    if (simmerv_step_retire(g_ctx, &ref) != 0) {
        std::fprintf(stderr, "cosim: simmerv_step_retire failed at seq %llu\n",
                     (unsigned long long)g_seqno);
        std::abort();
    }
    ref.seqno = g_seqno;

    // The probe reports the RVC-EXPANDED 32-bit insn for compressed instructions,
    // whereas simmerv reports the raw 16-bit parcel. Reconciling needs an RVC
    // un-expander; instead, when the retire is compressed (ref parcel's low 2 bits
    // != 11) we skip insn-equality -- pc/next_pc/rd/rd_val still pin the behavior,
    // and RVC expansion is separately verified (tb_rvc_expand, exhaustive).
    const bool ref_compressed = (ref.insn & 0x3) != 0x3;
    // NARROW exemption (real DUT anomaly, tracked separately -- do not widen): a
    // pending data fault can race an in-flight interrupt injection at the same pc
    // and get delivered on the OP_IRQ pseudo-op's checkpoint; the DUT then retires
    // insn=7f000073 carrying the SYNCHRONOUS cause while simmerv attributes the
    // identical trap (same pc/cause/tval) to the real instruction. Downstream
    // state matches (handler runs, sepc re-executes the op, the interrupt
    // re-injects), so tolerate the insn-word mismatch for exactly this shape
    // instead of aborting multi-hour hunts. First seen: tiny128-stress
    // @180,522,457 (COW fault in the dirty loop).
    const bool irq_op_sync_trap = dut.insn == 0x7f000073u && dut.trapped &&
                                  ref.trapped && dut.trap_cause == ref.trap_cause &&
                                  (dut.trap_cause >> 63) == 0;
    const bool insn_ok = ref_compressed || irq_op_sync_trap ||
                         (canon_insn(dut.insn) == canon_insn(ref.insn));
    const bool rdval_ok = dut.rd_kind == 0 || dut.rd_val == ref.rd_val;

    const bool ok =
        dut.pc         == ref.pc        &&
        dut.next_pc    == ref.next_pc   &&
        insn_ok                         &&
        dut.rd_kind    == ref.rd_kind   &&
        dut.rd_idx     == ref.rd_idx    &&
        dut.prv        == ref.prv       &&
        dut.trapped    == ref.trapped   &&
        rdval_ok                        &&
        dut.trap_cause == ref.trap_cause &&
        dut.trap_tval  == ref.trap_tval &&
        dut.mepc       == ref.mepc &&
        // MEMORY EFFECT.  Register results alone cannot see a store: it has no
        // architectural result, so a store to the WRONG PHYSICAL ADDRESS corrupts
        // REF/DUT memory silently and only surfaces much later as an unrelated fault
        // (GB5 retire #6979942, 2026-08-20: the DUT's page-table walk was correct
        // against its own memory -- the two models' page tables had already diverged).
        // Compare the PA on both loads and stores; only when BOTH sides agree an access
        // happened, so a model that reports no access never forces a false abort.
        (dut.mem_kind == 0 || ref.mem_kind == 0 ||
         (dut.mem_kind == ref.mem_kind && dut.mem_pa == ref.mem_pa));

    g_ring[g_ring_idx] = { dut, ref, true };
    g_ring_idx = (g_ring_idx + 1) % RING_N;

    if (g_seqno == 1 || (g_seqno % 1'000'000) == 0)
        std::fprintf(stderr, "cosim: %llu retirements ok (pc=%016llx)\n",
                     (unsigned long long)g_seqno, (unsigned long long)dut.pc);

    if (!ok) mismatch_abort(dut, ref);
}

// ---- device-DMA mirror (virtio-blk writes into DUT DDR) ----------------------------
// The TB forwards every device AXI write beat here in DPI order. Applying immediately
// would be one retire EARLY (probe_retire holds one retire to learn next_pc), so the
// beats queue and drain right after step_compare() -- i.e. between exactly the two
// retires the write chronologically separates. Keeps simmerv RAM == DUT DDR for
// non-coherent DMA without ever racing an in-flight compare.
struct DmaBeat { uint64_t off; uint64_t data; uint8_t strb; };
static std::vector<DmaBeat> g_dmaq;

void drain_dma_queue() {
    for (const auto& w : g_dmaq)
        for (int k = 0; k < 8; k++)
            if (w.strb & (1u << k)) {
                uint8_t b = (uint8_t)(w.data >> (8 * k));
                simmerv_write_memory(g_ctx, AXI_BASE + w.off + k, &b, 1);
            }
    g_dmaq.clear();
}

// +tracepc trigger: dump the ring (last RING_N retirements, DUT vs REF) the first time the
// target PC retires -- the failure window for a bug that trips neither mismatch_abort (no
// divergence) nor the hang watchdog (boot continues past it).
void dump_ring_window(const char* why, unsigned long long pc) {
    std::fprintf(stderr,
        "\n*** cosim TRACE (%s) pc=%016llx at retire #%llu -- last %zu retirements (DUT vs REF):\n",
        why, pc, (unsigned long long)g_seqno, RING_N);
    for (size_t i = 0; i < RING_N; i++) {
        size_t idx = (g_ring_idx + i) % RING_N;
        if (!g_ring[idx].valid) continue;
        dump_retire("DUT", g_ring[idx].dut);
        dump_retire("REF", g_ring[idx].ref);
    }
    std::fflush(stderr);
}

} // namespace

extern "C" void cosim_dma_write(unsigned long long off, unsigned long long data,
                                unsigned char strb) {
    g_dmaq.push_back({off, data, strb});
}

// Called by the TB watchdog when the DUT's fetch is stuck (no retirement progress) -- dumps
// the last RING_N retirements (DUT vs REF) so a HANG (which never reaches mismatch_abort) is
// as diagnosable as a divergence.
extern "C" void probe_dump_ring(unsigned long long fetch_pc) {
    std::fprintf(stderr,
        "\n*** cosim HANG: fetch stuck at pc=%016llx, no retirement (after retire #%llu) ***\n",
        (unsigned long long)fetch_pc, (unsigned long long)g_seqno);
    std::fprintf(stderr, "Last %zu retirements (DUT vs REF):\n", RING_N);
    for (size_t i = 0; i < RING_N; i++) {
        size_t idx = (g_ring_idx + i) % RING_N;
        if (!g_ring[idx].valid) continue;
        dump_retire("DUT", g_ring[idx].dut);
        dump_retire("REF", g_ring[idx].ref);
    }
    std::fflush(stderr);
}

// DPI callback from backend_top.v (one per committed instruction or trap).
extern "C" void probe_retire(
    unsigned long long pc,
    unsigned int       insn,
    unsigned char      rd_kind,
    unsigned char      rd_idx,
    unsigned char      prv,
    unsigned char      trapped,
    unsigned long long rd_val,
    unsigned long long trap_cause,
    unsigned long long trap_tval,
    unsigned long long mtime,
    unsigned long long mtimecmp,
    unsigned long long mepc,
    unsigned char      seip,
    unsigned char      mem_kind,
    unsigned long long mem_pa)
{
    if (!g_inited) cosim_init();

    if (!g_tp_parsed) { g_tp_parsed = true;
        const char* t = plusarg("tracepc");
        if (t) g_tracepc = std::strtoull(t, nullptr, 16); }
    if (g_tracepc && pc == g_tracepc && !g_traced) { g_traced = true; dump_ring_window("tracepc", pc); }

    SimmervRetire e{};
    e.pc = pc; e.insn = insn; e.rd_kind = rd_kind; e.rd_idx = rd_idx;
    e.prv = prv; e.trapped = trapped; e.rd_val = rd_val;
    e.trap_cause = trap_cause; e.trap_tval = trap_tval; e.mtime = mtime; e.mepc = mepc;
    e.mem_kind = mem_kind; e.mem_pa = mem_pa;   // memory effect, compared below

    if (g_have_prev) {
        g_prev.next_pc = pc;            // in-order commit: this retire's pc
        g_seqno++;
        g_prev.seqno = g_seqno;
        step_compare(g_prev, g_prev_mtimecmp, g_prev_seip);
    }
    drain_dma_queue();                  // DMA beats older than the retire just held
    g_prev = e;
    g_prev_mtimecmp = mtimecmp;
    g_prev_seip = (seip != 0);
    g_have_prev = true;
}
