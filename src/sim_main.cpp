#include "Vsmolrv64_tb.h"
#include "verilated.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef VERILATOR_COSIM
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include "simmerv_cosim.h"

namespace {

constexpr uint64_t SRAM_BASE = 0x70000000ULL;
constexpr uint64_t AXI_BASE  = 0x80000000ULL;
#ifndef COSIM_MEM_SIZE_LG2
#define COSIM_MEM_SIZE_LG2 27
#endif
constexpr size_t   MEM_BYTES = 1ULL << COSIM_MEM_SIZE_LG2;

SimmervCtx* g_ctx    = nullptr;
uint64_t    g_seqno  = 0;

struct RingEntry { SimmervRetire dut; SimmervRetire ref; bool valid; };
constexpr size_t RING_N = 32;
RingEntry g_ring[RING_N] = {};
size_t    g_ring_idx = 0;

// Matches Verilog $readmemh: supports @ADDR directives for sparse layouts.
// `parity` 0 = even file (bytes 16*idx .. +7), 1 = odd file (bytes 16*idx+8 .. +15).
bool load_sparse(const char* path, int parity, std::vector<uint8_t>& ram) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cosim: cannot open %s\n", path);
        return false;
    }
    std::string line;
    uint64_t idx = 0;
    while (std::getline(in, line)) {
        size_t s = line.find_first_not_of(" \t\r\n");
        if (s == std::string::npos) continue;
        line = line.substr(s);
        if (line.empty() || line[0] == '/') continue;
        if (line[0] == '@') {
            idx = std::stoull(line.substr(1), nullptr, 16);
            continue;
        }
        uint64_t w = std::stoull(line, nullptr, 16);
        uint64_t byte_off = idx * 16 + (parity ? 8 : 0);
        if (byte_off + 8 > ram.size()) {
            std::fprintf(stderr, "cosim: %s offset 0x%llx out of range\n",
                         path, (unsigned long long)byte_off);
            return false;
        }
        for (int b = 0; b < 8; b++) ram[byte_off + b] = (w >> (8*b)) & 0xff;
        idx++;
    }
    return true;
}

bool load_image(uint64_t base, const char* even_path, const char* odd_path) {
    // Pre-size to the full RAM so simmerv's pre-installed DTB is overwritten
    // with zeros past the loaded image; matches smolrv64's zeroed BRAM.
    std::vector<uint8_t> ram(MEM_BYTES, 0);
    if (!load_sparse(even_path, 0, ram)) return false;
    if (!load_sparse(odd_path,  1, ram)) return false;
    if (simmerv_write_memory(g_ctx, base, ram.data(), ram.size()) != 0) {
        std::fprintf(stderr, "cosim: simmerv_write_memory(image) failed\n");
        return false;
    }
    return true;
}

// rf.hex: 32 lines of hex, one per integer register x0..x31.
bool load_rf(const char* path) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cosim: cannot open %s\n", path);
        return false;
    }
    std::string line;
    uint32_t idx = 0;
    while (std::getline(in, line) && idx < 32) {
        size_t s = line.find_first_not_of(" \t\r\n");
        if (s == std::string::npos) continue;
        line = line.substr(s);
        if (line.empty() || line[0] == '/' || line[0] == '@') continue;
        uint64_t v = std::stoull(line, nullptr, 16);
        simmerv_write_register(g_ctx, idx, v);
        idx++;
    }
    return true;
}

void dump_retire(const char* label, const SimmervRetire& r) {
    std::fprintf(stderr,
        "  %s seq=%llu pc=%016llx npc=%016llx insn=%08x prv=%u trap=%u "
        "rd=(k%u,x%u)=%016llx cause=%016llx tval=%016llx mt=%llu mepc=%016llx\n",
        label,
        (unsigned long long)r.seqno,
        (unsigned long long)r.pc,
        (unsigned long long)r.next_pc,
        r.insn, r.prv, r.trapped, r.rd_kind, r.rd_idx,
        (unsigned long long)r.rd_val,
        (unsigned long long)r.trap_cause,
        (unsigned long long)r.trap_tval,
        (unsigned long long)r.mtime,
        (unsigned long long)r.mepc);
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
    std::fflush(stderr);
    std::abort();
}

static inline uint32_t canonicalize_retired_insn(uint32_t insn) {
    // Simmerv preserves the following halfword in bits [31:16] for compressed
    // retirements, while the DUT may report only the architectural 16-bit
    // opcode. Compare compressed instructions on their meaningful low halfword.
    return (insn & 0x3) == 0x3 ? insn : (insn & 0xffffu);
}

// If the retiring insn is a Zicsr read of a CSR whose value is hardware-
// dependent (and so simmerv would naturally diverge), return its csrno;
// otherwise return -1. Covers the free-running counters that drift
// between DUT and REF: cycle/time/instret and their M-mode aliases.
static inline int csr_read_to_override(uint32_t insn) {
    if ((insn & 0x7f) != 0x73) return -1;            // not SYSTEM
    const uint32_t f3 = (insn >> 12) & 0x7;
    // Zicsr funct3: 001/010/011 (CSRRW/RS/RC), 101/110/111 (immediate forms).
    if (f3 == 0 || f3 == 4) return -1;
    const uint32_t csrno = (insn >> 20) & 0xfff;
    switch (csrno) {
        case 0xC00:  // cycle
        case 0xC01:  // time
        case 0xC02:  // instret
        case 0xB00:  // mcycle
        case 0xB02:  // minstret
        case 0xF11:  // mvendorid  — implementation-defined ID, differs by model
        case 0xF12:  // marchid    — implementation-defined ID, differs by model
        case 0xF13:  // mimpid     — implementation/build stamp, differs by model
            return (int)csrno;
        default:
            // HPM counters (mhpmcounter3..31 = 0xB03..0xB1F, hpmcounter3..31 =
            // 0xC03..0xC1F) and event selectors (mhpmevent3..31 = 0x323..0x33F)
            // are model-specific, so let the DUT's read value win — same policy
            // as cycle/instret.
            if ((csrno >= 0xB03 && csrno <= 0xB1F) ||
                (csrno >= 0xC03 && csrno <= 0xC1F) ||
                (csrno >= 0x323 && csrno <= 0x33F))
                return (int)csrno;
            return -1;
    }
}

} // namespace

// DPI callback from smolrv64.v (one per retired instruction or trap).
extern "C" void cosim_retire(
    unsigned long long pc,
    unsigned long long next_pc,
    unsigned int       insn,
    unsigned char      rd_kind,
    unsigned char      rd_idx,
    unsigned char      prv,
    unsigned char      trapped,
    unsigned int       fflags,
    unsigned long long rd_val,
    unsigned long long trap_cause,
    unsigned long long trap_tval,
    unsigned long long mtime,
    unsigned long long mtimecmp,
    unsigned long long mepc,
    unsigned char      seip)
{
    g_seqno++;

    SimmervRetire dut{};
    dut.pc         = pc;
    dut.next_pc    = next_pc;
    dut.insn       = insn;
    dut.rd_kind    = rd_kind;
    dut.rd_idx     = rd_idx;
    dut.prv        = prv;
    dut.trapped    = trapped;
    dut.fflags     = fflags;
    dut.rd_val     = rd_val;
    dut.trap_cause = trap_cause;
    dut.trap_tval  = trap_tval;
    dut.mtime      = mtime;
    dut.seqno      = g_seqno;
    dut.mepc       = mepc;

    simmerv_set_mtime(g_ctx, mtime);
    // Only allow simmerv to take MTIP exactly when DUT does. DUT's mtip bit
    // latches as soon as mtime>=mtimecmp, but DUT only vectors at the next
    // fetch boundary; simmerv would otherwise fire immediately. Gate by the
    // actual trap retire with machine-timer cause (0x8000000000000007).
    const unsigned long long MTIP_CAUSE = 0x8000000000000007ULL;
    const bool dut_taking_mtip = trapped && trap_cause == MTIP_CAUSE;
    simmerv_set_mtimecmp(g_ctx, dut_taking_mtip ? mtimecmp : ~0ULL);
    // Same gating for the supervisor timer (Sstc STIP, cause ...5): the DUT's
    // registered pre_intr_pending vectors it a retire later than simmerv's
    // mtime>=stimecmp would. Let simmerv take STIP only when the DUT does.
    const unsigned long long STIP_CAUSE = 0x8000000000000005ULL;
    const bool dut_taking_stip = trapped && trap_cause == STIP_CAUSE;
    simmerv_set_stip_armed(g_ctx, dut_taking_stip);
    // Mirror DUT's PLIC→SEIP line: the two sims have independent UART/PLIC
    // state, so force simmerv's supervisor-external-interrupt bit to match.
    simmerv_set_seip(g_ctx, seip != 0);
    // Mirror DUT's only PLIC-connected IRQ (UART = 10) into simmerv's PLIC
    // pending mask so claim/pending MMIO reads return the same IRQ number.
    simmerv_set_plic_ip(g_ctx, 10, seip != 0);
    // For free-running counter CSRs (cycle/time/instret and M-aliases),
    // force simmerv's next read to return the DUT's read result. Each
    // counter advances at a model-specific rate, so direct comparison is
    // unstable — let the DUT's value win. Only meaningful when the DUT
    // actually wrote a destination register, in which case rd_val IS the
    // read result.
    if (!trapped && rd_kind != 0) {
        const int override_csr = csr_read_to_override(insn);
        if (override_csr >= 0) {
            simmerv_arm_csr_read(g_ctx, (uint16_t)override_csr, rd_val);
        }
    }
    SimmervRetire ref{};
    if (simmerv_step_retire(g_ctx, &ref) != 0) {
        std::fprintf(stderr, "cosim: simmerv_step_retire failed at seq %llu\n",
                     (unsigned long long)g_seqno);
        std::abort();
    }
    // simmerv counts retirements from 0 while the DUT path counts from 1;
    // they refer to the same retirement, so label them identically for the
    // history/mismatch dumps. (seqno is display-only — not in the compare.)
    ref.seqno = g_seqno;

    const uint32_t dut_insn_cmp = canonicalize_retired_insn(dut.insn);
    const uint32_t ref_insn_cmp = canonicalize_retired_insn(ref.insn);

    const bool ok =
        dut.pc         == ref.pc        &&
        dut.next_pc    == ref.next_pc   &&
        dut_insn_cmp   == ref_insn_cmp  &&
        dut.rd_kind    == ref.rd_kind   &&
        dut.rd_idx     == ref.rd_idx    &&
        dut.prv        == ref.prv       &&
        dut.trapped    == ref.trapped   &&
        (dut.rd_kind == 0 || dut.rd_val == ref.rd_val) &&
        dut.trap_cause == ref.trap_cause &&
        dut.trap_tval  == ref.trap_tval &&
        dut.mepc       == ref.mepc;

    g_ring[g_ring_idx] = { dut, ref, true };
    g_ring_idx = (g_ring_idx + 1) % RING_N;

    if (g_seqno == 1 || (g_seqno % 1'000'000) == 0) {
        std::fprintf(stderr, "cosim: %llu retirements ok (pc=%016llx)\n",
                     (unsigned long long)g_seqno, (unsigned long long)dut.pc);
    }

    if (!ok) mismatch_abort(dut, ref);
}

static const char* parse_plusarg(int argc, char** argv, const char* key) {
    size_t klen = std::strlen(key);
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '+' && !std::strncmp(argv[i] + 1, key, klen)
            && argv[i][1 + klen] == '=')
            return argv[i] + 2 + klen;
    }
    return nullptr;
}

static uint64_t parse_plusarg_hex(int argc, char** argv, const char* key, uint64_t fallback) {
    if (const char* val = parse_plusarg(argc, argv, key))
        return std::strtoull(val, nullptr, 16);
    return fallback;
}
#endif // VERILATOR_COSIM

// ---- Interactive UART RX -----------------------------------------------
// DPI-C counterpart of Icarus' tty_vpi.c $tty_read: hands host stdin bytes
// to the modeled UART. Returns the next byte (0..255) or -1 when none is
// available. The testbench imports this only when built under Verilator.
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace {
bool           g_tty_inited = false;
struct termios g_tty_orig;

void tty_restore() {
    if (g_tty_inited) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_tty_orig);
        g_tty_inited = false;
    }
}

void tty_init() {
    // Non-blocking so read() never stalls the simulation.
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    // Raw mode (no canonical line buffering / echo) for interactive use;
    // harmlessly skipped when stdin is a pipe or file.
    if (tcgetattr(STDIN_FILENO, &g_tty_orig) == 0) {
        struct termios raw = g_tty_orig;
        raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        raw.c_cflag &= ~(CSIZE | PARENB);
        raw.c_cflag |= CS8;
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        std::atexit(tty_restore);
    }
    g_tty_inited = true;
}
} // namespace

extern "C" int tty_read() {
    // Dispense buffered bytes one per clock so pasted input flows quickly...
    static unsigned char buf[256];
    static int have = 0, pos = 0;
    if (pos < have) return buf[pos++];
    // ...but only hit the read() syscall occasionally: human typing is
    // glacial next to simulated time, and polling every clock would swamp
    // Verilator with syscalls and gut its throughput.
    static unsigned throttle = 0;
    if (throttle++ & 0xfff) return -1;
    if (!g_tty_inited) tty_init();
    int n = read(STDIN_FILENO, buf, sizeof buf);
    if (n <= 0) return -1;   // EAGAIN (no data) or EOF
    have = n;
    pos  = 0;
    return buf[pos++];
}

int main(int argc, char** argv) {
    // Force stdout line-buffered so UART bytes from Verilog $write survive
    // SIGTERM / timeout when piped (e.g. `make run | tee …`).
    setvbuf(stdout, nullptr, _IOLBF, 0);
    Verilated::commandArgs(argc, argv);

#ifdef VERILATOR_COSIM
    g_ctx = simmerv_create(MEM_BYTES);
    if (!g_ctx) {
        std::fprintf(stderr, "cosim: simmerv_create failed\n");
        return 1;
    }
    const char* even = parse_plusarg(argc, argv, "even");
    const char* odd  = parse_plusarg(argc, argv, "odd");
    if (!even || !odd) {
        std::fprintf(stderr, "cosim: need +even=<path> +odd=<path>\n");
        return 1;
    }
    if (!load_image(AXI_BASE, even, odd)) return 1;
    if (const char* sram_even = parse_plusarg(argc, argv, "sram_even")) {
        const char* sram_odd = parse_plusarg(argc, argv, "sram_odd");
        if (!sram_odd) {
            std::fprintf(stderr, "cosim: need +sram_odd=<path> with +sram_even=<path>\n");
            return 1;
        }
        if (!load_image(SRAM_BASE, sram_even, sram_odd)) return 1;
    } else if (parse_plusarg(argc, argv, "sram_odd")) {
        std::fprintf(stderr, "cosim: need +sram_even=<path> with +sram_odd=<path>\n");
        return 1;
    }
    simmerv_zero_registers(g_ctx);
    if (const char* rf = parse_plusarg(argc, argv, "rf")) {
        if (!load_rf(rf)) return 1;
    }
    simmerv_set_pc(g_ctx, parse_plusarg_hex(argc, argv, "reset_pc", AXI_BASE));
    simmerv_set_mtime(g_ctx, 0);
#endif

    Vsmolrv64_tb* top = new Vsmolrv64_tb;
    while (!Verilated::gotFinish()) {
        top->clock = 0; top->eval();
        Verilated::timeInc(1);
        top->clock = 1; top->eval();
        Verilated::timeInc(1);
    }
    delete top;

#ifdef VERILATOR_COSIM
    simmerv_destroy(g_ctx);
    std::fprintf(stderr, "cosim: completed %llu retirements without divergence\n",
                 (unsigned long long)g_seqno);
#endif
    return 0;
}
