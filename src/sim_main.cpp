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

constexpr uint64_t MEM_BASE  = 0x80000000ULL;
constexpr size_t   MEM_BYTES = 128ULL * 1024 * 1024;   // matches MEM_SIZE_LG2=27
constexpr uint64_t RESET_PC  = 0x80000000ULL;          // matches smolrv64 default

SimmervCtx* g_ctx    = nullptr;
uint64_t    g_seqno  = 0;

struct RingEntry { SimmervRetire dut; SimmervRetire ref; bool valid; };
constexpr size_t RING_N = 32;
RingEntry g_ring[RING_N] = {};
size_t    g_ring_idx = 0;

bool read_hex_words(const char* path, std::vector<uint64_t>& out) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cosim: cannot open %s\n", path);
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        size_t s = line.find_first_not_of(" \t\r\n");
        if (s == std::string::npos) continue;
        line = line.substr(s);
        if (line.empty() || line[0] == '/' || line[0] == '@') continue;
        out.push_back(std::stoull(line, nullptr, 16));
    }
    return true;
}

bool load_image(const char* even_path, const char* odd_path) {
    std::vector<uint64_t> even, odd;
    if (!read_hex_words(even_path, even)) return false;
    if (!read_hex_words(odd_path,  odd))  return false;
    size_t n_pairs = std::max(even.size(), odd.size());
    std::vector<uint8_t> ram(n_pairs * 16, 0);
    for (size_t i = 0; i < even.size(); i++) {
        uint64_t w = even[i];
        for (int b = 0; b < 8; b++) ram[16*i + b] = (w >> (8*b)) & 0xff;
    }
    for (size_t i = 0; i < odd.size(); i++) {
        uint64_t w = odd[i];
        for (int b = 0; b < 8; b++) ram[16*i + 8 + b] = (w >> (8*b)) & 0xff;
    }
    if (simmerv_write_memory(g_ctx, MEM_BASE, ram.data(), ram.size()) != 0) {
        std::fprintf(stderr, "cosim: simmerv_write_memory(image) failed\n");
        return false;
    }
    // Zero the tail so simmerv's pre-installed DTB doesn't diverge from
    // smolrv64's zero-initialized BRAM beyond the loaded image.
    constexpr size_t CHUNK = 1 << 20;
    std::vector<uint8_t> zero(CHUNK, 0);
    for (uint64_t off = ram.size(); off < MEM_BYTES; off += CHUNK) {
        size_t n = std::min((uint64_t)CHUNK, MEM_BYTES - off);
        simmerv_write_memory(g_ctx, MEM_BASE + off, zero.data(), n);
    }
    return true;
}

void dump_retire(const char* label, const SimmervRetire& r) {
    std::fprintf(stderr,
        "  %s seq=%llu pc=%016llx npc=%016llx insn=%08x prv=%u trap=%u "
        "rd=(k%u,x%u)=%016llx cause=%016llx tval=%016llx mt=%llu\n",
        label,
        (unsigned long long)r.seqno,
        (unsigned long long)r.pc,
        (unsigned long long)r.next_pc,
        r.insn, r.prv, r.trapped, r.rd_kind, r.rd_idx,
        (unsigned long long)r.rd_val,
        (unsigned long long)r.trap_cause,
        (unsigned long long)r.trap_tval,
        (unsigned long long)r.mtime);
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
    unsigned long long mtime)
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

    simmerv_set_mtime(g_ctx, mtime);
    SimmervRetire ref{};
    if (simmerv_step_retire(g_ctx, &ref) != 0) {
        std::fprintf(stderr, "cosim: simmerv_step_retire failed at seq %llu\n",
                     (unsigned long long)g_seqno);
        std::abort();
    }

    const bool rd_writes = dut.rd_kind != 0 && dut.rd_idx != 0;
    const bool ok =
        dut.pc      == ref.pc      &&
        dut.next_pc == ref.next_pc &&
        dut.insn    == ref.insn    &&
        dut.prv     == ref.prv     &&
        dut.trapped == ref.trapped &&
        dut.rd_kind == ref.rd_kind &&
        dut.rd_idx  == ref.rd_idx  &&
        (!rd_writes || dut.rd_val == ref.rd_val) &&
        (!dut.trapped ||
            (dut.trap_cause == ref.trap_cause &&
             dut.trap_tval  == ref.trap_tval));

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
#endif // VERILATOR_COSIM

int main(int argc, char** argv) {
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
    if (!load_image(even, odd)) return 1;
    simmerv_zero_registers(g_ctx);
    simmerv_set_pc(g_ctx, RESET_PC);
    simmerv_set_mtime(g_ctx, 0);
#endif

    Vsmolrv64_tb* top = new Vsmolrv64_tb;
    while (!Verilated::gotFinish()) {
        top->clock = 0; top->eval();
        top->clock = 1; top->eval();
    }
    delete top;

#ifdef VERILATOR_COSIM
    simmerv_destroy(g_ctx);
    std::fprintf(stderr, "cosim: completed %llu retirements without divergence\n",
                 (unsigned long long)g_seqno);
#endif
    return 0;
}
