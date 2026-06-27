// DPI sink for the sharded-OoO performance event trace (docs/perf-observability-plan.md).
// Enabled by building a testbench with -DPERF_TRACE and adding this file to the
// verilator command; the RTL (backend_top.v, under `ifdef PERF_TRACE) calls perf_ev()
// once per pipeline event. Records are fixed 32-byte little-endian; perftool/ (Rust)
// reads them. Configured by environment (no plusarg plumbing):
//   PERF_TRACE_OUT   output path (default /tmp/perf_trace.bin)
//   PERF_TRACE_WIN   "start,len" in cycles -- capture only [start, start+len);
//                    unset = capture everything (use a short workload / window long runs!)
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
FILE*    g_fp   = nullptr;
bool     g_init = false;
bool     g_on   = false;        // tracing compiled + file opened ok
uint64_t g_lo   = 0;
uint64_t g_hi   = ~0ULL;        // capture window [lo, hi)
uint64_t g_n    = 0;            // records written

void init() {
   g_init = true;
   const char* out = std::getenv("PERF_TRACE_OUT");
   if (!out) out = "/tmp/perf_trace.bin";
   g_fp = std::fopen(out, "wb");
   if (!g_fp) { std::fprintf(stderr, "perf_trace: cannot open %s\n", out); return; }
   const char* win = std::getenv("PERF_TRACE_WIN");
   if (win) {
      unsigned long long s = 0, l = 0;
      if (std::sscanf(win, "%llu,%llu", &s, &l) == 2) { g_lo = s; g_hi = s + l; }
   }
   g_on = true;
   std::fprintf(stderr, "perf_trace: -> %s  window [%llu,%llu)\n", out,
                (unsigned long long)g_lo, (unsigned long long)g_hi);
}

struct Closer {                 // flush + report on normal $finish / exit
   ~Closer() {
      if (g_fp) { std::fflush(g_fp); std::fclose(g_fp); g_fp = nullptr; }
      if (g_on) std::fprintf(stderr, "perf_trace: %llu records\n", (unsigned long long)g_n);
   }
} g_closer;
}  // namespace

// One fixed 32-byte little-endian record (must match perftool/src/main.rs Rec):
//   [0..8) data u64  -- DISPATCH: pc,  WRITEBACK: wb_val
//   [16] kind u8   [17] seqno u8   [18] ckpid u8   [19] rdv u8
//   [20..22) pdst u16  [22..24) ps1 u16  [24..26) ps2 u16  [26..30) insn u32  [30..32) pad
// kind: 1=DISPATCH 2=SELECT 3=WRITEBACK 4=COMMIT 5=SQUASH.
extern "C" void perf_ev(long long cyc, int kind, int seq, int ckp, int rdv,
                        int pdst, int ps1, int ps2, long long data, int insn) {
   if (!g_init) init();
   if (!g_on) return;
   uint64_t c = (uint64_t)cyc;
   // cycle heartbeat (every 50M cyc) so a long run can be windowed: watch stderr to see
   // how cycles map to the workload, then set PERF_TRACE_WIN around the region of interest.
   static uint64_t hb = 0;
   if (c >= hb) {
      std::fprintf(stderr, "perf_trace: cyc %llu (%llu records captured)\n",
                   (unsigned long long)c, (unsigned long long)g_n);
      hb = c + 50000000ULL;
   }
   if (c < g_lo || c >= g_hi) return;
   uint8_t r[32];
   std::memset(r, 0, sizeof r);
   std::memcpy(r + 0, &c, 8);
   uint64_t dv = (uint64_t)data;
   std::memcpy(r + 8, &dv, 8);
   r[16] = (uint8_t)kind;
   r[17] = (uint8_t)seq;
   r[18] = (uint8_t)ckp;
   r[19] = (uint8_t)rdv;
   uint16_t u;
   u = (uint16_t)pdst; std::memcpy(r + 20, &u, 2);
   u = (uint16_t)ps1;  std::memcpy(r + 22, &u, 2);
   u = (uint16_t)ps2;  std::memcpy(r + 24, &u, 2);
   uint32_t iw = (uint32_t)insn; std::memcpy(r + 26, &iw, 4);
   std::fwrite(r, 1, sizeof r, g_fp);
   g_n++;
}
