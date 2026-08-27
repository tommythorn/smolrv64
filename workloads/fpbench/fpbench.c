// fpbench -- deterministic FP kernels, separating FPU LATENCY from FPU THROUGHPUT.
//
// WHY BOTH. The claim this exists to test is that ST_FPU regressed 0.973 -> 1.148 CPI
// when FP arithmetic was moved into the in-order scheduler, where it queues behind every
// load, mul/div, CSR and branch. That claim came from two different GB5 runs with
// different workload mixes, which is not evidence -- it is a correlation between two
// numbers that were never meant to be compared.
//
// A dependent chain cannot detect the defect: back-to-back dependent FP ops are limited by
// FPU latency no matter what the scheduler does. Only INDEPENDENT ops can expose an issue
// bottleneck. So both are measured, and the ratio is the interesting number:
//
//   lat  = cycles per op in ONE serial chain            -> FPU latency
//   thru = cycles per op with NCHAIN independent chains -> issue throughput
//
// If thru ~= lat, nothing overlaps and issue is the constraint (what P3 would fix).
// If thru << lat, the FPU is already pipelined and P3 buys little on this shape.
//
// Bare-metal M-mode, boots at 0x8000_0000:  FW=fpbench.bin ooo2/run-ooo2-linux.sh
// Measure at VDEFS="-DOOO2_HW=4" -- the sim default is a fetch width no hardware uses.

typedef unsigned char  uint8_t;
typedef unsigned int   uint32_t;
typedef unsigned long  uint64_t;

#define UART     ((volatile uint8_t *)0x10000000)
#define LSR_THRE 0x20
static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void putdec(uint64_t v) {
    char b[24]; int n = 0;
    if (!v) { putc_('0'); return; }
    while (v) { b[n++] = '0' + (v % 10); v /= 10; }
    while (n) putc_(b[--n]);
}
static void fixed2(uint64_t num, uint64_t den) {      // print num/den to 2 decimals
    uint64_t w = den ? num / den : 0, f = den ? (num * 100 / den) % 100 : 0;
    putdec(w); putc_('.'); if (f < 10) putc_('0'); putdec(f);
}
static uint64_t rdcycle(void) { uint64_t v; __asm__ volatile("rdcycle %0":"=r"(v)); return v; }
static uint64_t rdinstr(void) { uint64_t v; __asm__ volatile("rdinstret %0":"=r"(v)); return v; }

#define EV_ST_FPU 0x0303
#define EV_ST_MEM 0x0300
#define EV_FE_BUB 0x0310
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })

#define NCHAIN 8
#define NITER  200000

// volatile so the compiler cannot hoist, fold, or vectorise the chains away
static volatile double seed_a = 1.0000001, seed_b = 0.9999999;

int main(void) {
    SETEV(3, EV_ST_FPU); SETEV(4, EV_ST_MEM); SETEV(5, EV_FE_BUB);

    double a = seed_a, b = seed_b;
    double x[NCHAIN];
    for (int i = 0; i < NCHAIN; i++) x[i] = 1.0 + i * 0.125;

    // ---- LATENCY: one serial chain. Each fma waits for the previous result. ----
    uint64_t c0 = rdcycle(), i0 = rdinstr(), f0 = RDCNT(3);
    double s = x[0];
    for (int i = 0; i < NITER; i++) s = s * a + b;
    uint64_t lat_c = rdcycle() - c0, lat_i = rdinstr() - i0, lat_f = RDCNT(3) - f0;
    volatile double sink0 = s;

    // ---- THROUGHPUT: NCHAIN independent chains, same op count per chain. ----
    c0 = rdcycle(); i0 = rdinstr(); f0 = RDCNT(3);
    uint64_t m0 = RDCNT(4), e0 = RDCNT(5);
    double y0=x[0],y1=x[1],y2=x[2],y3=x[3],y4=x[4],y5=x[5],y6=x[6],y7=x[7];
    for (int i = 0; i < NITER; i++) {
        y0 = y0*a + b; y1 = y1*a + b; y2 = y2*a + b; y3 = y3*a + b;
        y4 = y4*a + b; y5 = y5*a + b; y6 = y6*a + b; y7 = y7*a + b;
    }
    uint64_t thr_c = rdcycle() - c0, thr_i = rdinstr() - i0, thr_f = RDCNT(3) - f0;
    uint64_t thr_m = RDCNT(4) - m0, thr_e = RDCNT(5) - e0;
    volatile double sink1 = y0+y1+y2+y3+y4+y5+y6+y7;
    (void)sink0; (void)sink1;

    const uint64_t lat_ops = NITER, thr_ops = (uint64_t)NITER * NCHAIN;

    puts_("fpbench: LATENCY  ops="); putdec(lat_ops);
    puts_(" cycles=");               putdec(lat_c);
    puts_("\nfpbench:   cyc/op=");   fixed2(lat_c, lat_ops);
    puts_("  insn/op=");             fixed2(lat_i, lat_ops);
    puts_("  ST_FPU/op=");           fixed2(lat_f, lat_ops);

    puts_("\nfpbench: THRUPUT ops="); putdec(thr_ops);
    puts_(" cycles=");                putdec(thr_c);
    puts_("\nfpbench:   cyc/op=");    fixed2(thr_c, thr_ops);
    puts_("  insn/op=");              fixed2(thr_i, thr_ops);
    puts_("  ST_FPU/op=");            fixed2(thr_f, thr_ops);
    puts_("\nfpbench:   ST_FPU=");     putdec(thr_f * 100 / thr_c);
    puts_("% ST_MEM=");                putdec(thr_m * 100 / thr_c);
    puts_("% FE_BUB=");                putdec(thr_e * 100 / thr_c);
    puts_("%\nfpbench:   overlap = lat/thru = ");
    fixed2(lat_c * 100 / lat_ops * thr_ops, thr_c * 100);
    puts_("x  (1.00 = nothing overlaps, issue-bound)\n");
    puts_("fpbench: done\n");
    for (;;);
}
