// mlbench -- the GB5 Machine Learning inner loop, from a hardware trace.
//
// It is a DOT PRODUCT, and the trace is 9 instructions:
//
//     add   s0, a1, a2      ; next row pointer
//     flw   f1, (a1)        ; A[i]         -- independent
//     flw   f2, (a3)        ; B[i]         -- independent
//     fmuls f1, f1, f2      ; A[i]*B[i]    -- independent of the accumulator
//     fadds f0, f1, f0      ; acc += ...   <- ONE serial chain across ALL iterations
//     c.addi a3, a3, 4
//     c.addi s1, s1, -1
//     c.mv  a1, s0
//     c.bnez s1
//
// WHY THIS WORKLOAD SCORES 0. `f0` is a single accumulator carried by every iteration, so
// the loop's critical path is one fadds latency per element -- 8.00 cycles measured
// (workloads/fpbench). No amount of reordering, window or FP throughput shortens a
// dependence chain. The only levers are FP add LATENCY, and whether the machine can keep
// the adder fed from one iteration to the next.
//
// That second part is why this differs from blurbench. Blur had a 5-level chain over 13
// instructions -- 40 cycles of chain, trivially covered. This is a 1-level chain over 9
// instructions, so the next iteration's loads and multiply must already be in flight, ~9
// instructions back, against a 16-entry ROB. Marginal by construction, so the ROB may
// actually matter here where it did not for blur.
//
// SERIAL is the traced code. SPLIT4 is the same arithmetic with four accumulators, which a
// compiler would emit under -ffast-math; it is not what GB5 runs, but it bounds how much
// of the gap is the chain rather than the machine.
//
// Measure at VDEFS="-DOOO2_HW=4".

typedef unsigned char  uint8_t;
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
static void fixed2(uint64_t num, uint64_t den) {
    uint64_t w = den ? num / den : 0, f = den ? (num * 100 / den) % 100 : 0;
    putdec(w); putc_('.'); if (f < 10) putc_('0'); putdec(f);
}
static uint64_t rdcycle(void) { uint64_t v; __asm__ volatile("rdcycle %0":"=r"(v)); return v; }
static uint64_t rdinstr(void) { uint64_t v; __asm__ volatile("rdinstret %0":"=r"(v)); return v; }
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })
#define EV_ST_FPU 0x0303
#define EV_ST_MEM 0x0300
#define EV_FE_BUB 0x0310

#define N     8192
#define PASS  16
static float A[N], B[N];

int main(void) {
    SETEV(3, EV_ST_FPU); SETEV(4, EV_ST_MEM); SETEV(5, EV_FE_BUB);
    for (int i = 0; i < N; i++) { A[i] = (float)((i*31)&255) * 0.00390625f;
                                  B[i] = (float)((i*17)&255) * 0.00390625f; }
    volatile float sink = 0.0f;

    // ---- SERIAL: one accumulator, as traced ----
    uint64_t c0 = rdcycle(), i0 = rdinstr(), f0c = RDCNT(3), m0 = RDCNT(4), b0 = RDCNT(5);
    for (int p = 0; p < PASS; p++) {
        float acc = 0.0f;
        for (int i = 0; i < N; i++) acc = acc + A[i] * B[i];
        sink += acc;
    }
    uint64_t sc = rdcycle() - c0, si = rdinstr() - i0, sf = RDCNT(3) - f0c;
    uint64_t sm = RDCNT(4) - m0, sb = RDCNT(5) - b0;

    // ---- SPLIT4: four accumulators, chain depth /4 ----
    c0 = rdcycle(); i0 = rdinstr(); f0c = RDCNT(3);
    for (int p = 0; p < PASS; p++) {
        float a0=0, a1=0, a2=0, a3=0;
        for (int i = 0; i < N; i += 4) {
            a0 = a0 + A[i+0]*B[i+0]; a1 = a1 + A[i+1]*B[i+1];
            a2 = a2 + A[i+2]*B[i+2]; a3 = a3 + A[i+3]*B[i+3];
        }
        sink += (a0+a1) + (a2+a3);
    }
    uint64_t tc = rdcycle() - c0, ti = rdinstr() - i0, tf = RDCNT(3) - f0c;
    (void)sink;

    const uint64_t el = (uint64_t)N * PASS;
    puts_("mlbench: SERIAL elems="); putdec(el);
    puts_(" cycles=");               putdec(sc);
    puts_("\nmlbench:   cyc/elem=");  fixed2(sc, el);
    puts_("  insn/elem=");            fixed2(si, el);
    puts_("  ST_FPU/elem=");          fixed2(sf, el);
    puts_("\nmlbench:   ST_FPU=");     putdec(sf * 100 / sc);
    puts_("% ST_MEM=");                putdec(sm * 100 / sc);
    puts_("% FE_BUB=");                putdec(sb * 100 / sc);
    puts_("%\nmlbench: SPLIT4 cyc/elem="); fixed2(tc, el);
    puts_("  insn/elem=");                 fixed2(ti, el);
    puts_("  ST_FPU/elem=");               fixed2(tf, el);
    puts_("\nmlbench:   split4 speedup = "); fixed2(sc, tc);
    puts_("x   (fadds latency is 8.00 cyc; serial floor is 8.00 cyc/elem)\n");
    puts_("mlbench: done\n");
    for (;;);
}
