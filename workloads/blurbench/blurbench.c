// blurbench -- the GB5 Gaussian Blur inner loop, from a hardware trace.
//
// The trace (5-tap FIR, one output pixel per iteration) has this shape:
//
//     flw f4,(a2); fmuls f4,f4,f0      taps: loads and muls all INDEPENDENT
//     flw f5,(a5); fmuls f5,f5,f3
//     fadds f4,f5,f4    <-+
//     flw f5,(s1)         |
//     fadds f4,f4,f5    <-+  SERIAL chain of 4 through the accumulator f4
//     flw f5,(a4); fmuls  |
//     fadds f4,f4,f5    <-+
//     flw f5,(a3); fmuls  |
//     fadds f4,f4,f5    <-+
//     fsw f4,(s1)
//
// So the reduction is LATENCY-bound, not throughput-bound, and FP multiple-in-flight
// (which took independent FP from 4.00 to 2.25 cyc/op and left dependent chains at 8.00)
// should do little for it. What CAN hide the chain is overlapping the next iteration --
// its loads and muls depend on nothing here -- and the loop is ~19 instructions against a
// 16-entry ROB, so the window is what decides how much overlaps. That is the likely reason
// 8/8 schedulers cost Gaussian Blur 6.8% and Ray Tracing 16.7% against 10/12.
//
// Two variants measure exactly that split:
//   serial = the traced code, one accumulator, 4-deep dependent chain
//   tree   = same arithmetic, pairwise reduction, chain depth 2
// If tree >> serial, the machine is latency-bound on the chain and the fix is window or
// code shape, not FPU throughput.
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
#define EV_ST_ROB 0x0305   /* dispatch blocked: ROB full */
#define EV_ST_MEM 0x0300
#define EV_FE_BUB 0x0310

#define N     16384            /* 64 KiB per plane */
#define PASS  8
static float src[N + 8], dst[N + 8];

int main(void) {
    SETEV(3, EV_ST_FPU); SETEV(4, EV_ST_MEM); SETEV(5, EV_FE_BUB);
    const float k0 = 0.0625f, k1 = 0.25f, k2 = 0.375f, k3 = 0.25f, k4 = 0.0625f;
    for (int i = 0; i < N + 8; i++) src[i] = (float)((i * 37) & 255) * 0.00390625f;

    // ---- SERIAL: the traced shape, one accumulator ----
    uint64_t c0 = rdcycle(), i0 = rdinstr(), f0 = RDCNT(3), m0 = RDCNT(4), b0 = RDCNT(5);
    for (int p = 0; p < PASS; p++)
        for (int i = 0; i < N; i++) {
            float a = src[i] * k0;
            a = src[i+1] * k1 + a;
            a = a + src[i+2] * k2;
            a = a + src[i+3] * k3;
            a = a + src[i+4] * k4;
            dst[i] = a;
        }
    uint64_t sc = rdcycle() - c0, si = rdinstr() - i0, sf = RDCNT(3) - f0;
    uint64_t sm = RDCNT(4) - m0, sb = RDCNT(5) - b0;

    // ---- TREE: identical arithmetic, chain depth 2 instead of 4 ----
    c0 = rdcycle(); i0 = rdinstr(); f0 = RDCNT(3);
    for (int p = 0; p < PASS; p++)
        for (int i = 0; i < N; i++) {
            float x = src[i] * k0 + src[i+1] * k1;      // independent...
            float y = src[i+2] * k2 + src[i+3] * k3;    // ...of this
            dst[i] = (x + y) + src[i+4] * k4;
        }
    uint64_t tc = rdcycle() - c0, ti = rdinstr() - i0, tf = RDCNT(3) - f0;

    volatile float sink = dst[0] + dst[N-1]; (void)sink;
    const uint64_t px = (uint64_t)N * PASS;

    puts_("blurbench: SERIAL px="); putdec(px);
    puts_(" cycles=");              putdec(sc);
    puts_("\nblurbench:   cyc/px=");fixed2(sc, px);
    puts_("  insn/px=");            fixed2(si, px);
    puts_("  ST_FPU/px=");          fixed2(sf, px);
    puts_("\nblurbench:   ST_FPU="); putdec(sf * 100 / sc);
    puts_("% ST_MEM=");              putdec(sm * 100 / sc);
    puts_("% FE_BUB=");              putdec(sb * 100 / sc);

    puts_("%\nblurbench: TREE   cyc/px="); fixed2(tc, px);
    puts_("  insn/px=");                   fixed2(ti, px);
    puts_("  ST_FPU/px=");                 fixed2(tf, px);
    puts_("\nblurbench:   tree speedup = "); fixed2(sc, tc);
    puts_("x  (>1 = the serial accumulator chain is the limit)\n");
    puts_("blurbench: done\n");
    for (;;);
}
