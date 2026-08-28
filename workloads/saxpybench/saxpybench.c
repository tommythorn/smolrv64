// saxpybench -- the GB5 Camera inner loop, from a hardware trace.
//
// Camera is classified INTEGER by GB5 and scores 1, so it drags the Integer geomean; its
// hot loop is nonetheless FP. The trace is 9 instructions per element:
//
//     2 flw    x[i], y[i]
//     1 fmuls  a*x[i]
//     1 fadds  + y[i]        <- NOT carried between iterations
//     1 fsw    y[i]
//     3 c.addi pointer bumps
//     1 c.bnez
//
// THE KEY DIFFERENCE FROM blurbench/mlbench: there is no accumulator carried across
// iterations. Every element is independent, so the dependence chain is only mul->add->store
// within one element. Nothing about the DATA stops the machine going as fast as it can
// issue -- which makes this a clean test of window and issue width rather than of latency.
//
// tools/trace-limit.py on the real trace: IW1/W16 0.93, IW2/W16 1.09, IW2/W64 2.00. So it
// is WINDOW-limited: 9 instructions per iteration cannot cover an 8-cycle FP latency
// without ~2 iterations in flight.
//
// L1-RESIDENT ON PURPOSE. The point is to isolate the window, so a cache miss must not be
// what limits it. STREAM (below) is the same loop over a footprint larger than the D$, for
// the case where that matters -- Camera does store every iteration, which is where a store
// buffer would tell.
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
#define EV_ST_ROB 0x0305

#define NR   2048               /* 8 KiB per array -- both fit the 64 KiB D$ */
#define PASS 64
#define NS   16384              /* 64 KiB per array -- 128 KiB total, 2x the D$ */
static float x[NS], y[NS];

int main(void) {
    SETEV(3, EV_ST_FPU); SETEV(4, EV_ST_MEM); SETEV(5, EV_FE_BUB); SETEV(6, EV_ST_ROB);
    const float a = 1.0000001f;
    for (int i = 0; i < NS; i++) { x[i] = (float)((i*13)&255) * 0.00390625f; y[i] = 1.0f; }

    // ---- RESIDENT: 8 KiB arrays, 64 passes. Window/width test, no misses. ----
    uint64_t c0 = rdcycle(), i0 = rdinstr(), f0 = RDCNT(3), m0 = RDCNT(4), b0 = RDCNT(5), r0 = RDCNT(6);
    for (int p = 0; p < PASS; p++)
        for (int i = 0; i < NR; i++) y[i] = a * x[i] + y[i];
    uint64_t rc = rdcycle()-c0, ri = rdinstr()-i0, rf = RDCNT(3)-f0;
    uint64_t rm = RDCNT(4)-m0, rb = RDCNT(5)-b0, rr = RDCNT(6)-r0;

    // ---- STREAM: 64 KiB arrays, 8 passes. Same element count, 2x the D$ footprint. ----
    c0 = rdcycle(); i0 = rdinstr(); f0 = RDCNT(3); m0 = RDCNT(4);
    for (int p = 0; p < 8; p++)
        for (int i = 0; i < NS; i++) y[i] = a * x[i] + y[i];
    uint64_t sc = rdcycle()-c0, si = rdinstr()-i0, sf = RDCNT(3)-f0, sm = RDCNT(4)-m0;

    const uint64_t rn = (uint64_t)NR * PASS, sn = (uint64_t)NS * 8;

    puts_("saxpybench: RESIDENT elems="); putdec(rn);
    puts_("\nsaxpybench:   cyc/elem=");   fixed2(rc, rn);
    puts_("  insn/elem=");                fixed2(ri, rn);
    puts_("\nsaxpybench:   ST_FPU=");      putdec(rf*100/rc);
    puts_("% ST_MEM=");                    putdec(rm*100/rc);
    puts_("% FE_BUB=");                    putdec(rb*100/rc);
    puts_("% ST_ROB=");                    putdec(rr*100/rc);
    puts_("%\nsaxpybench:   raw cyc=");    putdec(rc);
    puts_(" ST_FPU="); putdec(rf); puts_(" ST_MEM="); putdec(rm);
    puts_(" FE_BUB="); putdec(rb); puts_(" ST_ROB="); putdec(rr);
    puts_("\nsaxpybench: STREAM   cyc/elem="); fixed2(sc, sn);
    puts_("  insn/elem=");                     fixed2(si, sn);
    puts_("  ST_MEM=");                        putdec(sm*100/sc);
    puts_("%\nsaxpybench: done\n");
    for (;;);
}
