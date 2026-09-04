// stbench -- store THROUGHPUT, L1-resident, the twin of workloads/ldbench for the store path.
//
// The boot's stall composition (docs/OOO2-Spec.md P0, SB-WHERE2) puts dispatch held on a
// FULL STORE QUEUE ahead of every other stall, at DDR_LAT=4 and at 80 alike. Before touching
// the store path, measure what a store costs when it HITS, so the hit cost and the miss cost
// are separable:
//
//   thru   = cycles per store over NWAY independent streams  -> the drain rate of ooo2_sq
//   mix    = cycles per op, one store then one load, independent addresses -> port sharing
//   fwd    = cycles per pair, a store then a load of the SAME word -> the alias hold
//            (no forwarding: the load waits for the store to commit)
//
// Everything fits the 64 KiB D$. Bare-metal M-mode. Measure at VDEFS="-DOOO2_HW=4".

typedef unsigned char  uint8_t;
typedef unsigned int   uint32_t;
typedef unsigned long  uint64_t;

#define UART     ((volatile uint8_t *)0x10000000)
#define LSR_THRE 0x20
static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void putdec(uint64_t v) {
    char b[24]; int n = 0;
    do { b[n++] = '0' + v % 10; v /= 10; } while (v);
    while (n) putc_(b[--n]);
}
// v/d with two decimals
static void fixed2(uint64_t v, uint64_t d) {
    uint64_t q = v / d, r = ((v % d) * 100 + d / 2) / d;
    if (r >= 100) { q++; r -= 100; }
    putdec(q); putc_('.'); putc_('0' + r / 10); putc_('0' + r % 10);
}
static uint64_t rdcycle(void) { uint64_t v; __asm__ volatile("rdcycle %0":"=r"(v)); return v; }
static uint64_t rdinstr(void) { uint64_t v; __asm__ volatile("rdinstret %0":"=r"(v)); return v; }
#define EV_ST_MEM 0x0300
#define EV_DCACC  0x0100
#define EV_DCMISS 0x0102
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })

#define NSLOT  4096                    /* 32 KiB: fits the 64 KiB D$ */
#define NITER  200000
#define NWAY   8
static uint64_t buf[NSLOT];

static void report(const char *name, uint64_t n, uint64_t c, uint64_t i, uint64_t m,
                   uint64_t a, uint64_t s) {
    puts_("stbench: "); puts_(name); puts_(" ops="); putdec(n);
    puts_(" cycles="); putdec(c);
    puts_("\nstbench:   cyc/op="); fixed2(c, n);
    puts_("  insn/op="); fixed2(i, n);
    puts_("  ST_MEM/op="); fixed2(m, n);
    puts_("  D$acc="); putdec(a); puts_(" D$miss="); putdec(s);
    puts_("\n");
}

int main(void) {
    SETEV(3, EV_ST_MEM); SETEV(4, EV_DCACC); SETEV(5, EV_DCMISS);
    for (int i = 0; i < NSLOT; i++) buf[i] = i;                          // warm, all lines

    // ---- THROUGHPUT: 8 stores per iteration, one instruction each (sd with an immediate
    // offset off one base), to 8 different lines; the base then steps by 8 lines, wrapping
    // inside the buffer. About 1.4 instructions per store, so the IW=1 floor is ~1.4. ----
    volatile uint64_t *v = buf;
    uint64_t c0 = rdcycle(), i0 = rdinstr(), m0 = RDCNT(3), a0 = RDCNT(4), s0 = RDCNT(5);
    for (int i = 0; i < NITER; i++) {
        volatile uint64_t *p = v + ((i * 64) & (NSLOT - 1));
        p[0] = i; p[8] = i; p[16] = i; p[24] = i; p[32] = i; p[40] = i; p[48] = i; p[56] = i;
    }
    report("THRUPUT (store)", (uint64_t)NITER * 8, rdcycle() - c0, rdinstr() - i0,
           RDCNT(3) - m0, RDCNT(4) - a0, RDCNT(5) - s0);

    // ---- MIX: 4 stores and 4 loads per iteration, all to different lines, 1:1 like the boot.
    c0 = rdcycle(); i0 = rdinstr(); m0 = RDCNT(3); a0 = RDCNT(4); s0 = RDCNT(5);
    uint64_t acc = 0;
    for (int i = 0; i < NITER; i++) {
        volatile uint64_t *p = v + ((i * 64) & (NSLOT - 1));
        p[0] = i; acc += p[8]; p[16] = i; acc += p[24];
        p[32] = i; acc += p[40]; p[48] = i; acc += p[56];
    }
    report("MIX (store+load)", (uint64_t)NITER * 8, rdcycle() - c0, rdinstr() - i0,
           RDCNT(3) - m0, RDCNT(4) - a0, RDCNT(5) - s0);
    volatile uint64_t sink = acc;

    // ---- FWD: a store, then a load of the SAME word, 4 pairs per iteration on different
    // lines: the alias hold (no forwarding: the load waits for the store to commit). ----
    c0 = rdcycle(); i0 = rdinstr(); m0 = RDCNT(3); a0 = RDCNT(4); s0 = RDCNT(5);
    acc = 0;
    for (int i = 0; i < NITER / 2; i++) {
        volatile uint64_t *p = v + ((i * 64) & (NSLOT - 1));
        p[0] = i; acc += p[0]; p[16] = i; acc += p[16];
        p[32] = i; acc += p[32]; p[48] = i; acc += p[48];
    }
    report("FWD (store,load same word) pairs", (uint64_t)NITER * 2, rdcycle() - c0,
           rdinstr() - i0, RDCNT(3) - m0, RDCNT(4) - a0, RDCNT(5) - s0);
    sink = acc; (void)sink;

    puts_("stbench: done\n");
    for (;;);
}
