// membench -- the MISS-bound twins of ldbench/stbench: 1 MiB streams through a 64 KiB D$.
//
//   copy   cycles per byte, 8-byte loads and stores, src -> dst    (page copy)
//   fill   cycles per byte, 8-byte stores                          (memset, clear_page's shape)
//   sum    cycles per byte, 8-byte loads                           (checksum, scan)
//
// Every line misses, so the number is the D$'s fill path against the DDR model measured at
// 166.67 MHz: a line per ~35 cycles with no prefetch; the next-line stream buffer (plan item
// 6, 2026-09-05) overlaps the next fill with the current line's use. Bare-metal M-mode.
//   make && FW=$(pwd)/membench.bin CYC=40000000 ../../ooo2/run-ooo2-linux.sh

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART     ((volatile uint8_t *)0x10000000)
#define LSR_THRE 0x20
static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void putdec(uint64_t v) { char b[24]; int n = 0; do { b[n++] = '0' + v % 10; v /= 10; } while (v); while (n) putc_(b[--n]); }
static void fixed2(uint64_t v, uint64_t d) {
    uint64_t q = v / d, r = ((v % d) * 100 + d / 2) / d;
    if (r >= 100) { q++; r -= 100; }
    putdec(q); putc_('.'); putc_('0' + r / 10); putc_('0' + r % 10);
}
static uint64_t rdcycle(void) { uint64_t v; __asm__ volatile("rdcycle %0":"=r"(v)); return v; }
#define EV_DCACC  0x0100
#define EV_DCMISS 0x0102
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })

#define NW (1u << 17)                                   /* 128 Ki words = 1 MiB per buffer */
static uint64_t src[NW] __attribute__((aligned(64)));
static uint64_t dst[NW] __attribute__((aligned(64)));

static void report(const char *name, uint64_t bytes, uint64_t c, uint64_t a, uint64_t m) {
    puts_("membench: "); puts_(name); puts_(" bytes="); putdec(bytes); puts_(" cycles="); putdec(c);
    puts_("\nmembench:   cyc/byte="); fixed2(c, bytes); puts_("  cyc/line="); fixed2(c, bytes / 64);
    puts_("  D$acc="); putdec(a); puts_(" D$miss="); putdec(m); puts_("\n");
}

int main(void) {
    SETEV(4, EV_DCACC); SETEV(5, EV_DCMISS);
    volatile uint64_t *s = src, *d = dst;
    uint64_t c0, a0, m0, acc = 0;
    for (unsigned i = 0; i < NW; i++) s[i] = i;                          // src in DDR (dirty lines, later evicted)

    c0 = rdcycle(); a0 = RDCNT(4); m0 = RDCNT(5);
    for (unsigned i = 0; i < NW; i++) d[i] = s[i];
    report("copy (ld+st)", NW * 8, rdcycle() - c0, RDCNT(4) - a0, RDCNT(5) - m0);

    c0 = rdcycle(); a0 = RDCNT(4); m0 = RDCNT(5);
    for (unsigned i = 0; i < NW; i++) d[i] = i;
    report("fill (st)", NW * 8, rdcycle() - c0, RDCNT(4) - a0, RDCNT(5) - m0);

    c0 = rdcycle(); a0 = RDCNT(4); m0 = RDCNT(5);
    for (unsigned i = 0; i < NW; i++) acc += s[i];
    report("sum (ld)", NW * 8, rdcycle() - c0, RDCNT(4) - a0, RDCNT(5) - m0);
    volatile uint64_t sink = acc; (void)sink;

    puts_("membench: done\n");
    for (;;);
}
