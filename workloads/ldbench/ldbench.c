// ldbench -- load LATENCY vs load THROUGHPUT, both L1-resident.
//
// The FP equivalent of this (workloads/fpbench) found that fpnew was pipelined all along
// and its WRAPPER held one op; the fix was a tag, not a rewrite. The LSU is suspected of
// the same shape, so measure the same way before touching rv_cache.v:
//
//   lat  = cycles per load in a POINTER CHASE           -> load-to-use latency
//   thru = cycles per load over INDEPENDENT addresses   -> issue/cache throughput
//
// If thru ~= lat, nothing overlaps and one load is in flight at a time -- the wrapper is
// the limit. If thru << lat, the cache is already pipelining and the win is elsewhere.
//
// Everything fits the 64 KiB D$ on purpose: this measures the HIT path, which is where
// AES-XTS lives (0.28% miss rate) and where 54.7% of full-suite ST_MEM comes from at only
// 2.04% miss. A miss-bound benchmark would measure DRAM, not the machine.
//
// Bare-metal M-mode. Measure at VDEFS="-DOOO2_HW=4".

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
static void fixed2(uint64_t num, uint64_t den) {
    uint64_t w = den ? num / den : 0, f = den ? (num * 100 / den) % 100 : 0;
    putdec(w); putc_('.'); if (f < 10) putc_('0'); putdec(f);
}
static uint64_t rdcycle(void) { uint64_t v; __asm__ volatile("rdcycle %0":"=r"(v)); return v; }
static uint64_t rdinstr(void) { uint64_t v; __asm__ volatile("rdinstret %0":"=r"(v)); return v; }

#define EV_ST_MEM 0x0300
#define EV_DCACC  0x0100
#define EV_DCMISS 0x0102
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })

#define NSLOT  4096                    /* 32 KiB of pointers: fits the 64 KiB D$ */
#define NITER  200000
#define NWAY   8                       /* independent streams in the throughput loop */
static uint64_t buf[NSLOT];

int main(void) {
    SETEV(3, EV_ST_MEM); SETEV(4, EV_DCACC); SETEV(5, EV_DCMISS);

    // Pointer chase with a large stride so consecutive loads are on different lines and
    // different sets -- 512 sets, 64 B lines, so stride by a prime number of lines.
    for (uint64_t i = 0; i < NSLOT; i++)
        buf[i] = (uint64_t)&buf[(i + 613) & (NSLOT - 1)];
    for (int i = 0; i < NSLOT; i++) (void)*(volatile uint64_t *)&buf[i];   // warm

    // ---- LATENCY: each load's ADDRESS is the previous load's result. ----
    uint64_t c0 = rdcycle(), i0 = rdinstr(), m0 = RDCNT(3);
    uint64_t *p = &buf[0];
    for (int i = 0; i < NITER; i++) p = (uint64_t *)*p;
    uint64_t lat_c = rdcycle() - c0, lat_i = rdinstr() - i0, lat_m = RDCNT(3) - m0;
    volatile uint64_t sink0 = (uint64_t)p;

    // ---- THROUGHPUT: NWAY independent chases, no dependence between them. ----
    c0 = rdcycle(); i0 = rdinstr(); m0 = RDCNT(3);
    uint64_t a0 = RDCNT(4), s0 = RDCNT(5);
    uint64_t *q0=&buf[0],   *q1=&buf[311], *q2=&buf[631],  *q3=&buf[953];
    uint64_t *q4=&buf[1277],*q5=&buf[1601],*q6=&buf[1931], *q7=&buf[2267];
    for (int i = 0; i < NITER; i++) {
        q0=(uint64_t*)*q0; q1=(uint64_t*)*q1; q2=(uint64_t*)*q2; q3=(uint64_t*)*q3;
        q4=(uint64_t*)*q4; q5=(uint64_t*)*q5; q6=(uint64_t*)*q6; q7=(uint64_t*)*q7;
    }
    uint64_t thr_c = rdcycle() - c0, thr_i = rdinstr() - i0, thr_m = RDCNT(3) - m0;
    uint64_t thr_a = RDCNT(4) - a0, thr_s = RDCNT(5) - s0;
    // EVERY chain must be consumed. Sinking only q0 and q7 let gcc delete the other six
    // as dead, and the benchmark reported 0.50 instructions per load -- impossible, since a
    // load is at least one instruction. An unread result is a deleted chain.
    volatile uint64_t sink1 = (uint64_t)q0 ^ (uint64_t)q1 ^ (uint64_t)q2 ^ (uint64_t)q3
                            ^ (uint64_t)q4 ^ (uint64_t)q5 ^ (uint64_t)q6 ^ (uint64_t)q7;
    (void)sink0; (void)sink1;

    const uint64_t lat_n = NITER, thr_n = (uint64_t)NITER * NWAY;

    puts_("ldbench: LATENCY  loads="); putdec(lat_n);
    puts_(" cycles=");                 putdec(lat_c);
    puts_("\nldbench:   cyc/load=");   fixed2(lat_c, lat_n);
    puts_("  insn/load=");             fixed2(lat_i, lat_n);
    puts_("  ST_MEM/load=");           fixed2(lat_m, lat_n);

    puts_("\nldbench: THRUPUT loads="); putdec(thr_n);
    puts_(" cycles=");                  putdec(thr_c);
    puts_("\nldbench:   cyc/load=");    fixed2(thr_c, thr_n);
    puts_("  insn/load=");              fixed2(thr_i, thr_n);
    puts_("  ST_MEM/load=");            fixed2(thr_m, thr_n);
    puts_("\nldbench:   D$acc=");        putdec(thr_a);
    puts_(" D$miss=");                   putdec(thr_s);
    puts_("  ST_MEM=");                  putdec(thr_m * 100 / thr_c);
    puts_("%\nldbench:   overlap = lat/thru = ");
    fixed2(lat_c * 100 / lat_n * thr_n, thr_c * 100);
    puts_("x  (1.00 = nothing overlaps, one load in flight)\n");
    puts_("ldbench: done\n");
    for (;;);
}
