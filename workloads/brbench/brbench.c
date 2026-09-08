// brbench -- what a mispredicted branch COSTS, L1-resident, bare-metal M-mode.
//
// Four loops, each with one conditional branch per iteration and nothing else in flight:
//   pred      the branch is never taken (the predictor learns it): the loop's own cost
//   near      the branch takes a random direction (xorshift64, so no 12-bit history predicts
//             it); the taken target is a few bytes away, inside the fetch buffer's chunks
//   far       the same, with the taken target 1 KiB away: the buffer misses, the I$ hits
//   drain     the same near branch, but an OLDER load that misses the D$ is still in flight,
//             so the resolved mispredict must wait for the ROB head (RD_WAIT, plan item 5)
// The penalty per mispredict is the extra cycles per iteration over `pred`, divided by the
// redirects the counter saw. Run: make && FW=$(pwd)/brbench.bin CYC=40000000 ../../ooo2/run-ooo2-linux.sh

typedef unsigned char uint8_t; typedef unsigned long uint64_t;
#define UART ((volatile uint8_t *)0x10000000)
static void putc_(uint8_t c) { while (!(UART[5] & 0x20)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void putdec(uint64_t v) { char b[24]; int n = 0; do { b[n++] = '0' + v % 10; v /= 10; } while (v); while (n) putc_(b[--n]); }
static void fixed2(uint64_t v, uint64_t d) { uint64_t q = v / d, r = ((v % d) * 100 + d / 2) / d; if (r >= 100) { q++; r -= 100; } putdec(q); putc_('.'); putc_('0' + r / 10); putc_('0' + r % 10); }
static uint64_t rdcycle(void) { uint64_t v; __asm__ volatile("rdcycle %0" : "=r"(v)); return v; }
static uint64_t rdinstr(void) { uint64_t v; __asm__ volatile("rdinstret %0" : "=r"(v)); return v; }
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })
#define EV_REDIR 0x0005
#define EV_RDWAIT 0x0317
#define EV_FEBUB 0x0310
#define EV_REDBR 0x0006
#ifndef NITER
#define NITER 100000
#endif

static uint64_t seed = 0x9E3779B97F4A7C15ul;
static uint64_t big[1 << 16];               // 512 KiB: every stride-64 load misses the 64 KiB D$

static void report(const char *name, uint64_t c0, uint64_t c, uint64_t i, uint64_t r, uint64_t w, uint64_t f, uint64_t rb) {
    puts_("brbench: "); puts_(name); puts_(" start="); putdec(c0);
    puts_(" cyc/iter="); fixed2(c, NITER); puts_(" insn/iter="); fixed2(i, NITER);
    puts_(" redir/iter="); fixed2(r, NITER); puts_(" RD_WAIT/iter="); fixed2(w, NITER);
    puts_(" FE_BUB/iter="); fixed2(f, NITER); puts_(" RED_BR/iter="); fixed2(rb, NITER); puts_("\n");
}

#define MEASURE(name, body) do { \
    uint64_t x = seed, acc = 0, n = NITER; \
    uint64_t c0 = rdcycle(), i0 = rdinstr(), r0 = RDCNT(3), w0 = RDCNT(4), f0 = RDCNT(5), b0 = RDCNT(6); \
    __asm__ volatile(body : "+r"(x), "+r"(acc), "+r"(n) : "r"(big) : "t0", "t1", "t2", "memory"); \
    report(name, c0, rdcycle() - c0, rdinstr() - i0, RDCNT(3) - r0, RDCNT(4) - w0, RDCNT(5) - f0, RDCNT(6) - b0); \
    seed = x + acc; \
} while (0)

// xorshift64 in three shifts and xors; bit 0 of x decides the branch.
#define XS "slli t0, %0, 13\n xor %0, %0, t0\n srli t0, %0, 7\n xor %0, %0, t0\n slli t0, %0, 17\n xor %0, %0, t0\n"

int main(void) {
    SETEV(3, EV_REDIR); SETEV(4, EV_RDWAIT); SETEV(5, EV_FEBUB); SETEV(6, EV_REDBR);
    for (int k = 0; k < (1 << 16); k++) big[k] = k;

    MEASURE("pred ", "1:\n" XS "andi t0, %0, 0\n bnez t0, 2f\n addi %1, %1, 1\n 2: addi %2, %2, -1\n bnez %2, 1b\n");
    MEASURE("near ", "1:\n" XS "andi t0, %0, 1\n bnez t0, 2f\n addi %1, %1, 1\n j 3f\n 2: xori %1, %1, 5\n 3: addi %2, %2, -1\n bnez %2, 1b\n");
    MEASURE("far  ", "1:\n" XS "andi t0, %0, 1\n bnez t0, 4f\n addi %1, %1, 1\n 3: addi %2, %2, -1\n bnez %2, 1b\n j 5f\n"
                     ".skip 1024\n 4: xori %1, %1, 5\n j 3b\n 5:\n");
    // drain: a load of big[(x >> 8) & 0xffff] (a fresh line nearly every time) is OLDER than
    // the branch and independent of it; the branch resolves while the miss is in flight.
    MEASURE("drain", "1:\n" XS "srli t1, %0, 8\n slli t1, t1, 48\n srli t1, t1, 45\n add t1, t1, %3\n ld t2, 0(t1)\n"
                     "andi t0, %0, 1\n bnez t0, 2f\n addi %1, %1, 1\n j 3f\n 2: xori %1, %1, 5\n 3: add %1, %1, t2\n addi %2, %2, -1\n bnez %2, 1b\n");
    puts_("brbench: done\n");
    for (;;) ;
}
