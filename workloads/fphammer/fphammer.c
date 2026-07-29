// fphammer -- self-checking FP->INT chain hammer under timer-interrupt pressure.
//
// Target: the ubuntu-boot freeze. The FPGA wedges with grep spinning in gnulib's
// next_prime on a garbage hash-table size; the size comes from the chain
//     fcvt.s.lu -> fdiv.s -> (fle.s guard) -> fcvt.lu.s
// (gnulib compute_bucket_size). This runs exactly that chain over a fixed seed
// table, twice per iteration, under machine-timer interrupts re-armed at short
// pseudo-random intervals (1..8 CLINT ticks = 133..1064 core cycles) to sweep
// interrupt/rollback phases across the in-flight FP ops.
//
// A mismatch against the IRQs-off calibration values (or between the two
// back-to-back computations of the same iteration) prints the full context to
// the UART and keeps going (up to 16 reports). Heartbeat every 2^20 iterations.
//
// Bare-metal M-mode; boots via the monitor at 0x8000_0000 (FW=fphammer.bin).

typedef unsigned char      uint8_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

#define UART      ((volatile uint8_t  *)0x10000000)
#define LSR_THRE  0x20
#define MTIMECMP  ((volatile uint64_t *)0x02004000)
#define MTIME     ((volatile uint64_t *)0x0200BFF8)

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) {
    for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]);
}

static volatile uint64_t irq_count;
static uint32_t lfsr = 0xACE1u;

// aligned(4): mtvec's low 2 bits are the MODE field -- a 2-byte-aligned handler
// silently becomes MODE=2 (reserved) with the base truncated to the previous word.
__attribute__((interrupt("machine"), aligned(4)))
static void trap_handler(void) {
    lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);      // 16-bit Galois LFSR
    *MTIMECMP = *MTIME + 1 + (lfsr & 7);                 // 1..8 ticks ahead
    irq_count++;
}

// v2: operands come FROM MEMORY like grep's real code (ld count / flw threshold from
// the tuning struct), so this covers the LSU load path + NaN-boxing, not just FP arith.
// tune[] is volatile so every call re-loads; a bad load shows up as a wrong result AND
// (separately) as a bit-pattern mismatch on the reloaded float.
static volatile float    tune_threshold = 0.8f;
static volatile uint64_t tune_count;
static uint64_t bad_load;

static inline uint64_t chain_mem(uint64_t c) {
    float f, q, d; uint64_t r, guard, dbits;
    tune_count = c;
    __asm__ volatile ("ld  %0, %1"            : "=r"(c) : "m"(tune_count));
    __asm__ volatile ("flw %0, %1"            : "=f"(d) : "m"(tune_threshold));
    __asm__ volatile ("fmv.x.w %0, %1"        : "=r"(dbits) : "f"(d));
    if ((uint32_t)dbits != 0x3f4ccccdu) bad_load++;      // 0.8f bit pattern
    __asm__ volatile ("fcvt.s.lu %0, %1"      : "=f"(f) : "r"(c));
    __asm__ volatile ("fdiv.s    %0, %1, %2"  : "=f"(q) : "f"(f), "f"(d));
    __asm__ volatile ("fle.s     %0, %1, %2"  : "=r"(guard) : "f"(f), "f"(q));
    __asm__ volatile ("fcvt.lu.s %0, %1, rtz" : "=r"(r) : "f"(q));
    return r + (guard << 63);
}

// the exact gnulib-shaped chain, pinned to real instructions
static inline uint64_t chain(uint64_t c, float divisor) {
    float f, q; uint64_t r, guard;
    __asm__ volatile ("fcvt.s.lu %0, %1"      : "=f"(f) : "r"(c));
    __asm__ volatile ("fdiv.s    %0, %1, %2"  : "=f"(q) : "f"(f), "f"(divisor));
    __asm__ volatile ("fle.s     %0, %1, %2"  : "=r"(guard) : "f"(f), "f"(q));  // exercised, result folded in
    __asm__ volatile ("fcvt.lu.s %0, %1, rtz" : "=r"(r) : "f"(q));
    return r + (guard << 63);   // guard is 1 for all seeds (q > f when divisor < 1): folded so it can't be elided
}

#define NSEED 64
static const uint64_t seeds[NSEED] = {
    1, 2, 3, 5, 7, 10, 11, 13, 17, 20, 31, 37, 40, 53, 64, 79,
    100, 127, 160, 211, 320, 401, 640, 809, 1000, 1279, 1600, 2003, 3200, 4001, 6400, 8009,
    10000, 12800, 16001, 20011, 32000, 40009, 64000, 80021, 100000, 128000, 160001, 200003,
    320000, 400009, 640000, 800011, 1000000, 1280000, 1600033, 2000003, 3200000, 4000037,
    6400000, 8000009, 10000000, 12800000, 16000057, 20000003, 32000000, 40000003, 64000000, 80000023
};
static uint64_t expect[NSEED];

int main(void) {
    puts_("\nfphammer: gnulib fcvt/fdiv chain under IRQ pressure\n");

    __asm__ volatile ("csrs mstatus, %0" :: "r"(1ul << 13));   // FS = Initial (enable FPU)

    const float divisor = 0.8f;

    // calibration, interrupts off
    for (int i = 0; i < NSEED; i++) expect[i] = chain(seeds[i], divisor);
    puts_("calibrated; arming timer\n");

    __asm__ volatile ("csrw mtvec, %0" :: "r"((uint64_t)&trap_handler));
    *MTIMECMP = *MTIME + 4;
    __asm__ volatile ("csrs mie, %0" :: "r"(1ul << 7));        // MTIE
    __asm__ volatile ("csrs mstatus, %0" :: "r"(1ul << 3));    // MIE

    uint64_t iter = 0, bad = 0;
    for (;;) {
        for (int i = 0; i < NSEED; i++) {
            uint64_t a = chain(seeds[i], divisor);
            uint64_t b = chain_mem(seeds[i]);
            if ((a != expect[i] || b != expect[i]) && bad < 16) {
                bad++;
                puts_("MISMATCH iter="); puthex(iter);
                puts_(" seed=");  puthex(seeds[i]);
                puts_(" a=");     puthex(a);
                puts_(" b=");     puthex(b);
                puts_(" exp=");   puthex(expect[i]);
                puts_(" irqs=");  puthex(irq_count);
                puts_(" badld="); puthex(bad_load);
                puts_("\n");
            }
        }
        if ((++iter & 0xFFFFF) == 0 || iter == 8) {
            puts_("H iter="); puthex(iter);
            puts_(" irqs=");  puthex(irq_count);
            puts_(" bad=");   puthex(bad);
            puts_(" badld="); puthex(bad_load);
            puts_("\n");
        }
    }
}
