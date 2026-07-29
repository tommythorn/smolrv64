// fpseq -- replay grep's compute_bucket_size FP sequence EXACTLY, bare metal.
//
// The FPGA hang is deterministic: fcvt.lu.s of +0.0 writes {ffffffff,0} into an
// integer register instead of 0 (gdb on four hung greps, two boots, byte-identical).
// Every input is verified correct. fphammer (v1/v2) ran the same OPCODES 256M times
// cleanly -- but with different registers and a non-destructive divide. grep's actual
// sequence (file offset 0x179de..0x179fc) is distinctive:
//
//   fcvt.s.lu fa4, s1        (float)candidate
//   flw       fa5, thr       growth_threshold
//   flw       fa3, big       SIZE_MAX as float
//   fdiv.s    fa5, fa4, fa5  DESTRUCTIVE: dest == src2
//   fle.s     a5,  fa3, fa5  guard
//   fcvt.lu.s s1,  fa5, rtz  F2I into an INTEGER register
//
// so this pins the same registers, the same destructive divide, and the same F2I dest.
// Reported failure = the F2I result having any high bits set (a NaN-box on an integer).

typedef unsigned char      uint8_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

#define UART      ((volatile uint8_t  *)0x10000000)
#define LSR_THRE  0x20
#define MTIMECMP  ((volatile uint64_t *)0x02004000)
#define MTIME     ((volatile uint64_t *)0x0200BFF8)

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }

static volatile uint64_t irq_count;
static uint32_t lfsr = 0xACE1u;

__attribute__((interrupt("machine"), aligned(4)))
static void trap_handler(void) {
    lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
    *MTIMECMP = *MTIME + 1 + (lfsr & 7);
    irq_count++;
}

static volatile float thr = 0.8f;          // growth_threshold, loaded by flw
static volatile float big = 1.8446744e19f; // SIZE_MAX as float, loaded by flw

// grep's sequence, register-for-register. Returns the F2I result.
static inline uint64_t seq(uint64_t cand, uint64_t *guard_out) {
    uint64_t res, guard;
    __asm__ volatile (
        "fcvt.s.lu fa4, %2      \n"
        "flw       fa5, %3      \n"
        "flw       fa3, %4      \n"
        "fdiv.s    fa5, fa4, fa5\n"   // destructive: dest == src2
        "fle.s     %1,  fa3, fa5\n"
        "fcvt.lu.s %0,  fa5, rtz\n"
        : "=r"(res), "=r"(guard)
        : "r"(cand), "m"(thr), "m"(big)
        : "fa3", "fa4", "fa5");
    *guard_out = guard;
    return res;
}

int main(void) {
    puts_("\nfpseq: grep compute_bucket_size sequence, register-exact\n");

    // candidate 0 is what the hung greps actually had (fa4 = +0.0)
    static const uint64_t cands[8] = {0, 1, 2, 10, 53, 97, 1024, 65536};
    uint64_t iter = 0, bad = 0;

    __asm__ volatile ("csrw mtvec, %0" :: "r"((uint64_t)&trap_handler));
    *MTIMECMP = *MTIME + 4;
    __asm__ volatile ("csrs mie, %0" :: "r"(1ul << 7));
    __asm__ volatile ("csrs mstatus, %0" :: "r"(1ul << 3));

    for (;;) {
        for (int i = 0; i < 8; i++) {
            uint64_t g, r = seq(cands[i], &g);
            // expected: floor(cand / 0.8) in the low word, nothing in the high word
            if ((r >> 32) != 0 && bad < 12) {
                bad++;
                puts_("BOXED cand="); puthex(cands[i]);
                puts_(" r=");        puthex(r);
                puts_(" guard=");    puthex(g);
                puts_(" iter=");     puthex(iter);
                puts_(" irqs=");     puthex(irq_count);
                puts_("\n");
            }
        }
        if ((++iter & 0xFFFFF) == 0 || iter == 8) {
            puts_("H iter="); puthex(iter);
            puts_(" irqs=");  puthex(irq_count);
            puts_(" bad=");   puthex(bad);
            puts_("\n");
        }
    }
}
