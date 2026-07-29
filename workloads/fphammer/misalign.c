// misalign -- self-checking misaligned load/store hammer under timer-IRQ pressure.
//
// Motivation: the FPGA ubuntu wedge is grep spinning in gnulib next_prime on a garbage
// hash size, and the FP chain that computes that size is exonerated (fphammer v1+v2 on
// hardware: 384M executions, 23M IRQs, clean). The remaining way to get a garbage
// integer is a bad LOAD -- and the wedge onset in sim is thick with cause=4 misaligned
// traps, every one of which OpenSBI emulates in M-mode by decoding the instruction and
// reassembling bytes. A wrong result there is exactly a plausible-looking garbage value.
//
// This walks every byte offset 1..7 of an 8-byte window over a known pattern buffer,
// doing misaligned ld/lw/lwu/lh/lhu/sd/sw at each offset, checking every result against
// the value computed from the byte-wise reference. Timer interrupts are re-armed at
// LFSR-jittered 1..8 tick intervals so traps land at every phase of the emulation.
//
// Bare-metal M-mode at 0x8000_0000 (FW=misalign.bin).

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
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

#define NBYTE 64
static uint8_t buf[NBYTE + 16] __attribute__((aligned(8)));
static uint8_t sbuf[NBYTE + 16] __attribute__((aligned(8)));

// byte-wise reference (little-endian assembly), never misaligned itself
static uint64_t ref_load(const uint8_t *p, int n) {
    uint64_t v = 0;
    for (int i = n - 1; i >= 0; i--) v = (v << 8) | p[i];
    return v;
}

int main(void) {
    puts_("\nmisalign: unaligned ld/st emulation hammer under IRQ pressure\n");

    for (int i = 0; i < NBYTE + 16; i++) buf[i] = (uint8_t)(i * 7 + 1);

    __asm__ volatile ("csrw mtvec, %0" :: "r"((uint64_t)&trap_handler));
    *MTIMECMP = *MTIME + 4;
    __asm__ volatile ("csrs mie, %0" :: "r"(1ul << 7));
    __asm__ volatile ("csrs mstatus, %0" :: "r"(1ul << 3));

    uint64_t iter = 0, bad = 0;
    for (;;) {
        for (int off = 1; off < 8; off++) {
            const uint8_t *p = buf + off;
            uint64_t got, want;

            __asm__ volatile ("ld %0, 0(%1)" : "=r"(got) : "r"(p) : "memory");
            want = ref_load(p, 8);
            if (got != want && bad < 12) { bad++;
                puts_("BAD ld  off="); puthex(off); puts_(" got="); puthex(got);
                puts_(" want="); puthex(want); puts_(" irqs="); puthex(irq_count); puts_("\n"); }

            __asm__ volatile ("lwu %0, 0(%1)" : "=r"(got) : "r"(p) : "memory");
            want = ref_load(p, 4);
            if (got != want && bad < 12) { bad++;
                puts_("BAD lwu off="); puthex(off); puts_(" got="); puthex(got);
                puts_(" want="); puthex(want); puts_(" irqs="); puthex(irq_count); puts_("\n"); }

            __asm__ volatile ("lhu %0, 0(%1)" : "=r"(got) : "r"(p) : "memory");
            want = ref_load(p, 2);
            if (got != want && bad < 12) { bad++;
                puts_("BAD lhu off="); puthex(off); puts_(" got="); puthex(got);
                puts_(" want="); puthex(want); puts_(" irqs="); puthex(irq_count); puts_("\n"); }

            // misaligned store, then verify byte-wise
            uint64_t pat = 0x0123456789abcdefull ^ (off * 0x1111111111111111ull) ^ iter;
            uint8_t *q = sbuf + off;
            __asm__ volatile ("sd %0, 0(%1)" :: "r"(pat), "r"(q) : "memory");
            got = ref_load(q, 8);
            if (got != pat && bad < 12) { bad++;
                puts_("BAD sd  off="); puthex(off); puts_(" got="); puthex(got);
                puts_(" want="); puthex(pat); puts_(" irqs="); puthex(irq_count); puts_("\n"); }
        }
        if ((++iter & 0xFFFFF) == 0 || iter == 8) {
            puts_("H iter="); puthex(iter); puts_(" irqs="); puthex(irq_count);
            puts_(" bad="); puthex(bad); puts_("\n");
        }
    }
}
