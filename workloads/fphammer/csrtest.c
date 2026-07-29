// csrtest -- verify S-mode interrupt-enable CSR read/modify/write semantics.
//
// Motivation: during the ubuntu boot's S-timer storm the ILA shows mie.STIE(5) set in
// ALL 8192 samples across two timer traps. Linux's riscv_timer_interrupt begins with
// csr_clear(CSR_IE, IE_TIE), so STIE must drop on every entry. If clearing it does not
// stick, the handler can never mask the timer and livelocks before re-arming it.
//
// Runs in M-mode and drives the S-mode views (sie/sip are mie/mip through S_INT_MASK),
// then checks the same for the M-mode bits. Any FAIL line names the operation.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART      ((volatile uint8_t *)0x10000000)
#define LSR_THRE  0x20

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }

#define STIE (1ul << 5)
#define SSIE (1ul << 1)
#define SEIE (1ul << 9)
#define MTIE (1ul << 7)

static uint64_t bad;
static void chk(const char *what, uint64_t got, uint64_t want) {
    if (got != want) {
        bad++;
        puts_("FAIL "); puts_(what);
        puts_(" got="); puthex(got);
        puts_(" want="); puthex(want);
        puts_("\n");
    }
}

int main(void) {
    uint64_t v;
    puts_("\ncsrtest: sie/mie set-clear semantics\n");

    // start from a known state
    __asm__ volatile ("csrw mie, %0" :: "r"(0ul));

    // 1. set STIE via sie, read back via sie and mie
    __asm__ volatile ("csrs sie, %0" :: "r"(STIE));
    __asm__ volatile ("csrr %0, sie" : "=r"(v)); chk("sie after csrs STIE", v & STIE, STIE);
    __asm__ volatile ("csrr %0, mie" : "=r"(v)); chk("mie after csrs STIE", v & STIE, STIE);

    // 2. CLEAR STIE via sie -- the operation Linux's timer handler depends on
    __asm__ volatile ("csrc sie, %0" :: "r"(STIE));
    __asm__ volatile ("csrr %0, sie" : "=r"(v)); chk("sie after csrc STIE", v & STIE, 0ul);
    __asm__ volatile ("csrr %0, mie" : "=r"(v)); chk("mie after csrc STIE", v & STIE, 0ul);

    // 3. same through the mie view
    __asm__ volatile ("csrs mie, %0" :: "r"(STIE));
    __asm__ volatile ("csrr %0, sie" : "=r"(v)); chk("sie sees mie-set STIE", v & STIE, STIE);
    __asm__ volatile ("csrc mie, %0" :: "r"(STIE));
    __asm__ volatile ("csrr %0, sie" : "=r"(v)); chk("sie after mie-clear STIE", v & STIE, 0ul);

    // 4. clearing STIE must not disturb neighbours
    __asm__ volatile ("csrw mie, %0" :: "r"(STIE | SSIE | SEIE | MTIE));
    __asm__ volatile ("csrc sie, %0" :: "r"(STIE));
    __asm__ volatile ("csrr %0, mie" : "=r"(v));
    chk("STIE cleared",      v & STIE, 0ul);
    chk("SSIE preserved",    v & SSIE, SSIE);
    chk("SEIE preserved",    v & SEIE, SEIE);
    chk("MTIE preserved",    v & MTIE, MTIE);

    // 5. csrrc returns the OLD value and clears
    __asm__ volatile ("csrw mie, %0" :: "r"(STIE));
    __asm__ volatile ("csrrc %0, sie, %1" : "=r"(v) : "r"(STIE));
    chk("csrrc old value", v & STIE, STIE);
    __asm__ volatile ("csrr %0, sie" : "=r"(v)); chk("csrrc cleared", v & STIE, 0ul);

    // 6. write-whole-register through sie must not clobber M-only bits
    __asm__ volatile ("csrw mie, %0" :: "r"(MTIE));
    __asm__ volatile ("csrw sie, %0" :: "r"(STIE));
    __asm__ volatile ("csrr %0, mie" : "=r"(v));
    chk("sie write kept MTIE", v & MTIE, MTIE);
    chk("sie write set STIE",  v & STIE, STIE);

    puts_(bad ? "csrtest: FAILURES=" : "csrtest: ALL PASS (bad=");
    puthex(bad); puts_(")\n");
    for (;;) { }
}
