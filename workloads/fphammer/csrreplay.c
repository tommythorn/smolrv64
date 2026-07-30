// csrreplay -- does a replay-inducing load LOSE a neighbouring serializing CSR op?
//
// FPGA evidence (ILA, mtvec + write-strobe probes): in OpenSBI's MPRV accessor
//     e828  csrrw x12, mtvec, x12   install probe handler, save old vector in x12
//     e830  lbu   x17, 0(x10)       read S-mode memory via MPRV
//     e838  csrw  mtvec, x12        restore
// the csrrw produced NO mtvec write and never swapped x12, i.e. the whole serializing
// op vanished. x12 therefore still held the probe-handler address and the "restore"
// wrote it back, stranding mtvec at __sbi_expected_trap -- after which every S-mode
// ecall is skipped, sbi_set_timer never programs stimecmp, and the kernel storms.
//
// The load in that window goes through the MMU and can force a replay (device-load
// ordering / fault replay), which rolls back and refetches. This drives the same shape
// deliberately: a CSR swap immediately followed by a load that provokes a replay --
// here a DEVICE load (UART), which takes the devld_replay path by construction.
//
// Any iteration where mtvec does not come back is the bug. Bare metal M-mode.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART      ((volatile uint8_t *)0x10000000)
#define LSR_THRE  0x20

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }

__attribute__((naked, aligned(4)))
static void dummy_trap(void) { __asm__ volatile ("csrr t0, mepc\n addi t0,t0,4\n csrw mepc,t0\n mret\n"); }

int main(void) {
    uint64_t base, now, bad = 0, i;
    uint64_t probe = (uint64_t)&dummy_trap;
    volatile uint8_t *dev = UART + 5;      // LSR: a device (MMIO) address
    static volatile uint8_t mem;           // ordinary memory, for the control case

    puts_("\ncsrreplay: CSR swap next to a replay-inducing load\n");
    __asm__ volatile ("csrw mtvec, %0" :: "r"(probe));
    __asm__ volatile ("csrr %0, mtvec" : "=r"(base));
    // put a DIFFERENT known value in mtvec so a lost swap is visible
    base = probe;

    // A: device load between the swap and the restore (devld_replay path)
    for (i = 0; i < 200000; i++) {
        uint64_t v = probe, t;
        __asm__ volatile (
            "csrrw %0, mtvec, %0\n"     // install: mtvec<=v, v<=old mtvec
            "lbu   %1, 0(%2)\n"         // DEVICE load -> replay
            "csrw  mtvec, %0\n"         // restore from the swapped-out value
            : "+&r"(v), "=&r"(t) : "r"(dev) : "memory");
        __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
        if (now != base) {
            bad++;
            puts_("A-LOST mtvec="); puthex(now);
            puts_(" want=");        puthex(base);
            puts_(" iter=");        puthex(i);
            puts_("\n");
            __asm__ volatile ("csrw mtvec, %0" :: "r"(base));
            if (bad > 8) break;
        }
    }
    puts_("A done bad="); puthex(bad); puts_("\n");

    // B: control -- ordinary memory load in the same position
    uint64_t badB = 0;
    for (i = 0; i < 200000; i++) {
        uint64_t v = probe, t;
        __asm__ volatile (
            "csrrw %0, mtvec, %0\n"
            "lbu   %1, 0(%2)\n"
            "csrw  mtvec, %0\n"
            : "+&r"(v), "=&r"(t) : "r"(&mem) : "memory");
        __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
        if (now != base) {
            badB++;
            puts_("B-LOST mtvec="); puthex(now); puts_(" iter="); puthex(i); puts_("\n");
            __asm__ volatile ("csrw mtvec, %0" :: "r"(base));
            if (badB > 8) break;
        }
    }
    puts_("B done bad="); puthex(badB); puts_("\n");

    puts_((bad | badB) ? "csrreplay: FAILURES\n" : "csrreplay: ALL PASS\n");
    for (;;) { }
}
