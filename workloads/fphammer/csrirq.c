// csrirq -- the OpenSBI MPRV accessor window WITH a timer-interrupt storm landing in it.
//
// csrreplay (device-load replay) and csrmprv (MPRV + Sv39 + PTW + page fault) both pass
// 600k windows on hardware.  The one condition neither creates is the one the real boot
// has: an interrupt arriving while the serializing CSR op is in flight, which rolls the
// pipeline back to a checkpoint.  The hardware symptom -- a csrrw that performs NEITHER
// its CSR write NOR its rd write -- is what you get if such an op is squashed after issue
// and then never re-issued (the checkpoint resumes past it).
//
// So: same window as csrmprv, plus mtimecmp re-armed every iteration at a varying offset
// so the timer fires at a sliding position, frequently landing between the csrrw and the
// csrw.  mtvec inside the window is the probe handler, so those interrupts trap there --
// the handler therefore distinguishes interrupt (mcause MSB set: disarm MTIE, resume at
// mepc) from exception (skip the faulting 4-byte load), and the main loop re-arms.
//
// A lost csrrw shows up exactly as in OpenSBI: mtvec stranded at the probe handler.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART      ((volatile uint8_t *)0x10000000)
#define LSR_THRE  0x20

#define CLINT_MTIMECMP ((volatile uint64_t *)0x02004000)
#define CLINT_MTIME    ((volatile uint64_t *)0x0200bff8)

#define MSTATUS_MIE   (1UL << 3)
#define MSTATUS_MPP_S (1UL << 11)
#define MSTATUS_MPRV  (1UL << 17)
#define MIE_MTIE      (1UL << 7)

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)) { } UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }

// One handler for everything, since mtvec holds this address both inside and outside the
// window.  Only t0 is touched, parked in mscratch -- the window's own values live in
// registers gcc allocated, and a handler that spilled to memory would itself be an MPRV
// load/store while MPRV is still set.
__attribute__((naked, aligned(4)))
static void probe_trap(void)
{
    __asm__ volatile (
        "csrw  mscratch, t0\n"
        "csrr  t0, mcause\n"
        "bltz  t0, 1f\n"            // mcause MSB set => interrupt
        "csrr  t0, mepc\n"          // exception: skip the faulting 4-byte load
        "addi  t0, t0, 4\n"
        "csrw  mepc, t0\n"
        "csrr  t0, mscratch\n"
        "mret\n"
        "1:\n"
        "li    t0, 0x80\n"          // interrupt: disarm MTIE (main loop re-arms), resume
        "csrc  mie, t0\n"
        "csrr  t0, mscratch\n"
        "mret\n");
}

static uint64_t root_pt[512] __attribute__((aligned(4096)));

#define PTE_LEAF(pa) ((((uint64_t)(pa)) >> 12) << 10 | 0xcf)   /* V R W X A D, U=0 */

static uint64_t irqs;   // interrupts actually taken, so a silent no-storm run is visible

static inline uint64_t window(uint64_t probe, uint64_t base, volatile uint8_t *addr,
                              uint64_t *inside, uint64_t *swapped)
{
    uint64_t v = probe, t, in, save;
    __asm__ volatile (
        ".option push\n .option norvc\n"
        "csrr  %3, mstatus\n"
        "sfence.vma\n"
        "csrs  mstatus, %5\n"           // MPRV on
        "csrrw %0, mtvec, %0\n"         // install probe handler, save old vector
        "lbu   %1, 0(%4)\n"             // translated load -- and the interrupt window
        "csrr  %2, mtvec\n"             // did the install take effect?
        "csrw  mtvec, %0\n"             // restore
        "csrw  mstatus, %3\n"           // MPRV off, MPP back to S
        ".option pop\n"
        : "+&r"(v), "=&r"(t), "=&r"(in), "=&r"(save)
        : "r"(addr), "r"(MSTATUS_MPRV) : "memory");
    uint64_t now;
    __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
    *inside = in; *swapped = v;
    return (in != probe ? 1 : 0) | (v != base ? 2 : 0) | (now != base ? 4 : 0);
}

// Re-arm the timer so it fires `delta` ticks from now, sliding the arrival point across
// the window from one iteration to the next.
static inline void arm(uint64_t delta)
{
    *CLINT_MTIMECMP = *CLINT_MTIME + delta;
    __asm__ volatile ("csrs mie, %0" :: "r"(MIE_MTIE));
}

static uint64_t run(const char *name, uint64_t probe, uint64_t base,
                    volatile uint8_t *addr, uint64_t iters)
{
    uint64_t bad = 0, inside, swapped;
    for (uint64_t i = 0; i < iters; i++) {
        uint64_t before;
        arm(i & 7);
        __asm__ volatile ("csrr %0, mie" : "=r"(before));
        uint64_t f = window(probe, base, addr, &inside, &swapped);
        uint64_t after;
        __asm__ volatile ("csrr %0, mie" : "=r"(after));
        if ((before & MIE_MTIE) && !(after & MIE_MTIE)) irqs++;   // handler disarmed it
        if (f) {
            bad++;
            puts_(name); puts_(" LOST fail="); puthex(f);
            puts_(" inside=");  puthex(inside);
            puts_(" swapped="); puthex(swapped);
            puts_(" iter=");    puthex(i);
            puts_("\n");
            __asm__ volatile ("csrw mtvec, %0" :: "r"(base));
            if (bad > 8) break;
        }
    }
    puts_(name); puts_(" done bad="); puthex(bad);
    puts_(" irqs="); puthex(irqs); puts_("\n");
    return bad;
}

int main(void)
{
    static volatile uint8_t mem;
    uint64_t probe = (uint64_t)&probe_trap;
    uint64_t base;

    puts_("\ncsrirq: MPRV accessor window under a timer-interrupt storm\n");

    root_pt[0] = PTE_LEAF(0x00000000UL);          // devices
    root_pt[2] = PTE_LEAF(0x80000000UL);          // RAM
    // VA 0x4000_0000 (index 1) deliberately unmapped, for the faulting variant.

    uint64_t satp = (8UL << 60) | ((uint64_t)root_pt >> 12);
    __asm__ volatile ("sfence.vma\n csrw satp, %0\n sfence.vma" :: "r"(satp) : "memory");

    __asm__ volatile ("csrw mtvec, %0" :: "r"(probe));
    __asm__ volatile ("csrr %0, mtvec" : "=r"(base));
    // MPP=S so MPRV loads translate as S-mode; MIE=1 so the timer can actually land
    // inside the window (mstatus is saved/restored per window, so this survives).
    __asm__ volatile ("csrs mstatus, %0" :: "r"(MSTATUS_MPP_S | MSTATUS_MIE));

    puts_("satp="); puthex(satp); puts_(" probe="); puthex(probe);
    puts_(" base="); puthex(base); puts_("\n");

    uint64_t bad = 0;
    bad |= run("A-ram",  probe, base, &mem,                          200000);
    bad |= run("B-dev",  probe, base, UART + 5,                      200000);
    bad |= run("C-flt",  probe, base, (volatile uint8_t *)0x40000000UL, 200000);

    puts_(bad ? "csrirq: FAILURES\n" : "csrirq: ALL PASS\n");
    for (;;) { }
}
