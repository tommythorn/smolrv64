// csrmprv -- the OpenSBI MPRV accessor window, reproduced exactly.
//
// csrreplay showed a CSR swap next to a *device* load never loses the swap.  The
// difference from the real failure is that OpenSBI's load runs with mstatus.MPRV
// set and MPP=S, so it is translated by satp: it goes through the dMMU and, on a
// TLB miss, a page-table walk -- a different replay path than a device load.
//
// This builds the identical window bare-metal:
//     csrs  mstatus, MPRV        (MPP already = S)
//     csrrw v, mtvec, v          install probe handler, save old vector in v
//     lbu   t, 0(addr)           S-mode-translated load  <-- MMU / PTW / fault
//     csrw  mtvec, v             restore
//     csrc  mstatus, MPRV
//
// Three variants, all with an sfence.vma per iteration so the load always misses
// the TLB and forces a walk:
//   A  mapped RAM address        (PTW in the window)
//   B  mapped device address     (PTW + device-load replay in the window)
//   C  UNMAPPED address          (PTW -> page fault -> traps to the just-installed
//                                 probe handler, mepc+=4, mret) -- this is the case
//                                 the probe handler exists for in the first place.
//
// Checked per iteration: (1) mtvec inside the window really is the probe handler,
// i.e. the swap took effect, (2) the value swapped out is the old vector, (3) mtvec
// after the restore is back to base.  A lost csrrw fails (1) and (2) and strands
// mtvec, which is precisely the observed hardware symptom.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART      ((volatile uint8_t *)0x10000000)
#define LSR_THRE  0x20

#define MSTATUS_MPRV (1UL << 17)
#define MSTATUS_MPP_S (1UL << 11)

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }

// M-mode probe handler: swallow the fault, skip the faulting 4-byte load, return.
// Must be 4-byte aligned -- mtvec's low two bits are the mode field.
__attribute__((naked, aligned(4)))
static void probe_trap(void) { __asm__ volatile ("csrr t0, mepc\n addi t0,t0,4\n csrw mepc,t0\n mret\n"); }

// Sv39 root: two 1 GiB identity leaves -- VA 0 (devices) and VA 0x8000_0000 (RAM).
static uint64_t root_pt[512] __attribute__((aligned(4096)));

#define PTE_LEAF(pa) ((((uint64_t)(pa)) >> 12) << 10 | 0xcf)   /* V R W X A D, U=0 */

// One window.  Returns 0 on success, else a bit per broken check.
// Everything the window touches is in registers: with MPRV set, any compiler-
// generated spill would itself be translated, which is not what we are testing.
static inline uint64_t window(uint64_t probe, uint64_t base, volatile uint8_t *addr,
                              uint64_t *inside, uint64_t *swapped)
{
    uint64_t v = probe, t, in, save;
    // mstatus is saved and restored wholesale, exactly as OpenSBI does: a fault in
    // the window traps to M-mode (MPP:=M) and the probe handler's mret then leaves
    // MPP=U, which would translate every later iteration as U-mode and fault forever.
    // .option norvc keeps every instruction in the window 4 bytes, so the handler's
    // mepc+=4 lands on the instruction after the load.
    __asm__ volatile (
        ".option push\n .option norvc\n"
        "csrr  %3, mstatus\n"
        "sfence.vma\n"                  // force a walk for the load below
        "csrs  mstatus, %5\n"           // MPRV on: the load becomes S-mode translated
        "csrrw %0, mtvec, %0\n"         // install probe handler, save old vector
        "lbu   %1, 0(%4)\n"             // the translated load (may fault -> probe handler)
        "csrr  %2, mtvec\n"             // did the install actually take effect?
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

static uint64_t run(const char *name, uint64_t probe, uint64_t base,
                    volatile uint8_t *addr, uint64_t iters)
{
    uint64_t bad = 0, inside, swapped;
    for (uint64_t i = 0; i < iters; i++) {
        uint64_t f = window(probe, base, addr, &inside, &swapped);
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
    puts_(name); puts_(" done bad="); puthex(bad); puts_("\n");
    return bad;
}

int main(void)
{
    static volatile uint8_t mem;
    uint64_t probe = (uint64_t)&probe_trap;
    uint64_t base;

    puts_("\ncsrmprv: OpenSBI MPRV accessor window under Sv39\n");

    root_pt[0] = PTE_LEAF(0x00000000UL);          // devices
    root_pt[2] = PTE_LEAF(0x80000000UL);          // RAM
    // VA 0x4000_0000 (index 1) deliberately left unmapped, for variant C.

    uint64_t satp = (8UL << 60) | ((uint64_t)root_pt >> 12);
    __asm__ volatile ("sfence.vma\n csrw satp, %0\n sfence.vma" :: "r"(satp) : "memory");

    // MPP = S, so MPRV loads translate as S-mode.  MPRV itself is set only inside
    // the window; leaving it on here would translate our own console writes too.
    __asm__ volatile ("csrs mstatus, %0" :: "r"(MSTATUS_MPP_S));

    // Park a known vector so a lost swap is visible as mtvec stuck at the probe.
    __asm__ volatile ("csrw mtvec, %0" :: "r"(probe));
    __asm__ volatile ("csrr %0, mtvec" : "=r"(base));
    puts_("satp="); puthex(satp); puts_(" probe="); puthex(probe);
    puts_(" base="); puthex(base); puts_("\n");

    uint64_t bad = 0;
    bad |= run("A-ram",  probe, base, &mem,                          200000);
    bad |= run("B-dev",  probe, base, UART + 5,                      200000);
    bad |= run("C-flt",  probe, base, (volatile uint8_t *)0x40000000UL, 200000);

    puts_(bad ? "csrmprv: FAILURES\n" : "csrmprv: ALL PASS\n");
    for (;;) { }
}
