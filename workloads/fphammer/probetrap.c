// probetrap -- replicate OpenSBI's csr_read_allowed() probe sequence.
//
// On the FPGA, mtvec is stuck at OpenSBI's __sbi_expected_trap (ILA probe2 reads
// 0x80000738 in 8192/8192 samples during the storm), so every kernel ecall is skipped
// instead of serviced and the timer is never programmed. OpenSBI only installs that
// handler inside this macro:
//
//     csrrw mtvec, <probe_handler>   ; install, save old
//     csrr  <ret>,  <csr>            ; probed access; may trap -> handler does mepc+=4, mret
//     csrw  mtvec, <saved>           ; RESTORE
//
// If the restore is skipped (bad mepc on the trap -> handler returns PAST it) or the
// CSR write is lost, mtvec stays at the probe handler forever. This exercises the
// sequence against both a TRAPPING csr (mtopi 0xFB0, unimplemented here) and a
// non-trapping one, and verifies mtvec afterwards. Bare metal M-mode.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART      ((volatile uint8_t *)0x10000000)
#define LSR_THRE  0x20

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }

uint64_t trap_count, last_cause, last_epc;   /* global: the naked handler references them by name */

// mirrors __sbi_expected_trap: record, skip the faulting instruction, return
__attribute__((naked, aligned(4)))
static void probe_trap(void) {
    __asm__ volatile (
        "csrr t0, mcause\n"
        "la   t1, trap_count\n"
        "ld   t2, 0(t1)\n"
        "addi t2, t2, 1\n"
        "sd   t2, 0(t1)\n"
        "la   t1, last_cause\n"
        "sd   t0, 0(t1)\n"
        "csrr t0, mepc\n"
        "la   t1, last_epc\n"
        "sd   t0, 0(t1)\n"
        "addi t0, t0, 4\n"
        "csrw mepc, t0\n"
        "mret\n");
}

static uint64_t bad;
static void chk(const char *what, uint64_t got, uint64_t want) {
    if (got != want) { bad++;
        puts_("FAIL "); puts_(what); puts_(" got="); puthex(got);
        puts_(" want="); puthex(want); puts_("\n"); }
}

// the exact OpenSBI shape: install probe handler, read csr, restore mtvec
#define PROBE(csrname)                                              \
    ({ uint64_t v = (uint64_t)&probe_trap, r = 0;                   \
       __asm__ volatile ("csrrw %0, mtvec, %0\n"                    \
                         "csrr  %1, " csrname "\n"                  \
                         "csrw  mtvec, %0\n"                        \
                         : "+&r"(v), "=&r"(r) :: "memory");         \
       r; })

int main(void) {
    uint64_t orig, now, r;
    puts_("\nprobetrap: OpenSBI csr_read_allowed sequence\n");

    __asm__ volatile ("csrw mtvec, %0" :: "r"((uint64_t)&probe_trap));
    __asm__ volatile ("csrr %0, mtvec" : "=r"(orig));
    __asm__ volatile ("csrw mtvec, %0" :: "r"(orig));   // settle to a known vector

    // 1. probe a NON-trapping CSR (mscratch): no trap, restore must hold
    uint64_t base = orig;
    r = PROBE("0x340");
    __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
    chk("mtvec restored after non-trapping probe", now, base);
    chk("no trap taken", trap_count, 0ul);

    // 2. probe a TRAPPING CSR (mtopi 0xFB0, unimplemented -> illegal instruction).
    //    The handler must resume at the instruction AFTER the csrr, i.e. the restore.
    r = PROBE("0xFB0");
    __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
    puts_("after trapping probe: mtvec="); puthex(now);
    puts_(" traps="); puthex(trap_count);
    puts_(" cause="); puthex(last_cause);
    puts_(" epc=");   puthex(last_epc);
    puts_("\n");
    chk("trap taken on unimplemented csr", trap_count, 1ul);
    chk("mtvec RESTORED after trapping probe", now, base);

    // 3. repeat a few times: a lost restore may be intermittent
    for (int i = 0; i < 64; i++) {
        r = PROBE("0xFB0");
        __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
        if (now != base) { chk("mtvec restored in loop", now, base); break; }
    }
    (void)r;

    // 4. THE REAL QUESTION: run the probe window with an M-timer interrupt armed to
    //    fire inside it. OpenSBI runs these sequences with mstatus.MIE=0, so no
    //    interrupt may be delivered here. If one IS delivered, it lands on the probe
    //    handler, which does mepc+=4 and mret WITHOUT restoring mtvec -- and mtvec is
    //    then stuck at the probe handler forever, exactly as the FPGA shows.
    puts_("phase4: probe window with timer armed (MIE stays 0)\n");
    volatile uint64_t *mtimecmp = (volatile uint64_t *)0x02004000;
    volatile uint64_t *mtime    = (volatile uint64_t *)0x0200BFF8;
    __asm__ volatile ("csrs mie, %0" :: "r"(1ul << 7));       // MTIE enabled
    __asm__ volatile ("csrc mstatus, %0" :: "r"(1ul << 3));   // but MIE = 0
    uint64_t t0 = trap_count;
    for (int i = 0; i < 20000; i++) {
        *mtimecmp = *mtime + (i & 3);       // deadline lands inside/near the window
        r = PROBE("0xFB0");
        __asm__ volatile ("csrr %0, mtvec" : "=r"(now));
        if (now != base) {
            bad++;
            puts_("STUCK mtvec="); puthex(now);
            puts_(" iter=");       puthex((uint64_t)i);
            puts_(" traps=");      puthex(trap_count - t0);
            puts_(" cause=");      puthex(last_cause);
            puts_("\n");
            break;
        }
    }
    *mtimecmp = ~0ul;
    puts_("phase4 done, extra traps="); puthex(trap_count - t0); puts_("\n");

    puts_(bad ? "probetrap: FAILURES=" : "probetrap: ALL PASS (bad=");
    puthex(bad); puts_(")\n");
    for (;;) { }
}
