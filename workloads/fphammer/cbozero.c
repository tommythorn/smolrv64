// cbozero -- a store, a cbo.zero, a store: the shape that hung build L (2026-09-04).
//
// A CBO executes from M and must follow every OLDER store; the senior store queue commits
// stores before the LSU drains them, so the CBO in M has to wait for the queue. The first
// version waited for the queue to be EMPTY -- but entries are allocated at dispatch, so the
// store after the cbo.zero is in the queue before the CBO has finished, and it cannot get
// its address until M frees: a deadlock the tiny128 cosim never lined up (its boot issues
// no cbo.zero) and the board hit at SLUB init. The right wait is "an entry WITH AN ADDRESS",
// which only an older store can have (M translates in program order). Bare-metal M-mode,
// where cbo.zero needs no envcfg bit. Prints `cbozero: ok`; a hang prints nothing.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART      ((volatile uint8_t *)0x10000000)
#define LSR_THRE  0x20
static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void putdec(uint64_t v) { char b[24]; int n = 0; do { b[n++] = '0' + v % 10; v /= 10; } while (v); while (n) putc_(b[--n]); }

static volatile uint64_t buf[3 * 8] __attribute__((aligned(64)));   // three lines

int main(void) {
    volatile uint64_t *a = buf, *z = buf + 8, *b = buf + 16;
    for (uint64_t i = 1; i <= 2000; i++) {
        a[0] = i;                                                     // older: in the queue
        z[1] = i;                                                     // older, into the line to be zeroed
        __asm__ volatile ("cbo.zero (%0)" :: "r"(z) : "memory");      // waits for those two...
        b[0] = i;                                                     // ...while this one is allocated behind it
        z[3] = i;                                                     // younger, into the zeroed line
        if (a[0] != i || b[0] != i || z[0] != 0 || z[1] != 0 || z[3] != i || z[7] != 0) {
            puts_("cbozero: FAIL at "); putdec(i); puts_("\n"); for (;;);
        }
    }
    puts_("cbozero: ok\n");
    for (;;);
}
