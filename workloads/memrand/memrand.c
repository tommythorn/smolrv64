// memrand (B4, 2026-09-17): the random memory-ordering workload, run under the lockstep cosim.
// M-mode sets up Sv39 with three aliases of one 128 KiB data region (the identity gigapage, W0,
// W1) and an NC window W2 over the DMA region, delegates, enables PLIC source 11 for S-mode,
// and drops to S-mode main_s, which seeds the pointer page and runs the generated op stream
// (ops.S). The testbench's DMA agent (+dma_rand) writes bursts into the DMA region and raises
// source 11; the S-mode handler sums the window through W2, acks, completes. Every load's value
// and every store's bytes are judged by the lockstep; MEMRAND-DONE on the console ends the run.
#include <stdint.h>
#define UART  ((volatile uint8_t *)0x10000000)
#define PLIC  0x0C000000UL
#define W0    0x1000000000UL        /* VPN2 = 64:  alias 0 of the data region */
#define W1    0x2000000000UL        /* VPN2 = 128: alias 1 (remappable by the op stream) */
#define W2    0x3000000000UL        /* VPN2 = 192: NC window over the DMA region */
#define DATA_PA 0x81000000UL        /* 128 KiB region; the first 4 KiB is the pointer page */
#define DMA_PA  0x81200000UL        /* 4 KiB DMA window; +0xFF8 = the ack word */
#define PTE_V 1UL
#define PTE_R 2UL
#define PTE_W 4UL
#define PTE_X 8UL
#define PTE_A 64UL
#define PTE_D 128UL
#define PTE_NC (1UL << 61)
#define LEAF(pa, perm) ((((pa) >> 12) << 10) | PTE_A | PTE_D | PTE_V | (perm))

static uint64_t root[512]  __attribute__((aligned(4096)));
static uint64_t l1_w0[512] __attribute__((aligned(4096)));
uint64_t        l1_w1[512] __attribute__((aligned(4096)));   /* the op stream's remap writes entry 0 */
static uint64_t l1_w2[512] __attribute__((aligned(4096)));
extern const uint64_t region_init[0x20000 / 8];   /* ops.S: the region's initial contents (pointer page first) */
extern void run_ops(void);
extern void s_trap_entry(void);
volatile uint64_t dma_bursts, dma_sum;

static void putc_(uint8_t c) { while (!(UART[5] & 0x20)); UART[0] = c; }
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void puthex(uint64_t v) { for (int i = 60; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]); }
static inline void mmio_w32(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline uint32_t mmio_r32(uint64_t a) { return *(volatile uint32_t *)a; }

void trap_c(uint64_t cause, uint64_t epc, uint64_t tval) {
    if (cause == (1UL << 63 | 9)) {                    /* S external: the DMA agent's burst */
        uint32_t src = mmio_r32(PLIC + 0x201004);       /* claim (context 1) */
        if (src == 11) {
            uint64_t s = 0;
            for (uint64_t off = 0; off < 0xFF8; off += 8) s += *(volatile uint64_t *)(W2 + off);
            dma_sum += s;  dma_bursts++;
            *(volatile uint64_t *)(W2 + 0xFF8) = dma_bursts;    /* the ack the agent waits for */
        }
        mmio_w32(PLIC + 0x201004, src);                 /* complete */
        return;
    }
    puts_("MEMRAND-TRAP cause="); puthex(cause); puts_(" epc="); puthex(epc); puts_(" tval="); puthex(tval); puts_("\r\n");
    for (;;) ;
}

static void main_s(void) {
    volatile uint64_t *p = (volatile uint64_t *)W0;    /* the whole region, through alias 0 */
    for (int i = 0; i < 0x20000 / 8; i++) p[i] = region_init[i];
    asm volatile ("fence rw,rw" ::: "memory");
    asm volatile ("csrs sstatus, %0" :: "r"(2UL));      /* SIE */
    run_ops();
    asm volatile ("csrc sstatus, %0" :: "r"(2UL));
    uint64_t sum = 0;
    for (uint64_t off = 4096; off < 0x20000; off += 8) sum += *(volatile uint64_t *)(W1 + off);
    puts_("MEMRAND-DONE sum="); puthex(sum); puts_(" bursts="); puthex(dma_bursts); puts_(" dma="); puthex(dma_sum); puts_("\r\n");
    for (;;) ;
}

void main_m(void) {
    root[0]   = LEAF(0x00000000UL, PTE_R | PTE_W);           /* devices: UART, PLIC (gigapage) */
    root[2]   = LEAF(0x80000000UL, PTE_R | PTE_W | PTE_X);   /* identity: code, stack, tables, the region */
    root[64]  = ((((uint64_t)l1_w0) >> 12) << 10) | PTE_V;
    root[128] = ((((uint64_t)l1_w1) >> 12) << 10) | PTE_V;
    root[192] = ((((uint64_t)l1_w2) >> 12) << 10) | PTE_V;
    l1_w0[0]  = LEAF(DATA_PA, PTE_R | PTE_W);                /* 2 MiB megapages */
    l1_w1[0]  = LEAF(DATA_PA, PTE_R | PTE_W);
    l1_w2[0]  = LEAF(DMA_PA & ~0x1FFFFFUL, PTE_R | PTE_W) | PTE_NC;   /* W2 + 0 = the 2 MiB page holding DMA_PA */
    asm volatile ("csrw satp, %0" :: "r"((8UL << 60) | (((uint64_t)root) >> 12)));
    asm volatile ("sfence.vma" ::: "memory");
    mmio_w32(PLIC + 4 * 11, 1);                    /* source 11 priority 1 */
    mmio_w32(PLIC + 0x2080, 1u << 11);             /* enable for context 1 (S-mode) */
    mmio_w32(PLIC + 0x201000, 0);                  /* threshold */
    asm volatile ("csrw medeleg, %0" :: "r"(0xFFFFUL));
    asm volatile ("csrw mideleg, %0" :: "r"(0x222UL));
    asm volatile ("csrw stvec, %0" :: "r"((uint64_t)s_trap_entry));
    asm volatile ("csrw sie, %0" :: "r"(1UL << 9));             /* SEIE */
    asm volatile ("csrw mepc, %0" :: "r"((uint64_t)main_s));
    /* Written whole, never read-modify-written: the reset value of MPP is not architected and the
       two models differ (the DUT 0, simmerv M) -- no real software reads it before writing it. */
    asm volatile ("csrw mstatus, %0" :: "r"((1UL << 11) | (1UL << 5) | (3UL << 13)));   /* MPP = S, SPIE, FS = Dirty */
    asm volatile ("mret");
}
