// Bare-metal virtio-net TX DMA coherence test — Svpbmt NC variant.
//
// Same as virtio_coh.c, but the virtqueue is accessed through a non-cacheable
// (Svpbmt PBMT=01) alias, exactly like Linux's dma_alloc_coherent vring. This
// answers the open question: on real silicon, does an *NC* read see another
// master's DMA write without any cbo.flush?
//
// MMU layout (Sv39, 1 GiB superpages), run M-mode with MPRV/MPP=S so only data
// accesses translate (code fetch stays M-mode physical, like svpbmt_nc_test):
//   VA 0x00000000+ -> PA 0x00000000  PMA   (MMIO: UART, virtio @0x10003000)
//   VA 0x40000000+ -> PA 0x80000000  NC    (non-cacheable alias of DRAM)
//   VA 0x80000000+ -> PA 0x80000000  PMA   (code/data/stack, cacheable)
//
// Reads used->idx three ways after the device completes:
//   nc  used_idx  - via the NC alias (0x40102000) -> should re-fetch DRAM
//   cpu cached    - via the cacheable identity view (0x80102000), pre-primed
//   cpu flushed   - cacheable view after cbo.flush
// If nc==1 while cached==0, NC reads are coherent with device writes and the
// Linux fix is software/DT (make the vring NC). If nc==0, the NC read path is
// the RTL bug.

typedef unsigned char  u8;
typedef unsigned int   u32;
typedef unsigned long  u64;

#define UART     ((volatile u8 *)0x10000000UL)
#define LSR_THRE 0x20

#define VNET_BASE 0x10003000UL
#define VREG(off) (*(volatile u32 *)(VNET_BASE + (off)))

#define R_DRV_FEAT      0x020
#define R_DRV_FEAT_SEL  0x024
#define R_QUEUE_SEL     0x030
#define R_QUEUE_NUM     0x038
#define R_QUEUE_READY   0x044
#define R_QUEUE_NOTIFY  0x050
#define R_INT_STATUS    0x060
#define R_STATUS        0x070
#define R_DESC_LO       0x080
#define R_DESC_HI       0x084
#define R_DRIVER_LO     0x090
#define R_DRIVER_HI     0x094
#define R_DEVICE_LO     0x0a0
#define R_DEVICE_HI     0x0a4

#define D_NOTIFY    0xf04
#define D_RD_AVAIL  0xf08
#define D_EMPTY     0xf0c
#define D_RD_RING   0xf10
#define D_COMPLETE  0xf14
#define D_IRQ       0xf18
#define D_DMA_ERR   0xf1c
#define D_INDICES   0xf20
#define D_USED_HEAD 0xf24

#define S_ACK 1
#define S_DRIVER 2
#define S_DRIVER_OK 4
#define S_FEATURES_OK 8

// Physical ring addresses (also what the device is programmed with).
#define DESC_PA  0x80100000UL
#define AVAIL_PA 0x80101000UL
#define USED_PA  0x80102000UL
// NC alias: VA 0x40000000 maps PA 0x80000000.
#define NC(pa)   ((pa) - 0x80000000UL + 0x40000000UL)
#define DESC_NC  NC(DESC_PA)
#define AVAIL_NC NC(AVAIL_PA)
#define USED_NC  NC(USED_PA)

#define PT_PA 0x80200000UL   // root page table (Sv39)

static void putc_(u8 c)        { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s){ while (*s) putc_(*s++); }
static void hex(u64 v, int nyb){ for (int i = nyb - 1; i >= 0; i--) {
                                     u8 d = (v >> (i * 4)) & 0xf;
                                     putc_(d < 10 ? '0' + d : 'a' + d - 10); } }
static void row(const char *name, u32 v){ puts_(name); hex(v, 8); puts_("\r\n"); }

static void cbo_flush(u64 a){ asm volatile ("cbo.flush (%0)" :: "r"(a) : "memory"); }
static u32  used_idx_of(u32 word){ return (word >> 16) & 0xffff; }

static void enable_nc_mmu(void)
{
    volatile u64 *pt = (volatile u64 *)PT_PA;
    pt[0] = 0x000000CFUL;            // VA 0G->PA 0,          V|R|W|X|A|D, PMA
    pt[1] = 0x20000000200000CFUL;    // VA 1G->PA 0x80000000, +PBMT=01 (NC)
    pt[2] = 0x00000000200000CFUL;    // VA 2G->PA 0x80000000, PMA
    cbo_flush(PT_PA);                // PTW reads bypass the cache
    asm volatile ("fence" ::: "memory");
    asm volatile ("csrs 0x30a, %0" :: "r"(0x4000000000000000UL)); // menvcfg.PBMTE
    asm volatile ("csrw 0x180, %0" :: "r"(0x8000000000080200UL)); // satp Sv39,PPN=0x80200
    asm volatile ("sfence.vma");
    asm volatile ("csrs 0x300, %0" :: "r"(0x0000000000020800UL)); // mstatus MPRV|MPP=S
}

int main(void)
{
    puts_("\r\n=== virtio-net TX DMA coherence test (NC alias) ===\r\n");

    enable_nc_mmu();   // from here, data accesses are S-translated

    // bring the device up
    VREG(R_STATUS) = 0;
    VREG(R_STATUS) = S_ACK;
    VREG(R_STATUS) = S_ACK | S_DRIVER;
    VREG(R_DRV_FEAT_SEL) = 1; VREG(R_DRV_FEAT) = 0x3;
    VREG(R_DRV_FEAT_SEL) = 0; VREG(R_DRV_FEAT) = 0;
    VREG(R_STATUS) = S_ACK | S_DRIVER | S_FEATURES_OK;

    // build the TX virtqueue through the NC alias (writes go straight to DRAM)
    *(volatile u64 *)AVAIL_NC = 0x0000000000010000UL;  // flags0 idx1 ring[0]=0
    *(volatile u64 *)USED_NC  = 0;
    *(volatile u64 *)DESC_NC  = 0;
    asm volatile ("fence" ::: "memory");

    // program queue 1 with PHYSICAL addresses (device does raw DMA)
    VREG(R_QUEUE_SEL)  = 1;
    VREG(R_QUEUE_NUM)  = 8;
    VREG(R_DESC_LO)    = (u32)DESC_PA;  VREG(R_DESC_HI)   = 0;
    VREG(R_DRIVER_LO)  = (u32)AVAIL_PA; VREG(R_DRIVER_HI) = 0;
    VREG(R_DEVICE_LO)  = (u32)USED_PA;  VREG(R_DEVICE_HI) = 0;
    VREG(R_QUEUE_READY) = 1;
    VREG(R_STATUS) = S_ACK | S_DRIVER | S_FEATURES_OK | S_DRIVER_OK;

    // prime the *cacheable* view of used->idx (==0) for the contrast read
    u32 primed = *(volatile u32 *)USED_PA;

    VREG(R_QUEUE_NOTIFY) = 1;        // kick

    u32 dev_used_idx = 0, iters;
    for (iters = 0; iters < 2000000; iters++) {
        dev_used_idx = VREG(D_USED_HEAD) >> 16;
        if (dev_used_idx) break;
    }

    // the key read: NC alias, no cbo.flush
    u32 nc_used = *(volatile u32 *)USED_NC;
    // contrast: cacheable view (primed), then forced re-fetch
    u32 cached  = *(volatile u32 *)USED_PA;
    cbo_flush(USED_PA);
    u32 flushed = *(volatile u32 *)USED_PA;

    u32 d_notify = VREG(D_NOTIFY), d_rdav = VREG(D_RD_AVAIL),
        d_empty  = VREG(D_EMPTY),  d_rdring = VREG(D_RD_RING),
        d_compl  = VREG(D_COMPLETE), d_irq = VREG(D_IRQ),
        d_err    = VREG(D_DMA_ERR),  d_idx = VREG(D_INDICES),
        d_uh     = VREG(D_USED_HEAD), d_int = VREG(R_INT_STATUS);

    row("status      =0x", VREG(R_STATUS));
    row("notify_cnt  =0x", d_notify);
    row("rd_avail    =0x", d_rdav);
    row("empty_avail =0x", d_empty);
    row("rd_ring     =0x", d_rdring);
    row("complete    =0x", d_compl);
    row("irq_cnt     =0x", d_irq);
    row("dma_err     =0x", d_err);
    row("indices     =0x", d_idx);
    row("used_head   =0x", d_uh);
    row("int_status  =0x", d_int);
    row("poll_iters  =0x", iters);
    row("dev used_idx=0x", dev_used_idx);
    row("nc  used_idx=0x", used_idx_of(nc_used));
    row("cpu primed  =0x", used_idx_of(primed));
    row("cpu cached  =0x", used_idx_of(cached));
    row("cpu flushed =0x", used_idx_of(flushed));

    puts_("VERDICT: ");
    if (dev_used_idx != 1)
        puts_("device did NOT complete (device-side / config; see counters)\r\n");
    else if (used_idx_of(nc_used) == 1)
        puts_("NC read is COHERENT with device write -> noncoherent HW works; "
              "Linux fix is SW/DT (vring must be NC)\r\n");
    else
        puts_("NC read STALE (0) -> NC-read path does NOT see device DMA write "
              "-> RTL bug in the NC path\r\n");

    puts_("=== done ===\r\n");
    return 0;   // back to the monitor prompt
}
