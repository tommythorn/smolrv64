// Bare-metal virtio-net TX DMA coherence test for smolrv64 (real hardware).
//
// Reproduces, with no Linux in the way, the exact hop that fails when Ubuntu's
// virtio-net wedges after one packet: the device DMA-writes the used ring and
// the CPU never sees it. The virtio-net engine is "tx_drop" — it does NOT read
// the descriptor table or packet, only avail->idx + avail->ring[0], then writes
// used->{ring[0].id, idx} and raises an interrupt. So the minimal trigger is a
// one-entry avail ring + a used ring + the MMIO config + a kick.
//
// We run cacheable (no MMU) and use Zicbom CMO so each direction is explicit:
//   - avail (CPU->device): cbo.flush so the device reads fresh from DRAM.
//   - used  (device->CPU): prime the cache with used->idx==0 (as the Linux
//     driver would have, polling before completion), kick, then read used->idx
//     three ways and print all of them over the UART:
//        dev used_idx  - the device's own debug counter (MMIO, ground truth)
//        cpu cached    - a plain cacheable load (may return the stale 0)
//        cpu flushed   - a load after cbo.flush (forced re-fetch from DRAM)
//
// Verdict:
//   dev=1, cached=0, flushed=1 -> device write reaches DRAM; CPU read is stale
//                                  (device->CPU cache visibility) -> the Linux
//                                  NC-vring path is the thing to chase next.
//   dev=1, flushed=0           -> device write never reached DRAM (deeper bug).
//   dev!=1                     -> device never completed (see counters below).
//   dev=1, cached=1            -> no staleness at all.

typedef unsigned char  u8;
typedef unsigned int   u32;
typedef unsigned long  u64;

#define UART     ((volatile u8 *)0x10000000UL)
#define LSR_THRE 0x20

#define VNET_BASE 0x10003000UL
#define VREG(off) (*(volatile u32 *)(VNET_BASE + (off)))

// virtio-mmio register offsets
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
#define R_DRIVER_LO     0x090   // avail ring base
#define R_DRIVER_HI     0x094
#define R_DEVICE_LO     0x0a0   // used ring base
#define R_DEVICE_HI     0x0a4

// virtio-net TX-engine debug block (rk_xcku5p.v: virtio_net_debug_*)
#define D_NOTIFY    0xf04
#define D_RD_AVAIL  0xf08
#define D_EMPTY     0xf0c
#define D_RD_RING   0xf10
#define D_COMPLETE  0xf14
#define D_IRQ       0xf18
#define D_DMA_ERR   0xf1c
#define D_INDICES   0xf20   // {last_avail_idx, avail_idx}
#define D_USED_HEAD 0xf24   // {used_idx, head_desc}

// device_status bits
#define S_ACK         1
#define S_DRIVER      2
#define S_DRIVER_OK   4
#define S_FEATURES_OK 8

// TX virtqueue in DRAM (8-byte/page aligned, clear of code+stack)
#define DESC_ADDR  0x80100000UL
#define AVAIL_ADDR 0x80101000UL
#define USED_ADDR  0x80102000UL

static void putc_(u8 c)        { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s){ while (*s) putc_(*s++); }
static void hex(u64 v, int nyb){ for (int i = nyb - 1; i >= 0; i--) {
                                     u8 d = (v >> (i * 4)) & 0xf;
                                     putc_(d < 10 ? '0' + d : 'a' + d - 10); } }
static void row(const char *name, u32 v){ puts_(name); hex(v, 8); puts_("\r\n"); }

static void cbo_flush(u64 a){ asm volatile ("cbo.flush (%0)" :: "r"(a) : "memory"); }

// used->idx lives in bits[31:16] of the first word of the used ring.
static u32 used_idx_of(u32 word){ return (word >> 16) & 0xffff; }

int main(void)
{
    puts_("\r\n=== virtio-net TX DMA coherence test ===\r\n");

    // 1. bring the device up through feature negotiation.
    VREG(R_STATUS) = 0;
    VREG(R_STATUS) = S_ACK;
    VREG(R_STATUS) = S_ACK | S_DRIVER;
    VREG(R_DRV_FEAT_SEL) = 1; VREG(R_DRV_FEAT) = 0x3;   // VERSION_1 | ACCESS_PLATFORM
    VREG(R_DRV_FEAT_SEL) = 0; VREG(R_DRV_FEAT) = 0;
    VREG(R_STATUS) = S_ACK | S_DRIVER | S_FEATURES_OK;

    // 2. build a one-entry TX avail ring; zero the used ring.
    //    avail word = {ring[0]=0, idx=1, flags=0} -> head descriptor 0 posted.
    *(volatile u64 *)AVAIL_ADDR = 0x0000000000010000UL;
    *(volatile u64 *)USED_ADDR  = 0;
    *(volatile u64 *)DESC_ADDR  = 0;                    // device never reads desc
    cbo_flush(AVAIL_ADDR); cbo_flush(USED_ADDR); cbo_flush(DESC_ADDR);
    asm volatile ("fence" ::: "memory");

    // 3. program queue 1 (TX) and go live.
    VREG(R_QUEUE_SEL)  = 1;
    VREG(R_QUEUE_NUM)  = 8;
    VREG(R_DESC_LO)    = (u32)DESC_ADDR;  VREG(R_DESC_HI)   = 0;
    VREG(R_DRIVER_LO)  = (u32)AVAIL_ADDR; VREG(R_DRIVER_HI) = 0;
    VREG(R_DEVICE_LO)  = (u32)USED_ADDR;  VREG(R_DEVICE_HI) = 0;
    VREG(R_QUEUE_READY) = 1;
    VREG(R_STATUS) = S_ACK | S_DRIVER | S_FEATURES_OK | S_DRIVER_OK;

    // 4. prime the CPU cache with used->idx (==0): mimic a driver that polled
    //    the used ring before the device completed.
    u32 primed = *(volatile u32 *)USED_ADDR;

    // 5. kick the TX queue (notify value == queue index 1).
    VREG(R_QUEUE_NOTIFY) = 1;

    // 6. spin until the device reports the completion (MMIO debug, uncached).
    u32 dev_used_idx = 0, iters;
    for (iters = 0; iters < 2000000; iters++) {
        dev_used_idx = VREG(D_USED_HEAD) >> 16;
        if (dev_used_idx) break;
    }

    // 7. capture both CPU views immediately, before any DRAM access can evict.
    u32 cached  = *(volatile u32 *)USED_ADDR;   // possibly the stale cached 0
    cbo_flush(USED_ADDR);
    u32 flushed = *(volatile u32 *)USED_ADDR;   // forced re-fetch from DRAM

    // 8. snapshot device counters.
    u32 d_notify = VREG(D_NOTIFY), d_rdav = VREG(D_RD_AVAIL),
        d_empty  = VREG(D_EMPTY),  d_rdring = VREG(D_RD_RING),
        d_compl  = VREG(D_COMPLETE), d_irq = VREG(D_IRQ),
        d_err    = VREG(D_DMA_ERR),  d_idx = VREG(D_INDICES),
        d_uh     = VREG(D_USED_HEAD), d_int = VREG(R_INT_STATUS);

    // 9. report.
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
    row("cpu primed  =0x", used_idx_of(primed));
    row("cpu cached  =0x", used_idx_of(cached));
    row("cpu flushed =0x", used_idx_of(flushed));

    puts_("VERDICT: ");
    if (dev_used_idx != 1)
        puts_("device did NOT complete (device-side / config; see counters)\r\n");
    else if (used_idx_of(flushed) != 1)
        puts_("device write NOT visible even after cbo.flush -> never reached DRAM\r\n");
    else if (used_idx_of(cached) != 1)
        puts_("STALE CACHE: cbo.flush reveals it -> CPU read of used ring is stale "
              "(device->CPU coherence)\r\n");
    else
        puts_("coherent: CPU saw the completion without explicit invalidate\r\n");

    puts_("=== done ===\r\n");
    return 0;   // back to the monitor prompt
}
