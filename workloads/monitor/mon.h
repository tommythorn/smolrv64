// What the monitor's files share: the types, the console and the clock.
#ifndef MON_H
#define MON_H

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

// The mtime tick at the shipping clock (PROBE_CLK_DIV8=48): the DTS timebase-frequency that
// tools/check-dts-timebase.py holds the device trees to.
#define TIMEBASE_HZ 502008ul

// DDR scratch for the devices' queues and buffers and for boot.txt: their DMA reaches DDR, not
// the monitor's SRAM. Below the initrd and DTB that boot.txt places under the framebuffer, far
// above the payload at 0x8000_0000; the simulation's 512 MiB DDR takes it lower. The disk owns
// +0x0000..+0x5000, boot.txt +0x10000 (4 KiB), the network +0x20000..+0x40000.
#ifndef DMA_BASE
#define DMA_BASE 0xFE000000ul
#endif

// The framebuffer, RGB565 in DDR (video.c): the device tree's framebuffer node.
#ifndef FB_BASE
#define FB_BASE 0xFF000000ul
#endif

// The default boot's device tree, the page under the framebuffer.
#define DTB_BASE (FB_BASE - 0x1000ul)

// The board's DRAM in bytes; 0 probes for 2 or 4 GiB (memtest.c). A board with less states it:
// its controller is set for 16 Gb parts, so an address past its end is not a plain alias.
#ifndef DRAM_SIZE
#define DRAM_SIZE 0ul
#endif

// Integrity log (rv_soc_top.v, the window the fetch-buffer diagnostic used to own,
// 0x1000E000): the design's own invariants, latched in hardware by rv_errlog. 64-bit words:
// [0]={version,"ERRL"} magic, [1]=sticky, one bit per invariant that has fired since reset
// (D$ [15:0], I$ [31:16], LSU [47:32]; the bit numbers are the ones in rv_cache.v's and
// smolrv64_lsu.v's INTEGRITY LOG blocks), [2]={idx[55:48], cycle[47:0]} of the FIRST to fire.
// Linux userland reads the same words through /dev/mem (tools/errlog-read.sh).
#define ERRLOG_BASE     ((volatile uint64_t *)0x1000E000)
#define ERRLOG_MAGIC    0x4552524cu

static inline uint64_t now(void)
{
    uint64_t t;
    asm volatile ("csrr %0, time" : "=r"(t));
    return t;
}

void putc_(char c);
void puts_(const char *s);
void puthex8(uint32_t v);
void puthex32(uint32_t v);
void puthex64(uint64_t v);
void putdec(uint64_t v);
int  con_key(void);

// memtest.c: the `M` command
int  memtest(uint64_t a, uint64_t n, uint64_t passes);

// Zicbom over a range, by 64-byte block: 0 invalidate, 1 clean, 2 flush (vio.c)
void cbo(uint64_t a, uint64_t n, int op);
// vio.c: virtio-mmio devices and split virtqueues, polled. A queue occupies VQ_SPAN at its base;
// the caller cleans the descriptors it writes before vq_add.
#define VQ_MAX 256
#define VQ_SPAN 0x4000
#define D_NEXT 1
#define D_WRITE 2
struct vq_desc { uint64_t addr; uint32_t len; uint16_t flags; uint16_t next; };
struct vq { volatile uint32_t *dev; uint32_t sel; uint64_t base; uint16_t n, avail, used; };
#define VQ_DESC(q) ((volatile struct vq_desc *)(q)->base)
int  vio_start(volatile uint32_t *dev, uint32_t devid, const char *name);
int  vio_offers(volatile uint32_t *dev, int bit);
int  vio_queue(struct vq *q, volatile uint32_t *dev, uint32_t sel, uint64_t base, const char *name);
void vio_ready(volatile uint32_t *dev);
void vio_reset(volatile uint32_t *dev);
void vq_add(struct vq *q, uint16_t head);       // make a chain available (not yet visible)
void vq_kick(struct vq *q);                      // clean the queue, notify the device
int  vq_used(struct vq *q, uint32_t *len);       // the next used head, or -1
// net.c: virtio-net, DHCP and a TFTP client to the server on coffee
int  net_load(const char *path, uint64_t addr, uint64_t max, uint64_t *size);   // -2: no such file
void net_quiesce(void);

// video.c: the screen, a splash and a console mirroring the monitor's output
void video_init(void);
void video_putc(char c);
int  video_cmd(const char *args);                // V: show or set the mode (an X11 modeline)
void video_dtb(uint64_t dtb);                    // a device tree's framebuffer node to the mode

// disk.c: the SD card's EFI System Partition, read-only
int  disk_load(const char *path, uint64_t addr, uint64_t max, uint64_t *size);   // -2: no such file
int  disk_list(const char *path);
void disk_quiesce(void);

#endif
