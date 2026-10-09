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
// +0x0000..+0x2000, boot.txt +0x10000 (4 KiB), the network +0x20000..+0x44000.
#ifndef DMA_BASE
#define DMA_BASE 0xFE000000ul
#endif

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

// Zicbom over a range, by 64-byte block: 0 invalidate, 1 clean, 2 flush (vio.c)
void cbo(uint64_t a, uint64_t n, int op);
// vio.c: virtio-mmio devices and split virtqueues, polled. A queue occupies 4 KiB at its base.
#define VQ_MAX 64
#define D_NEXT 1
#define D_WRITE 2
struct vq_desc { uint64_t addr; uint32_t len; uint16_t flags; uint16_t next; };
struct vq { volatile uint32_t *dev; uint32_t sel; uint64_t base; uint16_t n, avail, used; };
#define VQ_DESC(q) ((volatile struct vq_desc *)(q)->base)
int  vio_start(volatile uint32_t *dev, uint32_t devid, const char *name);
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

// disk.c: the SD card's EFI System Partition, read-only
int  disk_load(const char *path, uint64_t addr, uint64_t max, uint64_t *size);   // -2: no such file
int  disk_list(const char *path);
void disk_quiesce(void);

#endif
