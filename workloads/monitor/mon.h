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

// DDR scratch for the disk's queue, metadata and boot.txt: the device's DMA reaches DDR, not
// the monitor's SRAM. Below the initrd and DTB that boot.txt places under the framebuffer, far
// above the payload at 0x8000_0000; the simulation's 512 MiB DDR takes it lower.
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

// Zicbom over a range, by 64-byte block: 0 invalidate, 1 clean, 2 flush (disk.c)
void cbo(uint64_t a, uint64_t n, int op);

// video.c: the screen, a splash and a console mirroring the monitor's output
void video_init(void);
void video_putc(char c);

// disk.c: the SD card's EFI System Partition, read-only
int  disk_load(const char *path, uint64_t addr, uint64_t max, uint64_t *size);   // -2: no such file
int  disk_list(const char *path);
void disk_quiesce(void);

#endif
