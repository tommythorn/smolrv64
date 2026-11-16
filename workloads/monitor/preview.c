// The splash on the host: video.c against a RAM framebuffer, the monitor's first lines through its
// console, written out at the RGB222 the scanout drives (make preview -> splash.ppm).
#include <stdio.h>
#include <stdint.h>
#define MON_H
typedef uint64_t u64;
static uint16_t fbmem[640 * 480] __attribute__((aligned(64)));
static uint32_t vgaregs[64];
void cbo(uint64_t a, uint64_t n, int op) { (void)a; (void)n; (void)op; }
static void putc_(char c) { (void)c; }
static void puts_(const char *s) { (void)s; }
static void putdec(uint64_t v) { (void)v; }
#define TIMEBASE_HZ 502008ul
static uint64_t now(void) { static uint64_t t; return t++; }   // the scanout never locks here
#define FB_BASE ((uint64_t)fbmem)
#define VGA vgaregs
#include "video.c"
int main(void)
{
    video_init();
    const char *s = "\nsmolrv64 monitor  rtl=2a9f0c41 fw=20261005104512 err=0000000000000000\n"
                    "go: boot over TFTP (or key[0]); B: from the SD card; ? for help\n"
                    "> go\n"
                    "net: 192.168.1.180 from DHCP for 02:53:7c:76:21:25, TFTP from 192.168.1.10 (next-server)\n"
                    "boot.txt: 160 bytes in 2 ms (blksize 1468)\n"
                    "boot> N80000000 fw_payload.bin\n"
                    "...................\nfw_payload.bin: 19657736 bytes in 2900 ms (blksize 1468)\n"
                    "boot> Nfefff000 smolrv64.dtb\nsmolrv64.dtb: 3319 bytes in 3 ms (blksize 1468)\n"
                    "boot> X80000000 0 fefff000\njumping...\n";
    for (; *s; s++) video_putc(*s);
    FILE *f = fopen("splash.ppm", "wb");
    fprintf(f, "P6 %d %d 255\n", W, H);
    for (int i = 0; i < W * H; i++) {
        uint16_t p = fbmem[i];
        int r = (p >> 14) & 3, g = (p >> 9) & 3, b = (p >> 3) & 3;   /* the top two bits, as the scanout */
        fputc(r * 85, f); fputc(g * 85, f); fputc(b * 85, f);
    }
    fclose(f);
    return 0;
}
