// The splash on the host: video.c against a RAM framebuffer, the monitor's first lines through its
// console, written out at the RGB222 the scanout drives (make preview -> splash.ppm).
#include <stdio.h>
#include <stdint.h>
#define MON_H
typedef uint64_t u64;
static uint16_t fbmem[640 * 480] __attribute__((aligned(64)));
static uint32_t vgaregs[64];
void cbo(uint64_t a, uint64_t n, int op) { (void)a; (void)n; (void)op; }
#define FB_BASE ((uint64_t)fbmem)
#define VGA vgaregs
#include "video.c"
int main(void)
{
    video_init();
    const char *s = "\nsmolrv64 monitor  rtl=2a9f0c41 fw=20261005104512 err=0000000000000000\n"
                    "autoboot from the SD card in 10 s (key[0] or any key: monitor) 10 9 8 7 \n"
                    "/smolrv64/boot.txt: 112 bytes in 3 ms\n"
                    "boot> L80000000 /smolrv64/fw_payload.bin\n"
                    "...................\n/smolrv64/fw_payload.bin: 19657736 bytes in 8123 ms\n"
                    "boot> Lffdff000 /smolrv64/smolrv64.dtb\n/smolrv64/smolrv64.dtb: 3323 bytes in 4 ms\n"
                    "boot> X80000000 0 ffdff000\njumping...\n";
    for (; *s; s++) video_putc(*s);
    FILE *f = fopen("splash.ppm", "wb");
    fprintf(f, "P6 640 480 255\n");
    for (int i = 0; i < 640 * 480; i++) {
        uint16_t p = fbmem[i];
        int r = (p >> 14) & 3, g = (p >> 9) & 3, b = (p >> 3) & 3;   /* the top two bits, as the scanout */
        fputc(r * 85, f); fputc(g * 85, f); fputc(b * 85, f);
    }
    fclose(f);
    return 0;
}
