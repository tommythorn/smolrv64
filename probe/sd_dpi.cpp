// DPI bridge: a single file-backed SpiSdCard (the behavioral SPI-mode SD card model)
// clocked by tb_virtio.v's SD-SPI pins each cycle. Backs virtio_blk's SD backend with a
// real disk image so the kernel can mount root=/dev/vda under simulation.
//   sd_attach(path) : open the image O_RDWR and attach it (sizes the card capacity).
//   sd_clock(sck,cs_n,mosi) -> miso : advance one SPI bit-clock edge.
#include "sd_spi_card_model.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

static SpiSdCard g_card;

extern "C" void sd_attach(const char* path) {
    int fd = open(path, O_RDWR);
    if (fd < 0) { fprintf(stderr, "[sd_dpi] FATAL: cannot open disk image %s\n", path); return; }
    g_card.attach_image(fd);
    off_t sz = lseek(fd, 0, SEEK_END);
    fprintf(stderr, "[sd_dpi] attached %s (%lld bytes, capacity %u sectors)\n",
            path, (long long)sz, (unsigned)((g_card.csd_csize + 1) * 1024));
}

extern "C" int sd_clock(int sck, int cs_n, int mosi) {
    g_card.clock_edge(sck & 1, cs_n & 1, mosi & 1);
    return g_card.miso & 1;
}
