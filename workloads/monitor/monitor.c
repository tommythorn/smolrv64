// Simple memory monitor for smolrv64
// Commands:
//   R<addr>          - read and display 64-bit word at address
//   W<addr> <val>    - write 64-bit word to address
//   WW<addr> <val>   - write 32-bit word to address
//   WH<addr> <val>   - write 16-bit half-word to address
//   WB<addr> <val>   - write 8-bit byte to address
//   T<addr>          - hexdump 256 bytes starting at address
//   Y<addr>          - receive XMODEM-1K upload to address
//   C<addr> <len>    - blake3-256 of len bytes at address
//   Z<addr> <len> [b] - fill len bytes at address with byte b (default 0)
//   S [sector]       - probe SD card, or dump 512-byte sector in hex
//   SL<sector> <count> <addr> - read SD sectors into memory
//   X<addr> [a0 [a1]] - jump to address and execute
//   P                - dump core debug counters; Pc clears them
//   ?                - help

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

#include "blake3.h"

// Firmware build stamp (YYYYMMDDHHMMSS as hex digits, like the RTL stamp).
// Injected by the Makefile; 0 when built without it.
#ifndef MONITOR_BUILD_STAMP
#define MONITOR_BUILD_STAMP 0
#endif

// NS16550A UART at 0x10000000
#define UART0_BASE  ((volatile uint8_t *)0x10000000)
#define CLK_FREQ    333333333
#define UART_SPEED  3000000

#define UART_THR  0
#define UART_RBR  0
#define UART_DLL  0
#define UART_IER  1
#define UART_DLH  1
#define UART_FCR  2
#define UART_LCR  3
#define UART_LSR  5

#define LCR_DLAB  0x80
#define LCR_8N1   0x03
#define LSR_THRE  0x20
#define LSR_DR    0x01

#define SD_SPI_BASE     ((volatile uint32_t *)0x10001000)
#define SD_CS_GPIO_BASE ((volatile uint32_t *)0x10001100)
#define SD_CD_GPIO_BASE ((volatile uint32_t *)0x10001200)

#define SD_SPI_RXDATA   0
#define SD_SPI_TXDATA   1
#define SD_SPI_STATUS   2
#define SD_SPI_CONTROL  3
#define SD_SPI_BAUD     4

#define SD_SPI_READY    0x01

static uint8_t sd_sector_buf[512];

static void uart_init(volatile uint8_t *base, int clk_freq, int baud)
{
    int div = clk_freq / (16 * baud);
    if (div < 1) div = 1;
    base[UART_LCR] = LCR_DLAB;
    base[UART_DLL] = div & 0xFF;
    base[UART_DLH] = (div >> 8) & 0xFF;
    base[UART_LCR] = LCR_8N1;
    base[UART_FCR] = 0x07;
    base[UART_IER] = 0x00;
}

static uint8_t uart_getc(volatile uint8_t *base)
{
    while (!(base[UART_LSR] & LSR_DR));
    return base[UART_RBR];
}

static void uart_putc(volatile uint8_t *base, uint8_t c)
{
    while (!(base[UART_LSR] & LSR_THRE));
    base[UART_THR] = c;
}

static void putc_(char c)
{
    if (c == '\n')
        uart_putc(UART0_BASE, '\r');
    uart_putc(UART0_BASE, c);
}

static void puts_(const char *s)
{
    while (*s)
        putc_(*s++);
}

static void puthex4(uint32_t v)
{
    v &= 0xf;
    putc_(v < 10 ? '0' + v : 'a' + (v - 10));
}

static void puthex8(uint32_t v)
{
    v &= 0xff;
    puthex4(v >> 4);
    puthex4(v);
}

static void puthex32(uint32_t v)
{
    puthex8(v >> 24);
    puthex8(v >> 16);
    puthex8(v >> 8);
    puthex8(v);
}

static void puthex64(uint64_t v)
{
    puthex32((uint32_t)(v >> 32));
    puthex32((uint32_t)v);
}

static uint64_t read_build_stamp(void)
{
    uint64_t build_stamp;
    asm volatile ("csrr %0, 0xfde" : "=r"(build_stamp));
    return build_stamp;
}

static void sync_cache_for_exec(void)
{
    asm volatile (".word 0x0000100f" ::: "memory"); /* fence.i */
}

// Parse hex digits; returns pointer past last digit consumed, or 0 on error.
static const char *parse_hex(const char *s, uint64_t *out)
{
    uint64_t v = 0;
    int count = 0;
    while (1) {
        char c = *s;
        int d;
        if      (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else break;
        v = (v << 4) | d;
        count++;
        s++;
    }
    if (!count) return 0;
    *out = v;
    return s;
}

#define LINE_MAX 128

// Poor-man's readline command history (ring buffer of recent lines).
#define HIST_N 8
static char hist[HIST_N][LINE_MAX];
static int  hist_head = 0;   // next slot to write
static int  hist_count = 0;  // valid entries, 0..HIST_N

// The k-th most recent history line (k = 1..hist_count).
static const char *hist_get(int k)
{
    return hist[(hist_head - k + HIST_N) % HIST_N];
}

static void hist_add(const char *line)
{
    char *dst = hist[hist_head];
    int i = 0;
    while (line[i] && i < LINE_MAX - 1) { dst[i] = line[i]; i++; }
    dst[i] = 0;
    hist_head = (hist_head + 1) % HIST_N;
    if (hist_count < HIST_N)
        hist_count++;
}

static void readline(char *buf)
{
    int n = 0;
    int browse = 0;   // 0 = fresh line, 1..hist_count = how far back in history
    for (;;) {
        char c = uart_getc(UART0_BASE);
        if (c == '\r' || c == '\n') {
            putc_('\n');
            buf[n] = 0;
            if (n > 0)
                hist_add(buf);
            return;
        } else if (c == 27) {           // ESC: handle CSI arrow keys (ESC [ A/B)
            if (uart_getc(UART0_BASE) != '[')
                continue;
            char arrow = uart_getc(UART0_BASE);
            int want = browse;
            if (arrow == 'A') want = browse + 1;        // up: older
            else if (arrow == 'B') want = browse - 1;   // down: newer
            if (want < 0) want = 0;
            if (want > hist_count) want = hist_count;
            if (want == browse)
                continue;
            browse = want;
            while (n > 0) { puts_("\b \b"); n--; }       // erase current line
            if (browse > 0) {
                const char *h = hist_get(browse);
                while (h[n] && n < LINE_MAX - 1) {
                    buf[n] = h[n];
                    putc_(h[n]);
                    n++;
                }
            }
        } else if (c == '\b' || c == 0x7f) {
            if (n > 0) {
                n--;
                puts_("\b \b");
            }
        } else if (c >= ' ' && n < LINE_MAX - 1) {
            buf[n++] = c;
            putc_(c);
        }
    }
}

// hexdump -C style: 16 bytes per row, address | hex | ascii
static void hexdump(uint64_t addr, int len)
{
    int i;
    for (i = 0; i < len; i += 16) {
        int j, row = len - i < 16 ? len - i : 16;
        puthex64(addr + i);
        puts_(":  ");
        for (j = 0; j < 16; j++) {
            if (j < row)
                puthex8(((volatile uint8_t *)(addr + i))[j]);
            else
                puts_("  ");
            putc_(j == 7 ? '-' : ' ');
        }
        putc_(' ');
        putc_('|');
        for (j = 0; j < row; j++) {
            uint8_t b = ((volatile uint8_t *)(addr + i))[j];
            putc_(b >= 0x20 && b < 0x7f ? b : '.');
        }
        putc_('|');
        putc_('\n');
    }
}

static void hexdump_bytes(const uint8_t *data, uint64_t base, int len)
{
    int i;
    for (i = 0; i < len; i += 16) {
        int j, row = len - i < 16 ? len - i : 16;
        puthex64(base + i);
        puts_(":  ");
        for (j = 0; j < 16; j++) {
            if (j < row)
                puthex8(data[i + j]);
            else
                puts_("  ");
            putc_(j == 7 ? '-' : ' ');
        }
        putc_(' ');
        putc_('|');
        for (j = 0; j < row; j++) {
            uint8_t b = data[i + j];
            putc_(b >= 0x20 && b < 0x7f ? b : '.');
        }
        putc_('|');
        putc_('\n');
    }
}

/* ── XMODEM-1K receive ───────────────────────────────────────────────── */
/* Simpler and more reliable than Y-modem: no header block, no batch end.
 * Host side: sx -k <file>   (lrzsz)
 *         or: sz --xmodem -k <file> */
#define XM_SOH  0x01   /* 128-byte block */
#define XM_STX  0x02   /* 1024-byte block */
#define XM_EOT  0x04
#define XM_ACK  0x06
#define XM_NAK  0x15
#define XM_CAN  0x18

static uint16_t xm_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    for (; len--; buf++) {
        crc ^= (uint16_t)*buf << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/* Non-blocking UART read with a spin-count timeout.
 * Returns 1 and stores byte in *out if a byte arrived; 0 on timeout. */
static int uart_getc_tmo(uint32_t tmo, uint8_t *out)
{
    while (tmo--) {
        if (UART0_BASE[UART_LSR] & LSR_DR) {
            *out = UART0_BASE[UART_RBR];
            return 1;
        }
    }
    return 0;
}

/* Receive XMODEM-1K to addr.
 * Sends 'C' every ~1 s until the sender responds (up to ~30 s total).
 * Returns bytes written (always a multiple of block size — last block may
 * contain up to 1023 padding bytes), or (uint64_t)-1 on cancel/timeout. */
static uint64_t xmodem1k_recv(uint64_t addr)
{
    static uint8_t blkbuf[1024]; /* static to keep off the stack */
    volatile uint8_t *dst = (volatile uint8_t *)addr;
    uint64_t total = 0;
    int next_blk = 1;
    uint8_t c;

    /* Send 'C' with ~1-second retries until the sender starts */
    for (int tries = 30; tries > 0; tries--) {
        uart_putc(UART0_BASE, 'C');
        if (uart_getc_tmo(CLK_FREQ / 5, &c))  /* ~200 ms per poll */
            goto got;
    }
    return (uint64_t)-1;  /* sender never responded */

got:
    for (;;) {
        if (c == XM_EOT) {
            uart_putc(UART0_BASE, XM_ACK);
            return total;
        }
        if (c == XM_CAN) {
            /* Two consecutive CANs = abort */
            if (uart_getc_tmo(CLK_FREQ / 10, &c) && c == XM_CAN) {
                uart_putc(UART0_BASE, XM_ACK);
                return (uint64_t)-1;
            }
            goto next;
        }
        if (c != XM_SOH && c != XM_STX) goto next;

        int blen    = (c == XM_STX) ? 1024 : 128;
        uint8_t blk = uart_getc(UART0_BASE);
        uint8_t inv = uart_getc(UART0_BASE);
        for (int i = 0; i < blen; i++)
            blkbuf[i] = uart_getc(UART0_BASE);
        uint16_t crc_got = ((uint16_t)uart_getc(UART0_BASE) << 8)
                          |  uart_getc(UART0_BASE);

        if ((uint8_t)(blk ^ inv) != 0xFF || xm_crc16(blkbuf, blen) != crc_got) {
            uart_putc(UART0_BASE, XM_NAK);
            goto next;
        }
        if (blk == (uint8_t)(next_blk & 0xFF)) {
            for (int i = 0; i < blen; i++) *dst++ = blkbuf[i];
            total += blen;
            next_blk++;
        } else if (blk != (uint8_t)((next_blk - 1) & 0xFF)) {
            /* Out-of-sequence block: cancel */
            uart_putc(UART0_BASE, XM_CAN);
            uart_putc(UART0_BASE, XM_CAN);
            return (uint64_t)-1;
        }
        /* Duplicate of previous block: ACK and ignore */
        uart_putc(UART0_BASE, XM_ACK);

next:
        /* Wait up to ~10 s for next byte; timeout = stalled transfer */
        if (!uart_getc_tmo((uint32_t)CLK_FREQ * 10, &c)) {
            uart_putc(UART0_BASE, XM_CAN);
            uart_putc(UART0_BASE, XM_CAN);
            return (uint64_t)-1;
        }
    }
}

static void sd_cs_assert(int assert)
{
    SD_CS_GPIO_BASE[0] = assert ? 0 : 1;
}

static void sd_spi_init(int baud)
{
    SD_SPI_BASE[SD_SPI_CONTROL] = 0; /* mode 0 */
    SD_SPI_BASE[SD_SPI_BAUD] = (uint32_t)baud;
}

static uint8_t sd_spi_xfer(uint8_t tx, int *ok)
{
    uint32_t tmo = 1000000;

    SD_SPI_BASE[SD_SPI_TXDATA] = tx;
    while (tmo--) {
        if (SD_SPI_BASE[SD_SPI_STATUS] & SD_SPI_READY)
            return (uint8_t)SD_SPI_BASE[SD_SPI_RXDATA];
    }

    *ok = 0;
    return 0xff;
}

static void sd_idle_clocks(int bytes, int *ok)
{
    while (bytes-- && *ok)
        (void)sd_spi_xfer(0xff, ok);
}

static int sd_cmd_raw(uint8_t cmd, uint32_t arg, uint8_t crc,
                      uint8_t *resp, int resp_len, int *ok)
{
    uint8_t r = 0xff;

    (void)sd_spi_xfer(0xff, ok);
    (void)sd_spi_xfer(0x40 | cmd, ok);
    (void)sd_spi_xfer((uint8_t)(arg >> 24), ok);
    (void)sd_spi_xfer((uint8_t)(arg >> 16), ok);
    (void)sd_spi_xfer((uint8_t)(arg >> 8), ok);
    (void)sd_spi_xfer((uint8_t)arg, ok);
    (void)sd_spi_xfer(crc, ok);

    for (int i = 0; i < 16 && *ok; i++) {
        r = sd_spi_xfer(0xff, ok);
        if ((r & 0x80) == 0)
            break;
    }

    if (!*ok || (r & 0x80))
        return -1;

    if (resp_len > 0)
        resp[0] = r;
    for (int i = 1; i < resp_len && *ok; i++)
        resp[i] = sd_spi_xfer(0xff, ok);

    return *ok ? 0 : -1;
}

static int sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc,
                  uint8_t *resp, int resp_len, int *ok)
{
    int rc;

    sd_cs_assert(1);
    rc = sd_cmd_raw(cmd, arg, crc, resp, resp_len, ok);
    sd_cs_assert(0);
    (void)sd_spi_xfer(0xff, ok);

    return rc;
}

static int sd_read_data(uint8_t cmd, uint32_t arg, volatile uint8_t *buf, int len, int *ok)
{
    uint8_t r1 = 0xff;

    sd_cs_assert(1);
    if (sd_cmd_raw(cmd, arg, 0x01, &r1, 1, ok) < 0 || r1 != 0) {
        sd_cs_assert(0);
        (void)sd_spi_xfer(0xff, ok);
        return r1;
    }

    for (uint32_t tmo = 100000; tmo && *ok; tmo--) {
        uint8_t token = sd_spi_xfer(0xff, ok);
        if (token == 0xfe) {
            for (int i = 0; i < len; i++)
                buf[i] = sd_spi_xfer(0xff, ok);
            (void)sd_spi_xfer(0xff, ok); /* crc */
            (void)sd_spi_xfer(0xff, ok);
            sd_cs_assert(0);
            (void)sd_spi_xfer(0xff, ok);
            return *ok ? 0 : -1;
        }
        if ((token & 0xf0) == 0)
            break;
    }

    sd_cs_assert(0);
    (void)sd_spi_xfer(0xff, ok);
    return -1;
}

static int sd_read_register(uint8_t cmd, uint8_t *buf, int *ok)
{
    return sd_read_data(cmd, 0, buf, 16, ok);
}

static int sd_sector_arg(uint64_t sector, int high_capacity, uint32_t *arg)
{
    if (high_capacity) {
        if (sector > 0xffffffff)
            return -1;
        *arg = (uint32_t)sector;
    } else {
        if (sector > 0x7fffff)
            return -1;
        *arg = (uint32_t)(sector << 9);
    }
    return 0;
}

static void print_sd_r1(const char *name, uint8_t r1)
{
    puts_(name);
    puts_(" R1=");
    puthex8(r1);
    if (r1 & 0x01) puts_(" idle");
    if (r1 & 0x02) puts_(" erase-reset");
    if (r1 & 0x04) puts_(" illegal-cmd");
    if (r1 & 0x08) puts_(" crc-err");
    if (r1 & 0x10) puts_(" erase-seq-err");
    if (r1 & 0x20) puts_(" addr-err");
    if (r1 & 0x40) puts_(" param-err");
    putc_('\n');
}

static void print_sd_csd_capacity(const uint8_t *csd)
{
    uint64_t capacity = 0;
    uint8_t csdver = csd[0] >> 6;

    puts_("CSD: ");
    for (int i = 0; i < 16; i++)
        puthex8(csd[i]);
    putc_('\n');

    if (csdver == 1) {
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3f) << 16) |
                          ((uint32_t)csd[8] << 8) |
                          csd[9];
        capacity = ((uint64_t)c_size + 1) << 19;
    } else if (csdver == 0) {
        uint32_t read_bl_len = csd[5] & 0x0f;
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) |
                          ((uint32_t)csd[7] << 2) |
                          ((csd[8] & 0xc0) >> 6);
        uint32_t c_size_mult = ((csd[9] & 0x03) << 1) |
                               ((csd[10] & 0x80) >> 7);
        capacity = ((uint64_t)c_size + 1) << (c_size_mult + 2 + read_bl_len);
    }

    puts_("CSD version=");
    puthex8(csdver);
    if (capacity) {
        puts_(" capacity=");
        puthex64(capacity);
        puts_(" bytes (");
        puthex64(capacity >> 20);
        puts_(" MiB)\n");
    } else {
        puts_(" capacity=unknown\n");
    }
}

static int sd_init_card(int verbose, int *high_capacity)
{
    int ok = 1;
    uint8_t r[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
    uint32_t cd_raw = SD_CD_GPIO_BASE[0] & 1;
    int initialized = 0;
    int v2_card = 0;
    uint32_t ocr = 0;

    if (high_capacity)
        *high_capacity = 0;

    if (verbose) {
        puts_("sd_cd raw=");
        puthex8(cd_raw);
        puts_(cd_raw ? " present=no (active-low)\n" : " present=yes (active-low)\n");
    }

    sd_spi_init(255); /* about 650 kHz from 333 MHz UI clock */
    sd_cs_assert(0);
    sd_idle_clocks(10, &ok);

    if (!ok) {
        if (verbose)
            puts_("SPI timeout during idle clocks\n");
        return -1;
    }

    if (sd_cmd(0, 0, 0x95, r, 1, &ok) < 0) {
        if (verbose)
            puts_("CMD0: no response\n");
        return -1;
    }
    if (verbose)
        print_sd_r1("CMD0", r[0]);

    if (sd_cmd(8, 0x000001aa, 0x87, r, 5, &ok) < 0) {
        if (verbose)
            puts_("CMD8: no response\n");
    } else {
        if (verbose) {
            print_sd_r1("CMD8", r[0]);
            puts_("CMD8 echo=");
            puthex8(r[3]);
            puthex8(r[4]);
            putc_('\n');
        }
        v2_card = !(r[0] & 0x04) && r[3] == 0x01 && r[4] == 0xaa;
    }

    for (int i = 0; i < 1000 && ok; i++) {
        uint8_t r55;
        uint32_t acmd41_arg = v2_card ? 0x40000000 : 0;
        if (sd_cmd(55, 0, 0x01, &r55, 1, &ok) < 0) {
            puts_("CMD55: no response\n");
            break;
        }
        if (sd_cmd(41, acmd41_arg, 0x01, r, 1, &ok) < 0) {
            puts_("ACMD41: no response\n");
            break;
        }
        if (r[0] == 0) {
            initialized = 1;
            if (verbose) {
                puts_("ACMD41 ready after ");
                puthex32(i + 1);
                puts_(" tries\n");
            }
            break;
        }
    }
    if (!initialized && verbose)
        print_sd_r1("ACMD41 last", r[0]);

    if (sd_cmd(58, 0, 0x01, r, 5, &ok) == 0) {
        ocr = ((uint32_t)r[1] << 24) |
              ((uint32_t)r[2] << 16) |
              ((uint32_t)r[3] << 8) |
              r[4];
        if (high_capacity)
            *high_capacity = (ocr & 0x40000000) != 0;
        if (verbose) {
            print_sd_r1("CMD58", r[0]);
            puts_("OCR=");
            puthex32(ocr);
            puts_((ocr & 0x40000000) ? " CCS=1\n" : " CCS=0\n");
        }
    } else {
        if (verbose)
            puts_("CMD58: no response\n");
    }

    if (!ok) {
        if (verbose)
            puts_("SPI timeout\n");
        return -1;
    }

    return initialized ? 0 : -1;
}

static void sd_probe(void)
{
    int ok = 1;
    uint8_t csd[16];

    if (sd_init_card(1, 0) == 0) {
        sd_spi_init(12); /* about 12.8 MHz */
        if (sd_read_register(9, csd, &ok) == 0)
            print_sd_csd_capacity(csd);
        else
            puts_("CMD9/CSD: failed\n");
    }

    if (!ok)
        puts_("SPI timeout\n");
}

static void sd_dump_sector(uint64_t sector)
{
    int ok = 1;
    int high_capacity = 0;
    uint32_t arg;
    int rc;

    if (sd_init_card(0, &high_capacity) < 0) {
        puts_("SD init failed; run S for details\n");
        return;
    }

    if (sd_sector_arg(sector, high_capacity, &arg) < 0) {
        puts_("sector too large for card addressing mode\n");
        return;
    }

    sd_spi_init(12); /* about 12.8 MHz */
    rc = sd_read_data(17, arg, sd_sector_buf, sizeof(sd_sector_buf), &ok);
    if (rc != 0 || !ok) {
        puts_("CMD17/read failed");
        if (!ok)
            puts_(" (SPI timeout)");
        putc_('\n');
        return;
    }

    puts_("sector=");
    puthex64(sector);
    puts_(" arg=");
    puthex32(arg);
    puts_(high_capacity ? " SDHC/SDXC\n" : " SDSC\n");
    hexdump_bytes(sd_sector_buf, sector << 9, sizeof(sd_sector_buf));
}

static void sd_load_sectors(uint64_t sector, uint64_t count, uint64_t addr)
{
    int ok = 1;
    int high_capacity = 0;
    volatile uint8_t *dst = (volatile uint8_t *)addr;

    if (count == 0) {
        puts_("count must be nonzero\n");
        return;
    }

    if (sd_init_card(0, &high_capacity) < 0) {
        puts_("SD init failed; run S for details\n");
        return;
    }

    sd_spi_init(12); /* about 12.8 MHz */
    for (uint64_t i = 0; i < count; i++) {
        uint32_t arg;
        int rc;

        if (sd_sector_arg(sector + i, high_capacity, &arg) < 0) {
            puts_("sector too large for card addressing mode\n");
            return;
        }

        rc = sd_read_data(17, arg, dst + (i << 9), 512, &ok);
        if (rc != 0 || !ok) {
            puts_("CMD17/read failed at sector=");
            puthex64(sector + i);
            if (!ok)
                puts_(" (SPI timeout)");
            putc_('\n');
            return;
        }
    }

    puts_("loaded sectors=");
    puthex64(sector);
    puts_(" count=");
    puthex64(count);
    puts_(" addr=");
    puthex64(addr);
    puts_(" bytes=");
    puthex64(count << 9);
    putc_('\n');
}

typedef void (*fn_t)(void);
typedef void (*fn_t2)(uint64_t, uint64_t);

int main(void)
{
    char buf[LINE_MAX];

    uart_init(UART0_BASE, CLK_FREQ, UART_SPEED);
    // Flush any spurious chars received during init or terminal connect
    while (UART0_BASE[UART_LSR] & LSR_DR)
        (void)UART0_BASE[UART_RBR];
    puts_("\nsmolrv64 monitor  rtl=");
    puthex64(read_build_stamp());
    puts_(" fw=");
    puthex64(MONITOR_BUILD_STAMP);
    putc_('\n');

    for (;;) {
        uint64_t addr, val;
        const char *p;

        puts_("> ");
        readline(buf);
        p = buf;

        if (*p == 'R' || *p == 'r') {
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: R<addr>\n"); continue; }
            val = *(volatile uint64_t *)addr;
            puthex64(addr); puts_(": "); puthex64(val); putc_('\n');

        } else if ((*p == 'W' || *p == 'w') &&
                   (p[1] == 'B' || p[1] == 'b')) {
            p = parse_hex(p + 2, &addr);
            if (!p) { puts_("usage: WB<addr> <val>\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &val);
            if (!p) { puts_("usage: WB<addr> <val>\n"); continue; }
            *(volatile uint8_t *)addr = (uint8_t)val;
            puts_("ok\n");

        } else if ((*p == 'W' || *p == 'w') &&
                   (p[1] == 'H' || p[1] == 'h')) {
            p = parse_hex(p + 2, &addr);
            if (!p) { puts_("usage: WH<addr> <val>\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &val);
            if (!p) { puts_("usage: WH<addr> <val>\n"); continue; }
            *(volatile uint16_t *)addr = (uint16_t)val;
            puts_("ok\n");

        } else if ((*p == 'W' || *p == 'w') &&
                   (p[1] == 'W' || p[1] == 'w')) {
            p = parse_hex(p + 2, &addr);
            if (!p) { puts_("usage: WW<addr> <val>\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &val);
            if (!p) { puts_("usage: WW<addr> <val>\n"); continue; }
            *(volatile uint32_t *)addr = (uint32_t)val;
            puts_("ok\n");

        } else if (*p == 'W' || *p == 'w') {
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: W<addr> <val>\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &val);
            if (!p) { puts_("usage: W<addr> <val>\n"); continue; }
            *(volatile uint64_t *)addr = val;
            puts_("ok\n");

        } else if (*p == 'T' || *p == 't') {
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: T<addr>\n"); continue; }
            hexdump(addr, 256);

        } else if (*p == 'Y' || *p == 'y') {
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: Y<addr>\n"); continue; }
            puts_("start XMODEM-1K send now\n");
            {
                uint64_t n = xmodem1k_recv(addr);
                if (n == (uint64_t)-1)
                    puts_("cancelled\n");
                else { puthex64(n); puts_(" bytes loaded\n"); }
            }

        } else if (*p == 'C' || *p == 'c') {
            uint64_t len;
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: C<addr> <len>\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &len);
            if (!p) { puts_("usage: C<addr> <len>\n"); continue; }
            {
                uint8_t hash[32];
                int i;
                blake3_hash((const void *)addr, len, hash);
                puts_("blake3: ");
                for (i = 0; i < 32; i++) puthex8(hash[i]);
                putc_('\n');
            }

        } else if (*p == 'Z' || *p == 'z') {
            uint64_t len, fill = 0;
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: Z<addr> <len> [byte]\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &len);
            if (!p) { puts_("usage: Z<addr> <len> [byte]\n"); continue; }
            while (*p == ' ') p++;
            if (*p) {
                const char *q = parse_hex(p, &fill);
                if (!q) { puts_("usage: Z<addr> <len> [byte]\n"); continue; }
            }
            {
                volatile uint8_t *d = (volatile uint8_t *)addr;
                uint8_t b = (uint8_t)fill;
                uint64_t i;
                for (i = 0; i < len; i++) d[i] = b;
            }
            puts_("ok\n");

        } else if (*p == 'S' || *p == 's') {
            uint64_t sector, count, load_addr;
            if (p[1] == 'L' || p[1] == 'l') {
                p += 2;
                while (*p == ' ') p++;
                p = parse_hex(p, &sector);
                if (!p) { puts_("usage: SL<sector> <count> <addr>\n"); continue; }
                while (*p == ' ') p++;
                p = parse_hex(p, &count);
                if (!p) { puts_("usage: SL<sector> <count> <addr>\n"); continue; }
                while (*p == ' ') p++;
                p = parse_hex(p, &load_addr);
                if (!p) { puts_("usage: SL<sector> <count> <addr>\n"); continue; }
                sd_load_sectors(sector, count, load_addr);
                continue;
            }

            p++;
            while (*p == ' ') p++;
            if (*p) {
                if (!parse_hex(p, &sector)) { puts_("usage: S [sector]\n"); continue; }
                sd_dump_sector(sector);
            } else {
                sd_probe();
            }

        } else if (*p == 'X' || *p == 'x') {
            uint64_t a0 = 0, a1 = 0;
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: X<addr> [a0 [a1]]\n"); continue; }
            if (*p == ' ') { const char *q = parse_hex(p + 1, &a0); if (q) p = q; }
            if (*p == ' ') { const char *q = parse_hex(p + 1, &a1); if (q) p = q; }
            puts_("jumping...\n");
            sync_cache_for_exec();
            ((fn_t2)addr)(a0, a1);
            puts_("returned\n");

        } else if (*p == 'P' || *p == 'p') {
            uint64_t mn, mx, tot, cnt, to, to_pc, to_tv, to_st, to_ca, to_ad;
            uint64_t build_stamp = read_build_stamp();
            uint64_t mcause, mtval, mepc, scause, stval, sepc;
            asm volatile ("csrr %0, 0xfc0" : "=r"(mn));
            asm volatile ("csrr %0, 0xfc1" : "=r"(mx));
            asm volatile ("csrr %0, 0xfc2" : "=r"(tot));
            asm volatile ("csrr %0, 0xfc3" : "=r"(cnt));
            asm volatile ("csrr %0, 0xfc4" : "=r"(to));
            asm volatile ("csrr %0, 0xfc5" : "=r"(to_pc));
            asm volatile ("csrr %0, 0xfc6" : "=r"(to_tv));
            asm volatile ("csrr %0, 0xfc7" : "=r"(to_st));
            asm volatile ("csrr %0, 0xfc8" : "=r"(to_ca));
            asm volatile ("csrr %0, 0xfc9" : "=r"(to_ad));
            asm volatile ("csrr %0, mcause" : "=r"(mcause));
            asm volatile ("csrr %0, mtval"  : "=r"(mtval));
            asm volatile ("csrr %0, mepc"   : "=r"(mepc));
            asm volatile ("csrr %0, scause" : "=r"(scause));
            asm volatile ("csrr %0, stval"  : "=r"(stval));
            asm volatile ("csrr %0, sepc"   : "=r"(sepc));
            if (p[1] == 'c' || p[1] == 'C') {
                asm volatile ("csrw 0xfc3, zero");
                puts_("cleared\n");
            } else {
                puts_("build stamp="); puthex64(build_stamp);
                putc_('\n');
                puts_("mig min="); puthex64(mn);
                puts_(" max=");    puthex64(mx);
                puts_(" total=");  puthex64(tot);
                puts_(" count=");  puthex64(cnt);
                puts_(" timeouts="); puthex64(to);
                if (cnt) {
                    puts_(" avg=");
                    puthex64(tot / cnt);
                }
                putc_('\n');
                puts_("scause="); puthex64(scause);
                puts_(" stval="); puthex64(stval);
                puts_(" sepc=");  puthex64(sepc);
                putc_('\n');
                puts_("mcause="); puthex64(mcause);
                puts_(" mtval="); puthex64(mtval);
                puts_(" mepc=");  puthex64(mepc);
                putc_('\n');
                if (to) {
                    puts_("first-to pc="); puthex64(to_pc);
                    puts_(" tval=");  puthex64(to_tv);
                    puts_(" state="); puthex64(to_st);
                    puts_(" cause="); puthex64(to_ca);
                    puts_(" addr=");  puthex64(to_ad);
                    putc_('\n');
                }
            }

        } else if (*p == '?' || *p == 'h' || *p == 'H') {
            puts_("R<addr>          read 64-bit word\n");
            puts_("W<addr> <val>    write 64-bit word\n");
            puts_("WW<addr> <val>   write 32-bit word\n");
            puts_("WH<addr> <val>   write 16-bit half-word\n");
            puts_("WB<addr> <val>   write 8-bit byte\n");
            puts_("T<addr>          hexdump 256 bytes\n");
            puts_("Y<addr>          receive XMODEM-1K upload (sx -k <file>)\n");
            puts_("C<addr> <len>    blake3-256 of len bytes at address\n");
            puts_("Z<addr> <len> [b] fill len bytes with byte b (default 0)\n");
            puts_("S [sector]       probe SD card, or dump 512-byte sector\n");
            puts_("SL<sec> <n> <addr> read n SD sectors into memory\n");
            puts_("X<addr> [a0 [a1]] execute from address\n");
            puts_("P                dump core debug counters; Pc clears them\n");

        } else if (*p != 0) {
            puts_("unknown command (? for help)\n");
        }
    }
}
