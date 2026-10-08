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
//   L<addr> <path>   - load a file from the SD card's EFI System Partition
//   D [path]         - list a directory there
//   B                - boot from the SD card now (what the autoboot does)
//   X<addr> [a0 [a1]] - jump to address and execute
//   P                - dump core debug counters; Pc clears them
//   ?                - help
//
// AUTOBOOT: after the banner the monitor counts down AUTOBOOT_S seconds, then runs
// /smolrv64/boot.txt from the SD card's EFI System Partition: monitor command lines, one per
// line, '#' starts a comment. Without boot.txt it loads /smolrv64/fw_payload.bin at
// 0x80000000 and /smolrv64/smolrv64.dtb at 0xffdff000 and jumps to the payload with the DTB.
// key[0] held at reset or pressed in the countdown, or any key on the console, stops it at
// the prompt; a key typed then starts the command line.

#include "mon.h"
#include "blake3.h"

#ifndef AUTOBOOT_S
#define AUTOBOOT_S 10
#endif
#ifndef BOOT_DIR
#define BOOT_DIR "/smolrv64"
#endif

// Firmware build stamp (YYYYMMDDHHMMSS as hex digits, like the RTL stamp).
// Injected by the Makefile; 0 when built without it.
#ifndef MONITOR_BUILD_STAMP
#define MONITOR_BUILD_STAMP 0
#endif

// NS16550A UART at 0x10000000
#define UART0_BASE  ((volatile uint8_t *)0x10000000)
#define CLK_FREQ    166666667       // the line is 3 Mbps in hardware; the divisor is not used
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

// The board's keys (rk_xcku5p.v @ 0x10006000): [3:0] held now, [11:8] pressed since the
// CPU's reset. key[0] is the monitor's.
#define KEYS_BASE       ((volatile uint32_t *)0x10006000)
#define KEY_MONITOR     0x101u

// Platform build-id block (rk_xcku5p.v @ 0x1000F000). Same registers Linux userland
// can read via /dev/mem. Layout: [0]="SMOL" magic, [1]=version, [2:3]=RTL build stamp,
// [4]=git commit (truncated HEAD), [5]=source-dirty flag.
#define BUILD_ID_BASE   ((volatile uint32_t *)0x1000F000)
#define BUILD_ID_MAGIC  0x534d4f4cu

// Integrity log (rv_soc_top.v, the window the fetch-buffer diagnostic used to own,
// 0x1000E000): the design's own invariants, latched in hardware by rv_errlog. 64-bit words:
// [0]={version,"ERRL"} magic, [1]=sticky, one bit per invariant that has fired since reset
// (D$ [15:0], I$ [31:16], LSU [47:32]; the bit numbers are the ones in rv_cache.v's and
// smolrv64_lsu.v's INTEGRITY LOG blocks), [2]={idx[55:48], cycle[47:0]} of the FIRST to fire.
// Linux userland reads the same words through /dev/mem (tools/errlog-read.sh).
#define ERRLOG_BASE     ((volatile uint64_t *)0x1000E000)
#define ERRLOG_MAGIC    0x4552524cu

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

void putc_(char c)
{
    if (c == '\n')
        uart_putc(UART0_BASE, '\r');
    uart_putc(UART0_BASE, c);
    video_putc(c);
}

void puts_(const char *s)
{
    while (*s)
        putc_(*s++);
}

static void puthex4(uint32_t v)
{
    v &= 0xf;
    putc_(v < 10 ? '0' + v : 'a' + (v - 10));
}

void puthex8(uint32_t v)
{
    v &= 0xff;
    puthex4(v >> 4);
    puthex4(v);
}

void puthex32(uint32_t v)
{
    puthex8(v >> 24);
    puthex8(v >> 16);
    puthex8(v >> 8);
    puthex8(v);
}

void puthex64(uint64_t v)
{
    puthex32((uint32_t)(v >> 32));
    puthex32((uint32_t)v);
}

void putdec(uint64_t v)
{
    char b[20];
    int n = 0;
    do { b[n++] = '0' + v % 10; v /= 10; } while (v);
    while (n) putc_(b[--n]);
}

static uint64_t read_build_stamp(void)
{
    // Prefer the platform build-id MMIO block: it works on both cores, whereas the
    // probe core (which runs this monitor on the FPGA) doesn't implement the 0xfde
    // stamp CSR -- reading it there yields zeros. Fall back to the CSR (scalar/sim).
    if (BUILD_ID_BASE[0] == BUILD_ID_MAGIC)
        return ((uint64_t)BUILD_ID_BASE[3] << 32) | BUILD_ID_BASE[2];
    uint64_t build_stamp;
    asm volatile ("csrr %0, 0xfde" : "=r"(build_stamp));
    return build_stamp;
}

static void sync_cache_for_exec(void)
{
    asm volatile ("fence.i" ::: "memory");
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

static void readline(char *buf, char c0)
{
    int n = 0;
    int browse = 0;   // 0 = fresh line, 1..hist_count = how far back in history
    for (;;) {
        char c = c0 ? c0 : uart_getc(UART0_BASE);
        c0 = 0;
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

/* Non-blocking UART read with a timeout in mtime ticks.
 * Returns 1 and stores byte in *out if a byte arrived; 0 on timeout. */
static int uart_getc_tmo(uint64_t tmo, uint8_t *out)
{
    uint64_t t0 = now();
    while (now() - t0 < tmo) {
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
        if (uart_getc_tmo(TIMEBASE_HZ / 5, &c))  /* 200 ms per poll */
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
            if (uart_getc_tmo(TIMEBASE_HZ / 10, &c) && c == XM_CAN) {
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
        /* Wait up to 10 s for next byte; timeout = stalled transfer */
        if (!uart_getc_tmo(TIMEBASE_HZ * 10, &c)) {
            uart_putc(UART0_BASE, XM_CAN);
            uart_putc(UART0_BASE, XM_CAN);
            return (uint64_t)-1;
        }
    }
}

typedef void (*fn_t)(void);
typedef void (*fn_t2)(uint64_t, uint64_t);

static int run_cmd(const char *p);

// The autoboot: /smolrv64/boot.txt's lines, each a monitor command, or the default. A failing
// line stops it at the prompt.
#define BOOT_TXT    (DMA_BASE + 0x10000)     // DDR scratch, above disk.c's queue
#define BOOT_TXT_MAX 4096
static const char *const boot_default[] = {
    "L80000000 /smolrv64/fw_payload.bin",
    "Lffdff000 /smolrv64/smolrv64.dtb",
    "X80000000 0 ffdff000",
    0
};

static int autoboot(void)
{
    static char line[LINE_MAX];
    uint64_t n;
    int r = disk_load(BOOT_DIR "/boot.txt", BOOT_TXT, BOOT_TXT_MAX - 1, &n);
    if (r == -1) { puts_("the SD card did not answer: autoboot stopped\n"); return -1; }
    if (r == 0) {
        const char *t = (const char *)BOOT_TXT;
        uint64_t i = 0;
        while (i < n) {
            int k = 0;
            while (i < n && t[i] != '\n') {
                if (t[i] != '\r' && k < LINE_MAX - 1) line[k++] = t[i];
                i++;
            }
            i++;
            line[k] = 0;
            const char *q = line;
            while (*q == ' ' || *q == '\t') q++;
            if (!*q || *q == '#') continue;
            puts_("boot> "); puts_(q); putc_('\n');
            if (run_cmd(q)) { puts_("autoboot stopped\n"); return -1; }
        }
        return 0;
    }
    puts_("no " BOOT_DIR "/boot.txt: the default\n");
    for (int k = 0; boot_default[k]; k++) {
        puts_("boot> "); puts_(boot_default[k]); putc_('\n');
        if (run_cmd(boot_default[k])) { puts_("autoboot stopped\n"); return -1; }
    }
    return 0;
}

// The countdown: key[0], held or pressed since reset, or a console key stops it. Returns the
// key typed (it starts the command line), or 0.
static char countdown(void)
{
    if (KEYS_BASE[0] & KEY_MONITOR) { puts_("key[0]: staying in the monitor\n"); return 0; }
    puts_("autoboot from the SD card in ");
    putdec(AUTOBOOT_S);
    puts_(" s (key[0] or any key: monitor) ");
    uint64_t t0 = now(), left = AUTOBOOT_S + 1;
    for (;;) {
        uint64_t el = (now() - t0) / TIMEBASE_HZ;
        if (el >= AUTOBOOT_S) break;
        if (AUTOBOOT_S - el != left) { left = AUTOBOOT_S - el; putdec(left); putc_(' '); }
        if (UART0_BASE[UART_LSR] & LSR_DR) { putc_('\n'); return UART0_BASE[UART_RBR]; }
        if (KEYS_BASE[0] & KEY_MONITOR) { puts_("\nkey[0]: staying in the monitor\n"); return 0; }
    }
    putc_('\n');
    autoboot();
    return 0;
}

static int run_cmd(const char *p)
{
    uint64_t addr, val;

    if (*p == 'R' || *p == 'r') {
        p = parse_hex(p + 1, &addr);
        if (!p) { puts_("usage: R<addr>\n"); return -1; }
        val = *(volatile uint64_t *)addr;
        puthex64(addr); puts_(": "); puthex64(val); putc_('\n');

    } else if ((*p == 'W' || *p == 'w') &&
               (p[1] == 'B' || p[1] == 'b')) {
        p = parse_hex(p + 2, &addr);
        if (!p) { puts_("usage: WB<addr> <val>\n"); return -1; }
        while (*p == ' ') p++;
        p = parse_hex(p, &val);
        if (!p) { puts_("usage: WB<addr> <val>\n"); return -1; }
        *(volatile uint8_t *)addr = (uint8_t)val;
        puts_("ok\n");

    } else if ((*p == 'W' || *p == 'w') &&
               (p[1] == 'H' || p[1] == 'h')) {
        p = parse_hex(p + 2, &addr);
        if (!p) { puts_("usage: WH<addr> <val>\n"); return -1; }
        while (*p == ' ') p++;
        p = parse_hex(p, &val);
        if (!p) { puts_("usage: WH<addr> <val>\n"); return -1; }
        *(volatile uint16_t *)addr = (uint16_t)val;
        puts_("ok\n");

    } else if ((*p == 'W' || *p == 'w') &&
               (p[1] == 'W' || p[1] == 'w')) {
        p = parse_hex(p + 2, &addr);
        if (!p) { puts_("usage: WW<addr> <val>\n"); return -1; }
        while (*p == ' ') p++;
        p = parse_hex(p, &val);
        if (!p) { puts_("usage: WW<addr> <val>\n"); return -1; }
        *(volatile uint32_t *)addr = (uint32_t)val;
        puts_("ok\n");

    } else if (*p == 'W' || *p == 'w') {
        p = parse_hex(p + 1, &addr);
        if (!p) { puts_("usage: W<addr> <val>\n"); return -1; }
        while (*p == ' ') p++;
        p = parse_hex(p, &val);
        if (!p) { puts_("usage: W<addr> <val>\n"); return -1; }
        *(volatile uint64_t *)addr = val;
        puts_("ok\n");

    } else if (*p == 'T' || *p == 't') {
        p = parse_hex(p + 1, &addr);
        if (!p) { puts_("usage: T<addr>\n"); return -1; }
        hexdump(addr, 256);

    } else if (*p == 'Y' || *p == 'y') {
        p = parse_hex(p + 1, &addr);
        if (!p) { puts_("usage: Y<addr>\n"); return -1; }
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
        if (!p) { puts_("usage: C<addr> <len>\n"); return -1; }
        while (*p == ' ') p++;
        p = parse_hex(p, &len);
        if (!p) { puts_("usage: C<addr> <len>\n"); return -1; }
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
        if (!p) { puts_("usage: Z<addr> <len> [byte]\n"); return -1; }
        while (*p == ' ') p++;
        p = parse_hex(p, &len);
        if (!p) { puts_("usage: Z<addr> <len> [byte]\n"); return -1; }
        while (*p == ' ') p++;
        if (*p) {
            const char *q = parse_hex(p, &fill);
            if (!q) { puts_("usage: Z<addr> <len> [byte]\n"); return -1; }
        }
        {
            volatile uint8_t *d = (volatile uint8_t *)addr;
            uint8_t b = (uint8_t)fill;
            uint64_t i;
            for (i = 0; i < len; i++) d[i] = b;
        }
        puts_("ok\n");

    } else if (*p == 'X' || *p == 'x') {
        uint64_t a0 = 0, a1 = 0;
        p = parse_hex(p + 1, &addr);
        if (!p) { puts_("usage: X<addr> [a0 [a1]]\n"); return -1; }
        if (*p == ' ') { const char *q = parse_hex(p + 1, &a0); if (q) p = q; }
        if (*p == ' ') { const char *q = parse_hex(p + 1, &a1); if (q) p = q; }
        puts_("jumping...\n");
        disk_quiesce();
        sync_cache_for_exec();
        ((fn_t2)addr)(a0, a1);
        puts_("returned\n");

    } else if (*p == 'L' || *p == 'l') {
        uint64_t n;
        p = parse_hex(p + 1, &addr);
        if (!p || *p != ' ') { puts_("usage: L<addr> <path>\n"); return -1; }
        while (*p == ' ') p++;
        if (disk_load(p, addr, ~0ul, &n)) return -1;

    } else if (*p == 'D' || *p == 'd') {
        p++;
        while (*p == ' ') p++;
        if (disk_list(p)) return -1;

    } else if (*p == 'B' || *p == 'b') {
        return autoboot();

    } else if (*p == 'E' || *p == 'e') {
        if ((uint32_t)ERRLOG_BASE[0] != ERRLOG_MAGIC) { puts_("no integrity log\n"); return -1; }
        uint64_t st = ERRLOG_BASE[1], fi = ERRLOG_BASE[2];
        puts_("errlog sticky="); puthex64(st);
        puts_("  (D$ [15:0]  I$ [31:16]  LSU [47:32])\n");
        if (st) {
            puts_("first: bit "); puthex64((fi >> 48) & 0xff);
            puts_(" at cycle "); puthex64(fi & 0xffffffffffffull);
            putc_('\n');
        }

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
        puts_("L<addr> <path>   load a file from the SD card's EFI System Partition\n");
        puts_("D [path]         list a directory there\n");
        puts_("B                boot from the SD card (/smolrv64/boot.txt) now\n");
        puts_("X<addr> [a0 [a1]] execute from address\n");
        puts_("P                dump core debug counters; Pc clears them\n");
        puts_("E                integrity log: which invariant fired, and when\n");

    } else if (*p != 0) {
        puts_("unknown command (? for help)\n");
        return -1;
    }
    return 0;
}

int main(void)
{
    static char buf[LINE_MAX];

    uart_init(UART0_BASE, CLK_FREQ, UART_SPEED);
    // Flush any spurious chars received during init or terminal connect
    while (UART0_BASE[UART_LSR] & LSR_DR)
        (void)UART0_BASE[UART_RBR];
    // rtl= identifies the loaded RTL by its git commit (read from the build-id MMIO block,
    // now probe-core-readable via soc_top). Falls back to the RTL build-stamp CSR on the
    // scalar core / sim, where the block isn't present (magic mismatch).
    puts_("\nsmolrv64 monitor  rtl=");
    if (BUILD_ID_BASE[0] == BUILD_ID_MAGIC) {
        puthex32(BUILD_ID_BASE[4]);   // git commit
        if (BUILD_ID_BASE[5] & 1)
            putc_('+');               // source tree was dirty at build time
    } else {
        puthex64(read_build_stamp()); // scalar/sim fallback: the 0xfde build-stamp CSR
    }
    puts_(" fw=");
    puthex64(MONITOR_BUILD_STAMP);
    // err= is the integrity log's sticky vector: 0 is the normal case; anything else is an
    // invariant that fired since reset, and 'E' says which one and when. A board that has
    // run for hours and comes back to this prompt carries its verdict in this one word.
    if ((uint32_t)ERRLOG_BASE[0] == ERRLOG_MAGIC) {
        puts_(" err=");
        puthex64(ERRLOG_BASE[1]);
    }
    putc_('\n');


    // The screen after the banner: drawing it takes a few hundred thousand cycles, and the
    // banner is what tells a console (and the netlist boot, rule F5) the core runs at all.
    video_init();
    char c0 = countdown();
    for (;;) {
        puts_("> ");
        readline(buf, c0);
        c0 = 0;
        run_cmd(buf);
    }
}
