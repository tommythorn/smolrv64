// Simple memory monitor for smolrv64
// Commands:
//   R<addr>          - read and display 64-bit word at address
//   W<addr> <val>    - write 64-bit word to address
//   WW<addr> <val>   - write 32-bit word to address
//   WH<addr> <val>   - write 16-bit half-word to address
//   WB<addr> <val>   - write 8-bit byte to address
//   T<addr>          - hexdump 256 bytes starting at address
//   L<addr>          - load base64-encoded binary to address (end with empty line)
//   Y<addr>          - receive XMODEM-1K upload to address
//   C<addr> <len>    - blake3-256 of len bytes at address
//   Z<addr> <len> [b] - fill len bytes at address with byte b (default 0)
//   X<addr> [a0 [a1]] - jump to address and execute
//   ?                - help

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

#include "blake3.h"

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

static void puthex8(uint8_t v)
{
    putc_("0123456789abcdef"[v >> 4]);
    putc_("0123456789abcdef"[v & 0xf]);
}

static void puthex16(uint16_t v)
{
    puthex8(v >> 8);
    puthex8(v & 0xff);
}

static void puthex32(uint32_t v)
{
    puthex16(v >> 16);
    puthex16(v & 0xffff);
}

static void puthex64(uint64_t v)
{
    puthex32(v >> 32);
    puthex32(v & 0xffffffff);
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

static void readline(char *buf)
{
    int n = 0;
    for (;;) {
        char c = uart_getc(UART0_BASE);
        if (c == '\r' || c == '\n') {
            putc_('\n');
            buf[n] = 0;
            return;
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

// Base64 decode table: -1=invalid, -2=padding
static int b64val(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    if (c == '=') return -2;
    return -1;
}

// Load base64-encoded data to addr.
// Reads lines until an empty line is received.
// Returns number of bytes written.
static uint64_t load_base64(uint64_t addr)
{
    char buf[LINE_MAX];
    uint64_t total = 0;
    volatile uint8_t *dst = (volatile uint8_t *)addr;

    for (;;) {
        int i, v0, v1, v2, v3;
        const char *p;

        readline(buf);
        if (buf[0] == 0)
            break;  // empty line = end of data

        p = buf;
        while (*p) {
            // skip whitespace
            while (*p == ' ' || *p == '\t') p++;
            if (!*p) break;

            v0 = b64val(*p++);
            v1 = *p ? b64val(*p++) : -1;
            v2 = *p ? b64val(*p++) : -1;
            v3 = *p ? b64val(*p++) : -1;

            if (v0 < 0 || v1 < 0) break;  // bad input

            *dst++ = (v0 << 2) | (v1 >> 4);
            total++;

            if (v2 == -2 || v2 < 0) break;  // padding or end
            *dst++ = ((v1 & 0xf) << 4) | (v2 >> 2);
            total++;

            if (v3 == -2 || v3 < 0) break;
            *dst++ = ((v2 & 0x3) << 6) | v3;
            total++;
        }
    }
    return total;
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

typedef void (*fn_t)(void);
typedef void (*fn_t2)(uint64_t, uint64_t);

int main(void)
{
    char buf[LINE_MAX];

    uart_init(UART0_BASE, CLK_FREQ, UART_SPEED);
    // Flush any spurious chars received during init or terminal connect
    while (UART0_BASE[UART_LSR] & LSR_DR)
        (void)UART0_BASE[UART_RBR];
    puts_("\nsmolrv64 monitor\n");

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

        } else if (*p == 'L' || *p == 'l') {
            uint64_t n;
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: L<addr> (then base64 lines, empty to end)\n"); continue; }
            puts_("send base64, empty line to finish:\n");
            n = load_base64(addr);
            puthex64(n); puts_(" bytes loaded\n");

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

        } else if (*p == 'X' || *p == 'x') {
            uint64_t a0 = 0, a1 = 0;
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: X<addr> [a0 [a1]]\n"); continue; }
            if (*p == ' ') { const char *q = parse_hex(p + 1, &a0); if (q) p = q; }
            if (*p == ' ') { const char *q = parse_hex(p + 1, &a1); if (q) p = q; }
            puts_("jumping...\n");
            ((fn_t2)addr)(a0, a1);
            puts_("returned\n");

        } else if (*p == 'P' || *p == 'p') {
            uint64_t mn, mx, tot, cnt, to, to_pc, to_tv, to_st, to_ca, to_ad;
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
            if (p[1] == 'c' || p[1] == 'C') {
                asm volatile ("csrw 0xfc3, zero");
                puts_("cleared\n");
            } else {
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
            puts_("L<addr>          load base64 blob (empty line ends)\n");
            puts_("Y<addr>          receive XMODEM-1K upload (sx -k <file>)\n");
            puts_("C<addr> <len>    blake3-256 of len bytes at address\n");
            puts_("Z<addr> <len> [b] fill len bytes with byte b (default 0)\n");
            puts_("X<addr> [a0 [a1]] execute from address\n");
            puts_("P                dump MIG latency stats; Pc clears them\n");

        } else if (*p != 0) {
            puts_("unknown command (? for help)\n");
        }
    }
}
