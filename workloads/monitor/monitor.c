// Simple memory monitor for smolrv64
// Commands:
//   R<addr>          - read and display 64-bit word at address
//   W<addr> <val>    - write 64-bit word to address
//   WW<addr> <val>   - write 32-bit word to address
//   WH<addr> <val>   - write 16-bit half-word to address
//   WB<addr> <val>   - write 8-bit byte to address
//   T<addr>          - hexdump 256 bytes starting at address
//   L<addr>          - load base64-encoded binary to address (end with empty line)
//   X<addr>          - jump to address and execute
//   ?                - help

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

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

typedef void (*fn_t)(void);

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

        } else if (*p == 'X' || *p == 'x') {
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: X<addr>\n"); continue; }
            puts_("jumping...\n");
            ((fn_t)addr)();
            puts_("returned\n");

        } else if (*p == '?' || *p == 'h' || *p == 'H') {
            puts_("R<addr>          read 64-bit word\n");
            puts_("W<addr> <val>    write 64-bit word\n");
            puts_("WW<addr> <val>   write 32-bit word\n");
            puts_("WH<addr> <val>   write 16-bit half-word\n");
            puts_("WB<addr> <val>   write 8-bit byte\n");
            puts_("T<addr>          hexdump 256 bytes\n");
            puts_("L<addr>          load base64 blob (empty line ends)\n");
            puts_("X<addr>          execute from address\n");

        } else if (*p != 0) {
            puts_("unknown command (? for help)\n");
        }
    }
}
