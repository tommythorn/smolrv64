// Simple memory monitor for smolrv64
// Commands:
//   R<hex>          - read and display 64-bit word at address
//   W<hex> <hex>    - write 64-bit word to address
//   ?               - help

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

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

static void puthex64(uint64_t v)
{
    int i;
    for (i = 60; i >= 0; i -= 4) {
        int d = (v >> i) & 0xf;
        putc_(d < 10 ? '0' + d : 'a' + d - 10);
    }
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

#define LINE_MAX 80

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
            puthex64(addr);
            puts_(": ");
            puthex64(val);
            putc_('\n');
        } else if (*p == 'W' || *p == 'w') {
            p = parse_hex(p + 1, &addr);
            if (!p) { puts_("usage: W<addr> <val>\n"); continue; }
            while (*p == ' ') p++;
            p = parse_hex(p, &val);
            if (!p) { puts_("usage: W<addr> <val>\n"); continue; }
            *(volatile uint64_t *)addr = val;
            puts_("ok\n");
        } else if (*p == '?' || *p == 'h' || *p == 'H') {
            puts_("R<addr>        read 64-bit word at addr\n");
            puts_("W<addr> <val>  write 64-bit word to addr\n");
        } else if (*p != 0) {
            puts_("unknown command (? for help)\n");
        }
    }
}
