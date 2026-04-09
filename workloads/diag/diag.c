// Minimal UART echo diagnostic for smolrv64
//
// No uart_init — UART works from reset (DLAB=0, TX always ready in model).
// Waits for first keystroke (so terminal can connect after programming),
// then echoes every received byte as [XX] hex.  CR also sends \r\n.
// This lets us verify RX and TX independently of any command logic.

typedef unsigned char  uint8_t;
typedef unsigned long  uint64_t;

#define UART   ((volatile uint8_t *)0x10000000)
#define LSR_DR    0x01   // Data Ready (bit 0)
#define LSR_THRE  0x20   // TX Holding Register Empty (bit 5)

static uint8_t getc_(void)
{
    while (!(UART[5] & LSR_DR));
    return UART[0];
}

static void putc_(uint8_t c)
{
    while (!(UART[5] & LSR_THRE));
    UART[0] = c;
}

static void puthex8(uint8_t v)
{
    uint8_t hi = (v >> 4) & 0xf;
    uint8_t lo =  v       & 0xf;
    putc_(hi < 10 ? '0' + hi : 'a' + hi - 10);
    putc_(lo < 10 ? '0' + lo : 'a' + lo - 10);
}

int main(void)
{
    for (;;) {
        uint8_t c = getc_();
        putc_('[');
        puthex8(c);
        putc_(']');
        if (c == '\r') {
            putc_('\r');
            putc_('\n');
        } else if (c == '@') {
            char *s = "Hello World here is a longer string to test things out a bit. ";
            while (*s)
                putc_(*s++);
        }
    }
}
