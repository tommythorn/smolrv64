// Trivial

typedef unsigned char uint8_t;

// NS16550A UART at 0x10000000
#define UART0_BASE  ((volatile uint8_t *)0x10000000)
#define CLK_FREQ    333333333
#define UART_SPEED  3000000

// Register offsets
#define UART_THR  0   // Transmit Holding Register (write, DLAB=0)
#define UART_RBR  0   // Receive Buffer Register   (read,  DLAB=0)
#define UART_DLL  0   // Divisor Latch LSB          (DLAB=1)
#define UART_IER  1   // Interrupt Enable Register  (DLAB=0)
#define UART_DLH  1   // Divisor Latch MSB          (DLAB=1)
#define UART_FCR  2   // FIFO Control Register      (write)
#define UART_LCR  3   // Line Control Register
#define UART_LSR  5   // Line Status Register

#define LCR_DLAB  0x80
#define LCR_8N1   0x03
#define LSR_THRE  0x20  // Transmit Holding Register Empty
#define LSR_DR    0x01  // Data Ready

static void uart_init(volatile uint8_t *base, int clk_freq, int baud) {
    int div = clk_freq / (16 * baud);
    if (div < 1) div = 1;
    base[UART_LCR] = LCR_DLAB;
    base[UART_DLL] = div & 0xFF;
    base[UART_DLH] = (div >> 8) & 0xFF;
    base[UART_LCR] = LCR_8N1;
    base[UART_FCR] = 0x07;   // enable + clear FIFOs
    base[UART_IER] = 0x00;   // no interrupts
}

static uint8_t uart_receive_blocking(volatile uint8_t *base) {
    while (!(base[UART_LSR] & LSR_DR));
    return base[UART_RBR];
}

static void uart_send_blocking(volatile uint8_t *base, uint8_t c) {
    while (!(base[UART_LSR] & LSR_THRE));
    base[UART_THR] = c;
}

static void myputc(char c) {
    if (c == '\n')
        uart_send_blocking(UART0_BASE, '\r');
    uart_send_blocking(UART0_BASE, c);
}

static void myputn(long n);
static void myputn(long n) {
    if (n < 0) {
        myputc('-');
        n = -n;
    }
    
    if (n >= 10)
        myputn(n / 10);
    myputc('0' + n % 10);
}

static void myputs(char *s) {
    while (*s) {
        myputc(*s++);
    }
}

static long numbers[999999+1];

int main(int argc, char **argv) {

    long cycle0, instret0, cycle, instret;

    asm("csrr  %0, mcycle" : "=r" (cycle0));
    asm("csrr  %0, minstret" : "=r" (instret0));

    numbers[0] = numbers[1] = 1;

    for (int i = 2; i < 1000000; ++i) {
        myputn(i);
        myputc(':');
        numbers[i] = numbers[i-1] + numbers[i-2];
        myputn(numbers[i]);
        myputc(' ');
    }

    asm("csrr  %0, mcycle" : "=r" (cycle));
    asm("csrr  %0, minstret" : "=r" (instret));

    cycle -= cycle0;
    instret -= instret0;

    myputs("\ncycle   = "); myputn(cycle);
    myputs("\ninstret = "); myputn(instret);
    myputs("\nCPI     = "); myputn(cycle / instret);
    myputs("\n");
}
