// Sieve of Eratosthenes -- Tommy Thorn 20231025
//
// Sieve of Eratosthenes is a classic method for generating prime
// numbers.  We start with a set of candidates (here odd numbers
// starting with 3 up to NN).  The invariant is that the smallest
// remaining candidate is a prime.  We remove that and remove all
// integer multiple of it from the set.  Repeat until the set is
// empty.
//
// As the largest possible prime factor of a composite number is the
// square root of the number, we can stop weeding out numbers when we
// reach that.  All remaining numbers are primes.

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

void uart_init(volatile uint8_t *base, int clk_freq, int baud) {
    int div = clk_freq / (16 * baud);
    if (div < 1) div = 1;
    base[UART_LCR] = LCR_DLAB;
    base[UART_DLL] = div & 0xFF;
    base[UART_DLH] = (div >> 8) & 0xFF;
    base[UART_LCR] = LCR_8N1;
    base[UART_FCR] = 0x07;   // enable + clear FIFOs
    base[UART_IER] = 0x00;   // no interrupts
}

uint8_t uart_receive_blocking(volatile uint8_t *base) {
    while (!(base[UART_LSR] & LSR_DR));
    return base[UART_RBR];
}

void uart_send_blocking(volatile uint8_t *base, uint8_t c) {
    while (!(base[UART_LSR] & LSR_THRE));
    base[UART_THR] = c;
}


#define N 80

static char primes[N*N+1+8];

static void myputc(char c) {
  if (c == '\n')
    uart_send_blocking(UART0_BASE, '\r');
  uart_send_blocking(UART0_BASE, c);
}

void myputs(char *s);
void myputn(unsigned n);

int main(int argc, char **argv) {
    int NN, p;
    char *primes_end, *cp, *pi;
    long *lp;

    uart_init(UART0_BASE, CLK_FREQ, UART_SPEED);

    myputs("\nPrimes:\n");

    NN = N*N;

    // 3 5 7 9 11 13 .. i*2+3
    // primes[NN] is the sential; this enables us to scan without
    // having to test against the array limit
    // XXX convert to bitvector for greater reach
    primes_end = primes + NN;

    for (;;) {
start_over:
        for (lp = (long *) primes;
             lp < (long *) primes + NN / 8 + 1;
             lp++)
            *lp = ~0ull;

        primes[NN] = 0;

        myputs("2\n");

        pi = primes;
        for (;;) {
            p = 3 + 2*(pi - primes);
            myputn(p);
            myputc('\n');
            cp = pi + p*(p/2);

            while (!*++pi) {
                if (pi > primes_end) {
                    goto start_over;
                }
            }

            while (cp < primes_end) {
                *cp = 0; // Sieve out multiples of p
                cp = cp + p;
            }

        }
    }
}

void myputs(char *s) {
  while (*s) {
    myputc(*s++);
  }
}

void myputn(unsigned n) {
  if (n >= 10)
    myputn(n / 10);
  myputc('0' + n % 10);
}
