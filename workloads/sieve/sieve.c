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
//
// Missed:
//  #define
//  initialized locals (and globals?)
//  global arrays
//  for (..)

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <locale.h>

// We take the follows claims as self evident
typedef int int32_t;
typedef unsigned char uint8_t;

// XXX This is problematic as different platforms _will_ have
// different frequencies and we currently don't have access to the
// frequency from software.  For Right Now, we assume 25 MHz and
// 115,200 bps
#define BASE_FREQUENCY 25000000
//#define UART_SPEED       115200
//#define UART_SPEED       460800
//#define UART_SPEED       921600
#define UART_SPEED        1500000
//#define UART_SPEED      3000000
#define UART0_BASE     ((volatile int32_t *)0x10000000)

enum SiFive_UART_Register {TXDATA, RXDATA, TXCTRL, RXCTRL, IE, IP, DIV, FREQ};
// Register bit definitions
#define TXDATA_FULL_BIT  31
#define RXDATA_EMPTY_BIT 31
#define TXCTRL_EN_BIT     0
#define TXCTRL_NSTOP_BIT  1
#define RXCTRL_EN_BIT     0
#define IE_TXWM_BIT       0
#define IE_RXWM_BIT       1

void uart_init(volatile int32_t *base, int32_t uart_speed) {
  // Rounding: f/g + 1/2 = f/g + 1/2*g/g = (f + g/2) / g
  int div = (base[FREQ] + uart_speed/2) / uart_speed;
  if (div < 1)
      div = 1; // avoid 0
  base[DIV] = div-1;
  base[TXCTRL] = 1 << TXCTRL_EN_BIT;  // Enable TX
  base[RXCTRL] = 1 << RXCTRL_EN_BIT;  // Enable RX
}

uint8_t uart_receive_blocking(volatile int32_t *base) {
  int c;
  do c = base[RXDATA]; while (c < 0);
  return (uint8_t) c;
}

void uart_send_blocking(volatile int32_t *base, uint8_t c) {
  while (base[TXDATA]);
  base[TXDATA] = c;
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

    uart_init(UART0_BASE, UART_SPEED);

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
