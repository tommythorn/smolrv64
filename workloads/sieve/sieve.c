// Sieve of Eratosthenes -- Tommy Thorn 20231025
//
// Sieve of Eratosthenes is a classic method for generating prime
// numbers.  We start with a set of candidates (here odd numbers
// starting with 3 upto NN).  The invariant is that the smallest
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
#define UART_SPEED       115200

enum SiFive_UART_Register {TXDATA, RXDATA, TXCTRL, RXCTRL, IE, IP, DIV};
// Register bit definitions
#define TXDATA_FULL_BIT  31
#define RXDATA_EMPTY_BIT 31
#define TXCTRL_EN_BIT     0
#define TXCTRL_NSTOP_BIT  1
#define RXCTRL_EN_BIT     0
#define IE_TXWM_BIT       0
#define IE_RXWM_BIT       1

void uart_init(volatile int32_t *base) {
  base[DIV] = (BASE_FREQUENCY + UART_SPEED/2) / UART_SPEED;
  base[TXCTRL] = 1 << TXCTRL_EN_BIT;  // Enable TX
  base[RXCTRL] = 1 << RXCTRL_EN_BIT;  // Enable TX
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

#define UART0 ((volatile int32_t *)0x10000000)

#define N 100

static char primes[N*N+1];

static void myputc(char c) {
  if (c == '\n')
    uart_send_blocking(UART0, '\r');
  uart_send_blocking(UART0, c);
}

void myputs(char *s);
void myputn(unsigned n);

int main(int argc, char **argv) {
    int NN, p;
    char *primes_end, *cp, *pi;
    long *lp;

    uart_init(UART0);

    NN = N*N;

    // 3 5 7 9 11 13 .. i*2+3
    // primes[NN] is the sential; this enables us to scan without
    // having to test against the array limit
    // XXX convert to bitvector for greater reach
    primes_end = primes + NN;

    for (;;) {
        start_over:
        for (lp = (long *)primes; lp < (long *)primes_end - 1 - 4*8; lp += 4) {
            lp[0] = 0x0101010101010101ull;
            lp[1] = 0x0101010101010101ull;
            lp[2] = 0x0101010101010101ull;
            lp[3] = 0x0101010101010101ull;
        }

        primes[NN] = 0;

        myputs("2\n");

        pi = primes;
        for (;;) {
            p = 3 + 2*(pi - primes);
            myputn(p);
            myputc('\n');
            cp = pi + p*(p/2);

            while (!*++pi)
                ;

            if (pi >= primes_end) { // XXX It would be a bug if it got beyond primes_end
                myputs("Bug?  pi = ");
                myputn(pi - primes_end);
                myputs("\n");
                goto start_over;
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
