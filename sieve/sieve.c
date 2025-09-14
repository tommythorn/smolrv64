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

#define N 100

static char primes[N*N+1];

static inline void myputc(char c) {
  *(volatile char *)0x10000000 = c;
}

void myputs(char *s);
void myputn(unsigned n);

int main(int argc, char **argv) {
    int NN, p;
    char *primes_end, *cp, *pi;
    long *lp;

    NN = N*N;

    // 3 5 7 9 11 13 .. i*2+3
    // primes[NN] is the sential; this enables us to scan without
    // having to test against the array limit
    // XXX convert to bitvector for greater reach
    primes_end = primes + NN;

    for (;;) {
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

            if (pi >= primes_end) // XXX It would be a bug if it got beyond primes_end
                break;

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
