// shabench: the sha256sum instruction mix with nothing else in the way, and the SAME 13
// counters `tools/perf-smol.sh cpi` reads on the board, printed in perf-stat format so
// `tools/perf-cpi-stack.py` parses either.  Bare metal (M-mode, the febench crt0) it
// programs mhpmevent3..15 itself; under Linux it is a plain program for perf to wrap.
//
//   make                              shabench.bin (bare metal) + shabench (Linux, dynamic)
//   FW=$(pwd)/shabench.bin INITRD= CYC=30000000 ../../ooo2/run-ooo2-linux.sh
//     | grep -E '^ +[0-9]+ +(cycles|instructions|r0)' | ../../tools/perf-cpi-stack.py
//   on the board: ./perf-smol.sh cpi ./shabench 64 2>&1 | ./perf-cpi-stack.py
//
// The digest is printed: the buffer is deterministic, so `python3 -c` with hashlib over the
// same bytes checks the bench hashes what it says (see the Makefile's `check` target).
#include "sha256.h"

#define BUF_BYTES 65536
static unsigned char buf[BUF_BYTES];
static const char *HEX = "0123456789abcdef";

static void fill(void)
{
   unsigned long i;
   for (i = 0; i < BUF_BYTES; i++) buf[i] = (unsigned char)((i * 2654435761ul) >> 24);
}

static void hash(unsigned long reps, unsigned char out[32])
{
   uint32_t st[8];
   unsigned long r;
   sha256_init(st);
   for (r = 0; r < reps; r++) sha256_blocks(st, buf, BUF_BYTES / 64);
   sha256_final(st, reps * BUF_BYTES, out);
}

#ifndef BARE
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv)
{
   unsigned long reps = argc > 1 ? strtoul(argv[1], 0, 0) : 64;
   unsigned char out[32];
   int i;
   fill();
   hash(reps, out);
   for (i = 0; i < 32; i++) { putchar(HEX[out[i] >> 4]); putchar(HEX[out[i] & 15]); }
   printf("  reps=%lu\n", reps);
   return 0;
}
#else
typedef unsigned char uint8_t; typedef unsigned long uint64_t;
#define UART ((volatile uint8_t *)0x10000000)
static void putc_(uint8_t c) { while (!(UART[5] & 0x20)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void putdec(uint64_t v) { char b[24]; int n = 0; do { b[n++] = '0' + v % 10; v /= 10; } while (v); while (n) putc_(b[--n]); }
// The event name is printed from EV[] as "r%04x": a static array of string POINTERS lands in
// .data.rel.ro, which the borrowed febench.ld places past a DATA_SEGMENT_ALIGN gap the flat
// image does not carry -- the first cut printed garbage names beside correct counts.
static void putev(unsigned code) { int i; putc_('r'); for (i = 12; i >= 0; i -= 4) putc_(HEX[(code >> i) & 15]); }
static void putrow(uint64_t v, const char *name) { puts_("      "); putdec(v); puts_("      "); puts_(name); puts_("\n"); }
static void putevrow(uint64_t v, unsigned code) { puts_("      "); putdec(v); puts_("      "); putev(code); puts_("\n"); }

// The cpi set of tools/perf-smol.sh, in mhpmcounter3..15 order.
static const unsigned EV[13] = { 0x300, 0x301, 0x302, 0x303, 0x304, 0x305, 0x311, 0x312,
                                 0x313, 0x314, 0x317, 0x005, 0x102 };
#define CSRW(n, v) __asm__ volatile("csrw " #n ", %0" :: "r"(v))
#define CSRR(n, v) __asm__ volatile("csrr %0, " #n : "=r"(v))
static void hpm_program(void)
{
   CSRW(0x323, EV[0]);  CSRW(0x324, EV[1]);  CSRW(0x325, EV[2]);  CSRW(0x326, EV[3]);
   CSRW(0x327, EV[4]);  CSRW(0x328, EV[5]);  CSRW(0x329, EV[6]);  CSRW(0x32a, EV[7]);
   CSRW(0x32b, EV[8]);  CSRW(0x32c, EV[9]);  CSRW(0x32d, EV[10]); CSRW(0x32e, EV[11]);
   CSRW(0x32f, EV[12]);
}
static void hpm_read(uint64_t v[13])
{
   CSRR(0xb03, v[0]);  CSRR(0xb04, v[1]);  CSRR(0xb05, v[2]);  CSRR(0xb06, v[3]);
   CSRR(0xb07, v[4]);  CSRR(0xb08, v[5]);  CSRR(0xb09, v[6]);  CSRR(0xb0a, v[7]);
   CSRR(0xb0b, v[8]);  CSRR(0xb0c, v[9]);  CSRR(0xb0d, v[10]); CSRR(0xb0e, v[11]);
   CSRR(0xb0f, v[12]);
}

int main(void)
{
   uint64_t c0, c1, i0, i1, h0[13], h1[13];
   unsigned char out[32];
   int i;
   fill();
   hpm_program();
   hash(1, out);                     // warm: code and buffer resident, as on the board
   CSRR(0xc00, c0); CSRR(0xc02, i0); hpm_read(h0);
   hash(2, out);
   CSRR(0xc00, c1); CSRR(0xc02, i1); hpm_read(h1);
   puts_("shabench digest ");
   for (i = 0; i < 32; i++) { putc_(HEX[out[i] >> 4]); putc_(HEX[out[i] & 15]); }
   puts_("  reps=2\n");
   putrow(c1 - c0, "cycles");
   putrow(i1 - i0, "instructions");
   for (i = 0; i < 13; i++) putevrow(h1[i] - h0[i], EV[i]);
   for (;;) ;
}
#endif
