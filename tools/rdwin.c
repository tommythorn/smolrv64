/* rdwin PHYS N [b]: print N 64-bit words of an MMIO window from /dev/mem using 64-bit loads
 * (the contract the rv_soc_top windows serve; byte loads through the device path return
 * lane 0 replicated). With a third argument 'b' the same words are assembled from byte loads,
 * for comparison. tools/errlog-read.sh sends it to the board and builds it there. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
int main(int argc, char **argv) {
   if (argc < 3) return 2;
   uint64_t phys = strtoull(argv[1], 0, 0); int n = atoi(argv[2]); int bytes = argc > 3;
   int f = open("/dev/mem", O_RDONLY | O_SYNC); if (f < 0) { perror("/dev/mem"); return 2; }
   volatile uint8_t *p = mmap(0, 4096, PROT_READ, MAP_SHARED, f, phys & ~0xfffULL);
   if (p == MAP_FAILED) { perror("mmap"); return 2; }
   p += phys & 0xfff;
   for (int i = 0; i < n; i++) {
      uint64_t v;
      if (bytes) { v = 0; for (int k = 0; k < 8; k++) v |= (uint64_t)p[8*i+k] << (8*k); }
      else v = *(volatile uint64_t *)(p + 8*i);
      printf("%016lx%s", (unsigned long)v, i + 1 < n ? " " : "\n");
   }
   return 0;
}
