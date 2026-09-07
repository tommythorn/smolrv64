// ddrhpm -- read the on-chip DDR latency histogram (src/ddr_hpm.v) from Linux
// userspace on the board, and print it in a form the sim model can be calibrated
// against.
//
// The gadget measures REQUEST->ACK latency of rv_soc_top's external 64-byte line port
// in core-clock cycles -- exactly what the simulation's DDR model approximates
// (tb_virtio / tb_cosim_linux `+ddr_real +ddr_lat=N`). Until this is read from a
// real boot, that model is a GUESS.
//
//   build:  riscv64-linux-gnu-gcc -O2 -static -o ddrhpm ddrhpm.c
//   run:    sudo ./ddrhpm            # snapshot
//           sudo ./ddrhpm -z         # snapshot, then zero the counters
//           sudo ./ddrhpm -z && <workload> && sudo ./ddrhpm    # isolate one phase
//
// MMIO map (from ddr_hpm.v), 256-byte window at 0x1800_0000:
//   0x00..0x38  rd_bin[0..7]   0x40..0x78  wr_bin[0..7]
//   0x80 rd_sum  0x88 wr_sum   0x90 rd_cnt  0x98 wr_cnt
// ANY write to the window clears all counters.
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define HPM_BASE 0x18000000UL
#define HPM_SIZE 0x100

// log2 buckets, matching perftool's bucket() convention
static const char *BIN_LABEL[8] = {
   "0", "1", "2-3", "4-7", "8-15", "16-31", "32-63", "64+"
};

static void print_hist(const char *what, const volatile uint64_t *bin,
                       uint64_t sum, uint64_t cnt)
{
   uint64_t total = 0, i;
   uint64_t v[8];
   for (i = 0; i < 8; i++) { v[i] = bin[i]; total += v[i]; }

   printf("\n%s: count=%llu", what, (unsigned long long)cnt);
   if (cnt)
      printf("  mean=%.2f cycles", (double)sum / (double)cnt);
   printf("\n");
   if (!total) { printf("  (no transactions)\n"); return; }

   for (i = 0; i < 8; i++) {
      double pct = 100.0 * (double)v[i] / (double)total;
      int bars = (int)(pct / 2.0 + 0.5), b;
      printf("  %-6s %12llu  %5.1f%%  ", BIN_LABEL[i], (unsigned long long)v[i], pct);
      for (b = 0; b < bars; b++) putchar('#');
      putchar('\n');
   }
}

int main(int argc, char **argv)
{
   int zero = (argc > 1 && strcmp(argv[1], "-z") == 0);
   int fd = open("/dev/mem", zero ? O_RDWR | O_SYNC : O_RDONLY | O_SYNC);
   if (fd < 0) { perror("open /dev/mem (need root)"); return 1; }

   void *map = mmap(NULL, HPM_SIZE, zero ? (PROT_READ | PROT_WRITE) : PROT_READ,
                    MAP_SHARED, fd, HPM_BASE);
   if (map == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

   volatile uint64_t *h = (volatile uint64_t *)map;
   uint64_t rd_sum = h[0x80 / 8], wr_sum = h[0x88 / 8];
   uint64_t rd_cnt = h[0x90 / 8], wr_cnt = h[0x98 / 8];

   printf("=== DDR line-port latency (core-clock cycles, request->ack) ===");
   print_hist("READ  (fills)",      &h[0x00 / 8], rd_sum, rd_cnt);
   print_hist("WRITE (write-back)", &h[0x40 / 8], wr_sum, wr_cnt);

   if (rd_cnt) {
      // The sim model takes a single base latency; report the value to use, and
      // the spread the model's jitter term should cover.
      printf("\nsim calibration:  +ddr_real +ddr_lat=%llu   (mean read latency)\n",
             (unsigned long long)(rd_sum / rd_cnt));
      printf("  compare the bin shape above against the model's distribution;\n"
             "  a long 64+ tail means refresh/bank conflicts the model must reproduce.\n");
   }

   if (zero) { h[0] = 0; printf("\ncounters cleared\n"); }

   munmap(map, HPM_SIZE);
   close(fd);
   return 0;
}
