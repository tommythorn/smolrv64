// hpmstat -- run a command and print the in-order core's CPI STACK from hardware.
//
// The stall-attribution counters (commit 3ad1da1e) charge every non-retiring cycle to
// exactly one cause, so sum(stalls)/instret + 1 reconstructs CPI. They have been in the
// bitstream since 2026-08-14 and, as far as the logs show, have never been read on silicon.
//
// The DTB exposes them through the raw-event catch-all
//     riscv,raw-event-to-mhpmcounters = <0x0 0x0 0xffffffff 0xffff0000 0x0000fff8>
// i.e. any raw event below 0x10000 maps to counters 3..15 -- thirteen counters for the ten
// events below, so nothing multiplexes and every number is a true count, not a sample.
//
//   build:  riscv64-linux-gnu-gcc -O2 -Wall -march=rv64gc -mabi=lp64d -static -o hpmstat hpmstat.c
//           (pin -march: coffee's gcc defaults to an RVA23 baseline that emits vector and
//            SIGILLs on this core -- see workloads/ddrhpm and workloads/ipcstat)
//   run:    ./hpmstat <command> [args...]        (as root on the board)
//
// The child stalls on a pipe until the parent has attached every counter, so nothing before
// exec is counted except the exec itself.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>

struct ev { const char *name; unsigned type; unsigned long long config; const char *what; };

// Order matters only for printing. ST_FPU is kept even though it is expected to read 0:
// a measured zero is evidence, an omitted counter is an assumption.
static struct ev EVENTS[] = {
   { "cycles",   PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES,   "total cycles"                  },
   { "instret",  PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS, "retired instructions"          },
   { "ST_MEM",   PERF_TYPE_RAW,      0x0300,                     "M stalled on the LSU (D$/dTLB/AMO)" },
   { "ST_DIV",   PERF_TYPE_RAW,      0x0301,                     "...on the iterative divider"   },
   { "ST_MUL",   PERF_TYPE_RAW,      0x0302,                     "...on the 3-cycle multiplier"  },
   { "ST_FPU",   PERF_TYPE_RAW,      0x0303,                     "...on the CVFPU"               },
   { "ST_SER",   PERF_TYPE_RAW,      0x0304,                     "serializing op holds frontend off" },
   { "FE_BUB",   PERF_TYPE_RAW,      0x0310,                     "X idle: frontend supplied nothing" },
   { "FE_MMU",   PERF_TYPE_RAW,      0x0311,                     "...because the iMMU was walking"   },
   { "FE_IC",    PERF_TYPE_RAW,      0x0312,                     "...because the I$ had no window"   },
};
#define NEV ((int)(sizeof EVENTS / sizeof EVENTS[0]))
#define I_CYCLES 0
#define I_INSTRET 1
#define I_FIRST_STALL 2        /* ST_MEM .. FE_BUB are the disjoint charge; FE_MMU/FE_IC
                                  are a BREAKDOWN of FE_BUB and must not be summed in. */
#define I_LAST_STALL 7         /* FE_BUB */

static int perf_open(int pid, unsigned type, unsigned long long config, const char *name)
{
   struct perf_event_attr a;
   memset(&a, 0, sizeof a);
   a.size    = sizeof a;
   a.type    = type;
   a.config  = config;
   a.disabled = 1;
   a.inherit  = 1;             /* follow children -- shells fork */
   // No perf group: the riscv SBI-PMU driver rejects grouped counters (EINVAL), since each
   // maps to an independent SBI counter. See workloads/ipcstat/ipcstat.c.
   int fd = (int)syscall(__NR_perf_event_open, &a, pid, -1, -1, 0);
   if (fd < 0) {
      fprintf(stderr, "perf_event_open(%s, type=%u, config=0x%llx): %s\n",
              name, type, config, strerror(errno));
      return -1;
   }
   return fd;
}

int main(int argc, char **argv)
{
   if (argc < 2) { fprintf(stderr, "usage: %s <command> [args...]\n", argv[0]); return 1; }

   int gate[2];
   if (pipe(gate)) { perror("pipe"); return 1; }

   pid_t pid = fork();
   if (pid < 0) { perror("fork"); return 1; }
   if (pid == 0) {
      char c; close(gate[1]);
      if (read(gate[0], &c, 1)) {}
      close(gate[0]);
      execvp(argv[1], &argv[1]);
      perror("execvp");
      _exit(127);
   }
   close(gate[0]);

   int fd[NEV];
   int nfail = 0;
   for (int i = 0; i < NEV; i++) {
      fd[i] = perf_open(pid, EVENTS[i].type, EVENTS[i].config, EVENTS[i].name);
      if (fd[i] < 0) nfail++;
   }
   if (nfail) fprintf(stderr, "hpmstat: %d counter(s) unavailable; those rows print as n/a\n", nfail);

   struct timeval t0, t1;
   gettimeofday(&t0, NULL);
   for (int i = 0; i < NEV; i++) if (fd[i] >= 0) ioctl(fd[i], PERF_EVENT_IOC_ENABLE, 0);

   if (write(gate[1], "g", 1) != 1) { perror("write"); return 1; }
   close(gate[1]);
   int status; while (waitpid(pid, &status, 0) < 0) {}
   gettimeofday(&t1, NULL);

   unsigned long long v[NEV];
   for (int i = 0; i < NEV; i++) {
      v[i] = 0;
      if (fd[i] >= 0) { ioctl(fd[i], PERF_EVENT_IOC_DISABLE, 0); if (read(fd[i], &v[i], 8) != 8) v[i] = 0; }
   }

   double secs = (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) / 1e6;
   double instret = (double)v[I_INSTRET];
   double cycles  = (double)v[I_CYCLES];
   if (instret <= 0) { fprintf(stderr, "hpmstat: instret is zero -- counters not working\n"); return 2; }

   printf("\n  wall %.3f s   cycles %llu   instret %llu\n", secs,
          (unsigned long long)v[I_CYCLES], (unsigned long long)v[I_INSTRET]);
   printf("  CPI = %.4f   IPC = %.4f\n\n", cycles / instret, instret / cycles);
   printf("  %-10s %14s %10s %9s   %s\n", "component", "count", "CPI", "% of CPI", "what");
   printf("  %-10s %14s %10.3f %8.1f%%   %s\n", "retire", "-", 1.0,
          100.0 / (cycles / instret), "the instruction itself");

   double summed = 1.0;
   for (int i = I_FIRST_STALL; i <= I_LAST_STALL; i++) {
      double cpi = (double)v[i] / instret;
      summed += cpi;
      printf("  %-10s %14llu %10.3f %8.1f%%   %s\n", EVENTS[i].name,
             (unsigned long long)v[i], cpi, 100.0 * cpi / (cycles / instret), EVENTS[i].what);
   }
   // FE_MMU and FE_IC are components OF FE_BUB, printed indented and excluded from the sum.
   for (int i = I_LAST_STALL + 1; i < NEV; i++) {
      double cpi = (double)v[i] / instret;
      printf("    %-8s %14llu %10.3f %8.1f%%   %s\n", EVENTS[i].name,
             (unsigned long long)v[i], cpi, 100.0 * cpi / (cycles / instret), EVENTS[i].what);
   }
   double measured = cycles / instret;
   printf("\n  sum(stalls)+1 = %.3f vs measured CPI %.3f   (unaccounted %.3f = %.1f%%)\n",
          summed, measured, measured - summed, 100.0 * (measured - summed) / measured);
   printf("  If unaccounted is large the taps are not charging every cycle to exactly one\n"
          "  cause, and the stack should not be trusted as a decomposition.\n\n");
   return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
}
