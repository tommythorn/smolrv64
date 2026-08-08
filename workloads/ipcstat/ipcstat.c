// ipcstat -- run a command and report cycles, instructions, IPC (and wall
// time) using perf_event_open, the same counters the kernel PMU driver
// (sscofpmf) exposes.  Raw rdcycle/rdinstret are blocked from userspace on
// modern kernels, so this is the portable way to get IPC on the board.
//
//   build:  riscv64-linux-gnu-gcc -O2 -static -o ipcstat ipcstat.c
//   run:    ./ipcstat <command> [args...]        (run as root on the board)
//
// The child stalls on a pipe until the parent has attached both counters, so
// nothing before exec is counted except the exec itself.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <linux/perf_event.h>

static int perf_open(int pid, unsigned type, unsigned long long config, int group)
{
   struct perf_event_attr a;
   memset(&a, 0, sizeof a);
   a.size = sizeof a;
   a.type = type;
   a.config = config;
   a.disabled = (group == -1);
   int fd = syscall(__NR_perf_event_open, &a, pid, -1, group, 0);
   if (fd < 0) { perror("perf_event_open"); exit(1); }
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
      char c;
      close(gate[1]);
      if (read(gate[0], &c, 1)) {}   // wait for the parent to attach counters
      close(gate[0]);
      execvp(argv[1], &argv[1]);
      perror("execvp");
      _exit(127);
   }

   close(gate[0]);
   // No perf group: the riscv SBI-PMU driver rejects grouped counters
   // (EINVAL) since each maps to an independent SBI counter. Two separate
   // events, enabled back-to-back, skew ~us -- fine at these run lengths.
   int fd_cyc = perf_open(pid, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES, -1);
   int fd_ins = perf_open(pid, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS, -1);

   struct timeval t0, t1;
   gettimeofday(&t0, NULL);
   ioctl(fd_cyc, PERF_EVENT_IOC_ENABLE, 0);
   ioctl(fd_ins, PERF_EVENT_IOC_ENABLE, 0);
   if (write(gate[1], "g", 1) != 1) { perror("write"); return 1; }
   close(gate[1]);

   int st;
   waitpid(pid, &st, 0);
   gettimeofday(&t1, NULL);
   ioctl(fd_cyc, PERF_EVENT_IOC_DISABLE, 0);
   ioctl(fd_ins, PERF_EVENT_IOC_DISABLE, 0);

   long long cyc = 0, ins = 0;
   if (read(fd_cyc, &cyc, 8) != 8 || read(fd_ins, &ins, 8) != 8) {
      perror("read counters");
      return 1;
   }

   double wall = (t1.tv_sec - t0.tv_sec) + 1e-6 * (t1.tv_usec - t0.tv_usec);
   fprintf(stderr, "ipcstat: cycles=%lld instret=%lld IPC=%.3f wall=%.2fs util=%.0f%%\n",
           cyc, ins, cyc ? (double)ins / (double)cyc : 0.0, wall,
           wall > 0 ? 100.0 * (double)cyc / 66666666.7 / wall : 0.0);
   return WIFEXITED(st) ? WEXITSTATUS(st) : 1;
}
