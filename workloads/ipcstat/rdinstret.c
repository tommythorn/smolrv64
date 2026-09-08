// rdinstret -- print the hart's retired-instruction counter, once. Under Simmerv (whose
// instret is its own instruction count) sampling this at each Geekbench "Running X" line
// gives instructions per subtest with no PMU driver involved (the SBI PMU path returns
// nonsense there: 27 instructions for `true`). Needs kernel.perf_user_access=2, else the
// CSR read traps to SIGILL on kernels since 6.6.
//   build: riscv64-linux-gnu-gcc -O2 -static -march=rv64gc -mabi=lp64d -o rdinstret rdinstret.c
#include <stdio.h>
int main(void)
{
   unsigned long v;
   __asm__ volatile("rdinstret %0" : "=r"(v));
   printf("%lu\n", v);
   return 0;
}
