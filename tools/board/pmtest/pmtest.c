// pmtest -- pointer masking (Supm) from user space under Linux: ask the kernel for PMLEN 7 with
// prctl(PR_SET_TAGGED_ADDR_CTRL), then load and store through a pointer with junk in its top 7
// bits. Without masking that pointer is non-canonical and the access is a SIGSEGV. The kernel
// needs CONFIG_RISCV_ISA_SUPM (else the prctl is refused) and a device tree that lists ssnpm.
//   cc -O2 -o pmtest pmtest.c && ./pmtest
#include <stdint.h>
#include <stdio.h>
#include <sys/prctl.h>

#ifndef PR_SET_TAGGED_ADDR_CTRL
#define PR_SET_TAGGED_ADDR_CTRL 55
#define PR_GET_TAGGED_ADDR_CTRL 56
#define PR_TAGGED_ADDR_ENABLE   (1UL << 0)
#endif
#ifndef PR_PMLEN_SHIFT
#define PR_PMLEN_SHIFT 24
#endif

int main(void) {
   if (prctl(PR_SET_TAGGED_ADDR_CTRL, PR_TAGGED_ADDR_ENABLE | (7UL << PR_PMLEN_SHIFT), 0, 0, 0)) {
      perror("PMTEST FAIL: prctl(PR_SET_TAGGED_ADDR_CTRL, PMLEN 7)");
      return 1;
   }
   long ctrl = prctl(PR_GET_TAGGED_ADDR_CTRL, 0, 0, 0, 0);
   static volatile long cell = 0x1122334455667788;
   volatile long *tagged = (volatile long *)((uintptr_t)&cell | (0x5aUL << 57));
   long seen = *tagged;
   *tagged = 0x0123456789abcdef;
   int ok = seen == 0x1122334455667788 && cell == 0x0123456789abcdef;
   printf("PMTEST %s: ctrl=%#lx load %#lx through %p, store landed %s\n", ok ? "PASS" : "FAIL", ctrl,
          (unsigned long)seen, (void *)tagged, cell == 0x0123456789abcdef ? "yes" : "no");
   return !ok;
}
