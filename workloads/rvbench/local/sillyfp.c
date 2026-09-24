#include <stdio.h>
#include <stdlib.h>

// sillyloop with every third c.nop replaced by an FP conversion, which executes on the FP port
// instead of an ALU: 22 c.nop + 10 fcvt.d.w + the loop's addiw and bnez.
int main(int c, char **v) {
  for (int i = 0; i < 1000000; ++i) {
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
    asm volatile("fcvt.d.w ft0, zero" ::: "ft0");
    asm("c.nop");
    asm("c.nop");
  }
}
