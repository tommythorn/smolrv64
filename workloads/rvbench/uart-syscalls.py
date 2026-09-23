#!/usr/bin/env python3
# uart-syscalls.py <riscv-tests/benchmarks/common/syscalls.c> -> stdout
# The benchmarks print through HTIF: every character is syscall(SYS_write, dev, &c, 1), a
# store of a request block's address to `tohost` and a spin on `fromhost`. Nothing here
# serves HTIF -- a testbench write to `fromhost` would sit behind the D$ and diverge the
# lockstep -- so the one function is replaced: SYS_write goes to the SoC's NS16550 at
# 0x1000_0000, polling LSR.THRE. Everything else in the file (printf, setStats, the tohost
# exit the testbench's +tohost= catches) is upstream's, unmodified.
import re, sys
src = open(sys.argv[1]).read()
m = re.search(r'static uintptr_t syscall\(uintptr_t which, uint64_t arg0, uint64_t arg1, uint64_t arg2\)\n\{.*?\n\}\n', src, re.S)
if not m:
    sys.exit("uart-syscalls.py: upstream syscall() not found -- riscv-tests changed; update the pattern")
uart = '''static uintptr_t syscall(uintptr_t which, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
  /* rvbench: the console is the SoC's NS16550 (THR +0, LSR +5, THRE = 0x20); HTIF is not served */
  if (which == SYS_write) {
    while (!(*(volatile uint8_t *)0x10000005 & 0x20))
      ;
    *(volatile uint8_t *)0x10000000 = *(const char *)arg1;
    return 1;
  }
  return (uintptr_t)-1;
}
'''
sys.stdout.write(src[:m.start()] + uart + src[m.end():])
