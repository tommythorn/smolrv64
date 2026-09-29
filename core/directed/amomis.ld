/* Mirrors the riscv-tests layout the in-order TB expects: entry at 0x80000000
   and tohost at 0x80001000 (the TB's default, also discoverable via nm). */
OUTPUT_ARCH(riscv)
ENTRY(_start)
SECTIONS
{
  . = 0x80000000;
  .text.init : { *(.text.init) }
  . = 0x80001000;
  .tohost    : { *(.tohost) }
  . = 0x80002000;
  .data      : { *(.data) }
}
