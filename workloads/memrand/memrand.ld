OUTPUT_ARCH(riscv)
ENTRY(_start)
SECTIONS
{
  . = 0x80000000;
  .text   : { *(.text.startup) *(.text*) }
  .rodata : { *(.rodata*) *(.srodata*) }
  .data   : { *(.data*) . = ALIGN(8); PROVIDE(__global_pointer$ = . + 0x800); *(.sdata*) }
  . = ALIGN(8);
  __bss_start = .;
  .bss    : { *(.bss*) *(.sbss*) *(COMMON) }
  . = ALIGN(4096);
  _end = .;
}
