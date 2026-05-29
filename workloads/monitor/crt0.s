        .globl _start
        .section .text.startup
_start:
        lla sp, __stack_top

        /* Set gp for gp-relative (small-data) access; norelax so the
           lla itself isn't turned into a gp-relative load. */
        .option push
        .option norelax
        lla gp, __global_pointer$
        .option pop

        lla t0, __bss_start
        lla t1, _end
1:
        bgeu t0, t1, 2f
        sd zero, 0(t0)
        addi t0, t0, 8
        j 1b
2:
        j main
