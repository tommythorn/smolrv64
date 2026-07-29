        .globl _start
        .section .text.startup
_start: li   sp, 0x80100000        # 1 MiB of headroom below; payload is tiny
        li   t0, 0x2000            # mstatus.FS = Initial BEFORE any compiled FP
        csrs mstatus, t0           # (gcc's prologue saves fs0 first thing in main)
        # gp MUST be set up: gcc addresses .sdata/.sbss (e.g. expect[]) gp-relative,
        # and with gp=0 those accesses silently land in nowhere.
        .option push
        .option norelax
        la   gp, __global_pointer$
        .option pop
        j    main
