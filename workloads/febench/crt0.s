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
        # zero .bss: nothing else does, and the XMODEM uploader pads its last block
        # with 0x1a -- uninitialized statics otherwise start life holding that padding.
        la   t0, __bss_start
        la   t1, _end
1:      bgeu t0, t1, 2f
        sd   zero, 0(t0)
        addi t0, t0, 8
        j    1b
2:      j    main
