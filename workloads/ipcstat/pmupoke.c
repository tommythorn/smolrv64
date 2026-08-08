// pmupoke -- freestanding perf_event_open trigger for the PMU-enable wedge.
// No libc (dodges the RVA23-vector-libc SIGILL): raw riscv64 Linux syscalls,
// stage markers on stdout so a sim/console log shows exactly where it dies.
//
//   build: riscv64-elf-gcc -nostdlib -ffreestanding -march=rv64gc -mabi=lp64d \
//            -static -no-pie -O2 -o pmupoke pmupoke.c
//
// Mirrors the kernel path perf stat / ipcstat take: open two hardware
// counters on self, enable, spin, read, report.
typedef unsigned long u64;
typedef unsigned int u32;

static inline long sys(long n, long a, long b, long c, long d, long e) {
    register long a7 __asm__("a7") = n;
    register long a0 __asm__("a0") = a;
    register long a1 __asm__("a1") = b;
    register long a2 __asm__("a2") = c;
    register long a3 __asm__("a3") = d;
    register long a4 __asm__("a4") = e;
    __asm__ volatile("ecall"
                     : "+r"(a0)
                     : "r"(a7), "r"(a1), "r"(a2), "r"(a3), "r"(a4)
                     : "memory");
    return a0;
}

static void out(const char *s) {
    long n = 0;
    while (s[n]) n++;
    sys(64, 1, (long)s, n, 0, 0);            // write(1, s, n)
}

static void outhex(u64 v) {
    char b[19];
    b[0] = '0'; b[1] = 'x';
    for (int i = 0; i < 16; i++) {
        int d = (v >> (60 - 4 * i)) & 0xf;
        b[2 + i] = d < 10 ? '0' + d : 'a' + d - 10;
    }
    b[18] = '\n';
    sys(64, 1, (long)b, 19, 0, 0);
}

static long popen_hw(u64 config) {
    u64 attr[16] = {0};                       // 128 bytes, zeroed
    attr[0] = 128ul << 32;                    // type=0 (HARDWARE), size=128
    attr[1] = config;                         // config @8
    attr[5] = 1;                              // flags @40: disabled=1
    return sys(241, (long)attr, 0, -1, -1, 0);  // perf_event_open(attr,pid=0,cpu=-1,group=-1,flags=0)
}

void _start(void) {
    out("PMUPOKE S1 open cycles\n");
    long fdc = popen_hw(0);
    outhex(fdc);
    out("PMUPOKE S2 open instret\n");
    long fdi = popen_hw(1);
    outhex(fdi);

    out("PMUPOKE S3 enable cycles\n");
    if (fdc >= 0) sys(29, fdc, 0x2400, 0, 0, 0);   // ioctl(ENABLE)
    out("PMUPOKE S4 enable instret\n");
    if (fdi >= 0) sys(29, fdi, 0x2400, 0, 0, 0);

    out("PMUPOKE S5 spin\n");
    volatile u64 x = 0;
    for (u64 i = 0; i < 3000000; i++) x += i;

    out("PMUPOKE S6 read\n");
    u64 vc = 0, vi = 0;
    if (fdc >= 0) sys(63, fdc, (long)&vc, 8, 0, 0);
    if (fdi >= 0) sys(63, fdi, (long)&vi, 8, 0, 0);
    out("cycles:  "); outhex(vc);
    out("instret: "); outhex(vi);

    out("PMUPOKE S7 disable\n");
    if (fdc >= 0) sys(29, fdc, 0x2401, 0, 0, 0);
    if (fdi >= 0) sys(29, fdi, 0x2401, 0, 0, 0);

    // ---- phase 2: child-attach, the exact perf-stat / ipcstat shape ----
    // Counters on ANOTHER task ride the scheduler's PMU context-switch path
    // (pmu->add/del -> SBI start/stop with IRQs off), which self-attach may
    // never exercise on an idle system.
    out("PMUPOKE S8 clone\n");
    long pid = sys(220, 17, 0, 0, 0, 0);       // clone(SIGCHLD) = fork
    if (pid == 0) {
        volatile u64 y = 0;                    // child: burn, then exit
        for (u64 i = 0; i < 2000000; i++) y += i;
        sys(93, 0, 0, 0, 0, 0);
    }
    out("PMUPOKE S9 attach child\n");
    u64 attr[16] = {0};
    attr[0] = 128ul << 32;                     // HARDWARE cycles
    attr[1] = 0;
    attr[5] = 1;                               // disabled
    long fdk = sys(241, (long)attr, pid, -1, -1, 0);
    outhex(fdk);
    out("PMUPOKE S10 enable child counter\n");
    if (fdk >= 0) sys(29, fdk, 0x2400, 0, 0, 0);
    out("PMUPOKE S11 wait4\n");
    long st = 0;
    sys(260, pid, (long)&st, 0, 0, 0);         // wait4(pid, &st, 0, NULL)
    out("PMUPOKE S12 read child counter\n");
    u64 vk = 0;
    if (fdk >= 0) sys(63, fdk, (long)&vk, 8, 0, 0);
    out("child cycles: "); outhex(vk);

    out("PMUPOKE DONE\n");
    sys(93, 0, 0, 0, 0, 0);                    // exit(0)
    for (;;) ;
}
