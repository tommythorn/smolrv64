/* sandbox-stress (nolibc): reproduce systemd's generator-sandbox shape.
 * Round: fork manager -> unshare(CLONE_NEWNS) + private tmpfs -> spawn NGEN
 * re-exec'd children (COW churn + tmpfs writes) -> waitall -> pipe barrier.
 * No libc: raw ecall shims only (the host cross-glibc is next-baseline/V-
 * tainted and must never touch the board). */
#include "nolibc.h"
#define ROUNDS 50
#define NGEN   16
#define COWSZ  (256*1024)

static const char *selfpath;

/* v2 generator variants: each round type exercises one more ingredient of
 * systemd's sandbox that v1 lacked, so a stall names its own syscall:
 *   mode 0: v1 COW/tmpfs round
 *   mode 1: ppoll-with-timeout barrier (lost-timer-wakeup probe: a child whose
 *           poll timeout never fires sleeps forever = the generator signature)
 *   mode 2: seccomp allow-all filter install (NO_NEW_PRIVS + BPF)
 *   mode 3: close_range + MS_MOVE mount dance */
static int be_gen_ppoll(void) {
    int p[2];
    if (xpipe2(p)) return 4;
    struct xpollfd pf; struct xtimespec ts;
    for (int i = 0; i < 20; i++) {
        pf.fd = p[0]; pf.events = 1; pf.revents = 0;   /* POLLIN, never ready */
        ts.sec = 0; ts.nsec = 10*1000*1000;             /* 10ms */
        long r = xppoll(&pf, 1, &ts);
        if (r != 0) return 5;                           /* must TIME OUT each pass */
    }
    xclose(p[0]); xclose(p[1]);
    return 0;
}
static int be_gen_seccomp(void) {
    static const struct xsock_filter allow = { 0x06, 0, 0, 0x7fff0000u }; /* RET ALLOW */
    struct xsock_fprog prog = { 1, &allow };
    if (xprctl(PR_SET_NO_NEW_PRIVS, 1)) return 6;
    if (xseccomp(SECCOMP_SET_MODE_FILTER, 0, &prog)) return 7;
    /* run a few syscalls THROUGH the filter */
    char b[8]; long fd = xopenc("/tmp/sbx/sc");
    if (fd >= 0) { xwrite(fd, b, 1); xclose(fd); }
    return 0;
}
static int be_gen_mounts(void) {
    if (xmkdir("/tmp/sbx/a")) return 8;
    if (xmkdir("/tmp/sbx/b")) return 8;
    if (xmount("tmpfs", "/tmp/sbx/a", "tmpfs", 0, "size=64k")) return 8;
    if (xmount("/tmp/sbx/a", "/tmp/sbx/b", 0, MS_MOVE, 0)) return 8;
    if (xclose_range(3, ~0ul, 0)) return 8;
    return 0;
}
static int be_generator(int idx) {
    char *m = xmmap(COWSZ);
    if ((long)m < 0) return 2;
    for (ulong i = 0; i < COWSZ; i += 4096) m[i] = (char)i;
    long c = xfork();
    if (c == 0) { for (ulong i = 0; i < COWSZ; i += 4096) m[i]++; xexit(0); }
    char path[24] = "/tmp/sbx/gXX";
    path[10] = '0' + idx / 10; path[11] = '0' + idx % 10;
    long fd = xopenc(path);
    if (fd >= 0) { xwrite(fd, path, 12); xclose(fd); }
    int st = -1; xwait4(c, &st);
    xmunmap(m, COWSZ);
    return (st == 0) ? 0 : 3;
}

int main(int argc, char **argv) {
    selfpath = argv[0];
    if (argc == 3 && argv[1][0]=='-' && argv[1][1]=='g') {
        int mode = argv[2][0]-'0';
        int idx  = (argv[2][1]-'0')*10 + (argv[2][2]-'0');
        if (mode == 1) return be_gen_ppoll();
        if (mode == 2) return be_gen_seccomp();
        if (mode == 3) return be_gen_mounts();
        return be_generator(idx);
    }
    puts1("sandbox-stress(nolibc): 50 rounds x 16 generators\n");
    for (int r = 0; r < ROUNDS; r++) {
        int rp[2];
        if (xpipe2(rp)) { puts1("pipe fail\n"); return 1; }
        long mgr = xfork();
        if (mgr == 0) {
            xclose(rp[0]);
            if (xunshare(CLONE_NEWNS))                          { xwrite(rp[1],"U",1); xexit(10); }
            if (xmount("none","/",0,MS_REC|MS_PRIVATE,0))       { xwrite(rp[1],"P",1); xexit(11); }
            if (xmount("tmpfs","/tmp","tmpfs",0,"size=4m"))     { xwrite(rp[1],"T",1); xexit(12); }
            if (xmkdir("/tmp/sbx"))                             { xwrite(rp[1],"D",1); xexit(13); }
            long g[NGEN]; char nb[4] = "000";
            for (int i = 0; i < NGEN; i++) {
                g[i] = xfork();
                if (g[i] == 0) {
                    nb[0] = '0' + (r & 3);              /* round mode cycles 0..3 */
                    nb[1] = '0' + i/10; nb[2] = '0' + i%10;
                    char *av[4]; av[0]=(char*)selfpath; av[1]="-g"; av[2]=nb; av[3]=0;
                    xexecve(selfpath, av, 0);
                    xexit(9);
                }
            }
            int bad = 0;
            for (int i = 0; i < NGEN; i++) { int st=-1; xwait4(g[i], &st); if (st) bad++; }
            xwrite(rp[1], bad ? "B" : "K", 1);
            xexit(bad ? 14 : 0);
        }
        xclose(rp[1]);
        char ack = 0;
        xread(rp[0], &ack, 1);
        xclose(rp[0]);
        int st = -1; xwait4(mgr, &st);
        if (ack != 'K') {                       /* name the failing stage: U/P/T/D/B or 0=no byte */
            char msg[] = "\nFAIL ack=?\n";
            msg[10] = ack ? ack : '0';
            xwrite(1, msg, sizeof(msg) - 1);
            return 1;
        }
        xwrite(1, ".", 1);
    }
    puts1("\nsandbox-stress PASS\n");
    return 0;
}

/* entry: set up argc/argv from the initial stack, call main, exit */
__asm__(
    ".global _start\n"
    "_start:\n"
    "  ld   a0, 0(sp)\n"        /* argc */
    "  addi a1, sp, 8\n"        /* argv */
    "  andi sp, sp, -16\n"
    "  call main\n"
    "  j    exit_shim\n");
void exit_shim(void) { register long a0 __asm__("a0"); xexit((int)a0); }

/* gcc may synthesize calls to these; provide scalar versions locally */
void *memset(void *d, int c, unsigned long n) {
    char *p = d; while (n--) *p++ = (char)c; return d;
}
void *memcpy(void *d, const void *s, unsigned long n) {
    char *p = d; const char *q = s; while (n--) *p++ = *q++; return d;
}
