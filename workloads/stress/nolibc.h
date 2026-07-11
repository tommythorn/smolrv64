/* minimal riscv64 syscall shims -- no libc, no toolchain sysroot contamination */
#ifndef NOLIBC_H
#define NOLIBC_H
typedef unsigned long ulong; typedef long ssize_t; typedef unsigned long size_t;
static inline long sysc(long n, long a, long b, long c, long d, long e, long f) {
    register long a7 __asm__("a7") = n;
    register long a0 __asm__("a0") = a; register long a1 __asm__("a1") = b;
    register long a2 __asm__("a2") = c; register long a3 __asm__("a3") = d;
    register long a4 __asm__("a4") = e; register long a5 __asm__("a5") = f;
    __asm__ volatile("ecall" : "+r"(a0)
                     : "r"(a7), "r"(a1), "r"(a2), "r"(a3), "r"(a4), "r"(a5)
                     : "memory");
    return a0;
}
#define SYS_openat 56
#define SYS_close 57
#define SYS_pipe2 59
#define SYS_read 63
#define SYS_write 64
#define SYS_exit 93
#define SYS_exit_group 94
#define SYS_clone 220
#define SYS_execve 221
#define SYS_mmap 222
#define SYS_munmap 215
#define SYS_wait4 260
#define SYS_unshare 97
#define SYS_mount 40
#define SYS_mkdirat 34
#define AT_FDCWD (-100)
#define O_CREAT 0100
#define O_WRONLY 01
#define PROT_READ 1
#define PROT_WRITE 2
#define MAP_PRIVATE 2
#define MAP_ANONYMOUS 0x20
#define CLONE_NEWNS 0x00020000
#define MS_REC 16384
#define MS_PRIVATE (1<<18)
#define SIGCHLD 17
static inline long xwrite(int fd, const void *p, ulong n){ return sysc(SYS_write, fd, (long)p, n,0,0,0); }
static inline long xread (int fd, void *p, ulong n)      { return sysc(SYS_read,  fd, (long)p, n,0,0,0); }
static inline long xclose(int fd)                        { return sysc(SYS_close, fd,0,0,0,0,0); }
static inline long xpipe2(int *f)                        { return sysc(SYS_pipe2, (long)f,0,0,0,0,0); }
static inline long xfork(void)                           { return sysc(SYS_clone, SIGCHLD,0,0,0,0,0); }
static inline long xexecve(const char*p,char*const*a,char*const*e){ return sysc(SYS_execve,(long)p,(long)a,(long)e,0,0,0); }
static inline long xwait4(long pid,int*st)               { return sysc(SYS_wait4, pid,(long)st,0,0,0,0); }
static inline long xunshare(long fl)                     { return sysc(SYS_unshare, fl,0,0,0,0,0); }
static inline long xmount(const char*s,const char*t,const char*ty,ulong fl,const void*d){ return sysc(SYS_mount,(long)s,(long)t,(long)ty,fl,(long)d,0); }
static inline long xmkdir(const char*p)                  { return sysc(SYS_mkdirat, AT_FDCWD,(long)p,0755,0,0,0); }
static inline long xopenc(const char*p)                  { return sysc(SYS_openat, AT_FDCWD,(long)p,O_CREAT|O_WRONLY,0644,0,0); }
static inline void*xmmap(ulong len)                      { return (void*)sysc(SYS_mmap,0,len,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0); }
static inline long xmunmap(void*p,ulong l)               { return sysc(SYS_munmap,(long)p,l,0,0,0,0); }
static inline void xexit(int c)                          { sysc(SYS_exit_group,c,0,0,0,0,0); __builtin_unreachable(); }
static inline ulong xstrlen(const char*s){ ulong n=0; while(s[n]) n++; return n; }
static inline void puts1(const char*s){ xwrite(1,s,xstrlen(s)); }
#endif
/* v2 additions: sandbox-shaped syscalls beyond fork/mount/unshare */
#define SYS_ppoll 73
#define SYS_prctl 167
#define SYS_seccomp 277
#define SYS_close_range 436
#define PR_SET_NO_NEW_PRIVS 38
#define SECCOMP_SET_MODE_FILTER 1
#define MS_MOVE 8192
struct xpollfd { int fd; short events; short revents; };
struct xtimespec { long sec; long nsec; };
struct xsock_filter { unsigned short code; unsigned char jt, jf; unsigned int k; };
struct xsock_fprog { unsigned short len; const struct xsock_filter *filter; };
static inline long xppoll(struct xpollfd *f, ulong n, struct xtimespec *ts) {
    return sysc(SYS_ppoll, (long)f, n, (long)ts, 0, 8, 0);   /* sigsetsize=8, no mask */
}
static inline long xprctl(long op, long a) { return sysc(SYS_prctl, op, a, 0,0,0,0); }
static inline long xseccomp(ulong op, ulong fl, const void *p) { return sysc(SYS_seccomp, op, fl, (long)p,0,0,0); }
static inline long xclose_range(ulong lo, ulong hi, ulong fl) { return sysc(SYS_close_range, lo, hi, fl,0,0,0); }
