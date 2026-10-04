/*
 * fbdemo -- a plasma on the Linux framebuffer, for the graphics console (simmerv --graphics,
 * smolrv64's VGA scanout).
 *
 *   fbdemo              run until Ctrl-C
 *   fbdemo 300          run 300 frames, then exit
 *
 * Truly dependency-free: no libc at all, just Linux system calls, built -march=rv64gc. A static
 * glibc will not do -- Ubuntu 26.04's riscv64 glibc is compiled for RVA23, so even its memset
 * executes vector instructions, which smolrv64 (RVA22) does not have. Only the kernel's
 * <linux/fb.h> and <linux/kd.h> are used, for the ioctl structures.
 *
 * Each frame is drawn into an ordinary RAM buffer and copied to /dev/fb0 in one sequential
 * pass: on smolrv64 the framebuffer mapping is uncached, so pixel-at-a-time stores into it
 * would each be a DDR round trip. 16-bit RGB565 and 32-bit XRGB framebuffers both work.
 * While it runs, the console is in graphics mode so fbcon's cursor does not draw over it;
 * Ctrl-C restores it.
 */
#include <linux/fb.h>
#include <linux/kd.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long ulong;

/* ---------------- the runtime: system calls (asm-generic numbers) ---------------- */
static long sys(long n, long a, long b, long c, long d, long e, long f)
{
    register long a0 asm("a0") = a, a1 asm("a1") = b, a2 asm("a2") = c;
    register long a3 asm("a3") = d, a4 asm("a4") = e, a5 asm("a5") = f, a7 asm("a7") = n;
    asm volatile("ecall" : "+r"(a0) : "r"(a1), "r"(a2), "r"(a3), "r"(a4), "r"(a5), "r"(a7) : "memory");
    return a0;
}
#define SYS_ioctl 29
#define SYS_openat 56
#define SYS_close 57
#define SYS_write 64
#define SYS_exit_group 94
#define SYS_clock_gettime 113
#define SYS_rt_sigaction 134
#define SYS_mmap 222
#define AT_FDCWD (-100)
#define O_RDWR 2
#define PROT_RW 3
#define MAP_SHARED 1
#define MAP_PRIVATE_ANON 0x22
#define SIGINT 2
#define SIGTERM 15

static int sys_open(const char *p) { return (int)sys(SYS_openat, AT_FDCWD, (long)p, O_RDWR, 0, 0, 0); }
static int sys_ioctl(int fd, ulong req, void *arg) { return (int)sys(SYS_ioctl, fd, (long)req, (long)arg, 0, 0, 0); }
static void *sys_mmap(ulong len, int flags, int fd) { return (void *)sys(SYS_mmap, 0, (long)len, PROT_RW, flags, fd, 0); }
static int mmap_failed(void *p) { return (ulong)p > (ulong)-4096; }

void *memcpy(void *d, const void *s, ulong n)       /* the compiler may call these */
{
    ulong *dw = d; const ulong *sw = s;
    for (; n >= 8; n -= 8) *dw++ = *sw++;            /* the framebuffer copy: 8-byte stores */
    u8 *db = (u8 *)dw; const u8 *sb = (const u8 *)sw;
    while (n--) *db++ = *sb++;
    return d;
}
void *memset(void *d, int c, ulong n)
{
    u8 *p = d;
    while (n--) *p++ = (u8)c;
    return d;
}

static void put(const char *s)
{
    ulong n = 0;
    while (s[n]) n++;
    sys(SYS_write, 2, (long)s, (long)n, 0, 0, 0);
}
static void putn(long v)
{
    char b[24]; int i = 23;
    b[i] = 0;
    if (v < 0) { put("-"); v = -v; }
    do { b[--i] = (char)('0' + v % 10); v /= 10; } while (v);
    put(b + i);
}
static long now_ms(void)
{
    long ts[2];
    sys(SYS_clock_gettime, 1 /* CLOCK_MONOTONIC */, (long)ts, 0, 0, 0, 0);
    return ts[0] * 1000 + ts[1] / 1000000;
}

static volatile int stop;
static void on_signal(int sig) { (void)sig; stop = 1; }
static void catch(int sig)
{
    struct { void (*handler)(int); ulong flags; ulong mask; } sa = { on_signal, 0, 0 };
    sys(SYS_rt_sigaction, sig, (long)&sa, 0, 8, 0, 0);   /* the vDSO supplies the sigreturn */
}

/* ---------------- the demo ---------------- */
static signed char sine[256];   /* sin(2*pi*i/256) * 127, from a parabola: smooth enough */
static u32 colour[256];         /* a hue wheel, in the framebuffer's pixel format */
static u8 col[4096], row[4096];

static int run(long frames)
{
    int fd = sys_open("/dev/fb0");
    if (fd < 0) { put("fbdemo: cannot open /dev/fb0\n"); return 1; }
    struct fb_var_screeninfo var;
    struct fb_fix_screeninfo fix;
    if (sys_ioctl(fd, FBIOGET_VSCREENINFO, &var) || sys_ioctl(fd, FBIOGET_FSCREENINFO, &fix)) {
        put("fbdemo: FBIOGET_*SCREENINFO failed\n"); return 1;
    }
    int w = (int)var.xres, h = (int)var.yres, bpp = (int)var.bits_per_pixel, stride = (int)fix.line_length;
    if ((bpp != 16 && bpp != 32) || w > 4096 || h > 4096) { put("fbdemo: unsupported mode\n"); return 1; }
    ulong size = (ulong)stride * (ulong)h;
    u8 *fb = sys_mmap(size, MAP_SHARED, fd);
    u8 *back = sys_mmap(size, MAP_PRIVATE_ANON, -1);
    if (mmap_failed(fb) || mmap_failed(back)) { put("fbdemo: mmap failed\n"); return 1; }
    put("fbdemo: "); putn(w); put("x"); putn(h); put(", "); putn(bpp); put(" bpp, stride "); putn(stride); put("\n");

    for (int i = 0; i < 256; i++) {
        int x = i & 127, y = (4 * 127 * x * (128 - x)) / (128 * 128);
        sine[i] = (signed char)(i < 128 ? y : -y);
    }
    for (int i = 0; i < 256; i++) {
        u32 r = (u32)(128 + sine[i]), g = (u32)(128 + sine[(i + 85) & 255]), b = (u32)(128 + sine[(i + 170) & 255]);
        colour[i] = bpp == 16 ? ((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3)) : (r << 16 | g << 8 | b);
    }

    catch(SIGINT);
    catch(SIGTERM);
    int tty = sys_open("/dev/tty0");                 /* best effort: keep fbcon off the picture */
    if (tty >= 0 && sys_ioctl(tty, KDSETMODE, (void *)KD_GRAPHICS) != 0) { sys(SYS_close, tty, 0, 0, 0, 0, 0); tty = -1; }

    long t0 = now_ms(), last = t0, n = 0;
    for (int t = 0; !stop && n != frames; t++, n++) {
        /* Per-column and per-row terms once a frame; three lookups and two adds a pixel. */
        for (int x = 0; x < w; x++) col[x] = (u8)(sine[(x * 3 + t * 2) & 255] + sine[(x + t * 5) & 255] / 2);
        for (int y = 0; y < h; y++) row[y] = (u8)(sine[(y * 2 - t * 3) & 255] + sine[(y + t) & 255] / 2);
        for (int y = 0; y < h; y++) {
            u8 *line = back + (ulong)y * (ulong)stride;
            int ry = row[y], dy = (y + t * 4) & 255;
            if (bpp == 16) {
                u16 *p = (u16 *)line;
                for (int x = 0; x < w; x++) p[x] = (u16)colour[(u8)(col[x] + ry + sine[(x + dy) & 255])];
            } else {
                u32 *p = (u32 *)line;
                for (int x = 0; x < w; x++) p[x] = colour[(u8)(col[x] + ry + sine[(x + dy) & 255])];
            }
        }
        memcpy(fb, back, size);
        long s = now_ms();
        if (s - last >= 2000) {
            long tenths = (n + 1) * 10000 / (s - t0);
            put("fbdemo: "); putn(tenths / 10); put("."); putn(tenths % 10); put(" frames/s\n");
            last = s;
        }
    }
    put("fbdemo: "); putn(n); put(" frames in "); putn((now_ms() - t0) / 1000); put(" s\n");
    if (tty >= 0) sys_ioctl(tty, KDSETMODE, (void *)KD_TEXT);
    memset(fb, 0, size);
    return 0;
}

static long atol_(const char *s)
{
    long v = 0;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    return v;
}

/* The kernel starts us with sp -> argc, argv[0], ... */
__attribute__((used)) static void start_c(long *sp)
{
    long argc = sp[0];
    char **argv = (char **)(sp + 1);
    sys(SYS_exit_group, run(argc > 1 ? atol_(argv[1]) : -1), 0, 0, 0, 0, 0);
}
asm(".globl _start\n_start:\n"
    "  .option push\n  .option norelax\n  lla gp, __global_pointer$\n  .option pop\n"
    "  mv a0, sp\n  andi sp, sp, -16\n  call start_c\n");
