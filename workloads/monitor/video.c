// The monitor's screen: vga_scanout at 640x480@60 (its reset mode), a splash across the top and,
// below it, a text console that mirrors everything the monitor prints, in Spleentt 5x8.
//
// The framebuffer is RGB565 in DDR (FB_BASE, the device tree's framebuffer node) and the scanout
// reads DDR, not the D$: every pixel written is cleaned to DDR (cbo.clean) before it can show.
// The scanout drives the top two bits of each channel (RGB222), so the splash's ramps are
// ordered-dithered to those 64 colours.
#include "mon.h"
#include "font.h"

#ifndef VGA
#define VGA         ((volatile uint32_t *)0x10005000)
#endif
#ifndef FB_BASE
#define FB_BASE     0xFFF00000ul
#endif
#define FB          ((volatile uint16_t *)FB_BASE)
#define W           640
#define H           480
#define STRIDE      (W * 2)
#define SPLASH_H    120                          // the splash's rows; the console is below
#define COLS        (W / FONT_W)
#define ROWS        ((H - SPLASH_H) / FONT_H)

static int on, row, col;
// what is printed before the screen is up (the banner): it goes out on the UART at once and is
// replayed into the console once the splash is drawn
static char early[256];
static int  nearly;
void video_putc(char c);

// a channel level 0..255 to 0..3, ordered-dithered by a 4x4 Bayer matrix
static const uint8_t bayer[16] = { 0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5 };
static int dither(int v, int x, int y)
{
    int v3 = v * 3, l = v3 / 255;
    return l + ((v3 - l * 255) * 16 > (bayer[(y & 3) * 4 + (x & 3)] * 2 + 1) * 255 / 2);
}
// a pixel from 0..3 levels, each replicated through its RGB565 field
static uint16_t rgb(int r, int g, int b)
{
    return (uint16_t)(((r << 3 | r << 1 | r >> 1) << 11) | ((g << 4 | g << 2 | g) << 5) | (b << 3 | b << 1 | b >> 1));
}
static uint16_t ramp(int r, int g, int b, int x, int y)
{
    return rgb(dither(r, x, y), dither(g, x, y), dither(b, x, y));
}

#define FG rgb(2, 3, 3)                       // the console's text, a pale cyan
#define BG 0

// glyph c at pixel (x, y), scaled s times; set pixels only (the background is already drawn)
static void glyph(int c, int x, int y, int s, int r0, int g0, int b0, int r1, int g1, int b1, int x0, int span)
{
    if (c < 0x20 || c > 0x7e) c = '?';
    const uint8_t *g = font[c - 0x20];
    for (int gy = 0; gy < FONT_H * s; gy++)
        for (int gx = 0; gx < FONT_W * s; gx++)
            if (g[gy / s] & (0x80 >> (gx / s))) {
                int px = x + gx, py = y + gy, t = span ? ((px - x0) * 256 / (span - x0)) : 0;
                if (t > 255) t = 255;
                FB[py * W + px] = ramp(r0 + (r1 - r0) * t / 256, g0 + (g1 - g0) * t / 256,
                                       b0 + (b1 - b0) * t / 256, px, py);
            }
}

static void text(const char *s, int x, int y, int sc, int r0, int g0, int b0, int r1, int g1, int b1, int span)
{
    for (int x0 = x; *s; s++, x += FONT_W * sc) glyph(*s, x, y, sc, r0, g0, b0, r1, g1, b1, x0, span);
}

static int width(const char *s, int sc) { int n = 0; while (s[n]) n++; return n * FONT_W * sc; }

void video_init(void)
{
    VGA[0] = 0;                                   // stopped while its registers change
    VGA[1] = (uint32_t)FB_BASE;
    VGA[2] = STRIDE;
    VGA[4] = 640;  VGA[5] = 656;  VGA[6] = 752;  VGA[7] = 800;      // VESA 640x480@60
    VGA[8] = 480;  VGA[9] = 490;  VGA[10] = 492; VGA[11] = 525;
    // the splash: a ramp from deep blue at the top to black at the console's edge
    for (int y = 0; y < SPLASH_H; y++)
        for (int x = 0; x < W; x++)
            FB[y * W + x] = ramp(8 + 16 * x / W, 0, 150 - 130 * y / SPLASH_H, x, y);
    for (uint64_t *p = (uint64_t *)(FB_BASE + SPLASH_H * STRIDE); p < (uint64_t *)(FB_BASE + H * STRIDE); p++)
        *p = 0;
    static const char title[] = "SmolRV64", sub[] = "RISC-V out-of-order soft core";
    int tx = (W - width(title, 6)) / 2, sx = (W - width(sub, 2)) / 2;
    text(title, tx + 4, 16, 6, 0, 0, 0, 0, 0, 0, 0);                          // its shadow
    text(title, tx, 12, 6, 40, 255, 255, 255, 60, 255, tx + width(title, 6));  // cyan to magenta
    text(sub, sx, 66, 2, 200, 200, 255, 200, 200, 255, 0);
    for (int x = 40; x < W - 40; x++)                                          // a rule under it
        FB[(SPLASH_H - 6) * W + x] = ramp(255 - 255 * x / W, 120, 255 * x / W, x, SPLASH_H - 6);
    cbo(FB_BASE, H * STRIDE, 1);
    VGA[0] = 1;                                   // on: negative sync polarities, as VESA's 640x480
    on = 1;  row = 0;  col = 0;
    for (int i = 0; i < nearly; i++) video_putc(early[i]);
}

// the console's cell (c, r): set pixels FG, clear ones BG, then clean its rows to DDR
static void cell(int ch, int c, int r)
{
    const uint8_t *g = font[(ch < 0x20 || ch > 0x7e ? '?' : ch) - 0x20];
    int y0 = SPLASH_H + r * FONT_H, x0 = c * FONT_W;
    for (int y = 0; y < FONT_H; y++) {
        volatile uint16_t *p = FB + (y0 + y) * W + x0;
        for (int x = 0; x < FONT_W; x++) p[x] = (g[y] & (0x80 >> x)) ? FG : BG;
        cbo((uint64_t)p, FONT_W * 2, 1);
    }
}

static void scroll(void)
{
    uint64_t top = FB_BASE + SPLASH_H * STRIDE, line = FONT_H * STRIDE, end = FB_BASE + H * STRIDE;
    volatile uint64_t *d = (volatile uint64_t *)top, *s = (volatile uint64_t *)(top + line);
    while ((uint64_t)s < end) *d++ = *s++;
    while ((uint64_t)d < end) *d++ = 0;
    cbo(top, end - top, 1);
}

void video_putc(char c)
{
    if (!on) { if (nearly < (int)sizeof early) early[nearly++] = c; return; }
    if (c == '\r') { col = 0; return; }
    if (c == '\b') { if (col) col--; return; }
    if (c == '\n') { col = 0; if (++row == ROWS) { scroll(); row = ROWS - 1; } return; }
    if (col == COLS) { col = 0; if (++row == ROWS) { scroll(); row = ROWS - 1; } }
    cell((unsigned char)c, col++, row);
}
