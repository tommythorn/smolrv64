// The monitor's screen: vga_scanout in the mode the V command sets (640x480@60, its reset mode,
// until then), a splash across the top and, below it, a text console that mirrors everything the
// monitor prints, in Spleentt 5x8. The splash and the console follow the mode's size.
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
#ifndef FB_SIZE
#define FB_SIZE     (16ul << 20)                 // the framebuffer region (the device tree's node)
#endif
#define FB          ((volatile uint16_t *)FB_BASE)
static int W = 640, H = 480;                     // the mode's active pixels and lines (video_mode)
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

// The splash -- a ramp from deep blue at the top to black at the console's edge, the name across
// it -- and an empty console, at the mode's size; cleaned to DDR whole.
static void draw(void)
{
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
    row = 0;  col = 0;
}

// A mode: the pixel clock asked for (kHz), the horizontal and vertical active / sync start / sync
// end / total, and the sync polarities (1: active high) -- an X11 modeline's fields.
struct vmode { uint32_t khz; uint32_t h[4], v[4]; int hpos, vpos; };
static struct vmode cur;
static uint32_t cur_d;                           // CLKOUT0's divider in cur

// THE PIXEL CLOCK is vga_mmcm_inst's CLKOUT0, the 1000 MHz VCO / D for an integer D (rk_xcku5p.v):
// the nearest D to the mode's clock, the refresh moving instead. The bitstream comes up at D = 6
// and is timed there, so no mode is faster than 166.67 MHz. Only CLKOUT0's divider is rewritten,
// read-modify-written as XAPP888 does (tools/vga-try.py on the board): the VCO, and with it the
// lock and filter settings, never moves.
#define VCO_KHZ     1000000u
#define D_MIN       6
#define DRP         16                           // VGA register indices: 0x40, 0x44, 0x30
#define PIXRST      17
#define STATUS      12
// VGA[reg] & mask reads want within a millisecond (an MMCM locks within 100 us, DS922); 0 if not
static int await(int reg, uint32_t mask, uint32_t want)
{
    for (uint64_t t0 = now(); now() - t0 <= TIMEBASE_HZ / 1000; ) if ((VGA[reg] & mask) == want) return 1;
    return 0;
}
static int drp_lost;                             // an access the MMCM never answered
static uint32_t drp(uint32_t cmd)                // one access: {we, addr, data}; a read's data
{
    VGA[DRP] = cmd;
    drp_lost |= !await(DRP, 1u << 31, 0);
    return VGA[DRP] & 0xffff;
}

static void put2(uint32_t v100)                  // v100 / 100, two decimals
{
    putdec(v100 / 100); putc_('.'); putc_('0' + v100 / 10 % 10); putc_('0' + v100 % 10);
}

// The mode, checked, then set: the scan off, the divider rewritten with the MMCM held in reset,
// the lock awaited, the registers written, the splash and console drawn at the new size, the scan
// on. A mode that fails a check leaves the screen as it was. Returns 0, or -1 having said why.
static int video_mode(const struct vmode *m)
{
    const char *bad = 0;
    for (int i = 0; i < 2 && !bad; i++) {
        const uint32_t *t = i ? m->v : m->h;
        if (!(t[0] <= t[1] && t[1] <= t[2] && t[2] <= t[3] && t[3] <= 4095))
            bad = "timings must be active <= sync start <= sync end <= total <= 4095";
    }
    uint32_t d = m->khz ? (VCO_KHZ + m->khz / 2) / m->khz : 0;
    if (bad) ;
    else if (m->h[0] % 32)                bad = "the width must be a multiple of 32 (a stride of 64-byte lines)";
    else if (m->h[0] < 320 || m->v[0] < 240) bad = "the mode is at least 320x240 (the splash and a console)";
    else if ((uint64_t)m->h[0] * m->v[0] * 2 > FB_SIZE) bad = "the frame does not fit the 16 MiB framebuffer region";
    else if (d < D_MIN)                   bad = "the pixel clock is above 166.67 MHz, the bitstream's";
    else if (d > 128)                     bad = "the pixel clock is below 7.8 MHz";
    if (bad) { puts_("V: "); puts_(bad); putc_('\n'); return -1; }

    VGA[0] = 0;                                   // stopped while its registers change
    VGA[PIXRST] = 1;                              // the MMCM held in reset
    drp_lost = 0;
    uint32_t r1 = drp(0x08u << 16), r2 = drp(0x09u << 16), high = d / 2 & 63, low = (d - d / 2) & 63;
    drp(1u << 31 | 0x08u << 16 | (r1 & 0xf000) | high << 6 | low);   // 64 is 0; keep the phase mux, reserved bit
    drp(1u << 31 | 0x09u << 16 | (r2 & 0xff3f) | (d & 1) << 7);      // edge for an odd divider; counting on
    VGA[PIXRST] = 0;
    int locked = await(STATUS, 1, 1);
    VGA[1] = (uint32_t)FB_BASE;
    VGA[2] = m->h[0] * 2;
    for (int i = 0; i < 4; i++) { VGA[4 + i] = m->h[i]; VGA[8 + i] = m->v[i]; }
    W = (int)m->h[0];  H = (int)m->v[0];
    cur = *m;  cur_d = d;
    draw();
    VGA[0] = 1u | (uint32_t)m->hpos << 1 | (uint32_t)m->vpos << 2;
    if (drp_lost || !locked) {
        puts_(drp_lost ? "V: the pixel clock's MMCM did not answer its DRP\n" : "V: the pixel clock did not lock\n");
        return -1;
    }
    return 0;
}

// The mode as set: size, pixel clock, line and frame rates, polarities, and the scanout's view.
static void video_show(void)
{
    uint32_t hz = 1000000000u / cur_d, st = VGA[STATUS], frame = cur.h[3] * cur.v[3];
    putdec(cur.h[0]); putc_('x'); putdec(cur.v[0]);
    puts_(": pixel clock "); put2((hz + 5000) / 10000); puts_(" MHz (1000 / "); putdec(cur_d);
    puts_("), "); put2((uint32_t)(((uint64_t)hz * 100 + frame / 2) / frame)); puts_(" Hz, ");
    putc_(cur.hpos ? '+' : '-'); puts_("hsync "); putc_(cur.vpos ? '+' : '-'); puts_("vsync; ");
    puts_(st & 1 ? "locked" : "NOT locked"); puts_(", frames not shown whole: "); putdec(st >> 16);
    putc_('\n');
}

void video_init(void)
{
#ifdef VIDEO_720P
    // 1280x720 at 1000 / 14 = 71.43 MHz (50.0 Hz), the mode the HDMI board's OLED monitor takes:
    // the HDMI transmitter's MMCM multiplies it by ten, a VCO of 714 MHz, under the 800 MHz
    // rating (src/hdmi_tx.v); it locks and runs.
    static const struct vmode boot = { 71430, { 1280, 1376, 1424, 1904 }, { 720, 725, 730, 750 }, 1, 1 };
#else
    static const struct vmode boot = { 25175, { 640, 656, 752, 800 }, { 480, 490, 492, 525 }, 0, 0 };
#endif
    video_mode(&boot);                            // VESA 640x480@60, at 25 MHz (59.5 Hz), or 720p
    on = 1;
    for (int i = 0; i < nearly; i++) video_putc(early[i]);
}

// V [modeline]: with no arguments, the mode; with an X11 modeline -- the clock in MHz, the four
// horizontal and the four vertical timings, +/-hsync and +/-vsync; a leading `Modeline "name"` is
// skipped -- sets it, e.g. V 162 1600 1664 1856 2160 1200 1201 1204 1250 +hsync +vsync.
static const char *skip(const char *s) { while (*s == ' ' || *s == '\t') s++; return s; }
static const char *word(const char *s, const char *w)   // s past w and a space, or 0
{
    while (*w) if ((*s | 0x20) != (*w | 0x20)) return 0; else s++, w++;
    return (*s == ' ' || *s == '\t' || !*s) ? s : 0;
}
static const char *num(const char *s, uint32_t *v, uint32_t scale)   // decimal; fraction if scale > 1
{
    uint32_t n = 0, f = 0, k = 1;
    int digits = 0;
    while (*s >= '0' && *s <= '9') { n = n * 10 + (uint32_t)(*s++ - '0'); digits++; }
    if (scale > 1 && *s == '.')
        for (s++; *s >= '0' && *s <= '9'; s++) if (k < scale) { f = f * 10 + (uint32_t)(*s - '0'); k *= 10; }
    if (!digits || (*s != ' ' && *s != '\t' && *s)) return 0;
    *v = n * scale + f * (scale / k);
    return s;
}
int video_cmd(const char *p)
{
    p = skip(p);
    if (!*p) { video_show(); return 0; }
    const char *q = word(p, "Modeline");
    if (q) p = skip(q);
    if (*p == '"') { for (p++; *p && *p != '"'; p++) ; if (*p) p++; p = skip(p); }
    struct vmode m = { 0, { 0 }, { 0 }, -1, -1 };
    p = num(p, &m.khz, 1000);
    for (int i = 0; p && i < 8; i++) p = num(skip(p), i < 4 ? &m.h[i] : &m.v[i - 4], 1);
    while (p && *(p = skip(p))) {
        int pos = *p == '+';
        if (*p != '+' && *p != '-') { p = 0; break; }
        if ((q = word(p + 1, "hsync"))) m.hpos = pos;
        else if ((q = word(p + 1, "vsync"))) m.vpos = pos;
        else { p = 0; break; }
        p = q;
    }
    if (!p || m.hpos < 0 || m.vpos < 0) {
        puts_("usage: V [<MHz> <h active> <h sync start> <h sync end> <h total> <v ...> +|-hsync +|-vsync]\n");
        return -1;
    }
    if (video_mode(&m)) return -1;
    video_show();
    return 0;
}

// The device tree a payload is started with says the mode the screen is in: its framebuffer node
// (compatible "simple-framebuffer") has its width, height and stride cells rewritten in place --
// the same sizes, so the tree's layout does not move -- and the kernel's simplefb draws what the
// scanout shows. A tree is the flattened one (magic d00dfeed); anything else is left alone.
static uint32_t fdt32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void fdt32w(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static int fdt_is(const uint8_t *s, const char *t) { while (*t) if (*s++ != (uint8_t)*t++) return 0; return *s == 0; }
static uint8_t *fdt_al(uint8_t *p) { return (uint8_t *)(((uint64_t)p + 3) & ~3ul); }
void video_dtb(uint64_t a)
{
    uint8_t *f = (uint8_t *)a;
    if (!on || fdt32(f) != 0xd00dfeedu) return;
    uint8_t *p = f + fdt32(f + 8), *end = p + fdt32(f + 36), *strs = f + fdt32(f + 12);
    uint8_t *w = 0, *h = 0, *st = 0;
    int fb = 0;
    while (p < end) {
        uint32_t tok = fdt32(p);
        p += 4;
        if (tok == 1) {                              // a node begins: its name, then its properties
            while (*p) p++;
            p = fdt_al(p + 1);
            fb = 0;  w = h = st = 0;
        } else if (tok == 3) {                       // a property: length, name, value
            uint32_t len = fdt32(p);
            const uint8_t *nm = strs + fdt32(p + 4);
            p += 8;
            if (fdt_is(nm, "compatible") && fdt_is(p, "simple-framebuffer")) fb = 1;
            else if (len == 4 && fdt_is(nm, "width"))  w = p;
            else if (len == 4 && fdt_is(nm, "height")) h = p;
            else if (len == 4 && fdt_is(nm, "stride")) st = p;
            p = fdt_al(p + len);
        } else if (tok == 2) {                       // the node ends (the framebuffer's has no children)
            if (fb && w && h && st) {
                fdt32w(w, (uint32_t)W);  fdt32w(h, (uint32_t)H);  fdt32w(st, (uint32_t)STRIDE);
                puts_("dtb: the framebuffer is ");  putdec((uint64_t)W);  putc_('x');  putdec((uint64_t)H);  putc_('\n');
                return;
            }
            fb = 0;
        } else if (tok != 4) break;                  // the end, or not a tree
    }
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
