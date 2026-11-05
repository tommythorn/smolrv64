// The memory test (`M`): DRAM written and read back through the D$, every line cleaned to DRAM
// and dropped from the D$ in between, so what is read back is what DRAM kept. The phases:
//
//   data lines     each of the 64 data bits alone set and alone clear, alternating over a line's
//                  eight beats
//   address lines  a value at every power-of-two offset from the base: an address bit stuck, or
//                  tied to another, makes two of those words one
//   own address    every word holds its own address, complemented on even passes
//   random data    a pseudo-random value per line, the lines in order: the full bandwidth
//   random order   the lines in a scattered order, so nearly every access opens a row in another
//                  bank: the activate and precharge timing rather than the bandwidth
//
// Each pass draws new values, and passes repeat until a key is pressed (seen between phases) or
// the count runs out: a soak. The monitor's own DDR, the devices' scratch and the framebuffer in
// [DMA_BASE, FB_BASE + 1 MiB), is never written: a range across it is tested on either side.
#include "mon.h"

#define LINE   64ul
#define DRAM_A 0x80000000ul                 // the DRAM window's base (src/dram_window.vh)
#define HOLE_A DMA_BASE
#define HOLE_E (FB_BASE + 0x100000ul)
#define GOLD   0x9e3779b97f4a7c15ul         // word w of a line holds the line's value + w x GOLD

static uint64_t nerr, shown;

static void bad(const char *what, uint64_t a, uint64_t want, uint64_t got)
{
    if (shown++ < 8) {
        puts_("    "); puts_(what); putc_(' '); puthex64(a);
        puts_(": wrote "); puthex64(want); puts_(", read "); puthex64(got);
        puts_(" (xor "); puthex64(want ^ got); puts_(")\n");
    }
    nerr++;
}

// One word through DRAM: its line is flushed after the access, so the next access misses. The
// memory port serves a waiting read before a waiting write, so a read of another address can
// overtake a write -- and reach the same DRAM word when an address line is stuck, or past a
// 2 GiB board's top. put() therefore reads its word back: a miss on a line still being written
// back waits for DRAM to acknowledge the write, so every later access comes after it.
static uint64_t get(uint64_t a) { uint64_t v = *(volatile uint64_t *)a; cbo(a, 8, 2); return v; }
static void put(uint64_t a, uint64_t v) { *(volatile uint64_t *)a = v; cbo(a, 8, 2); get(a); }

static int hole(uint64_t a) { return a >= HOLE_A && a < HOLE_E; }

static uint64_t mix(uint64_t z)             // splitmix64's finalizer
{
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ul;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebul;
    return z ^ (z >> 31);
}

static uint64_t xs(uint64_t x)              // xorshift64, never 0 from a non-zero x
{
    x ^= x << 13;
    x ^= x >> 7;
    return x ^ (x << 17);
}

static void data_lines(uint64_t a)
{
    volatile uint64_t *l = (volatile uint64_t *)a;
    for (int b = 0; b < 64; b++) {
        uint64_t v = 1ul << b;
        for (int w = 0; w < 8; w++) l[w] = w & 1 ? ~v : v;
        cbo(a, LINE, 2);
        for (int w = 0; w < 8; w++) {
            uint64_t want = w & 1 ? ~v : v, got = l[w];
            if (got != want) bad("data lines", a + 8 * w, want, got);
        }
        cbo(a, LINE, 2);
    }
}

static void address_lines(uint64_t a, uint64_t n)
{
    const uint64_t P = 0xaaaaaaaaaaaaaaaaul, Q = ~P;
    uint64_t o, t, v;
    for (o = 8; o < n; o <<= 1)
        if (!hole(a + o)) put(a + o, P);
    put(a, Q);
    for (o = 8; o < n; o <<= 1)                         // a bit stuck at 1 sends a to a + o
        if (!hole(a + o) && (v = get(a + o)) != P) bad("address lines", a + o, P, v);
    put(a, P);
    for (t = 8; t < n; t <<= 1) {
        if (hole(a + t)) continue;
        put(a + t, Q);
        if ((v = get(a)) != P) bad("address lines", a, P, v);   // one stuck at 0 sends a + t to a
        for (o = 8; o < n; o <<= 1)                     // two bits tied send a + t to a + o
            if (o != t && !hole(a + o) && (v = get(a + o)) != P) bad("address lines", a + o, P, v);
        put(a + t, P);
    }
}

static void own_write(uint64_t a, uint64_t e, uint64_t k)
{
    for (; a < e; a += LINE) {
        volatile uint64_t *l = (volatile uint64_t *)a;
        l[0] = a ^ k;        l[1] = (a + 8) ^ k;  l[2] = (a + 16) ^ k; l[3] = (a + 24) ^ k;
        l[4] = (a + 32) ^ k; l[5] = (a + 40) ^ k; l[6] = (a + 48) ^ k; l[7] = (a + 56) ^ k;
    }
}

static void own_check(uint64_t a, uint64_t e, uint64_t k)
{
    for (; a < e; a += 8) {
        uint64_t v = *(volatile uint64_t *)a;
        if (v != (a ^ k)) bad("own address", a, a ^ k, v);
    }
}

static void line_put(uint64_t a, uint64_t r)
{
    volatile uint64_t *l = (volatile uint64_t *)a;
    l[0] = r;            l[1] = r + GOLD;     l[2] = r + 2 * GOLD; l[3] = r + 3 * GOLD;
    l[4] = r + 4 * GOLD; l[5] = r + 5 * GOLD; l[6] = r + 6 * GOLD; l[7] = r + 7 * GOLD;
}

static void line_check(const char *what, uint64_t a, uint64_t r)
{
    for (int w = 0; w < 8; w++, a += 8, r += GOLD) {
        uint64_t v = *(volatile uint64_t *)a;
        if (v != r) bad(what, a, r, v);
    }
}

// The random data's lines in order, each the next value of one xorshift stream from r.
static uint64_t stream(uint64_t a, uint64_t e, uint64_t r, int check)
{
    for (; a < e; a += LINE) {
        r = xs(r);
        if (check) line_check("random data", a, r);
        else line_put(a, r);
    }
    return r;
}

// Every line of [a, e) once, `step` lines after the last (mod the line count), its value from
// its address. A prime step larger than the line count visits them all.
static void scatter(uint64_t a, uint64_t e, uint64_t seed, uint64_t step, int check)
{
    uint64_t n = (e - a) / LINE, j = 0;
    step %= n;
    for (uint64_t i = 0; i < n; i++) {
        uint64_t l = a + j * LINE;
        if (check) line_check("random order", l, mix(l ^ seed));
        else line_put(l, mix(l ^ seed));
        if ((j += step) >= n) j -= n;
    }
}

static void field(const char *s, int w)     // s, then spaces out to w columns
{
    for (; *s; s++, w--) putc_(*s);
    while (w-- > 0) putc_(' ');
}

static void mibs(uint64_t bytes, uint64_t ticks)
{
    putdec(ticks ? bytes * TIMEBASE_HZ / ticks >> 20 : 0);    // bytes x TIMEBASE_HZ < 2^52
    puts_(" MiB/s");
}

// The DRAM window is 4 GiB, but a 2 GiB board ignores the controller's top address bit: its
// upper 2 GiB repeats the lower, and a test there would overwrite the monitor's own DDR. The two
// words the probe writes get their values back.
static uint64_t dram_size(void)
{
    if (DRAM_SIZE) return DRAM_SIZE;
    uint64_t lo = get(DRAM_A), hi = get(DRAM_A + (2ul << 30)), size;
    put(DRAM_A, 0);
    put(DRAM_A + (2ul << 30), ~0ul);
    size = get(DRAM_A) ? 2ul << 30 : 4ul << 30;
    put(DRAM_A + (2ul << 30), hi);
    put(DRAM_A, lo);
    return size;
}

// `M [<addr> <len> [<passes>]]`: no range is all of DRAM, no count is until a key.
int memtest(uint64_t a, uint64_t n, uint64_t passes)
{
    static const char *const name[] = { "data lines", "address lines", "own address",
                                        "random data", "random order" };
    uint64_t pa[2], pe[2], e, total = 0, pass, size = dram_size();
    int np = 0, stop = 0;

    if (!n) { a = DRAM_A; n = size; }
    e = (a + n) & ~(LINE - 1);
    a = (a + LINE - 1) & ~(LINE - 1);
    if (a < DRAM_A || e > DRAM_A + size || e <= a) {
        puts_("M: "); puthex64(a); puts_(".."); puthex64(e); puts_(" is not a range in this board's DRAM ");
        puthex64(DRAM_A); puts_(".."); puthex64(DRAM_A + size); putc_('\n');
        return -1;
    }
    if (hole(a)) {
        puts_("M: "); puthex64(a); puts_(" is in the monitor's own DDR "); puthex64(HOLE_A);
        puts_(".."); puthex64(HOLE_E); putc_('\n');
        return -1;
    }
    if (a < HOLE_A) { pa[np] = a; pe[np++] = e < HOLE_A ? e : HOLE_A; }
    if (e > HOLE_E) { pa[np] = a > HOLE_E ? a : HOLE_E; pe[np++] = e; }

    puts_("memory test, "); putdec(size >> 30); puts_(" GiB of DRAM:");
    for (int i = 0; i < np; i++) { putc_(' '); puthex64(pa[i]); puts_(".."); puthex64(pe[i]); }
    puts_(passes ? "" : ", until a key"); putc_('\n');

    for (pass = 1; !stop && (!passes || pass <= passes); pass++) {
        uint64_t seed = mix(pass), k = pass & 1 ? 0 : ~0ul, t = now(), r;
        nerr = 0;
        puts_("pass "); putdec(pass); putc_('\n');
        for (int ph = 0; ph < 5 && !stop; ph++) {
            uint64_t t0 = now(), t1 = t0, bytes = 0, before = nerr;
            shown = 0;
            if (ph == 0) data_lines(pa[0]);
            else if (ph == 1) address_lines(a, e - a);
            else {
                r = seed | 1;
                for (int i = 0; i < np; i++) {
                    if (ph == 2) own_write(pa[i], pe[i], k);
                    else if (ph == 3) r = stream(pa[i], pe[i], r, 0);
                    else scatter(pa[i], pe[i], seed, 2654435761ul, 0);
                    cbo(pa[i], pe[i] - pa[i], 2);
                    bytes += pe[i] - pa[i];
                }
                t1 = now();
                r = seed | 1;
                for (int i = 0; i < np; i++) {
                    if (ph == 2) own_check(pa[i], pe[i], k);
                    else if (ph == 3) r = stream(pa[i], pe[i], r, 1);
                    else scatter(pa[i], pe[i], seed, 2246822519ul, 1);
                }
            }
            puts_("  "); field(name[ph], 15);
            if (nerr == before) puts_("ok");
            else { putdec(nerr - before); puts_(" errors"); }
            if (bytes) {
                puts_(", write "); mibs(bytes, t1 - t0);
                puts_(", read "); mibs(bytes, now() - t1);
            }
            putc_('\n');
            stop = con_key();
        }
        total += nerr;
        puts_("pass "); putdec(pass); puts_(": "); putdec(nerr); puts_(" errors, ");
        putdec((now() - t) / TIMEBASE_HZ); puts_(" s");
        if ((uint32_t)ERRLOG_BASE[0] == ERRLOG_MAGIC && ERRLOG_BASE[1]) {
            puts_(", integrity log sticky "); puthex64(ERRLOG_BASE[1]);
        }
        putc_('\n');
    }
    puts_("memory test: "); putdec(total); puts_(" errors\n");
    return total ? -1 : 0;
}
