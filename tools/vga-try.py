#!/usr/bin/env python3
"""Try a video mode on the board's VGA scanout from the ROM monitor, with a test pattern.

    tools/vga-try.py 'Modeline "960x600_60.00" 45.25 960 992 1088 1216 600 603 609 624 -hsync +vsync'
    tools/vga-try.py 45.25 960 992 1088 1216 600 603 609 624 -hsync +vsync   # the same, unquoted
    tools/vga-try.py --cvt 960x600          # whatever `cvt 960 600 60` says
    tools/vga-try.py --cvt-r 960x600@60     # cvt's reduced-blanking mode
    tools/vga-try.py --pattern bars ...     # card (default), bars, grid, checker, white, black
    tools/vga-try.py --dry-run ...          # print what would be sent; touch nothing
    tools/vga-try.py --off                  # scanout off

The board must be at the ROM monitor's `>` prompt (just programmed, or key[1] pressed), with
the serial console in a `screen -L` session, as for ubuntu-boot.sh: SESSION (default `board`)
and LOG (default ~/smolrv64/workloads/ubuntu/screenlog.0). The script

  1. turns the scanout off and holds the pixel-clock MMCM in reset;
  2. sets the pixel clock: CLKOUT0 = 1000 MHz / D for the integer D nearest the modeline's
     clock. The divider is read back over DRP and only its fields are rewritten (XAPP888's
     read-modify-write), so the VCO -- and its lock and filter settings -- never move;
  3. programs the timings, polarities, framebuffer base and stride, and enables the scanout;
  4. uploads a test pattern straight into the framebuffer over XMODEM (~4 s for 960x600);
  5. reads STATUS back: pixel clock locked, and lines fetched late (the DRAM bandwidth check).

The pattern's outermost pixels are a 1-pixel white border, so a monitor's auto-adjust has edges
to lock onto and any cropping is plain to see. The framebuffer goes where simmerv's
`--graphics WxH` would put it, and the script prints the matching DTS lines, so a mode that
looks right can be booted (ubuntu-nfs.dts.in, then `VGA=1` after setting the mode).

Limits: the width must be a multiple of 32 (the stride, W*2, a multiple of 64), at most 2048;
totals at most 4095. The pixel clock is 1000/D MHz, so the refresh rate moves a little from
the modeline's (printed). The bitstream's pixel-clock logic was timed at 40 MHz; much faster
clocks are untested, as is any clock change on the board at all (the DRP path, 2026-10-04).
"""
import argparse
import os
import re
import struct
import subprocess
import sys
import tempfile
import time

VCO_MHZ = 1000.0
PAGE = 0x1000_5000
DRAM_TOP = 0x1_0000_0000
CTRL, FB_BASE, STRIDE, H0, V0, STATUS, DRP, PIXRST = 0x00, 0x04, 0x08, 0x10, 0x20, 0x30, 0x40, 0x44


def parse_modeline(words):
    """(MHz, (ha, hss, hse, ht), (va, vss, vse, vt), hsync_pos, vsync_pos)"""
    text = ' '.join(words)
    text = re.sub(r'^\s*Modeline\s*', '', text, flags=re.I)
    text = re.sub(r'^\s*"[^"]*"\s*', '', text)
    toks = text.split()
    try:
        nums = [float(toks[0])] + [int(t) for t in toks[1:9]]
    except (ValueError, IndexError):
        sys.exit(f'not a modeline: {text!r}\n  want: <MHz> ha hss hse ht va vss vse vt [+-hsync] [+-vsync]')
    flags = ' '.join(toks[9:]).lower()
    hpos = '+hsync' in flags or '-hsync' not in flags
    vpos = '+vsync' in flags or '-vsync' not in flags
    return nums[0], tuple(nums[1:5]), tuple(nums[5:9]), hpos, vpos


def cvt(spec, reduced):
    m = re.fullmatch(r'(\d+)x(\d+)(?:@([\d.]+))?', spec)
    if not m:
        sys.exit(f'--cvt wants WxH[@Hz], got {spec!r}')
    args = ['cvt'] + (['-r'] if reduced else []) + [m.group(1), m.group(2), m.group(3) or '60']
    out = subprocess.run(args, capture_output=True, text=True, check=True).stdout
    line = [l for l in out.splitlines() if l.startswith('Modeline')][0]
    print(f'# {line}')
    return parse_modeline([line])


def check(mhz, hs, vs):
    w, h = hs[0], vs[0]
    bad = []
    if w % 32:
        bad.append(f'width {w} is not a multiple of 32 (stride {w * 2} must be a multiple of 64)')
    if w > 2048:
        bad.append(f'width {w} exceeds the 2048-pixel line buffer')
    for name, t in (('horizontal', hs), ('vertical', vs)):
        if not (t[0] <= t[1] <= t[2] <= t[3]) or t[3] > 4095:
            bad.append(f'{name} timings {t} are not active <= sync start <= sync end <= total <= 4095')
    if bad:
        sys.exit('\n'.join(bad))
    d = max(1, min(128, round(VCO_MHZ / mhz)))
    return d


def placement(w, h):
    size = max(4096, 1 << (w * h * 2 - 1).bit_length())
    return (DRAM_TOP - size) & ~(size - 1), size


# ----------------------------------------------------------------- the test pattern (RGB565)
def rgb(r, g, b):
    return (r >> 3) << 11 | (g >> 2) << 5 | b >> 3


WHITE, BLACK = rgb(255, 255, 255), 0
BARS = [rgb(255, 255, 255), rgb(255, 255, 0), rgb(0, 255, 255), rgb(0, 255, 0),
        rgb(255, 0, 255), rgb(255, 0, 0), rgb(0, 0, 255), rgb(0, 0, 0)]


def pattern(kind, w, h):
    px = [BLACK] * (w * h)

    def put(x, y, c):
        if 0 <= x < w and 0 <= y < h:
            px[y * w + x] = c

    if kind == 'white':
        px = [WHITE] * (w * h)
    elif kind == 'bars':
        for y in range(h):
            for x in range(w):
                px[y * w + x] = BARS[x * 8 // w]
    elif kind == 'checker':      # 1-pixel checkerboard: the sharpest thing a scaler can show
        for y in range(h):
            for x in range(w):
                px[y * w + x] = WHITE if (x ^ y) & 1 else BLACK
    elif kind in ('grid', 'card'):
        grey = rgb(96, 96, 96)
        for y in range(h):
            for x in range(w):
                if x % 32 == 0 or y % 32 == 0:
                    px[y * w + x] = grey
        if kind == 'card':
            # Colour bars across the middle third, a 1-pixel vertical-line block and a
            # checkerboard block for sharpness, and four 4-level ramps (RGB222 shows 4).
            y0, y1 = h // 3, h // 2
            for y in range(y0, y1):
                for x in range(w // 8, w - w // 8):
                    px[y * w + x] = BARS[(x - w // 8) * 8 // (w - w // 4)]
            for y in range(y1 + 8, y1 + 8 + h // 8):
                for x in range(w // 8, w // 2 - 8):
                    px[y * w + x] = WHITE if x & 1 else BLACK
                for x in range(w // 2 + 8, w - w // 8):
                    px[y * w + x] = WHITE if (x ^ y) & 1 else BLACK
            ry = y1 + 16 + h // 8
            for i, (cr, cg, cb) in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 1))):
                for y in range(ry + i * (h // 32), ry + (i + 1) * (h // 32) - 2):
                    for x in range(w // 8, w - w // 8):
                        lvl = (x - w // 8) * 256 // (w - w // 4)
                        px[y * w + x] = rgb(cr * lvl, cg * lvl, cb * lvl)
            # A crosshair at the centre, and an arrow in each corner pointing outwards.
            for x in range(w // 2 - 20, w // 2 + 21):
                put(x, h // 2 - 1, WHITE)
            for y in range(h // 2 - 21, h // 2 + 20):
                put(w // 2, y, WHITE)
            for k in range(2, 24):
                for (cx, cy, sx, sy) in ((0, 0, 1, 1), (w - 1, 0, -1, 1), (0, h - 1, 1, -1), (w - 1, h - 1, -1, -1)):
                    put(cx + sx * k, cy + sy * k, rgb(255, 0, 0))
                    put(cx + sx * k, cy + sy * 2, rgb(255, 0, 0))
                    put(cx + sx * 2, cy + sy * k, rgb(255, 0, 0))
    # Always: the outermost pixels white, so cropping is visible and auto-adjust has edges.
    for x in range(w):
        put(x, 0, WHITE)
        put(x, h - 1, WHITE)
    for y in range(h):
        put(0, y, WHITE)
        put(w - 1, y, WHITE)
    return struct.pack(f'<{w * h}H', *px)


# ----------------------------------------------------------------- the serial console
class Monitor:
    def __init__(self, session, log, dry):
        self.session, self.log, self.dry = session, log, dry
        if dry:
            return
        if not os.path.exists(log):
            sys.exit(f'{log} not found: start the console with `screen -L` (see ubuntu-boot.sh)')
        subprocess.run(['screen', '-S', session, '-X', 'logfile', 'flush', '1'], check=True)

    def mark(self):
        return 0 if self.dry else os.path.getsize(self.log)

    def since(self, pos):
        with open(self.log, 'rb') as f:
            f.seek(pos)
            return f.read().decode(errors='replace').replace('\r', '')

    def send(self, line):
        if self.dry:
            print(line)
            return
        for ch in line + '\r':                # a character at a time, as ubuntu-boot.sh does
            subprocess.run(['screen', '-S', self.session, '-X', 'stuff', ch], check=True)
            time.sleep(0.02)

    def wait_for(self, pos, pattern, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            m = re.search(pattern, self.since(pos))
            if m:
                return m
            time.sleep(0.2)
        return None

    def ww(self, off, val):
        self.send(f'WW{PAGE + off:08x} {val & 0xffffffff:x}')

    def rd(self, off):
        if self.dry:
            self.send(f'R{PAGE + off:08x}')
            return 0
        pos = self.mark()
        self.send(f'R{PAGE + off:08x}')
        m = self.wait_for(pos, rf'{PAGE + off:016x}: ([0-9a-fA-F]{{16}})')
        if not m:
            sys.exit(f'no answer to R{PAGE + off:08x}: is the board at the monitor prompt?')
        return int(m.group(1), 16) & 0xffffffff

    def at_prompt(self):
        if self.dry:
            return True
        pos = self.mark()
        self.send('')
        return self.wait_for(pos, r'(^|\n)>\s*$', 3.0) is not None

    def upload(self, addr, path):
        if self.dry:
            print(f'Y{addr:08x}   + sx -k {path}')
            return True
        pos = self.mark()
        self.send(f'Y{addr:08x}')
        time.sleep(1)
        subprocess.run(['screen', '-S', self.session, '-X', 'exec', '!!', 'sx', '-k', os.path.realpath(path)], check=True)
        return self.wait_for(pos, r'Transfer complete', 120.0) is not None


def drp(mon, reg, val=None):
    """One DRP access to the pixel-clock MMCM: a read, or a write of val."""
    if val is None:
        mon.ww(DRP, reg << 16)
        return mon.rd(DRP) & 0xffff
    mon.ww(DRP, 1 << 31 | reg << 16 | val)
    return val


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('modeline', nargs='*')
    ap.add_argument('--cvt', metavar='WxH[@Hz]')
    ap.add_argument('--cvt-r', metavar='WxH[@Hz]')
    ap.add_argument('--pattern', default='card', choices=['card', 'bars', 'grid', 'checker', 'white', 'black'])
    ap.add_argument('--off', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--save-pattern', metavar='FILE', help='also write the pattern (raw RGB565) here')
    a = ap.parse_args()
    mon = Monitor(os.environ.get('SESSION', 'board'),
                  os.environ.get('LOG', os.path.expanduser('~/smolrv64/workloads/ubuntu/screenlog.0')),
                  a.dry_run)
    if a.off:
        mon.ww(CTRL, 0)
        return
    if a.cvt or a.cvt_r:
        mhz, hs, vs, hpos, vpos = cvt(a.cvt or a.cvt_r, bool(a.cvt_r))
    elif a.modeline:
        mhz, hs, vs, hpos, vpos = parse_modeline(a.modeline)
    else:
        ap.error('give a modeline, --cvt or --cvt-r')
    d = check(mhz, hs, vs)
    real = VCO_MHZ / d
    w, h = hs[0], vs[0]
    base, size = placement(w, h)
    print(f'{w}x{h}: pixel clock {real:.3f} MHz (1000 / {d}; the modeline asks {mhz:.3f}, '
          f'{(real / mhz - 1) * 100:+.1f}%), {real * 1e6 / hs[3] / 1e3:.2f} kHz, '
          f'{real * 1e6 / (hs[3] * vs[3]):.2f} Hz; {"+" if hpos else "-"}hsync {"+" if vpos else "-"}vsync')
    print(f'framebuffer {size // 1024} KiB at {base:#x}, stride {w * 2}')
    if real > 40.5:
        print(f'note: {real:.1f} MHz is above the 40 MHz the bitstream\'s pixel-clock logic was timed at')
    if not mon.at_prompt():
        sys.exit('the board is not at the monitor\'s ">" prompt (press key[1], or reprogram)')

    # 1-2: off, MMCM in reset, CLKOUT0's divider read-modify-written, MMCM out of reset.
    mon.ww(CTRL, 0)
    mon.ww(PIXRST, 1)
    r1, r2 = drp(mon, 0x08), drp(mon, 0x09)
    high, low = (1, 1) if d == 1 else (d // 2, d - d // 2)
    n1 = (r1 & 0xf000) | high << 6 | low                                 # keep phase mux, reserved
    n2 = (r2 & 0xff3f) | (d & 1 and d > 1) << 7 | (d == 1) << 6           # edge, no_count
    if not a.dry_run:
        print(f'DRP CLKOUT0: reg 0x08 {r1:#06x} -> {n1:#06x}, reg 0x09 {r2:#06x} -> {n2:#06x}')
    drp(mon, 0x08, n1)
    drp(mon, 0x09, n2)
    mon.ww(PIXRST, 0)

    # 3: the mode, then on.
    mon.ww(FB_BASE, base)
    mon.ww(STRIDE, w * 2)
    for i, v in enumerate(hs):
        mon.ww(H0 + 4 * i, v)
    for i, v in enumerate(vs):
        mon.ww(V0 + 4 * i, v)
    mon.ww(CTRL, 1 | hpos << 1 | vpos << 2)

    # 4: the pattern.
    data = pattern(a.pattern, w, h)
    if a.save_pattern:
        open(a.save_pattern, 'wb').write(data)
    with tempfile.NamedTemporaryFile(suffix='.rgb565', delete=False) as f:
        f.write(data)
    print(f'uploading the {a.pattern} pattern, {len(data)} bytes ...')
    if not mon.upload(base, f.name):
        sys.exit('the XMODEM upload did not complete')
    os.unlink(f.name)

    # 5: what the hardware says.
    st = mon.rd(STATUS)
    time.sleep(1.5)
    st2 = mon.rd(STATUS)
    if not a.dry_run:
        print(f'pixel clock {"LOCKED" if st & 1 else "NOT LOCKED"}; lines fetched late: '
              f'{st2 >> 16} (+{(st2 >> 16) - (st >> 16)} in the last ~2 s)')
    print('\n# to boot Linux in this mode -- ubuntu-nfs.dts.in, and simmerv --graphics '
          f'{w}x{h}:\n/memreserve/ {base:#x} {size:#x};')
    print(f'\tframebuffer@{base:x} {{ compatible = "simple-framebuffer"; reg = <0 {base:#x} 0 {size:#x}>; '
          f'width = <{w}>; height = <{h}>; stride = <{w * 2}>; format = "r5g6b5"; }};')


if __name__ == '__main__':
    main()
