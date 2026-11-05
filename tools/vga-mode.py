#!/usr/bin/env python3
"""A video mode for vga_scanout: the ROM monitor's V command.

    tools/vga-mode.py 1024x768@60      # the V line
    tools/vga-mode.py --list

The pixel clock is vga_mmcm_inst's CLKOUT0 = VCO / divider, with the VCO fixed at 1000 MHz
(smolrv64_shell.v: ui_clk 333.33 MHz / 2 * 6), and the bitstream is timed at the divider it comes up
at, 6 (166.67 MHz). The monitor's V command takes an X11 modeline, picks the integer divider
nearest its clock and rewrites only CLKOUT0's divider over DRP (workloads/monitor/video.c). An
integer divider cannot hit every VESA clock exactly; the refresh rate moves instead, and the
table says by how much.

The framebuffer is the base of a fixed 16 MiB region at the top of 2 GiB of DRAM (0xFF00_0000),
whatever the mode: room for any mode up to 4K30 at 16 bpp. The device trees reserve the region
(workloads/ubuntu/ubuntu-nfs.dts.in), and the monitor writes the mode it is in into the tree's
framebuffer node when it starts a payload, so a V line before the boot is all a mode takes.
(simmerv's `--graphics WxH` places its own framebuffer and describes it in its own device tree.)
"""
import argparse
import sys

VCO_MHZ = 1000.0
FB_SIZE = 16 << 20                       # the framebuffer region, at 0xFF00_0000

# name: (VESA pixel clock MHz, h active/sync start/sync end/total, v ..., hsync+, vsync+)
MODES = {
    '640x480@60':   (25.175, (640, 656, 752, 800),     (480, 490, 492, 525),   False, False),
    '800x600@56':   (36.0,   (800, 824, 896, 1024),    (600, 601, 603, 625),   True,  True),
    '800x600@60':   (40.0,   (800, 840, 968, 1056),    (600, 601, 605, 628),   True,  True),
    '800x600@72':   (50.0,   (800, 856, 976, 1040),    (600, 637, 643, 666),   True,  True),
    '1024x768@60':  (65.0,   (1024, 1048, 1184, 1344), (768, 771, 777, 806),   False, False),
    '1280x1024@60': (108.0,  (1280, 1328, 1440, 1688), (1024, 1025, 1028, 1066), True, True),
    # VESA's 1600x1200@60 timing (162 MHz) at the fastest clock, 166.67 MHz: 61.7 Hz
    '1600x1200@60': (162.0,  (1600, 1664, 1856, 2160), (1200, 1201, 1204, 1250), True, True),
    # at the fastest clock, 52.0 Hz: the BenQ Q24W5 on the TinyVGA adapter takes it (Tommy, 2026-10-25)
    '1920x1200@52': (166.67, (1920, 2032, 2232, 2560), (1200, 1203, 1213, 1252), False, True),
}


def divider(mhz):
    return round(VCO_MHZ / mhz)


def describe(name):
    mhz, hs, vs, hpos, vpos = MODES[name]
    d = divider(mhz)
    real = VCO_MHZ / d
    refresh = real * 1e6 / (hs[3] * vs[3])
    return d, real, refresh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', nargs='?', help='e.g. 800x600@60')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list or not a.mode:
        for name in MODES:
            d, real, refresh = describe(name)
            print(f'{name:14} VESA {MODES[name][0]:7.3f} MHz -> /{d:<3} = {real:7.3f} MHz, {refresh:5.1f} Hz')
        return
    if a.mode not in MODES:
        sys.exit(f'unknown mode {a.mode}; --list shows them')
    mhz, hs, vs, hpos, vpos = MODES[a.mode]
    d, real, refresh = describe(a.mode)
    w, h = hs[0], vs[0]
    stride = w * 2
    if stride % 64:
        sys.exit(f'{a.mode}: stride {stride} is not a multiple of 64 bytes')
    if w * h * 2 > FB_SIZE:
        sys.exit(f'{a.mode}: {w * h * 2} bytes do not fit the {FB_SIZE >> 20} MiB framebuffer region')

    print(f'# {a.mode}: pixel clock {real:.3f} MHz (VCO {VCO_MHZ:.0f} / {d}), {refresh:.2f} Hz,'
          f' {w * h * 2 // 1024} KiB of the framebuffer region')
    print('# the monitor command, before X (ubuntu-boot.sh sends it with VGA=<mode>):')
    sync = lambda pos, n: ('+' if pos else '-') + n
    print('V', f'{mhz:g}', *hs, *vs, sync(hpos, 'hsync'), sync(vpos, 'vsync'))
    print(f'# simmerv: --graphics {w}x{h}')


if __name__ == '__main__':
    main()
