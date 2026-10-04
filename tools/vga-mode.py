#!/usr/bin/env python3
"""Program vga_scanout for a video mode: the ROM-monitor commands and the matching device tree.

    tools/vga-mode.py 1024x768@60      # monitor commands, then the DTS lines
    tools/vga-mode.py --list

The pixel clock is vga_mmcm_inst's CLKOUT0 = VCO / divider, with the VCO fixed at 1000 MHz
(rk_xcku5p.v: ui_clk 333.33 MHz / 2 * 6). Only CLKOUT0's divider is rewritten over DRP: the
VCO never moves, so the lock and filter settings, which depend only on M, stay valid. An
integer divider cannot hit every VESA clock exactly; the refresh rate moves instead, and the
table says by how much.

The framebuffer is placed as simmerv's `--graphics WxH` places it (src/lib.rs
setup_framebuffer): W*H*2 bytes rounded up to a power of two, at the top of 2 GiB of DRAM,
aligned to that size. The DTS lines replace the 800x600 ones in workloads/ubuntu/ubuntu-nfs.dts.in,
and the DTB and initrd must load below the framebuffer (ubuntu-boot.sh: DTB_ADDR, INITRD_ADDR).

DRP layout (CLKOUT0, XAPP888's MMCME3/MMCME4 tables -- CHECK ON THE BOARD, untested so far):
  0x08 ClkReg1: [15:13] phase mux  [12] reserved  [11:6] high time  [5:0] low time
  0x09 ClkReg2: [15] reserved  [14:12] frac  [11] frac_en  [10] frac_wf_r  [9:8] mx
                [7] edge  [6] no_count  [5:0] delay time
XAPP888 read-modify-writes these, keeping the reserved bits; this writes them as the board
reads them back (ClkReg1 bit 12 set), and tools/vga-try.py does the read-modify-write.
"""
import argparse
import sys

VCO_MHZ = 1000.0
PAGE = 0x1000_5000
DRAM_TOP = 0x1_0000_0000

# name: (VESA pixel clock MHz, h active/sync start/sync end/total, v ..., hsync+, vsync+)
MODES = {
    '640x480@60':   (25.175, (640, 656, 752, 800),     (480, 490, 492, 525),   False, False),
    '800x600@56':   (36.0,   (800, 824, 896, 1024),    (600, 601, 603, 625),   True,  True),
    '800x600@60':   (40.0,   (800, 840, 968, 1056),    (600, 601, 605, 628),   True,  True),
    '800x600@72':   (50.0,   (800, 856, 976, 1040),    (600, 637, 643, 666),   True,  True),
    '1024x768@60':  (65.0,   (1024, 1048, 1184, 1344), (768, 771, 777, 806),   False, False),
    '1280x1024@60': (108.0,  (1280, 1328, 1440, 1688), (1024, 1025, 1028, 1066), True, True),
}


def divider(mhz):
    return max(1, min(128, round(VCO_MHZ / mhz)))


def clkout0_regs(d):
    if d == 1:
        return 0x1041, 0x0040                       # high = low = 1, no_count
    high, low = d // 2, d - d // 2
    # Bit 12 of ClkReg1 is reserved and reads back SET in this bitstream's MMCM (0x130d at /25,
    # read over DRP on the board, 2026-10-04): written blind, it must stay set. tools/vga-try.py
    # read-modify-writes instead.
    return 0x1000 | (high << 6) | low, (d & 1) << 7  # edge for an odd divider


def placement(w, h):
    size = max(4096, 1 << (w * h * 2 - 1).bit_length())
    return (DRAM_TOP - size) & ~(size - 1), size


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
    base, size = placement(w, h)
    reg1, reg2 = clkout0_regs(d)

    print(f'# {a.mode}: pixel clock {real:.3f} MHz (VCO {VCO_MHZ:.0f} / {d}), {refresh:.2f} Hz;'
          f' framebuffer {size // 1024} KiB at {base:#x}')
    print('# monitor commands, before X (ubuntu-boot.sh sends these with VGA=<mode>):')
    cmds = [(0x00, 0)]                                        # disable while the mode changes
    cmds += [(0x44, 1)]                                       # hold the MMCM in reset
    cmds += [(0x40, (1 << 31) | (0x08 << 16) | reg1), (0x40, (1 << 31) | (0x09 << 16) | reg2)]
    cmds += [(0x44, 0)]                                       # release; wait for STATUS[0]
    cmds += [(0x04, base), (0x08, stride)]
    cmds += [(0x10 + 4 * i, x) for i, x in enumerate(hs)] + [(0x20 + 4 * i, x) for i, x in enumerate(vs)]
    cmds += [(0x00, 1 | (hpos << 1) | (vpos << 2))]
    for off, val in cmds:
        print(f'WW{PAGE + off:08x} {val:x}')
    print(f'# then R{PAGE + 0x30:08x}: bit 0 (pixel clock locked) must be 1')
    print()
    print('# device tree (ubuntu-nfs.dts.in):')
    print(f'/memreserve/ {base:#x} {size:#x};')
    print(f'\tframebuffer@{base:x} {{\n\t\tcompatible = "simple-framebuffer";\n'
          f'\t\treg = <0 {base:#x} 0 {size:#x}>;\n\t\twidth = <{w}>;\n\t\theight = <{h}>;\n'
          f'\t\tstride = <{stride}>;\n\t\tformat = "r5g6b5";\n\t}};')
    print(f'# simmerv: --graphics {w}x{h}')


if __name__ == '__main__':
    main()
