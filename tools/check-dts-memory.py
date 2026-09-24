#!/usr/bin/env python3
"""Verify that a board DTS describes no memory the core would fault on.

ooo2_core faults every physical address at or above its DRAM top, 0x8000_0000 + 2^DRAM_LG2
(THE PHYSICAL-ADDRESS CAP). A memory node reaching past that top hands Linux pages the core
refuses, so each node must lie inside [0x8000_0000, top). DRAM_LG2 is read from the RTL's
board branch, so the cap has one home.

    tools/check-dts-memory.py workloads/ubuntu/ubuntu-nfs.dts workloads/gb5/gb5-fpga.dts
"""
import pathlib, re, sys

CORE = pathlib.Path(__file__).resolve().parent.parent / 'ooo2' / 'ooo2_core.v'
BASE = 0x8000_0000


def board_dram_lg2():
    # the `else` arm of the COSIM_MEM_SIZE_LG2 conditional is the board's configuration
    m = re.search(r'`else\s*\n\s*localparam integer DRAM_LG2 = (\d+);', CORE.read_text())
    if not m:
        sys.exit(f'{CORE}: no board DRAM_LG2 found')
    return int(m.group(1))


def cells(text):
    return [int(x, 0) for x in text.split()]


def main(paths):
    top = BASE + (1 << board_dram_lg2())
    bad = 0
    for p in paths:
        text = pathlib.Path(p).read_text()
        nodes = re.findall(r'memory@[0-9a-fA-F]+\s*\{[^}]*?reg\s*=\s*<([^>]*)>', text)
        if not nodes:
            print(f'{p}: no memory node'); bad += 1; continue
        for reg in nodes:
            c = cells(reg)
            if len(c) != 4:
                print(f'{p}: memory reg <{reg}> is not <addr-hi addr-lo size-hi size-lo>'); bad += 1; continue
            lo = (c[0] << 32) | c[1]
            hi = lo + ((c[2] << 32) | c[3])
            if lo < BASE or hi > top:
                print(f'{p}: memory [{lo:#x}, {hi:#x}) reaches outside the core\'s DRAM [{BASE:#x}, {top:#x})')
                bad += 1
    if bad:
        sys.exit(1)
    print(f'dts memory: within [{BASE:#x}, {top:#x}) in {len(paths)} file(s)')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
