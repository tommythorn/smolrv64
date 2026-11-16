#!/usr/bin/env python3
"""The boot SRAM (rv_soc_top.v lmem: 4096 lines of 512 bits) as the routed design's 57 RAMB36
INIT/INITP strings, for tools/monitor-eco.sh. Chain c = cells 4c..4c+3 holds line bits [36c+35:36c]
(32 in INIT, 4 in INITP), cell 4c+j the lines 1024j..1024j+1023; cell 56 (9 x 4K) holds bits
[511:504]. Vivado chose this packing; `check` proves it against a checkpoint before anything is
written, so a synthesis that packs differently is refused, never patched.

  lmem-init.py check <dump> <monitor.bin>   0 when the dump's INITs are exactly this image's
  lmem-init.py tcl <monitor.bin> <cell prefix>   the set_property lines that load the image"""
import sys, re
NL = 4096
def lines(binf):
    b = open(binf, 'rb').read(); b += bytes(-len(b) % 64)
    L = [int.from_bytes(b[i:i+64], 'little') for i in range(0, len(b), 64)]
    if len(L) > NL: sys.exit('%s: %d lines, the SRAM holds %d' % (binf, len(L), NL))
    return L + [0] * (NL - len(L))
def inits(L):
    out = {}
    for cell in range(57):
        init = [0] * 128; initp = [0] * 16
        if cell < 56:
            c, j = divmod(cell, 4)
            for a in range(1024):
                v = (L[1024 * j + a] >> (36 * c)) & ((1 << 36) - 1)
                d, p = v & 0xffffffff, v >> 32
                init[a // 8] |= d << (32 * (a % 8))
                initp[a // 64] |= p << (4 * (a % 64))
        else:
            for a in range(4096):
                d = (L[a] >> 504) & 0xff
                init[a // 32] |= d << (8 * (a % 32))
        for i in range(128): out[(cell, 'INIT_%02X' % i)] = init[i]
        for i in range(16): out[(cell, 'INITP_%02X' % i)] = initp[i]
    return out
def parse(dump):
    got, cell = {}, None
    for ln in open(dump):
        m = re.match(r'cell \S+lmem_reg_bram_(\d+)\b', ln)
        if m: cell = int(m.group(1)); continue
        m = re.match(r'\s+(INITP?_[0-9A-F]{2}) 256\'h([0-9A-F]+)', ln)
        if m: got[(cell, m.group(1))] = int(m.group(2), 16)
    return got
if __name__ == '__main__':
    if sys.argv[1] == "check":
        got, want = parse(sys.argv[2]), inits(lines(sys.argv[3]))
        bad = [k for k in want if got.get(k) != want[k]]
        print('%d INIT strings, %d differ' % (len(want), len(bad)), bad[:8])
        sys.exit(1 if bad else 0)
    if sys.argv[1] == "tcl":
        for (cell, k), v in sorted(inits(lines(sys.argv[2])).items()):
            print("set_property %s 256'h%064X [get_cells %s%d]" % (k, v, sys.argv[3], cell))
