#!/usr/bin/env python3
"""B6: tiny128-blk.cpio = tiny128.cpio (zstd newc) with /etc/init.d/S99blkcheck added as ONE
archive: the base is decompressed, its TRAILER dropped, the script appended as a newc entry (mode
0755, root), a new TRAILER written, and the whole recompressed. One archive, not a concatenation,
so nothing rests on how the kernel treats a second compressed segment."""
import subprocess, sys
base, script, out = sys.argv[1:4]
d = subprocess.run(['zstd', '-dc', base], capture_output=True, check=True).stdout
def hdr(name, size, mode, ino):
    f = [0x070701, ino, mode, 0, 0, 1, 0, size, 0, 0, 0, 0, len(name) + 1, 0]
    return ('070701' + ''.join(f'{x:08X}' for x in f[1:])).encode() + name.encode() + b'\0'
def pad(b): return b + b'\0' * (-len(b) % 4)
# walk to the TRAILER
off = 0; ino_max = 0
while True:
    assert d[off:off + 6] == b'070701', (off, d[off:off + 6])
    fld = lambda i: int(d[off + 6 + 8 * i: off + 14 + 8 * i], 16)
    fsz, nsz = fld(6), fld(11); name = d[off + 110: off + 110 + nsz - 1].decode()
    if name == 'TRAILER!!!': break
    ino_max = max(ino_max, fld(0)); off += 110 + nsz; off = (off + 3) & ~3; off += fsz; off = (off + 3) & ~3
body = open(script, 'rb').read()
entry = pad(hdr('etc/init.d/S99blkcheck', len(body), 0o100755, ino_max + 1)) + pad(body)
trailer = pad(hdr('TRAILER!!!', 0, 0, 0))
merged = d[:off] + entry + trailer
open(out, 'wb').write(subprocess.run(['zstd', '-q', '-19', '-c'], input=merged, capture_output=True, check=True).stdout)
print(f'{out}: {len(merged)} bytes uncompressed, S99blkcheck added ({len(body)} bytes)')
