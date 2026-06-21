#!/usr/bin/env python3
# Emit a smolrv64-monitor command script that loads a flat binary into DRAM and
# runs it. XMODEM upload is broken, so we use W (64-bit write) commands. The
# region is zeroed first (Z) so only non-zero dwords need a W line, then X jumps.
#
#   ./gen_w.py virtio_coh.bin [0x80000000] > load.mon
#
# Paste the output into the monitor prompt (it parses bare hex, no 0x prefix).
import struct
import sys

path = sys.argv[1]
base = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x80000000

data = open(path, "rb").read()
if len(data) % 8:
    data += b"\x00" * (8 - len(data) % 8)

out = [f"Z{base:x} {len(data):x} 0"]
for off in range(0, len(data), 8):
    (val,) = struct.unpack_from("<Q", data, off)
    if val:
        out.append(f"W{base + off:x} {val:x}")
out.append(f"X{base:x}")

sys.stdout.write("\n".join(out) + "\n")
sys.stderr.write(f"{len(data)} bytes, {len(out) - 2} W lines\n")
