#!/usr/bin/env python3
# Convert a flat binary into 512-bit (64-byte) line hex for $readmemh into a
# `reg [511:0] lmem [...]` array (probe soc_top boot SRAM). Byte j of a line sits
# at bits [j*8 +: 8], so byte 0 is the LSB => emit each 64-byte chunk MSB-first
# (chunk reversed), one 128-hex-digit word per line. Mirrors evenodd.py's
# per-word byte-reversal, widened from 8 to 64 bytes.
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <binary_filename>", file=sys.stderr)
    sys.exit(1)

with open(sys.argv[1], 'rb') as f:
    data = f.read()

for i in range(0, len(data), 64):
    chunk = data[i:i+64]
    if len(chunk) < 64:
        chunk = chunk.ljust(64, b'\x00')
    print(chunk[::-1].hex())
