#!/usr/bin/env python3
"""
mkhex.py - Generate sparse memory hex files for smolrv64 simulation

Usage: mkhex.py <base_name> <hex_offset>:<file> [<hex_offset>:<file> ...]

Generates <base_name>.even and <base_name>.odd for use with smolrv64-sim.
Each <hex_offset> is a hex byte offset within the simulated memory region.

Memory layout (each 16-byte block at offset B):
  mem0 (even file): bytes B+0 .. B+7
  mem1 (odd  file): bytes B+8 .. B+15
"""
import sys


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <base_name> <hex_offset>:<file> [...]")
        sys.exit(1)

    base_name = sys.argv[1]
    segments = []
    for arg in sys.argv[2:]:
        off_str, filename = arg.split(':', 1)
        offset = int(off_str, 16)
        with open(filename, 'rb') as f:
            data = f.read()
        segments.append((offset, data))

    for parity in (0, 1):
        suffix = 'even' if parity == 0 else 'odd'
        words = {}  # array_index -> 8-byte bytearray
        for byte_offset, data in segments:
            start_block = byte_offset // 16
            end_byte = byte_offset + len(data)
            end_block = (end_byte + 15) // 16
            for block in range(start_block, end_block):
                if parity == 0:
                    blk_start = block * 16
                else:
                    blk_start = block * 16 + 8
                word = bytearray(8)
                for i in range(8):
                    abs_pos = blk_start + i
                    if byte_offset <= abs_pos < end_byte:
                        word[i] = data[abs_pos - byte_offset]
                if any(word):
                    words[block] = word

        with open(f"{base_name}.{suffix}", 'w') as out:
            prev = None
            for addr in sorted(words.keys()):
                if prev is None or addr != prev + 1:
                    out.write(f"@{addr:08x}\n")
                out.write(words[addr][::-1].hex() + "\n")
                prev = addr


if __name__ == '__main__':
    main()
