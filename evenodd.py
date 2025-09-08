#!/usr/bin/env python3
import sys

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <binary_filename> <parity>")
    print("parity: '0' or '1'")
    sys.exit(1)

filename = sys.argv[1]
parity = sys.argv[2]

if parity not in ('0', '1'):
    print("Error: parity must be '0' or '1'")
    sys.exit(1)

parity_int = int(parity)

with open(filename, 'rb') as f:
    data = f.read()

for i in range(0, len(data), 8):
    word_index = i // 8
    if word_index % 2 == parity_int:
        word_bytes = data[i:i+8]
        if len(word_bytes) < 8:
            word_bytes = word_bytes.ljust(8, b'\x00')
        hex_word = word_bytes[::-1].hex()
        print(hex_word)
