#!/usr/bin/env python3
"""Build the SD autoboot simulation's disk image: a GPT whose EFI System Partition (entry 15,
as on the Ubuntu cloud image the board's card carries) is FAT32 with 512-byte clusters, holding
/smoltest/boot.txt, a payload and a data file under long names, the data file fragmented.

   mkimage.py <out.img> <hello.bin> <data-bytes>

Needs sgdisk, mkfs.vfat and the pyfatfs module (a virtualenv: pip install pyfatfs 'setuptools<70').
"""
import os, subprocess, sys, tempfile, hashlib, random
import fs
from pyfatfs.PyFatFS import PyFatFS

out, hello, nbytes = sys.argv[1], sys.argv[2], int(sys.argv[3])
SEC, PART_LBA, PART_SECTORS = 512, 34816, 120 * 2048       # the ESP: 60 MiB at 17 MiB
size = (PART_LBA + PART_SECTORS + 2048) * SEC
with open(out, "wb") as f:
    f.truncate(size)
subprocess.run(["sgdisk", "-Z", out], check=True, capture_output=True)
subprocess.run(["sgdisk", "-n", "1:2048:+16M", "-t", "1:8300",
                "-n", f"15:{PART_LBA}:+{PART_SECTORS - 1}", "-t", "15:EF00", out],
               check=True, capture_output=True)
with tempfile.TemporaryDirectory() as d:
    part = os.path.join(d, "esp.img")
    with open(part, "wb") as f:
        f.truncate(PART_SECTORS * SEC)
    subprocess.run(["mkfs.vfat", "-F", "32", "-S", "512", "-s", "1", "-n", "UEFI", part],
                   check=True, capture_output=True)
    rnd = random.Random(20261005)
    data = bytes(rnd.getrandbits(8) for _ in range(nbytes))
    prog = open(hello, "rb").read()
    v = PyFatFS(part)
    v.makedirs("/smoltest/sub dir")
    v.writebytes("/smoltest/spacer-a.bin", b"a" * 20000)        # its clusters come free below
    with v.openbin("/smoltest/sub dir/data with a long name.bin", "w") as f:
        f.write(data[: nbytes // 2])
    v.writebytes("/smoltest/spacer-b.bin", b"b" * 3000)         # pins the run's end
    v.remove("/smoltest/spacer-a.bin")
    with v.openbin("/smoltest/sub dir/data with a long name.bin", "a") as f:
        f.write(data[nbytes // 2:])                             # continues in spacer-a's clusters
    v.writebytes("/smoltest/Hello World Program.bin", prog)
    boot = ("# the SD autoboot simulation\n"
            f"L9c000000 /smoltest/sub dir/data with a long name.bin\n"
            f"C9c000000 {nbytes:x}\n"
            "\n"
            f"L9d000000 /SMOLTEST/hello world program.BIN\n"
            f"C9d000000 {len(prog):x}\n"
            "D /smoltest\n"
            "X9d000000\n")
    v.writetext("/smoltest/boot.txt", boot)
    v.close()
    with open(out, "r+b") as f, open(part, "rb") as p:
        f.seek(PART_LBA * SEC)
        f.write(p.read())
try:
    import blake3
    h = lambda b: blake3.blake3(b).hexdigest()
except ImportError:
    h = lambda b: "(pip install blake3 for the expected hashes)"
print(f"data blake3: {h(data)}")
print(f"prog blake3: {h(prog)}")
