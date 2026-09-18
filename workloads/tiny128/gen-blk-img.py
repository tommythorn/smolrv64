#!/usr/bin/env python3
"""B6: the small ext4 image the disk-backed cosim mounts: data.bin (64 KiB, deterministic) and
its SHA256SUMS, in a 4 MiB journal-less ext4 built with mkfs.ext4 -d (no root needed). 64 KiB is
128 sectors -- about 2.5 M cycles through the SPI-mode SD model -- so the check fits in the
300 M-cycle cosim; the image must be >= 512 KiB for the card model's CSD granularity."""
import hashlib, os, random, subprocess, sys, tempfile
out = sys.argv[1]
with tempfile.TemporaryDirectory() as d:
    data = random.Random(0x5B0C).randbytes(65536)
    open(os.path.join(d, 'data.bin'), 'wb').write(data)
    open(os.path.join(d, 'SHA256SUMS'), 'w').write(hashlib.sha256(data).hexdigest() + '  data.bin\n')
    if os.path.exists(out): os.unlink(out)
    with open(out, 'wb') as f: f.truncate(4 << 20)
    subprocess.check_call(['mkfs.ext4', '-q', '-F', '-O', '^has_journal', '-d', d, out])
print(f'{out}: 4 MiB ext4, data.bin sha256 {hashlib.sha256(data).hexdigest()[:16]}...')
