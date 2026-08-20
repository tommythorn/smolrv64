#!/usr/bin/env python3
"""Verify a DTS's timebase-frequency against the CLINT rate the RTL actually produces.

The DTB's timebase-frequency is what Linux uses for EVERY deadline -- scheduler ticks, TCP
timeouts, NFS retries. It is not a free parameter: it must equal the rate the CLINT really
ticks at, which is

    probe_clk = (1_000_000_000 // PROBE_CLK_DIV8) * 8      # rk_xcku5p.v / ino_soc_top.v
    SCALE_DIV = probe_clk // 501_253                       # integer -> NOT exact
    real      = probe_clk // SCALE_DIV

The integer SCALE_DIV is the trap. At DIV8=120 the arithmetic lands exactly on 501253, which
is where that constant came from. At DIV8=72 the real rate is 502765, so the DTS was 0.302%
fast from the 111 MHz milestone (2026-08) until 2026-08-19 -- the GB5 run carried it. The DTS
comment said "MUST track SCALE_DIV" the whole time; nothing checked that it did.

    tools/check-dts-timebase.py 72 workloads/ubuntu/ubuntu-nfs.dts
"""
import re, sys, pathlib

def real_rate(div8):
    hz = (1_000_000_000 // div8) * 8
    return hz, hz // (hz // 501_253)

def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    div8 = int(sys.argv[1])
    hz, want = real_rate(div8)
    bad = 0
    for f in sys.argv[2:]:
        p = pathlib.Path(f)
        if not p.exists():
            print(f"  {f}: MISSING"); bad = 1; continue
        m = re.search(r"timebase-frequency\s*=\s*<\s*(\d+)\s*>", p.read_text())
        if not m:
            print(f"  {f}: no timebase-frequency"); continue
        got = int(m.group(1))
        if got != want:
            err = 100.0 * (got - want) / want
            print(f"  {f}: timebase-frequency={got} but PROBE_CLK_DIV8={div8} "
                  f"({hz:,} Hz) really ticks at {want} ({err:+.3f}% off)")
            bad = 1
        else:
            print(f"  {f}: timebase-frequency={got} ok for DIV8={div8} ({hz:,} Hz)")
    sys.exit(bad)

main()
