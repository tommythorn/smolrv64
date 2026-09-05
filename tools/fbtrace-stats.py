#!/usr/bin/env python3
"""Summarise an FB_TRACE run (the [FB] lines rv_soc_top prints when built with
VDEFS=-DFB_TRACE and run with +trace_from/+trace_to) over a cycle window: how often fetch
emitted a bundle and, when it did not, whether it had bytes; requests, answers, slides,
hits, arrival-served cycles. These are the numbers tools/fe-pipe-model.py predicts, so a
frontend change is judged the same way before and after it is built. 2026-09-05: on the
sha256 kernel the as-built buffer emitted a bundle in 74.6% of cycles (model 75.1%), the
run-ahead buffer (item 10e) in 99.0%.

    VDEFS=-DFB_TRACE BUILD=1 FW=.../shabench.bin CYC=8100000 ooo2/run-ooo2-linux.sh \
        +trace_from=8000000 +trace_to=8100000 > trace.out
    tools/fbtrace-stats.py trace.out 8000000 8100000"""
import re, sys
log = sys.argv[1]; lo = int(sys.argv[2]) if len(sys.argv) > 2 else 0; hi = int(sys.argv[3]) if len(sys.argv) > 3 else 1 << 62
n = fx0_nobytes = fx0_bytes = req = ack = val = in1 = hit = arr = 0
pcs = set()
for line in open(log, errors="ignore"):
    if not line.startswith("[FB]"): continue
    d = dict(re.findall(r'(\w+)=([0-9a-fx]+)', line))
    c = int(d['c'])
    if c < lo or c >= hi: continue
    n += 1
    if d['fx'] == '0':
        if int(d['avail']) == 0: fx0_nobytes += 1
        else: fx0_bytes += 1
    req += d['req'] == '1'; ack += d['ack'] == '1'; val += d['val'] == '1'
    in1 += d['in1'] == '1'; hit += d['hit'] == '1'; arr += d['arr'] == '1'
    pcs.add(d['pc'])
if not n: sys.exit("no [FB] lines in the window")
print("cycles %d  fetch emitted a bundle %d (%.1f%%)  nothing: no bytes %d (%.1f%%), bytes but no bundle %d (%.1f%%)"
      % (n, n - fx0_nobytes - fx0_bytes, 100.0 * (n - fx0_nobytes - fx0_bytes) / n, fx0_nobytes, 100.0 * fx0_nobytes / n, fx0_bytes, 100.0 * fx0_bytes / n))
print("requests presented %d  accepted %d  answered %d  slides %d  hits %d  arrival-served %d  distinct PCs %d"
      % (req, ack, val, in1, hit, arr, len(pcs)))
