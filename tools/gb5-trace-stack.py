#!/usr/bin/env python3
"""Per-subtest CPI stacks from a Geekbench run traced by /var/tmp/gb5-trace.sh on the board.

The trace is `perf stat -a -x, -I 10000` (system-wide, one sample per 10 s, 12 events)
next to Geekbench's own output with every line stamped `date +%s`. Each "Running X" line
opens a window that the next one closes; the samples inside a window are summed and
reduced to CPI, the stall fractions and the miss rates. Written 2026-09-07 because the
preview binary has no per-workload switch and Machine Learning scored 0 at 0.01
images/sec on the full two-wide stack, unchanged from June while everything else moved.

    tools/gb5-trace-stack.py gb5-trace-perf.csv gb5-trace-raw.log          # the table
    tools/gb5-trace-stack.py ... --min-seconds 30                           # skip short windows

Events (docs/smolrv64-perf-events.json): r0005 redirects, r0008 trap/system redirects,
r0300 LSU stall, r0303 FPU stall, r0304 serializing stall, r0310 frontend bubble,
r0100/r0102 D$ accesses/misses, r0110/r0112 I$ accesses/misses. System-wide counts include
whatever else the board runs; it is otherwise idle.
"""
import sys, time, re, argparse, calendar
from collections import defaultdict

ap = argparse.ArgumentParser()
ap.add_argument("csv"); ap.add_argument("raw")
ap.add_argument("--min-seconds", type=float, default=20.0)
a = ap.parse_args()

# --- the samples: epoch -> {event: count}
start = None
samples = defaultdict(dict)
for line in open(a.csv, errors="replace"):
    if line.startswith("# started on"):
        # the board keeps UTC, so its header is UTC: timegm, never the host's mktime (7 h off once)
        start = calendar.timegm(time.strptime(line[len("# started on"):].strip(), "%a %b %d %H:%M:%S %Y"))
        continue
    f = line.rstrip("\n").split(",")
    if len(f) < 4 or not f[0].strip(): continue
    try: t = float(f[0]); c = int(f[1])
    except ValueError: continue
    samples[start + t][f[3]] = c
assert start is not None, "no '# started on' line in the csv"
# the board stamps the raw log with UTC epochs and perf's header is board-local; both are UTC there
times = sorted(samples)

# --- the windows: "Running X" lines, closed by the next one or the end
section = None; windows = []
for line in open(a.raw, errors="replace"):
    m = re.match(r"(\d+)\s+(.*)$", line.rstrip("\n"))
    if not m: continue
    ts, txt = int(m.group(1)), m.group(2).strip()
    if txt in ("Single-Core", "Multi-Core"): section = txt[:2]
    elif txt.startswith("Running "):
        if windows: windows[-1][2] = ts
        windows.append([f"{section or '??'} {txt[8:]}", ts, None])
    elif txt.startswith("Uploading") or "trace end" in txt:
        if windows and windows[-1][2] is None: windows[-1][2] = ts
if windows and windows[-1][2] is None: windows[-1][2] = int(times[-1]) if times else windows[-1][1]   # still running: up to the last sample

def col(x, w=9): return f"{x:>{w}}"
print(f"{'subtest':<26}{col('secs',6)}{col('IPC',6)}{col('CPI',6)}{col('trap/K',8)}{col('redir/K',8)}"
      f"{col('LSU%',6)}{col('FPU%',6)}{col('SER%',6)}{col('FE%',6)}{col('D$miss%',9)}{col('D$MPKI',8)}{col('I$miss%',9)}")
for name, t0, t1 in windows:
    if t1 - t0 < a.min_seconds: continue
    tot = defaultdict(int)
    for t in times:
        if t0 <= t < t1:
            for k, v in samples[t].items(): tot[k] += v
    cyc, ins = tot["cycles"], tot["instructions"]
    if not cyc or not ins: continue
    pct = lambda e: 100.0 * tot[e] / cyc
    print(f"{name:<26}{col(t1 - t0, 6)}{col(f'{ins/cyc:.3f}', 6)}{col(f'{cyc/ins:.2f}', 6)}"
          f"{col(f'{1000*tot['r0008']/ins:.2f}', 8)}{col(f'{1000*tot['r0005']/ins:.1f}', 8)}"
          f"{col(f'{pct('r0300'):.0f}', 6)}{col(f'{pct('r0303'):.0f}', 6)}{col(f'{pct('r0304'):.0f}', 6)}{col(f'{pct('r0310'):.0f}', 6)}"
          f"{col(f'{100*tot['r0102']/max(tot['r0100'],1):.2f}', 9)}{col(f'{1000*tot['r0102']/ins:.1f}', 8)}"
          f"{col(f'{100*tot['r0112']/max(tot['r0110'],1):.2f}', 9)}")
