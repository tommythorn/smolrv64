#!/usr/bin/env python3
"""Track GB5 progression across RTL commits.

  bench-track.py add <date> <rtl> <mhz> < pasted-geekbench-page.txt
  bench-track.py diff <rtl-old> <rtl-new>
  bench-track.py series [workload]
  bench-track.py runs

Compares RATE, never score. A GB5 subscore is a small integer: AES-XTS scored 1 both
before and after a 16.7% throughput gain, so score cannot see a change worth shipping.
Rate can.

The noise floor is real and must be respected: 95aff227 measured 814.5 and 754.4 kB/s on
AES-XTS on two different days -- 7.4% apart on identical RTL. `diff` marks anything under
8% as WITHIN NOISE rather than calling it a win or a regression.
"""
import sys, pathlib, re

TSV = pathlib.Path(__file__).resolve().parent.parent / "bench" / "gb5-results.tsv"
NOISE = 8.0          # percent; from the 95aff227 same-commit spread

def rows():
    out = []
    for ln in TSV.read_text().splitlines():
        if ln.startswith("#") or not ln.strip():
            continue
        f = ln.split("\t")
        if len(f) >= 7:
            out.append(dict(date=f[0], rtl=f[1], mhz=f[2], wl=f[3],
                            score=f[4], rate=float(f[5]), unit=f[6]))
    return out

def parse_page(text):
    """Pull (workload, score, rate, unit) out of a pasted Geekbench result page.

    The page lists them on consecutive lines: name, score, then "rate unit".
    """
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    out, i = [], 0
    while i + 2 < len(lines):
        nm, sc, rt = lines[i], lines[i+1], lines[i+2]
        m = re.match(r"^([\d.]+)\s+(\S+)$", rt)
        if re.match(r"^\d+$", sc) and m and not re.match(r"^[\d.]+$", nm):
            out.append((nm, sc, m.group(1), m.group(2)))
            i += 3
        else:
            i += 1
    return out

def cmd_add(date, rtl, mhz):
    got = parse_page(sys.stdin.read())
    if not got:
        sys.exit("no workload rows parsed from stdin")
    with TSV.open("a") as f:
        for nm, sc, rate, unit in got:
            f.write("%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (date, rtl, mhz, nm, sc, rate, unit))
    print("added %d rows for %s (%s)" % (len(got), rtl, date))

def cmd_diff(a, b):
    ra = {r["wl"]: r for r in rows() if r["rtl"].startswith(a)}
    rb = {r["wl"]: r for r in rows() if r["rtl"].startswith(b)}
    both = sorted(set(ra) & set(rb))
    if not both:
        sys.exit("no workloads in common between %s and %s" % (a, b))
    print("%-24s %12s %12s %9s" % ("workload", a, b, "change"))
    wins = regs = noise = 0
    for w in both:
        x, y = ra[w]["rate"], rb[w]["rate"]
        ch = (y / x - 1) * 100
        tag = ""
        if abs(ch) < NOISE:
            tag, noise = "  (within noise)", noise + 1
        elif ch > 0:
            wins += 1
        else:
            tag, regs = "  REGRESSION", regs + 1
        print("%-24s %12.2f %12.2f %+8.1f%%%s" % (w, x, y, ch, tag))
    print("\n%d improved, %d regressed, %d within +/-%.0f%% noise"
          % (wins, regs, noise, NOISE))
    if regs:
        print("Regressions are real only if they exceed the same-commit spread; re-run "
              "before acting on any single one.")

def cmd_series(wl=None):
    rs = [r for r in rows() if wl is None or r["wl"].lower().startswith(wl.lower())]
    for w in sorted({r["wl"] for r in rs}):
        s = sorted([r for r in rs if r["wl"] == w], key=lambda r: r["date"])
        print("\n%s (%s)" % (w, s[0]["unit"]))
        prev = None
        for r in s:
            ch = "" if prev is None else "  %+.1f%%" % ((r["rate"] / prev - 1) * 100)
            print("  %s  %-10s %7s MHz  %10.2f%s" % (r["date"], r["rtl"], r["mhz"], r["rate"], ch))
            prev = r["rate"]

def cmd_runs():
    seen = {}
    for r in rows():
        seen.setdefault((r["date"], r["rtl"], r["mhz"]), 0)
        seen[(r["date"], r["rtl"], r["mhz"])] += 1
    for (d, rtl, mhz), n in sorted(seen.items()):
        print("%s  %-10s %7s MHz  %2d workloads" % (d, rtl, mhz, n))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    c = sys.argv[1]
    if   c == "add"    and len(sys.argv) == 5: cmd_add(*sys.argv[2:5])
    elif c == "diff"   and len(sys.argv) == 4: cmd_diff(sys.argv[2], sys.argv[3])
    elif c == "series":                        cmd_series(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "runs":                          cmd_runs()
    else: sys.exit(__doc__)
