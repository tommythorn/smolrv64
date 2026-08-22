#!/usr/bin/env python3
"""Turn `perf stat` output from the smolrv64 in-order core into a CPI stack + MPKI.

The counters charge every non-retiring cycle to exactly ONE cause (see
workloads/ipcstat/hpmstat.c), so

    CPI = 1 (issue) + sum(backend stalls)/instret + frontend-bubbles/instret

reconstructs the measured CPI exactly.  This script checks that it does and prints the
residual, so a silently-wrong event map shows up as a broken identity rather than a
plausible-looking table.

Event names come from docs/smolrv64-perf-events.json, which tools/gen-perf-events.py
generates from src/csr_file.v and src/lint.sh verifies -- so this cannot drift from the
RTL.  Do not hardcode codes here.

Usage:
    perf stat -e cycles,instructions,r0005,r0100,r0102,r0110,r0112,\\
        r0300,r0301,r0302,r0303,r0304,r0310,r0311,r0312 CMD 2>&1 | tools/perf-cpi-stack.py
    tools/perf-cpi-stack.py saved-perf-output.txt
"""
import json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
CATALOG = os.path.join(HERE, "..", "docs", "smolrv64-perf-events.json")

# FE_MMU and FE_IC are SUBSETS of FE_BUB ("X idle ... because"), so they are reported as a
# breakdown underneath it and never added alongside it.
BACKEND = [("ST_MEM", "LSU  (D$ / dTLB / AMO)"), ("ST_DIV", "divider"),
           ("ST_MUL", "multiplier"), ("ST_FPU", "FPU"), ("ST_SER", "serializing op")]
FE_SUB  = [("FE_MMU", "iMMU walking"), ("FE_IC", "I$ had no window")]

def load_names():
    with open(CATALOG) as f:
        cat = json.load(f)
    # "0x0300" -> "r0300", matching how perf echoes a raw event back
    return {"r%04x" % int(e["EventCode"], 16): e["EventName"] for e in cat}

def parse(text, names):
    """Pull counts out of perf's columnar output.  Tolerates thousands separators, the
    trailing multiplex percentage, '<not counted>' and '<not supported>'."""
    vals, unknown = {}, []
    for line in text.splitlines():
        line = line.split("#")[0]                      # drop perf's derived-metric comment
        m = re.match(r"\s*([\d,.]+|<[^>]+>)\s+([A-Za-z_][\w.:-]*)\s*(\(\d[\d.]*%\))?\s*$", line)
        if not m:
            continue
        raw, ev = m.group(1), m.group(2)
        if raw.startswith("<"):
            vals[ev] = None
            continue
        if "seconds" in ev:
            continue
        n = int(raw.replace(",", "").split(".")[0])
        key = {"cycles": "CYCLES", "instructions": "INSTRET"}.get(ev) or names.get(ev.lower())
        if key is None:
            unknown.append(ev); continue
        vals[key] = n
    return vals, unknown

def main():
    text = open(sys.argv[1]).read() if len(sys.argv) > 1 else sys.stdin.read()
    names = load_names()
    v, unknown = parse(text, names)
    g = lambda k: (v.get(k) or 0)

    cyc, ins = g("CYCLES"), g("INSTRET")
    if not cyc or not ins:
        sys.exit("error: need both cycles and instructions; got %s" % sorted(v))

    cpi, per_k = cyc / ins, lambda n: 1000.0 * n / ins
    fe_bub = g("FE_BUB")
    stalls = sum(g(k) for k, _ in BACKEND) + fe_bub
    issue  = cyc - stalls

    print("  cycles %-16d instructions %-16d IPC %.3f   CPI %.3f" % (cyc, ins, ins/cyc, cpi))
    print("\n  CPI stack (each cycle charged to exactly one cause)")
    print("    %-30s %10.3f  %5.1f%%" % ("issue / retire", issue/ins, 100.0*issue/cyc))
    for k, label in BACKEND:
        if v.get(k) is not None and g(k):
            print("    %-30s %10.3f  %5.1f%%" % ("stall: " + label, g(k)/ins, 100.0*g(k)/cyc))
    if fe_bub:
        print("    %-30s %10.3f  %5.1f%%" % ("frontend bubble", fe_bub/ins, 100.0*fe_bub/cyc))
        other = fe_bub - sum(g(k) for k, _ in FE_SUB)
        for k, label in FE_SUB:
            if g(k):
                print("      %-28s %10.3f  %5.1f%%" % ("- " + label, g(k)/ins, 100.0*g(k)/cyc))
        print("      %-28s %10.3f  %5.1f%%" % ("- other: had bytes, X idle", other/ins, 100.0*other/cyc))

    total = issue/ins + stalls/ins
    resid = cpi - total
    flag = "" if abs(resid) < 0.005 else "   <-- CHECK: events do not account for CPI"
    print("    %-30s %10.3f%s" % ("residual vs measured CPI", resid, flag))

    print("\n  MPKI (per 1000 instructions)")
    for code, label, acc in (("REDIR", "pipeline redirects", None),
                             ("DCMISS", "D$ misses", "DCACC"),
                             ("ICMISS", "I$ misses", "ICACC")):
        if v.get(code) is None:
            continue
        rate = "" if not acc or not g(acc) else "     (%.3f%% of %s accesses)" % (
            100.0*g(code)/g(acc), acc[:2])
        print("    %-30s %10.3f%s" % (label, per_k(g(code)), rate))

    # The number that usually explains a bad CPI here: stall per ACCESS, not per miss.
    if g("ST_MEM") and g("DCACC"):
        print("\n  LSU stall per D$ access %.2f cycles" % (g("ST_MEM")/g("DCACC")))
        if g("DCMISS"):
            print("  LSU stall per D$ MISS   %.0f cycles  -- if this is implausibly large the"
                  "\n                                     stall is hit latency, not misses"
                  % (g("ST_MEM")/g("DCMISS")))
    for code, label in (("LOAD", "loads"), ("STORE", "stores")):
        if g(code):
            print("  %-8s %12d  (%.1f%% of instructions)" % (label, g(code), 100.0*g(code)/ins))
    if unknown:
        print("\n  ignored unrecognised events: %s" % ", ".join(sorted(set(unknown))))

main()
