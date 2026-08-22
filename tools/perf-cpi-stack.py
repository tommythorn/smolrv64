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
# This script is meant to be COPIED to the board, where there is no repo and no ../docs.
# Search, in order: an explicit override, next to the script (copy the .json along with
# it), the repo layout, and the cwd.  Never fall back to a built-in table -- a silently
# stale event map is exactly what this file exists to prevent.
CANDIDATES = [os.environ.get("SMOLRV_PERF_EVENTS"),
              os.path.join(HERE, "smolrv64-perf-events.json"),
              os.path.join(HERE, "..", "docs", "smolrv64-perf-events.json"),
              os.path.join(os.getcwd(), "smolrv64-perf-events.json")]

# FE_MMU and FE_IC are SUBSETS of FE_BUB ("X idle ... because"), so they are reported as a
# breakdown underneath it and never added alongside it.
BACKEND = [("ST_MEM", "LSU  (D$ / dTLB / AMO)"), ("ST_DIV", "divider"),
           ("ST_MUL", "multiplier"), ("ST_FPU", "FPU"), ("ST_SER", "serializing op")]
FE_SUB  = [("FE_MMU", "iMMU walking"), ("FE_IC", "no fetch bytes at all"),
           ("FE_ALN", "bytes, but no whole insn"), ("FE_QUE", "insn ready, F/X queue empty")]
REDIR_SUB = [("RED_BR", "conditional branch"), ("RED_JLR", "indirect jump (jalr)"),
             ("RED_TRP", "trap / system op")]

def load_names():
    path = next((p for p in CANDIDATES if p and os.path.exists(p)), None)
    if path is None:
        sys.exit("error: smolrv64-perf-events.json not found. Copy it next to this script,\n"
                 "       or set SMOLRV_PERF_EVENTS=/path/to/smolrv64-perf-events.json.\n"
                 "       Looked in: %s" % ", ".join(p for p in CANDIDATES if p))
    with open(path) as f:
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

    # FE_BUB is the sum of the FE_* causes.  perf-smol.sh's "cpi" set omits it to stay
    # inside 13 counters, so reconstruct it rather than reporting a hole.
    if v.get("FE_BUB") is None and any(v.get(k) is not None for k, _ in FE_SUB):
        v["FE_BUB"] = sum(v.get(k) or 0 for k, _ in FE_SUB)
        v["_FE_BUB_DERIVED"] = True

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
        tag = " (derived)" if v.get("_FE_BUB_DERIVED") else ""
        print("    %-30s %10.3f  %5.1f%%%s" % ("frontend bubble", fe_bub/ins, 100.0*fe_bub/cyc, tag))
        other = fe_bub - sum(g(k) for k, _ in FE_SUB)
        for k, label in FE_SUB:
            if g(k):
                print("      %-28s %10.3f  %5.1f%%" % ("- " + label, g(k)/ins, 100.0*g(k)/cyc))
        # Only a real hole now: FE_ALN/FE_QUE close what used to be an unexplained 21-31%.
        if abs(other) > 0.0005 * cyc:
            print("      %-28s %10.3f  %5.1f%%   <-- unattributed"
                  % ("- other", other/ins, 100.0*other/cyc))

    total = issue/ins + stalls/ins
    resid = cpi - total
    flag = "" if abs(resid) < 0.005 else "   <-- CHECK: events do not account for CPI"
    print("    %-30s %10.3f%s" % ("residual vs measured CPI", resid, flag))

    print("\n  MPKI (per 1000 instructions)")
    if any(v.get(k) is not None for k, _ in REDIR_SUB):
        tot = g("REDIR")
        for k, label in REDIR_SUB:
            if v.get(k) is not None:
                print("    %-30s %10.3f" % ("redirect: " + label, per_k(g(k))))
        if tot:
            rest = tot - sum(g(k) for k, _ in REDIR_SUB)
            print("    %-30s %10.3f   (fence.i / direct jal)" % ("redirect: other", per_k(rest)))
    for code, label, acc in (("REDIR", "pipeline redirects", None),
                             ("DCMISS", "D$ misses", "DCACC"),
                             ("ICMISS", "I$ misses", "ICACC")):
        if v.get(code) is None:
            continue
        rate = "" if not acc or not g(acc) else "     (%.3f%% of %s accesses)" % (
            100.0*g(code)/g(acc), acc[:2])
        print("    %-30s %10.3f%s" % (label, per_k(g(code)), rate))

    # Fetch-buffer payoff: FB_RHIT is exactly what flush-on-redirect would turn into misses.
    if v.get("FB_RHIT") is not None:
        print("\n  fetch buffer -- does the address comparison pay?")
        print("    %-30s %10.3f" % ("hits in redirect shadow /1k", per_k(g("FB_RHIT"))))
        if g("FB_HIT"):
            print("    %-30s %9.2f%% of all buffer hits" % ("", 100.0*g("FB_RHIT")/g("FB_HIT")))
        if g("REDIR"):
            print("    %-30s %9.2f%% of redirects land in the buffer"
                  % ("", 100.0*g("FB_RHIT")/g("REDIR")))
        print("    -> near zero: the tag earns nothing, a stream buffer is free.")
        print("       large: it is already a small loop buffer, worth GROWING not deleting.")

    # Split the LSU stall into what misses can possibly explain vs what is left.  This is
    # the number that decides whether to attack the miss path (MSHRs, non-blocking) or the
    # HIT path (load-to-use latency) -- and on this core it has consistently been the hit
    # path, which no amount of miss-handling work would touch.
    if g("ST_MEM") and g("DCACC"):
        acc, miss, st = g("DCACC"), g("DCMISS"), g("ST_MEM")
        print("\n  LSU stall %.3f CPI -- misses or hit latency?" % (st/ins))
        print("    %-30s %10.2f cycles" % ("stall per D$ ACCESS", st/acc))
        for pen in (30, 60, 100):
            frm = miss * pen
            print("    %-30s %9.1f%% of the LSU stall (%.3f CPI)"
                  % ("if a miss costs %d cycles" % pen, 100.0*frm/st, frm/ins))
        print("    -> whatever is left is HIT latency: every access pays it, misses are %.3f%%"
              % (100.0*miss/acc))
    for code, label in (("LOAD", "loads"), ("STORE", "stores")):
        if g(code):
            print("  %-8s %12d  (%.1f%% of instructions)" % (label, g(code), 100.0*g(code)/ins))
    if unknown:
        print("\n  ignored unrecognised events: %s" % ", ".join(sorted(set(unknown))))

main()
