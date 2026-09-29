#!/usr/bin/env python3
"""Turn `perf stat` output from the SmolRV64 core into a top-down breakdown or a CPI stack + MPKI.

TOP-DOWN (the `td` set, or any run with the TD_* events): the core charges every cycle to exactly
one of bad speculation (TD_BS), front-end (TD_FE), back-end (TD_BE) or dispatching (the rest), by
one classifier (smolrv64_core td_k, SmolRV64-Spec 11.2) -- the same one the simulator's TOPDOWN-SIM
prints -- so the four buckets are cycles by construction, and the depth events are subsets of
their parent. DPATCH (instructions dispatched) gives the slot view against IW x cycles.

    tools/perf-smol.sh td CMD 2>&1 | tools/perf-cpi-stack.py

The CPI-stack view below is the older wait-cycle view: its events overlap (a cycle waits on
several things at once) and it is kept for its depth, never summed as a partition.

The counters charge every non-retiring cycle to exactly ONE cause (see
workloads/ipcstat/hpmstat.c), so

    CPI = 1 (issue) + sum(backend stalls)/instret + frontend-bubbles/instret

reconstructs the measured CPI exactly.  This script checks that it does and prints the
residual, so a silently-wrong event map shows up as a broken identity rather than a
plausible-looking table.

Event names come from docs/smolrv64-perf-events.json, which tools/gen-perf-events.py
generates from src/csr_file.v and src/lint.sh verifies -- so this cannot drift from the
RTL.  Do not hardcode codes here.

Usage (13 programmable counters -- one set per run, never the union, see perf-smol.sh):
    tools/perf-smol.sh cpi CMD 2>&1 | tools/perf-cpi-stack.py     # the stack + ROB-full,
                                                                  # drain, redirects, D$ misses
    tools/perf-smol.sh br  CMD 2>&1 | tools/perf-cpi-stack.py     # redirects by cause
    tools/perf-smol.sh mem CMD 2>&1 | tools/perf-cpi-stack.py     # D$/I$ traffic, loads/stores
    tools/perf-cpi-stack.py saved-perf-output.txt
    tools/perf-cpi-stack.py --width 2 ...        # a narrower build than the shipping IW=3

The identity this checks: dispatch is WIDTH instructions per cycle (3, the shipping IW), so
    cycles = instructions/WIDTH + sum(named stall cycles) + frontend bubbles + UNATTRIBUTED
and "unattributed" is what no event names.  On a two-wide core a cycle that dispatches ONE
instruction has no counter yet: half of it lands in unattributed, which is where the
frontend's single-instruction bundles (the chunk-boundary cap, item 10e) show up.  A run with only the FE_* events is not a stack:
every backend stall lands there (2026-09-05: 10.7% of sha256sum's cycles), and the tool says
so rather than folding it into a "retire" line.  Events can overlap (a cycle blocked on a load
AND on ROB space counts in both), so a small negative is overlap, not an error.
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
           ("ST_MUL", "multiplier"), ("ST_FPU", "FPU"),
           ("ST_DSP", "dispatch held (scheduler / rename / queues / serializing)"),
           ("ST_ROB", "dispatch: ROB full")]
# ST_IQ..ST_SRZ are the disjoint parts of ST_DSP (the `hold` set): a breakdown underneath it.
DSP_SUB = [("ST_IQ", "the scheduler is full"), ("ST_RN", "rename: a free list is empty"),
           ("ST_SQ", "store queue full"), ("ST_LQ", "load queue full"), ("ST_SRZ", "serializing op")]
# What ST_MEM is made of (2026-09-17, the memory backend program): cycle buckets printed
# underneath it as shares of all cycles, never added (they overlap ST_MEM and each other),
# plus the per-1k counts and the mean queue occupancies that go with them.
MEM_SUB = [("MEM_HITSER", "a ready load the door did not take"),
           ("MEM_LDINFL", "a load access in flight (hit ~3 cycles; the rest is miss wait)"),
           ("MEM_STDOOR", "a store at the D$ door, unaccepted"),
           ("MEM_ALIAS_UNK", "load blocked: older store address unknown"),
           ("MEM_ALIAS_OVL", "load blocked: known older store overlaps"),
           ("MEM_DEVWAIT", "device load waiting for the head")]
MEM_CNT = [("MEM_REORD", "loads issued past an uncommitted older store"),
           ("MEM_WPKILL", "wrong-path loads killed after their access ran")]
FE_SUB  = [("FE_MMU", "iMMU walking"), ("FE_IC", "no fetch bytes at all"),
           ("FE_ALN", "bytes, but no whole insn"), ("FE_QUE", "insn ready, decoupling queue empty")]
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

def topdown(v, width):
    """The closing top-down report. Returns False if the run has no TD_* events."""
    if v.get("TD_BE") is None or v.get("TD_FE") is None or v.get("TD_BS") is None:
        return False
    g = lambda k: (v.get(k) or 0)
    cyc, ins = g("CYCLES"), g("INSTRET")
    bs, fe, be = g("TD_BS"), g("TD_FE"), g("TD_BE")
    disp = cyc - bs - fe - be
    pc = lambda n: 100.0 * n / cyc
    print("  cycles %-16d instructions %-16d IPC %.3f   (%d-wide)" % (cyc, ins, ins / cyc, width))
    print("\n  TOP-DOWN, level 1 (every cycle charged to exactly one; SmolRV64-Spec 11.2)")
    for name, n in (("dispatching", disp), ("bad speculation", bs), ("front-end", fe), ("back-end", be)):
        print("    %-34s %6.2f%%" % (name, pc(n)))
    if disp < 0 or min(bs, fe, be) < 0:
        sys.exit("error: the level-1 buckets exceed the cycles -- the event map is wrong (docs/smolrv64-perf-events.json)")
    def sub(parent, rows):
        rest = parent
        for label, k in rows:
            if v.get(k) is not None:
                print("      %-32s %6.2f%%" % ("- " + label, pc(g(k))))
                rest -= g(k)
        if any(v.get(k) is not None for _, k in rows):
            print("      %-32s %6.2f%%" % ("- the rest", pc(rest)))
    print("\n  level 2")
    print("    %-34s %6.2f%%" % ("bad speculation", pc(bs)))
    if v.get("RD_WAIT") is not None:
        drain = min(g("RD_WAIT"), bs)
        print("      %-32s %6.2f%%" % ("- a resolved restart waiting (drain)", pc(drain)))
        print("      %-32s %6.2f%%" % ("- the redirect itself", pc(bs - drain)))
    print("    %-34s %6.2f%%" % ("front-end", pc(fe)))
    sub(fe, [("latency: iMMU walk / no fetch bytes", "TD_FE_LAT")])
    print("    %-34s %6.2f%%" % ("back-end", pc(be)))
    sub(be, [("memory: M on a memory op / a load result", "TD_BE_MEM"),
             ("the ROB is full", "TD_BE_ROB"),
             ("a scheduler or load/store queue is full", "TD_BE_IQ")])
    if v.get("DPATCH") is not None:
        slots = width * cyc
        print("\n  slots (%d x cycles)" % width)
        print("    %-34s %6.2f%%" % ("dispatched", 100.0 * g("DPATCH") / slots))
        print("    %-34s %6.2f%%" % ("retired", 100.0 * ins / slots))
        print("    %-34s %6.2f%%" % ("dispatched, squashed (wrong path)", 100.0 * (g("DPATCH") - ins) / slots))
    rows = [("conditional branch", "RED_BR"), ("indirect jump", "RED_JLR"), ("trap / system op", "RED_TRP"),
            ("D$ line fills", "DCMISS"), ("I$ misses", "ICMISS"), ("dTLB walks", "DTLB_MISS")]
    got = [(l, k) for l, k in rows if v.get(k) is not None]
    if got:
        print("\n  per 1000 instructions")
        for l, k in got:
            print("    %-34s %10.3f" % (l, 1000.0 * g(k) / ins))
    return True

def main():
    args, width = sys.argv[1:], 3
    if "--width" in args:
        i = args.index("--width"); width = int(args[i + 1]); del args[i:i + 2]
    text = open(args[0]).read() if args else sys.stdin.read()
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
    if topdown(v, width):
        if unknown:
            print("\n  ignored unrecognised events: %s" % ", ".join(sorted(set(unknown))))
        return

    cpi, per_k = cyc / ins, lambda n: 1000.0 * n / ins
    fe_bub = g("FE_BUB")
    named  = sum(g(k) for k, _ in BACKEND) + fe_bub
    floor  = ins / width                  # the dispatch floor: every cycle full
    unattr = cyc - floor - named          # cycles beyond the floor that nothing names
    no_backend = not any(v.get(k) is not None for k, _ in BACKEND)

    print("  cycles %-16d instructions %-16d IPC %.3f   CPI %.3f" % (cyc, ins, ins/cyc, cpi))
    print("  (the wait-cycle view: its events overlap, so it is depth, not a partition -- the td set closes)")
    print("\n  CPI stack (each cycle charged to one cause; dispatch is %d per cycle, so %.3f is the floor)"
          % (width, 1.0 / width))
    print("    %-30s %10.3f  %5.1f%%" % ("dispatch floor (%d per cycle)" % width, 1.0 / width, 100.0*floor/cyc))
    for k, label in BACKEND:
        if v.get(k) is not None and g(k):
            print("    %-30s %10.3f  %5.1f%%" % ("stall: " + label, g(k)/ins, 100.0*g(k)/cyc))
            # DT_WALK is a SUBSET of ST_MEM (a walk holds the LSU), reported underneath it, never
            # added: the dTLB is 16 entries direct-mapped, and a layout that thrashes it costs a
            # walk per load with no D$ miss to show for it (2026-09-05).
            if k == "ST_MEM" and v.get("DT_WALK") is not None:
                print("      %-28s %10.3f  %5.1f%%" % ("- of which dTLB walking", g("DT_WALK")/ins, 100.0*g("DT_WALK")/cyc))
            if k == "ST_MEM" and any(v.get(kk) is not None for kk, _ in MEM_SUB):
                for kk, ll in MEM_SUB:
                    if v.get(kk) is not None:
                        print("      %-28s %10.3f  %5.1f%%" % ("- " + ll, g(kk)/ins, 100.0*g(kk)/cyc))
                if v.get("MEM_LDINFL") is not None and v.get("LOAD") is not None and g("LOAD"):
                    est = g("MEM_LDINFL") - 3 * g("LOAD")
                    print("      %-28s %10.3f  %5.1f%%   (LDINFL - 3 x loads: estimate)" % ("- ...of which miss wait", est/ins, 100.0*est/cyc))
                for kk, ll in MEM_CNT:
                    if v.get(kk) is not None:
                        print("      %-28s %10.3f per 1k insns" % ("- " + ll, per_k(g(kk))))
                for kk, ll in (("MEM_LQOCC", "load queue"), ("MEM_SQOCC", "store queue")):
                    if v.get(kk) is not None:
                        print("      %-28s %10.2f entries" % ("- mean %s occupancy" % ll, g(kk)/cyc))
            # ST_DSP's disjoint parts (the `hold` set): a breakdown, never added alongside it.
            if k == "ST_DSP" and any(v.get(kk) is not None for kk, _ in DSP_SUB):
                for kk, ll in DSP_SUB:
                    if g(kk):
                        print("      %-28s %10.3f  %5.1f%%" % ("- " + ll, g(kk)/ins, 100.0*g(kk)/cyc))
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
    flag = ""
    if unattr > 0.005 * cyc:
        flag = ("   <-- no backend events in this run: use perf-smol.sh cpi" if no_backend
                else "   <-- a cause with no counter (or events dropped from the set)")
    elif unattr < -0.005 * cyc:
        flag = "   (events overlap: a cycle blocked on two units counts in both)"
    print("    %-30s %10.3f  %5.1f%%%s" % ("unattributed", unattr/ins, 100.0*unattr/cyc, flag))

    mpki = []
    if any(v.get(k) is not None for k, _ in REDIR_SUB):
        tot = g("REDIR")
        for k, label in REDIR_SUB:
            if v.get(k) is not None:
                mpki.append("    %-30s %10.3f" % ("redirect: " + label, per_k(g(k))))
        if tot:
            rest = tot - sum(g(k) for k, _ in REDIR_SUB)
            mpki.append("    %-30s %10.3f   (fence.i / direct jal)" % ("redirect: other", per_k(rest)))
    if v.get("RD_WAIT") is not None:
        # a redirect resolved in M sits there until it is the ROB head (head_block): the cycles
        # a rename walk-back (P7) would recover. Cycles per instruction and share of cycles.
        mpki.append("    %-30s %10.3f   (%.1f%% of cycles: a resolved redirect waiting for the head)"
                    % ("redirect drain, cycles/insn", g("RD_WAIT")/ins, 100.0*g("RD_WAIT")/cyc))
    if v.get("DTLB_MISS") is not None:
        per_walk = "" if not g("DTLB_MISS") or v.get("DT_WALK") is None else "     (%.1f cycles per walk)" % (g("DT_WALK")/g("DTLB_MISS"))
        mpki.append("    %-30s %10.3f%s" % ("dTLB misses (walks)", per_k(g("DTLB_MISS")), per_walk))
    for code, label, acc in (("REDIR", "pipeline redirects", None),
                             ("DCMISS", "D$ line fills", "DCACC"),
                             ("ICMISS", "I$ misses", "ICACC")):
        if v.get(code) is None:
            continue
        rate = "" if not acc or not g(acc) else "     (%.3f%% of %s accesses)" % (
            100.0*g(code)/g(acc), acc[:2])
        mpki.append("    %-30s %10.3f%s" % (label, per_k(g(code)), rate))
    if mpki:
        print("\n  MPKI (per 1000 instructions)")
        print("\n".join(mpki))
    else:
        print("\n  MPKI: no redirect / miss / drain events in this run (perf-smol.sh cpi, br, mem)")

    for code, label in (("LOAD", "loads"), ("STORE", "stores")):
        if g(code):
            print("  %-8s %12d  (%.1f%% of instructions)" % (label, g(code), 100.0*g(code)/ins))
    if unknown:
        print("\n  ignored unrecognised events: %s" % ", ".join(sorted(set(unknown))))

main()
