#!/usr/bin/env python3
"""perf-smol's event names, and `perf-smol rec`'s report of where sampled events landed.

    perf-smol-rec.py list                  # every event: raw code, name, what it counts
    perf-smol-rec.py codes SPEC            # SPEC's raw codes, comma-separated, for perf stat -e
    perf-smol-rec.py rec SPEC              # the events perf record samples for SPEC
    perf script -F event,period,ip,sym,dso --hide-call-graph | perf-smol-rec.py report SPEC

SPEC is `td`, the sixteen top-down causes outside dispatching (each on its fixed counter), or
event names or raw codes, comma-separated (TDK_BE_DEPLD,DCMISS or r0102). The names and codes
come from the event table perf-cpi-stack.py reads, beside this script.

A sample stands for its period's worth of its event, so a function's weight in an event is the
sum of its samples' periods, whatever period perf's frequency mode chose. For td, perf-smol rec
samples every td_k bucket, dispatching ones included, so a function's cycles are its buckets' sum
and its causes are shares of them; for named events it samples cycles beside them.
"""
import importlib.util, json, os, re, sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("perf_cpi_stack", os.path.join(HERE, "perf-cpi-stack.py"))
pcs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pcs)


def table():
    """{raw code: name}, and {name: the short tag a cause's description ends with, e.g. be:M-mem}."""
    names, path = pcs.load_names()
    with open(path) as f:
        cat = json.load(f)
    tag = {}
    for e in cat:
        m = re.search(r"\(([a-z]+:[^()]+)\)\s*$", e["BriefDescription"])
        tag[e["EventName"]] = m.group(1) if m else e["EventName"]
    return names, tag, cat


# td_k's every bucket: the sixteen causes outside dispatching (the fixed counters) and dispatched
# 1-4 (general counters). Each cycle is in exactly one, so a function's cycles are the sum of its.
PARTITION = pcs.STALL_TDK + [k for k, _ in pcs.TDK["dispatching"]]


def codes(spec, names, td=pcs.STALL_TDK):
    byname = {n: c for c, n in names.items()}
    out = []
    for w in (td if spec == "td" else spec.split(",")):
        c = w.lower() if re.fullmatch(r"r[0-9a-fA-F]{4}", w) else byname.get(w.upper())
        if c not in names:
            sys.exit("perf-smol: no event %r (perf-smol events lists them)" % w)
        out.append(c)
    return out


def rec_events(spec, names):
    """What perf record samples: for td the whole partition, else cycles and the events named."""
    return codes(spec, names, PARTITION) if spec == "td" else ["cycles"] + codes(spec, names)


def report(spec, names, tag):
    """Weights by function and event from perf script's lines, then the hottest functions."""
    want = [names.get(c, "CYCLES") for c in rec_events(spec, names)]
    w = defaultdict(lambda: defaultdict(int))
    tot, nsamp = defaultdict(int), defaultdict(int)
    for line in sys.stdin:
        # "  20000 r0510:  3f807b3792 do_lookup_x (/usr/lib/riscv64-linux-gnu/ld-linux-riscv64-lp64d.so.1)"
        m = re.match(r"\s*(\d+)\s+([^\s:]+)\S*:\s+([0-9a-f]+)\s+(.*?)\s*\(([^()]*)\)\s*$", line)
        if not m:
            continue
        per, ev, ip, sym, dso = int(m.group(1)), m.group(2), m.group(3), m.group(4), m.group(5)
        name = "CYCLES" if ev == "cycles" else names.get(ev.lower(), ev)
        if sym == "[unknown]":
            # a stripped binary by its file; the kernel's symbols are hidden from a user's perf
            sym = "[kernel]" if dso == "[unknown]" and int(ip, 16) >> 63 else "[%s]" % os.path.basename(dso)
        w[sym][name] += per
        tot[name] += per
        nsamp[name] += 1
    if not sum(nsamp.values()):
        sys.exit("perf-smol rec: no samples (did the command run, and does perf record work here?)")
    top = int(os.environ.get("PERF_TOP", "20"))
    pct = lambda n, d: 100.0 * n / d if d else 0.0
    print("  perf-smol rec: %d samples of %d events" % (sum(nsamp.values()), len(want)))
    if spec == "td":
        # A function's cycles are its buckets' sum, so its causes are shares of its own cycles:
        # the function's top-down. Rows by stall cycles, where the time is lost.
        cyc = {f: sum(e[n] for n in PARTITION) for f, e in w.items()}
        stall = {f: sum(e[n] for n in pcs.STALL_TDK) for f, e in w.items()}
        allc, alls = sum(cyc.values()), sum(stall.values())
        print("  %cyc: the function's share of the cycles; %stall: of the cycles lost to a cause (td_k,\n"
              "  SmolRV64-Spec 11.2). Then the function's own cycles: the share lost, its largest causes.\n")
        print("  %6s %6s  %-32s %s" % ("%cyc", "%stall", "function", "its own cycles: lost, and to what"))
        for f in sorted(stall, key=stall.get, reverse=True)[:top]:
            causes = sorted(pcs.STALL_TDK, key=lambda n: w[f][n], reverse=True)[:3]
            by = "  ".join("%s %.0f" % (tag[n], pct(w[f][n], cyc[f])) for n in causes if w[f][n])
            print("  %6.2f %6.2f  %-32s %3.0f%%  %s" % (pct(cyc[f], allc), pct(stall[f], alls), f[:32],
                                                     pct(stall[f], cyc[f]), by))
    else:
        # Any event: each function's share of it. Rows by the first event named.
        print("  each column: the function's share of that event over the whole run\n")
        print("  %6s " % "%cyc" + " ".join("%10s" % ("%" + n[:9]) for n in want[1:]) + "  function")
        for f in sorted(w, key=lambda f: w[f][want[1]], reverse=True)[:top]:
            print("  %6.2f " % pct(w[f]["CYCLES"], tot["CYCLES"])
                  + " ".join("%10.2f" % pct(w[f][n], tot[n]) for n in want[1:]) + "  " + f)
    few = [n for n in want if nsamp[n] < 20]
    if few:
        print("\n  few samples (under 20), so their shares are noise: %s" % ", ".join(
            "%s %d" % (n, nsamp[n]) for n in few))
    print("\n  A sample lands up to ~30 cycles after its event, plus the interrupt: the function is"
          "\n  right, the instruction is not.")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("list", "codes", "rec", "report") or (sys.argv[1] != "list" and len(sys.argv) < 3):
        sys.exit(__doc__)
    names, tag, cat = table()
    if sys.argv[1] == "list":
        for e in cat:
            print("r%04x  %-16s %s" % (int(e["EventCode"], 16), e["EventName"], e["BriefDescription"]))
    elif sys.argv[1] == "codes":
        print(",".join(codes(sys.argv[2], names)))
    elif sys.argv[1] == "rec":
        print(",".join(rec_events(sys.argv[2], names)))
    else:
        report(sys.argv[2], names, tag)


main()
