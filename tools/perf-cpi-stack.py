#!/usr/bin/env python3
"""Turn `perf stat` output from the SmolRV64 core into a top-down breakdown or a CPI stack + MPKI.

TOP-DOWN (the `td` set, or any run with the TD_* events): the core charges every cycle to exactly
one of bad speculation (TD_BS), front-end (TD_FE), back-end (TD_BE) or dispatching (the rest), by
one classifier (smolrv64_core td_k, SmolRV64-Spec 11.2) -- the same one the simulator's TOPDOWN-SIM
prints -- so the four buckets are cycles by construction, and the depth events are subsets of
their parent. DPATCH (instructions dispatched) gives the slot view against IW x cycles. The TDK_*
events are td_k itself, one per TOPDOWN-SIM bucket: the tdbe set counts the back-end's ten causes
(they sum to TD_BE), the tdfe set bad speculation's and the front-end's and the dispatch width.

    perf-smol td   CMD > /dev/null      # levels 1 and 2 (tools/perf-smol.sh, installed on the board)
    perf-smol tdbe CMD > /dev/null      # the back-end's causes
    perf-smol tdfe CMD > /dev/null      # bad speculation, front-end, width

perf-smol runs this script on its counts when the command ends; run it by hand to re-read a
saved file (perf-smol.<set>.txt).

The wait-cycle view below reads the ST_* events: they overlap (a cycle waits on several things
at once) and are depth, never a partition.

A dispatch stall is the queue head holding an instruction it has no credit for (hd_wait,
SmolRV64-Spec 3.2): ST_DSP and ST_ROB, and ST_IQ..ST_SRZ beneath ST_DSP, all describe the head.
ST_MUL and ST_DIV are occupancy (a multiply in flight in a lane, the MD stage holding a divide),
not stalls, and are reported apart. SmolRV64-Spec 11, "Reading the report", says what each line
points at.

Event names come from docs/smolrv64-perf-events.json, which tools/gen-perf-events.py
generates from src/csr_file.v and src/lint.sh verifies -- so this cannot drift from the
RTL.  Do not hardcode codes here.

Usage (13 programmable counters -- one set per run, never the union, see perf-smol.sh):
    perf-smol cpi CMD                   # the stack + ROB-full, drain, redirects, D$ misses
    perf-smol br  CMD                   # redirects by cause
    perf-smol mem CMD                   # D$/I$ traffic, loads/stores
    tools/perf-cpi-stack.py perf-smol.cpi.txt     # re-read a run's counts
    tools/perf-cpi-stack.py --width 3 ...        # a narrower build than the shipping IW=4

The wait-cycle view's arithmetic: dispatch is WIDTH instructions per cycle (4, the shipping IW), so
    cycles = instructions/WIDTH + sum(wait cycles) + frontend bubbles + UNATTRIBUTED
and "unattributed" is what no event names, including every cycle that dispatches fewer than
WIDTH. A run with only the FE_* events names no back-end wait: every one lands in unattributed,
and the tool says so. The wait events overlap (a cycle blocked on a load AND on ROB space counts
in both), so a negative unattributed is overlap, not an error.
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

# The wait events (they overlap); ST_DIV and ST_MUL are occupancies, printed apart.
BACKEND = [("ST_MEM", "a head op in M, or a wait on an integer load"), ("ST_FPU", "stage F held, or a wait on an FP result"),
           ("ST_DSP", "queue head without a credit (scheduler / rename / queues / serializing)"),
           ("ST_ROB", "queue head without ROB room")]
OCCUPANCY = [("ST_DIV", "a divide in the MD stage"), ("ST_MUL", "a multiply in flight in a lane")]
# ST_IQ..ST_SRZ are the disjoint parts of ST_DSP (the `hold` set): a breakdown underneath it.
DSP_SUB = [("ST_IQ", "the head's lane's scheduler, the FP queue, the head-op register or the divide credit"),
           ("ST_RN", "rename: a shard below LOWAT, or the map's sweep after a squash"),
           ("ST_SQ", "no store-queue credit"), ("ST_LQ", "no load-queue credit"),
           ("ST_SRZ", "serialising, a CSR op or a CBO waiting")]
# The memory events: cycle counts printed as shares of all cycles (they overlap ST_MEM and each
# other), the per-1k counts, the mean queue occupancies, and the fast-load latency.
MEM_SUB = [("MEM_HITSER", "a ready load the LSU port did not take"),
           ("MEM_STDOOR", "a write at the D$ door with no room"),
           ("MEM_ALIAS_UNK", "load blocked: an older store's address unknown"),
           ("MEM_ALIAS_OVL", "load blocked: a known older store overlaps"),
           ("MEM_DEVWAIT", "a device load waiting for the head"),
           ("DT_WALK", "dMMU walker busy (second level or walk)")]
MEM_CNT = [("MEM_REORD", "loads taken with an older store live"),
           ("MEM_WPKILL", "load landings dropped by a kill")]
FE_SUB  = [("FE_MMU", "iMMU cannot translate"), ("FE_IC", "no fetch bytes"),
           ("FE_ALN", "bytes, but no whole instruction"), ("FE_QUE", "an instruction not yet in the IR")]
REDIR_SUB = [("RED_BR", "conditional branch"), ("RED_JLR", "indirect jump (jalr)"),
             ("RED_TRP", "exception / interrupt / fault / replay")]
# td_k's causes (TDK_*), under their level-1 parent; each parent's list is all of its causes.
TDK = {"bad speculation": [("TDK_REDIRECT", "a redirect"), ("TDK_DRAIN", "a resolved restart waiting")],
       "front-end": [("TDK_FE_IMMU", "the iMMU"), ("TDK_FE_ICACHE", "no fetch bytes"),
                     ("TDK_FE_ALIGN", "bytes, no whole instruction"), ("TDK_FE_QUEUE", "an instruction not yet in the IR")],
       "back-end": [("TDK_BE_MMEM", "M holds a head op"), ("TDK_BE_ROB", "no ROB room"),
                    ("TDK_BE_DEPLD", "a wait on an integer load / AMO / CSR read"),
                    ("TDK_BE_DEPFP", "a wait on an FP / FP-load / divide result"),
                    ("TDK_BE_IQ", "no scheduler credit"), ("TDK_BE_RN", "rename below LOWAT or sweeping"),
                    ("TDK_BE_SQ", "no store-queue credit"), ("TDK_BE_LQ", "no load-queue credit"),
                    ("TDK_BE_SER", "serialising / CSR / CBO"), ("TDK_BE_HELD", "the IR held after a redirect or resteer")],
       "dispatching": [("TDK_OK1", "1 instruction"), ("TDK_OK2", "2"), ("TDK_OK3", "3"), ("TDK_OK4", "4")]}

# Every event name the reports read: the table must name them all (see main).
USED = ({k for k, _ in BACKEND + OCCUPANCY + DSP_SUB + MEM_SUB + MEM_CNT + FE_SUB + REDIR_SUB}
        | {k for rows in TDK.values() for k, _ in rows}
        | {"TD_BS", "TD_FE", "TD_BE", "TD_FE_LAT", "TD_BE_MEM", "TD_BE_ROB", "TD_BE_IQ", "DPATCH",
           "RD_WAIT", "REDIR", "DCMISS", "ICMISS", "DTLB_MISS", "MEM_LDINFL", "MEM_LDSENT",
           "MEM_LQOCC", "MEM_SQOCC", "FE_BUB"})

def load_names():
    path = next((p for p in CANDIDATES if p and os.path.exists(p)), None)
    if path is None:
        sys.exit("error: smolrv64-perf-events.json not found. Copy it next to this script,\n"
                 "       or set SMOLRV_PERF_EVENTS=/path/to/smolrv64-perf-events.json.\n"
                 "       Looked in: %s" % ", ".join(p for p in CANDIDATES if p))
    with open(path) as f:
        cat = json.load(f)
    # "0x0300" -> "r0300", matching how perf echoes a raw event back
    return {"r%04x" % int(e["EventCode"], 16): e["EventName"] for e in cat}, path

def parse(text, names):
    """Pull counts out of perf's columnar output, and each event's trailing multiplex
    percentage.  Tolerates thousands separators, '<not counted>' and '<not supported>'."""
    vals, unknown, mux = {}, [], {}
    for line in text.splitlines():
        pm = re.search(r"\((\d[\d.]*)%\)\s*$", line)     # the multiplex share ends the line
        line = line.split("#")[0]                      # drop perf's derived-metric comment
        m = re.match(r"\s*([\d,.]+|<[^>]+>)\s+([A-Za-z_][\w.:-]*)\s*(\(\d[\d.]*%\))?\s*$", line)
        if not m:
            continue
        raw, ev = m.group(1), m.group(2)
        if pm:
            mux[ev] = float(pm.group(1))
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
    return vals, unknown, mux

class Report:
    """Rows collected, then printed numbers first -- right-aligned in a column as wide as the
    block's widest -- and the label after, indented by its depth, so a long cause never pushes
    its numbers out of line and the report stays narrow."""
    def __init__(self):
        self.items = []
    def text(self, s=""):
        self.items.append(s)
    def row(self, depth, label, num, note=""):
        self.items.append((depth, label, num, note))
    def show(self):
        i = 0
        while i < len(self.items):
            if isinstance(self.items[i], str):
                print(self.items[i]); i += 1; continue
            j = i
            while j < len(self.items) and not isinstance(self.items[j], str):
                j += 1
            w = max(len(n) for _, _, n, _ in self.items[i:j])
            for d, l, n, note in self.items[i:j]:
                print("    " + n.rjust(w) + "   " + "  " * d + l + ("   " + note if note else ""))
            i = j

def topdown(v, width, R):
    """The closing top-down report. Returns False if the run has no TD_* events."""
    if v.get("TD_BE") is None or v.get("TD_FE") is None or v.get("TD_BS") is None:
        return False
    g = lambda k: (v.get(k) or 0)
    cyc, ins = g("CYCLES"), g("INSTRET")
    bs, fe, be = g("TD_BS"), g("TD_FE"), g("TD_BE")
    disp = cyc - bs - fe - be
    pc = lambda n: "%6.2f%%" % (100.0 * n / cyc)
    R.text("  cycles %-16d instructions %-16d IPC %.3f   (%d-wide)" % (cyc, ins, ins / cyc, width))
    R.text("\n  TOP-DOWN, level 1 (every cycle charged to exactly one; SmolRV64-Spec 11.2)")
    for name, n in (("dispatching", disp), ("bad speculation", bs), ("front-end", fe), ("back-end", be)):
        R.row(0, name, pc(n))
    if disp < 0 or min(bs, fe, be) < 0:
        sys.exit("error: the level-1 buckets exceed the cycles -- the event map is wrong (docs/smolrv64-perf-events.json)")
    def sub(parent, rows, rest_label):
        rest = parent
        for label, k in rows:
            if v.get(k) is not None:
                R.row(1, "- " + label, pc(g(k)))
                rest -= g(k)
        if any(v.get(k) is not None for _, k in rows):
            R.row(1, "- " + rest_label, pc(rest))
    have3 = tdk(v, {"bad speculation": bs, "front-end": fe, "back-end": be, "dispatching": disp}, pc, R)
    R.text("\n  level 2")
    R.row(0, "bad speculation", pc(bs))
    if v.get("RD_WAIT") is not None:
        drain = min(g("RD_WAIT"), bs)
        R.row(1, "- a resolved restart waiting (drain)", pc(drain))
        R.row(1, "- the redirect itself", pc(bs - drain))
    R.row(0, "front-end", pc(fe))
    sub(fe, [("latency: iMMU walk / no fetch bytes", "TD_FE_LAT")],
        "bytes but no whole instruction, or not yet in the IR" + ("" if have3 else " (tdfe splits)"))
    R.row(0, "back-end", pc(be))
    sub(be, [("memory: a head op in M / an integer-load wait", "TD_BE_MEM"),
             ("no ROB room", "TD_BE_ROB"),
             ("no scheduler / load / store queue room", "TD_BE_IQ")],
        "rename, serialising, FP results, the held IR" + ("" if have3 else " (tdbe splits)"))
    if v.get("DPATCH") is not None:
        slots = width * cyc
        R.text("\n  slots (%d x cycles)" % width)
        R.row(0, "dispatched", "%6.2f%%" % (100.0 * g("DPATCH") / slots))
        R.row(0, "retired", "%6.2f%%" % (100.0 * ins / slots))
        R.row(0, "dispatched, squashed (wrong path)", "%6.2f%%" % (100.0 * (g("DPATCH") - ins) / slots))
    rows = [("conditional branch", "RED_BR"), ("indirect jump", "RED_JLR"), ("exception / interrupt / fault / replay", "RED_TRP"),
            ("D$ line fills", "DCMISS"), ("I$ misses", "ICMISS"), ("dMMU walker starts", "DTLB_MISS")]
    got = [(l, k) for l, k in rows if v.get(k) is not None]
    if got:
        R.text("\n  per 1000 instructions")
        for l, k in got:
            R.row(0, l, "%.3f" % (1000.0 * g(k) / ins))
    if not have3:
        R.text("\n  level 3: perf-smol top (every cause in one run, on a bitstream with the fixed counters),"
               "\n  or perf-smol tdbe and perf-smol tdfe, each a run of its own")
    return True

# td_k's causes outside dispatching, one counter each (perf-smol top). Level 1 and level 2 are
# sums of them, so a run that counted every one has no use for the TD_* events.
STALL_TDK = [k for p in ("bad speculation", "front-end", "back-end") for k, _ in TDK[p]]

def from_causes(v, cyc):
    """Level 1 and 2 from td_k's causes, when the run counted every cause and not the TD_*
    events: bad speculation, the front-end and the back-end are their causes' sums, and
    dispatching 4 is the cycles every other cause leaves (td_k charges each cycle once)."""
    if v.get("TD_BS") is not None or any(v.get(k) is None for k in STALL_TDK):
        return
    g = lambda k: v.get(k) or 0
    v["TD_BS"] = g("TDK_REDIRECT") + g("TDK_DRAIN")
    v["TD_FE"] = sum(g(k) for k, _ in TDK["front-end"])
    v["TD_BE"] = sum(g(k) for k, _ in TDK["back-end"])
    v["TD_FE_LAT"] = g("TDK_FE_IMMU") + g("TDK_FE_ICACHE")
    v["TD_BE_MEM"] = g("TDK_BE_MMEM") + g("TDK_BE_DEPLD")
    v["TD_BE_ROB"] = g("TDK_BE_ROB")
    v["TD_BE_IQ"] = g("TDK_BE_IQ") + g("TDK_BE_SQ") + g("TDK_BE_LQ")
    v["RD_WAIT"] = g("TDK_DRAIN")
    if all(v.get(k) is not None for k in ("TDK_OK1", "TDK_OK2", "TDK_OK3")) and v.get("TDK_OK4") is None:
        v["TDK_OK4"] = (cyc - v["TD_BS"] - v["TD_FE"] - v["TD_BE"]
                        - g("TDK_OK1") - g("TDK_OK2") - g("TDK_OK3"))

def tdk(v, parents, pc, R):
    """td_k's causes under their level-1 parent, as far as the run counted them; a parent whose
    every cause was counted is checked to close (td_k is one cause per cycle). Returns whether
    the run counted any."""
    if not any(v.get(k) is not None for rows in TDK.values() for k, _ in rows):
        return False
    R.text("\n  level 3 (td_k, the TOPDOWN-SIM buckets)")
    for parent, rows in TDK.items():
        got = [(k, l) for k, l in rows if v.get(k) is not None]
        if not got:
            continue
        R.row(0, parent, pc(parents[parent]))
        for k, l in got:
            R.row(1, "- " + l, pc(v[k]))
        if len(got) == len(rows):
            tot = sum(v[k] for k, _ in rows)
            if abs(tot - parents[parent]) > 0.001 * max(parents[parent], 1):
                R.row(1, "- the causes' sum", pc(tot), "<-- does not close: %d counted, %d the parent"
                      % (tot, parents[parent]))
    return True

def main():
    args, width = sys.argv[1:], 4
    if "--width" in args:
        i = args.index("--width"); width = int(args[i + 1]); del args[i:i + 2]
    text = open(args[0]).read() if args else sys.stdin.read()
    names, table = load_names()
    v, unknown, mux = parse(text, names)
    # A table that lacks a name this script reads, or a raw event it does not name, is older than
    # the RTL's event map: its counts would be dropped or misnamed and the report would quietly
    # fall back to a view without them (a September table beside current scripts turned a
    # Top-Down run into a wait-cycle view, and named r0304 ST_SER). The three files travel together.
    lacks = sorted(USED - set(names.values()))
    stale = sorted(set(e for e in unknown if re.fullmatch(r"r[0-9a-fA-F]{4}", e)))
    if lacks or stale:
        miss = lacks + stale
        sys.exit("error: %s is older than this script: it lacks %d of the events it reads (%s%s).\n"
                 "       Install the three files together from the tree: tools/perf-smol-install.sh, or\n"
                 "       copy docs/smolrv64-perf-events.json beside this script."
                 % (table, len(miss), ", ".join(miss[:6]), ", ..." if len(miss) > 6 else ""))
    g = lambda k: (v.get(k) or 0)
    R = Report()
    # The events of one set fit the PMU (cycle, instret and 13 counters), so a share under 100%
    # means another perf session held counters: the counts are scaled estimates and the
    # rotation's SBI calls slowed the run (a loop beside `perf stat -a`: -19%, instructions +9%).
    shared = [p for p in mux.values() if p < 98.0]
    if shared:
        R.text("  WARNING: %d events multiplexed (counted %.0f-%.0f%% of the run): every count is perf's\n"
               "  scaled estimate, and the run shared the PMU with another perf session -- not\n"
               "  comparable with an unshared run.\n" % (len(shared), min(shared), max(shared)))

    # FE_BUB is the sum of the FE_* causes.  perf-smol.sh's "cpi" set omits it to stay
    # inside 13 counters, so reconstruct it rather than reporting a hole.
    if v.get("FE_BUB") is None and any(v.get(k) is not None for k, _ in FE_SUB):
        v["FE_BUB"] = sum(v.get(k) or 0 for k, _ in FE_SUB)
        v["_FE_BUB_DERIVED"] = True

    cyc, ins = g("CYCLES"), g("INSTRET")
    if not cyc or not ins:
        sys.exit("error: need both cycles and instructions; got %s" % sorted(v))
    from_causes(v, cyc)
    if topdown(v, width, R):
        if unknown:
            R.text("\n  ignored unrecognised events: %s" % ", ".join(sorted(set(unknown))))
        R.show()
        return

    cpi, per_k = cyc / ins, lambda n: 1000.0 * n / ins
    fe_bub = g("FE_BUB")
    named  = sum(g(k) for k, _ in BACKEND) + fe_bub      # ST_DIV / ST_MUL are occupancies: not here
    floor  = ins / width                  # the dispatch floor: every cycle full
    unattr = cyc - floor - named          # cycles beyond the floor that nothing names
    no_backend = not any(v.get(k) is not None for k, _ in BACKEND)
    share = lambda n: "%7.3f  %5.1f%%" % (n / ins, 100.0 * n / cyc)    # cycles per insn, share of cycles

    R.text("  cycles %-16d instructions %-16d IPC %.3f   CPI %.3f" % (cyc, ins, ins/cyc, cpi))
    R.text("  (the wait-cycle view: its events overlap, so it is depth, not a partition -- the td sets close)")
    R.text("\n  wait cycles per instruction (dispatch is %d per cycle, so %.3f is the floor)"
           % (width, 1.0 / width))
    R.row(0, "dispatch floor (%d per cycle)" % width, "%7.3f  %5.1f%%" % (1.0 / width, 100.0*floor/cyc))
    for k, label in BACKEND:
        if v.get(k) is not None and g(k):
            R.row(0, "stall: " + label, share(g(k)))
            # the memory events sit beside ST_MEM, not inside it: each is its own condition
            if k == "ST_MEM" and any(v.get(kk) is not None for kk, _ in MEM_SUB + MEM_CNT + [("MEM_LDINFL", "")]):
                for kk, ll in MEM_SUB:
                    if v.get(kk) is not None:
                        R.row(1, "~ " + ll, share(g(kk)))
                if v.get("MEM_LDINFL") is not None:
                    R.row(1, "~ mean fast loads in flight", "%.2f" % (g("MEM_LDINFL")/cyc), "loads")
                    if g("MEM_LDSENT"):
                        R.row(1, "~ mean fast-load latency", "%.2f" % (g("MEM_LDINFL")/g("MEM_LDSENT")), "cycles")
                for kk, ll in MEM_CNT:
                    if v.get(kk) is not None:
                        R.row(1, "~ " + ll, "%.3f" % per_k(g(kk)), "per 1k insns")
                for kk, ll in (("MEM_LQOCC", "load queue"), ("MEM_SQOCC", "store queue")):
                    if v.get(kk) is not None:
                        R.row(1, "~ mean %s occupancy" % ll, "%.2f" % (g(kk)/cyc), "entries")
            # ST_DSP's disjoint parts (the `hold` set): a breakdown, never added alongside it.
            if k == "ST_DSP" and any(v.get(kk) is not None for kk, _ in DSP_SUB):
                for kk, ll in DSP_SUB:
                    if g(kk):
                        R.row(1, "- " + ll, share(g(kk)))
    for k, label in OCCUPANCY:
        if v.get(k) is not None:
            R.row(0, label, share(g(k)), "(occupancy, not in the sum)")
    if fe_bub:
        tag = " (derived)" if v.get("_FE_BUB_DERIVED") else ""
        R.row(0, "frontend bubble" + tag, share(fe_bub))
        other = fe_bub - sum(g(k) for k, _ in FE_SUB)
        for k, label in FE_SUB:
            if g(k):
                R.row(1, "- " + label, share(g(k)))
        # FE_MMU + FE_IC + FE_ALN + FE_QUE = FE_BUB in the RTL: a remainder is a broken event map
        if abs(other) > 0.0005 * cyc:
            R.row(1, "- other", share(other), "<-- unattributed")
    flag = ""
    if unattr > 0.005 * cyc:
        flag = ("<-- no backend events in this run: use perf-smol cpi" if no_backend
                else "<-- a cause with no counter (or events dropped from the set)")
    elif unattr < -0.005 * cyc:
        flag = "(events overlap: a cycle blocked on two units counts in both)"
    R.row(0, "unattributed", share(unattr), flag)

    mpki = []
    if any(v.get(k) is not None for k, _ in REDIR_SUB):
        tot = g("REDIR")
        for k, label in REDIR_SUB:
            if v.get(k) is not None:
                mpki.append(("redirect: " + label, "%.3f" % per_k(g(k)), ""))
        if tot:
            rest = tot - sum(g(k) for k, _ in REDIR_SUB)
            mpki.append(("redirect: other", "%.3f" % per_k(rest), "(xRET, sfence.vma, CSR-write redirects, fence.i, jal)"))
    if v.get("RD_WAIT") is not None:
        # a mispredict's restart, from its resolve to its release or its squash at the head
        mpki.append(("redirect drain, cycles/insn", "%.3f" % (g("RD_WAIT")/ins),
                     "(%.1f%% of cycles: a resolved mispredict's restart)" % (100.0*g("RD_WAIT")/cyc)))
    if v.get("DTLB_MISS") is not None:
        per_walk = "" if not g("DTLB_MISS") or v.get("DT_WALK") is None else "(%.1f busy cycles per start)" % (g("DT_WALK")/g("DTLB_MISS"))
        mpki.append(("dMMU walker starts", "%.3f" % per_k(g("DTLB_MISS")), per_walk))
    for code, label, acc in (("REDIR", "pipeline redirects", None),
                             ("DCMISS", "D$ line fills", "DCACC"),
                             ("ICMISS", "I$ misses", "ICACC")):
        if v.get(code) is None:
            continue
        rate = "" if not acc or not g(acc) else "(%.3f%% of %s accesses)" % (
            100.0*g(code)/g(acc), acc[:2])
        mpki.append((label, "%.3f" % per_k(g(code)), rate))
    if mpki:
        R.text("\n  MPKI (per 1000 instructions)")
        for label, val, note in mpki:
            R.row(0, label, val, note)
    else:
        R.text("\n  MPKI: no redirect / miss / drain events in this run (perf-smol.sh cpi, br, mem)")

    for code, label in (("LOAD", "loads"), ("STORE", "stores")):
        if g(code):
            R.text("  %-8s %12d  (%.1f%% of instructions)" % (label, g(code), 100.0*g(code)/ins))
    if unknown:
        R.text("\n  ignored unrecognised events: %s" % ", ".join(sorted(set(unknown))))
    R.show()

main()
