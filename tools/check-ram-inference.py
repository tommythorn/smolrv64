#!/usr/bin/env python3
"""Fail the build if an array that MUST be a RAM came back as flops.

WHY THIS EXISTS.  Whether an array becomes a RAM or a mux over flops is decided by
synthesis, silently, from the array's ACCESS PATTERN -- and the difference is invisible in
the RTL.  Two mechanisms demote an array, both of them innocuous-looking edits:

  broadcast READ    reading every entry in a loop (a wakeup CAM, a search, a priority pick).
                    Distributed RAM has one read port per instance, so this forces flops.
                    smolrv64_iq.v's e_ps was demoted this way and became the tail of the
                    critical path; e_prd and e_rob, in the SAME module with the same entry
                    count, stayed RAM because they are read only at [sel].

  broadcast WRITE   writing every entry in one cycle -- almost always a `for` loop in a
                    reset branch.  No RAM can do it.  This pinned smolrv64_rename's free lists
                    and rename maps (~3,400 bits) into flops purely to initialise them,
                    which configuration does for free on an FPGA.

Neither shows up in lint, in a testbench, or in the cosim.  It shows up as slack, months
later, on a design that is 65-83% route-bound (docs/rtl-rules.md I7).  So the intent is
recorded here and CHECKED, rather than left to be rediscovered.

Usage:  check-ram-inference.py <synthesis .vds> [manifest]
        manifest defaults to tools/ram-manifest.txt next to this script. A count `x{expr}` is
        evaluated in IW, the environment's SMOLRV64_IW (the build sets it to the width it built).
Exit 0 = every required array is a RAM.  Exit 1 = at least one regressed.
"""
import re
import sys
import os


def inferred_rams(logpath):
    """Names of arrays synthesis implemented as RAM, from BOTH places it says so.

    The RAM utilization TABLE is authoritative.  The `The RAM "..."` INFO messages are NOT
    complete -- they omitted the PRF and the BTB when this was first written, which produced
    two false 'this is flops' conclusions.  Block RAMs are reported in a third form again.
    """
    names = set()
    final = set()        # (scope, path) of every array in the FINAL mapping reports: once each
    in_final = False
    in_block = False     # inside a Block RAM mapping table: every row of it is a block RAM
    with open(logpath, errors="ignore") as fh:
        for line in fh:
            m0 = re.match(r"(Distributed|Block) RAM: (Final|Preliminary) Mapping Report", line)
            if m0:
                in_final = "Final" in line
                in_block = m0.group(1) == "Block"
            elif line.strip() and not line.startswith(("|", "+")):
                in_block = False
            # 1. utilization table: |module | path/name_reg | Implied | D x W | RAM32M16 x n |
            #    A Block RAM table's true-dual-port rows carry their RAMB counts in later columns
            #    (|module | name_reg | A: D x W | W | R | B: D x W | W | R | ... | RAMB18 | RAMB36 |),
            #    so in that table every row is a RAM, whatever its fifth column says.
            if line.startswith("|"):
                f = [x.strip() for x in line.split("|")]
                blk = in_block and len(f) >= 3 and f[2] not in ("", "RTL Object")
                if blk or (len(f) >= 6 and re.search(r"RAM\d|RAMB|BRAM", f[5])):
                    names.add(_base(f[2]))
                    if in_final:
                        final.add((f[1], _path(f[2])))
            # 2. INFO: ... The RAM "scope/name_reg" ... (incomplete, but free to include)
            m = re.search(r'The RAM "([^"]+)"', line)
            if m:
                names.add(_base(m.group(1)))
            # 3. INFO: ... instance <path>/name_reg_bram_0 (implemented as a Block RAM)
            m = re.search(r"instance (\S+) \(implemented as a Block RAM\)", line)
            if m:
                names.add(_base(re.sub(r"_bram_\d+$", "", m.group(1))))
    return names, final


def _path(qualified):
    """The array's path inside its report row's scope, its _reg and cascade index stripped."""
    n = re.sub(r"_\d+$", "", qualified.strip())
    return n[:-4] if n.endswith("_reg") else n


def _base(qualified):
    n = qualified.split("/")[-1].strip()
    # A deep block RAM is split into a cascade of primitives named <array>_reg_0, _reg_1, ...
    # (e.g. the 8K BTB -> btb_reg_0..13). Strip that trailing cascade index so it reduces to
    # the array base like the single-primitive case. This only broadens what counts as a RAM,
    # so it can never turn a genuine flop-demotion into a false pass.
    n = re.sub(r"_\d+$", "", n)
    return n[:-4] if n.endswith("_reg") else n


def load_manifest(path):
    req = []
    with open(path) as fh:
        for raw in fh:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            name, _, why = line.partition(":")
            req.append((name.strip(), why.strip()))
    return req


def required_present(entry, have, final):
    """An entry names one array, or `block[*].array xN`: N generate instances, every one a RAM.
    The N are counted in the final mapping reports, where a row is one array in its scope, so a
    module instantiated more than once (smolrv64_bankq, smolrv64_mwram) counts each instance's
    banks; an array those reports do not name (a block RAM reported only by instance) is counted
    by its name, as before."""
    m = re.fullmatch(r"(.+)\s+x(\d+|\{[^}]*\})", entry)
    if not m:
        return entry in have
    want = m.group(2)
    if want.startswith("{"):
        if "SMOLRV64_IW" not in os.environ:
            sys.exit("check-ram-inference: %s needs SMOLRV64_IW, the width the build used" % entry)
        want = eval(want[1:-1], {}, {"IW": int(os.environ["SMOLRV64_IW"])})
    pat = re.compile(r"(^|/)" + re.escape(m.group(1).strip()).replace(r"\[\*\]", r"\[\d+\]") + "$")
    n = sum(1 for _, p in final if pat.search(p))
    if n == 0:
        n = sum(1 for p in have if pat.search(p))
    return n == int(want)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: check-ram-inference.py <synthesis .vds> [manifest]")
    log = sys.argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    manifest = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "ram-manifest.txt")
    if not os.path.exists(log):
        print("ram-check: no synthesis log at %s -- not a pass" % log)
        return 1
    have, final = inferred_rams(log)
    if not have:
        print("ram-check: parsed NO rams out of %s." % log)
        print("           That is not a pass -- the log format changed or synthesis did not")
        print("           run. A check that cannot fail must not report success.")
        return 1
    missing = [(n, w) for n, w in load_manifest(manifest) if not required_present(n, have, final)]
    if missing:
        print("\n*** RAM INFERENCE REGRESSED ***")
        for n, why in missing:
            print("    %-10s is no longer a RAM%s" % (n, (" -- " + why) if why else ""))
        print("""
    An array in tools/ram-manifest.txt was demoted to flops. Something now reads EVERY
    entry (a loop, a search, a CAM-style compare) or writes every entry in one cycle
    (usually a `for` loop in a reset branch). Find it and index the array instead, or --
    if the broadcast is genuinely required -- delete the line from the manifest AND say
    in the commit message why that array is allowed to be flops now.

    See docs/rtl-rules.md I7. Arrays inferred this run: %s""" % " ".join(sorted(have)))
        return 1
    print("ram-check: %d required arrays all inferred as RAM" % len(load_manifest(manifest)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
