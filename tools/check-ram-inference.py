#!/usr/bin/env python3
"""Fail the build if an array that MUST be a RAM came back as flops.

WHY THIS EXISTS.  Whether an array becomes a RAM or a mux over flops is decided by
synthesis, silently, from the array's ACCESS PATTERN -- and the difference is invisible in
the RTL.  Two mechanisms demote an array, both of them innocuous-looking edits:

  broadcast READ    reading every entry in a loop (a wakeup CAM, a search, a priority pick).
                    Distributed RAM has one read port per instance, so this forces flops.
                    ooo2_iq.v's e_ps was demoted this way and became the tail of the
                    critical path; e_prd and e_rob, in the SAME module with the same entry
                    count, stayed RAM because they are read only at [sel].

  broadcast WRITE   writing every entry in one cycle -- almost always a `for` loop in a
                    reset branch.  No RAM can do it.  This pinned ooo2_rename's free lists
                    and rename maps (~3,400 bits) into flops purely to initialise them,
                    which configuration does for free on an FPGA.

Neither shows up in lint, in a testbench, or in the cosim.  It shows up as slack, months
later, on a design that is 65-83% route-bound (docs/rtl-rules.md I7).  So the intent is
recorded here and CHECKED, rather than left to be rediscovered.

Usage:  check-ram-inference.py <synth-runme.log> [manifest]
        manifest defaults to tools/ram-manifest.txt next to this script.
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
    with open(logpath, errors="ignore") as fh:
        for line in fh:
            # 1. utilization table: |module | path/name_reg | Implied | D x W | RAM32M16 x n |
            if line.startswith("|"):
                f = [x.strip() for x in line.split("|")]
                if len(f) >= 6 and re.search(r"RAM\d|RAMB|BRAM", f[5]):
                    names.add(_base(f[2]))
            # 2. INFO: ... The RAM "scope/name_reg" ... (incomplete, but free to include)
            m = re.search(r'The RAM "([^"]+)"', line)
            if m:
                names.add(_base(m.group(1)))
            # 3. INFO: ... instance <path>/name_reg_bram_0 (implemented as a Block RAM)
            m = re.search(r"instance (\S+) \(implemented as a Block RAM\)", line)
            if m:
                names.add(_base(re.sub(r"_bram_\d+$", "", m.group(1))))
    return names


def _base(qualified):
    n = qualified.split("/")[-1].strip()
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


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: check-ram-inference.py <synth-runme.log> [manifest]")
    log = sys.argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    manifest = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "ram-manifest.txt")
    if not os.path.exists(log):
        print("ram-check: no synth log at %s -- SKIPPED" % log)
        return 0
    have = inferred_rams(log)
    if not have:
        print("ram-check: parsed NO rams out of %s." % log)
        print("           That is not a pass -- the log format changed or synthesis did not")
        print("           run. A check that cannot fail must not report success.")
        return 1
    missing = [(n, w) for n, w in load_manifest(manifest) if n not in have]
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
