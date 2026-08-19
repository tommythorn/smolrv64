#!/usr/bin/env python3
"""Generate docs/smolrv64-perf-events.json from the event map in src/csr_file.v.

The JSON is a Linux perf event file: `perf stat -e smolrv64/ST_MEM/` resolves a NAME
through it to a raw EventCode. It used to be maintained by hand, and had drifted to
describe a core that no longer exists -- TLB_LOOKUP/CACHE_FILL_BEAT/AXI_READ are
implemented nowhere, and its 0x0300..0x0304 named TLB events while the RTL counts the
in-order stall attribution there. Anyone resolving by name got the wrong counter.

csr_file.v is the only place the mapping is real, so it is the source. Run:

    tools/gen-perf-events.py            # rewrite docs/smolrv64-perf-events.json
    tools/gen-perf-events.py --check    # exit 1 if the JSON is stale (for CI/lint)
"""
import json, re, sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC  = ROOT / "src" / "csr_file.v"
OUT  = ROOT / "docs" / "smolrv64-perf-events.json"

# one event per line:  HPMEV_NAME = 16'hCODE,   // description
EV = re.compile(r"HPMEV_(\w+)\s*=\s*16'h([0-9a-fA-F]{4})\s*[,;]\s*//\s*(.+?)\s*$", re.M)

def expand(desc, prev):
    """'...on the divider' continues the previous description rather than repeating it."""
    if not desc.startswith("...") or not prev:
        return desc
    tail = desc[3:]
    for sep in (" on the ", ": "):
        if sep in prev:
            head = prev.split(sep)[0]
            return f"{head} {tail}" if sep == " on the " else f"{head}: {tail}"
    return f"{prev} {tail}"

def build():
    events, prev = [], None
    for name, code, desc in EV.findall(SRC.read_text()):
        desc = expand(desc, prev); prev = desc
        events.append({"EventName": name, "EventCode": f"0x{code.lower()}",
                       "BriefDescription": desc})
    if not events:
        sys.exit(f"{SRC}: no HPMEV_ definitions matched -- did the block format change?")
    return events

def main():
    events = build()
    text = json.dumps(events, indent=2) + "\n"
    if "--check" in sys.argv:
        cur = OUT.read_text() if OUT.exists() else ""
        if cur != text:
            sys.exit(f"{OUT} is stale -- run tools/gen-perf-events.py")
        print(f"perf-events: up to date ({len(events)} events)")
        return
    OUT.write_text(text)
    print(f"perf-events: wrote {OUT.relative_to(ROOT)} ({len(events)} events)")

main()
