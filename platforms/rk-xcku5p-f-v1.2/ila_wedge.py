#!/usr/bin/env python3
"""Decode probe7 (backend_top's dbg_wedge) from an ILA capture.

The board freezes in U-mode with a pending+enabled S-timer interrupt it never takes and
fetch parked on a single PA. A constant fetch PA means the frontend is frozen, which means
accept=0, which means a bundle is ready but can_dispatch is false. This prints which term
is holding it, and whether commit is still advancing.

Prediction to check: cc_full=1 with cc_commit never pulsing and cc_committed constant --
i.e. a checkpoint whose count never reaches zero (a lost deferred op), which blocks commit,
fills the checkpoints, blocks dispatch, freezes the frontend, and starves interrupt
injection. Any other pattern (e.g. !disp_ready or fe_stall alone) points elsewhere.

    Usage: ila_wedge.py <capture.csv>
"""
import csv, sys, collections

BITS = [
    (3,  "any_valid"),      (4,  "can_dispatch"),  (5,  "accept"),
    (6,  "cc_full"),        (7,  "fe_stall"),      (8,  "sched_not_ready"),
    (9,  "sb_full"),        (10, "lq_full"),       (11, "eb_redirect"),
    (12, "lsu_dfault_v"),   (13, "ill_v"),         (14, "cc_commit"),
    (15, "cc_empty"),       (16, "fe_red_v"),      (17, "immu_ready"),
    (18, "immu_fault"),     (19, "icache_empty"),  (20, "csr_irq_v"),
    (21, "inject_inflight"),(22, "irq_inject"),    (23, "roll_v"),
    (24, "replay_v"),       (25, "devld_solo_v"),  (26, "pend_iflt"),
    (27, "dflt_replay"),    (28, "dflt_fire"),     (29, "iflt_fire"),
    # what commit is waiting on (see backend_top dbg_wedge [49:38])
    (39, "cc_stall_barrier"), (40, "amo_gap"),     (41, "devrd_pending"),
    (42, "amo_busy"),       (43, "sb_any"),        (44, "lq_any"),
    (45, "unit_busy"),      (46, "rs_any_live"),   (47, "rs_stuck_not_elig"),
]


def num(s):
    s = s.strip()
    try:
        return int(s, 16)
    except ValueError:
        return int(s, 2)


def main(path):
    rows = list(csv.reader(open(path, newline="")))
    hdr = next(i for i, r in enumerate(rows) if r and "Sample in Buffer" in r[0])
    cols = rows[hdr]
    data = [r for r in rows[hdr + 1:]
            if r and len(r) == len(cols) and not r[0].startswith("Radix")]
    col = next((c for c in cols if "wedge" in c or "probe7" in c), None)
    if col is None:
        sys.exit(f"no wedge probe in {path}; columns: {cols}")
    vals = [num(r[cols.index(col)]) for r in data]
    print(f"samples: {len(vals)}")

    # Per-bit: how many samples asserted. A wedge shows constants, not traffic.
    print("\nbit                 asserted/total")
    for pos, name in BITS:
        n = sum((v >> pos) & 1 for v in vals)
        mark = "  <== CONSTANT" if n in (0, len(vals)) else ""
        print(f"  {name:<18} {n:>6}/{len(vals)}{mark}")

    cur = collections.Counter((v >> 30) & 0xF for v in vals)
    cmt = collections.Counter((v >> 34) & 0xF for v in vals)
    print(f"\ncur ckpt      : {dict(cur)}")
    print(f"committed ckpt: {dict(cmt)}")
    cnt = collections.Counter((v >> 48) & 0x3 for v in vals)
    print(f"count[committed] : {dict(cnt)}   (commit fires only at 0)")
    commits = sum((v >> 14) & 1 for v in vals)
    print(f"cc_commit pulses in window: {commits}")
    if commits == 0 and len(cmt) == 1:
        print("  -> commit is NOT advancing: a checkpoint's count never reaches zero.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ila_now.csv")
