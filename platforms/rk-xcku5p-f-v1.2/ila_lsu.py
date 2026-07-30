#!/usr/bin/env python3
"""Decode probe5 (lsu.v dbg_lsu) from an ILA capture.

The live bug is a fault-delivery deadlock: a data fault is latched (df_v) for an op whose
checkpoint is NOT the oldest live one, so it can never be delivered (df_oldest is false in
backend_top), while lsu_dfault_v simultaneously freezes dispatch. If the OLDEST checkpoint's
load/store then never completes, commit can never advance to the faulting checkpoint.

lsu.v:657 documents this class and guards two ways in (concurrent load/store faults -> pick the
older; an older fault preempting a latched younger one). This says which op is actually stuck:
compare the latched fault's ckpt/seq against the selected load's and store's.

    Usage: ila_lsu.py <capture.csv>
"""
import csv, sys, collections

BITS = [
    (0, "ld_sel_v"),   (1, "ldx_ready"),   (2, "ldx_fault"),  (3, "ld_xpage"),
    (4, "ld_xok"),     (5, "ld_is_dev"),   (6, "ld_committed"),(7, "ld_olds_any"),
    (8, "ld_olds_same"),(9, "sel_fire"),   (10, "merge_adv"), (11, "p_v"),
    (12, "mem_rvalid"),(13, "amo_v"),      (14, "ck_v"),      (15, "st_need_xl"),
    (16, "stx_ready"), (17, "stx_fault"),  (18, "st_xpage"),  (19, "st_ck_done"),
    (20, "st_ck_flt"), (21, "df_v"),       (22, "ld_xflt"),   (23, "amo_need_xl"),
    (57, "xlate"),     (58, "rollback"),
]
FIELDS = [("ld_ckpt", 24, 0x7), ("st_ckpt", 27, 0x7), ("flt_ckpt", 30, 0x7),
          ("ld_seq", 33, 0xFF), ("st_seq", 41, 0xFF), ("flt_seq", 49, 0xFF)]


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
    col = next((c for c in cols if "lsu" in c.lower() or "probe5" in c), None)
    if col is None:
        sys.exit(f"no LSU probe in {path}; columns: {cols}")
    vals = [num(r[cols.index(col)]) for r in data]
    print(f"samples: {len(vals)}\n")
    print("bit                asserted/total")
    for pos, name in BITS:
        n = sum((v >> pos) & 1 for v in vals)
        mark = "  <== CONSTANT" if n in (0, len(vals)) else ""
        print(f"  {name:<16} {n:>6}/{len(vals)}{mark}")
    print()
    for name, sh, mask in FIELDS:
        c = collections.Counter((v >> sh) & mask for v in vals)
        print(f"  {name:<9}: {dict(c)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ila_frozen.csv")
