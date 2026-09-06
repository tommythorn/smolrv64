#!/usr/bin/env python3
# Compare two probe logs cycle by cycle (hex fields normalised), print the first differing cycles.
import re, sys
def load(p):
    d = {}
    for l in open(p, errors='replace'):
        m = re.match(r'\[(\d+)\] P (.*)', l)
        if not m: continue
        cyc = int(m.group(1)); fields = {}
        for grp in m.group(2).split(' '):
            k, v = grp.split('=', 1)
            fields[k] = [x.lstrip('0') or '0' for x in v.lower().split(',')]
        d[cyc] = fields
    return d
a, b = load(sys.argv[1]), load(sys.argv[2])
off = int(sys.argv[4]) if len(sys.argv) > 4 else 0
b = {c - off: v for c, v in b.items()}   # B sampled after the edge: B[c] ~ A[c+off]
names = {'rn': 'r_valid,r_rd,r_rs1,r_rs2,r_prd,r_sprs1,r_sprs2', 'rnb': 'r_valid_b,r_rd_b,r_prd_b',
         'prf': 'ra1,ra2,rd1,rd2,we_ie,wa_ie,wd_ie,we_ld,wa_ld,wd_ld', 'alu': 'rs1,rs2,imm,pc,result', 'rob': 'd_valid,d_idx,c_valid,d_valid2'}
shown = 0
for cyc in sorted(set(a) & set(b)):
    diffs = []
    for g in a[cyc]:
        for i, (x, y) in enumerate(zip(a[cyc][g], b[cyc].get(g, []))):
            if x != y: diffs.append(f"{g}.{names[g].split(',')[i]}: {x} vs {y}")
    if diffs:
        print(f"[{cyc}] " + " | ".join(diffs)); shown += 1
        if shown >= int(sys.argv[3]) if len(sys.argv) > 3 else 12: break
if not shown: print("IDENTICAL over", len(set(a) & set(b)), "cycles")
