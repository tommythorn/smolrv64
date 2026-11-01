#!/usr/bin/env python3
"""Group failing endpoints into families, across one or more draws (tools/timing-census.sh).

    tools/timing-census.py <census.txt>...    (each a timing-census.tcl output: a WNS line, then paths)

A family is (start module/register -> end module/register), the core's hierarchy prefix dropped and
bit indices folded. Per draw: its WNS, its endpoint count, and the worst slack inside the frontend
(core/fe) against everything else -- a cut that helps one side cannot move WNS past the other's.
Then the families, those failing in the most draws first: draws, endpoints, worst slack, levels.
"""
import collections, os, re, sys

def fam(pin):
    p = pin.replace('probe_core/core/', '').replace('probe_core/', '')
    p = re.sub(r'_reg.*', '', p)
    p = re.sub(r'\[\d+\]', '[]', p)
    parts = p.split('/')
    return '/'.join(parts[:2]) if len(parts) > 1 else parts[0]

cnt = collections.defaultdict(dict); worst = collections.defaultdict(dict); lv = collections.defaultdict(list)
print(f"{'draw':40s} {'WNS':>7s} {'endpts':>6s} {'frontend':>9s} {'rest':>7s}")
for path in sys.argv[1:]:
    lines = open(path).read().splitlines()
    name = os.path.splitext(os.path.basename(path))[0]
    wns = lines[0].split()[1] if lines and lines[0].startswith('WNS') else '?'
    fe, rest = [], []
    for l in lines[1:]:
        f = l.split()
        s = float(f[0]); k = fam(f[4]) + ' -> ' + fam(f[6])
        cnt[k][name] = cnt[k].get(name, 0) + 1
        worst[k][name] = min(worst[k].get(name, 0.0), s)
        lv[k].append(int(f[1][1:]))
        (fe if ('/core/fe/' in f[4] or '/core/fe/' in f[6]) else rest).append(s)
    print(f"{name:40s} {float(wns):+7.3f} {len(lines)-1:6d} {min(fe) if fe else 0.0:+9.3f} {min(rest) if rest else 0.0:+7.3f}")
rows = sorted(((len(cnt[k]), sum(cnt[k].values()), min(worst[k].values()), k) for k in cnt), key=lambda r: (-r[0], -r[1]))
print(f"\n{'draws':>5s} {'endpts':>6s} {'worst':>7s} {'levels':>7s}  family")
for nd, tot, w, k in rows[:40]:
    print(f"{nd:5d} {tot:6d} {w:+7.3f} {min(lv[k]):3d}-{max(lv[k]):<3d}  {k}")
