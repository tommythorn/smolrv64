#!/usr/bin/env python3
"""Instructions per Geekbench subtest, per build, from runs stamped by /var/tmp/sv-gb7.sh:
every "Running X" line and the stop line carry the epoch second and the hart's instret
(read by /var/tmp/rdinstret), so a subtest's count is the difference to the next stamp.
Under Simmerv instret is the simulator's own instruction count, which makes this the one
meaningful number there. Prints one row per subtest with each build's count and the ratio
to the first build, so the B/Zicond/V builds' savings read directly.

  tools/gb7-mix.py /srv/ubuntu-root/var/tmp/sv-gb7-{rv64gc,rv64gcbv,rva23}-raw.log
"""
import re, sys

def per_subtest(raw):
    stamps = []                                   # (instret, name or None)
    for line in open(raw, errors='replace'):
        m = re.match(r'\d+ (\d+) (?:.*?)Running (.+?)\s*$', line)
        if m: stamps.append((int(m.group(1)), m.group(2).strip())); continue
        m = re.match(r'\d+ (\d+) === (?:single-core done|end)', line)
        if m: stamps.append((int(m.group(1)), None)); break   # multi-core repeats the names: stop here
    res = {}
    for i, (n0, name) in enumerate(stamps):
        if name is None or i + 1 >= len(stamps): continue
        res[name] = stamps[i + 1][0] - n0
    return res

if __name__ == '__main__':
    builds = []
    for raw in sys.argv[1:]:
        b = re.search(r'sv-gb7-([a-z0-9]+)-raw', raw)
        builds.append((b.group(1) if b else raw, per_subtest(raw)))
    names = list(builds[0][1])
    w = max(len(n) for n in names) + 2
    print(f"{'subtest':<{w}}" + "".join(f"{b:>14}" for b, _ in builds) + "".join(f"{b + '/' + builds[0][0]:>16}" for b, _ in builds[1:]))
    tot = [0] * len(builds)
    for n in names:
        base = builds[0][1].get(n, 0)
        row = f"{n:<{w}}"
        for i, (_, d) in enumerate(builds):
            tot[i] += d.get(n, 0); row += f"{d.get(n, 0) / 1e9:>12.2f} G"
        row += "".join(f"{(d.get(n, 0) / base if base else 0):>15.2f}x" for _, d in builds[1:])
        print(row)
    print(f"{'total':<{w}}" + "".join(f"{t / 1e9:>12.2f} G" for t in tot) + "".join(f"{(t / tot[0] if tot[0] else 0):>15.2f}x" for t in tot[1:]))
