#!/usr/bin/env python3
"""Render the Geekbench 5 single-core score over time as an SVG for the front of the README.

  tools/gb5-progress-chart.py workloads/gb5/progress.tsv > docs/images/gb5-progress.svg

Reads `YYYYMMDD<TAB>score` rows ('#' lines are comments). The x axis is calendar time, so the
spacing between results is honest; each result is a point, joined in order, and a score is
labelled where it first appears. No dependencies; the SVG is written by hand, like
tools/gb5-chart.py, so the chart is reproducible from the table alone.
"""
import datetime, math, sys

def read(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"): continue
        d, s = line.split()
        rows.append((datetime.date(int(d[:4]), int(d[4:6]), int(d[6:])), float(s)))
    return sorted(rows)

def svg(rows):
    w, h = 640, 300
    x0, x1, y0, y1 = 48, w - 24, h - 40, 52            # plot box: left, right, bottom, top
    d0 = rows[0][0].replace(day=1)
    last = rows[-1][0]
    d1 = (last.replace(day=1) + datetime.timedelta(days=32)).replace(day=1)   # the next month
    days = (d1 - d0).days
    ymax = max(2, math.ceil(max(s for _, s in rows) + 1))
    X = lambda d: x0 + (x1 - x0) * (d - d0).days / days
    Y = lambda s: y0 - (y0 - y1) * s / ymax
    out = [f'<text x="12" y="24" font-size="15" font-weight="bold">Geekbench 5 single-core score</text>',
           f'<text x="12" y="41" fill="#555">SmolRV64 on the XCKU5P board at 166.67 MHz, one point per result</text>']
    for s in range(0, ymax + 1, 1 if ymax <= 8 else 2):                      # gridlines + y labels
        out.append(f'<line x1="{x0}" y1="{Y(s):.1f}" x2="{x1}" y2="{Y(s):.1f}" stroke="#e2e2e2" stroke-width="1"/>')
        out.append(f'<text x="{x0-8}" y="{Y(s)+4:.1f}" text-anchor="end" fill="#555">{s}</text>')
    m = d0
    while m <= d1:                                                           # month ticks
        out.append(f'<line x1="{X(m):.1f}" y1="{y0}" x2="{X(m):.1f}" y2="{y0+5}" stroke="#555555" stroke-width="1"/>')
        if m < d1:
            mid = X(m + datetime.timedelta(days=15))
            out.append(f'<text x="{mid:.1f}" y="{y0+20}" text-anchor="middle" fill="#555">{m.strftime("%b %Y")}</text>')
        m = (m + datetime.timedelta(days=32)).replace(day=1)
    out.append(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}" stroke="#555555" stroke-width="1"/>')
    path = "M " + " L ".join(f"{X(d):.1f} {Y(s):.1f}" for d, s in rows)
    out.append(f'<path d="{path}" fill="none" stroke="#2b6cb0" stroke-width="2"/>')
    seen = set()
    for d, s in rows:
        out.append(f'<circle cx="{X(d):.1f}" cy="{Y(s):.1f}" r="3.5" fill="#2b6cb0"/>')
        if s not in seen:
            seen.add(s)
            lab = f"{s:g}"
            out.append(f'<text x="{X(d)-7:.1f}" y="{Y(s)-8:.1f}" text-anchor="end" font-weight="bold" '
                       f'fill="#2b6cb0">{lab}</text>')
    body = "\n".join(out)
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
            f'font-family="Helvetica, Arial, sans-serif" font-size="12">\n'
            f'<rect width="{w}" height="{h}" fill="#ffffff"/>\n{body}\n</svg>\n')

if __name__ == "__main__":
    sys.stdout.write(svg(read(sys.argv[1])))
