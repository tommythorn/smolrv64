#!/usr/bin/env python3
"""Render the per-subtest Geekbench 5 single-core scores as an SVG bar chart for the README.

  tools/gb5-chart.py workloads/gb5/result-2026-09-07.md > docs/images/gb5-single-core.svg

Reads the `| subtest | score | rate |` table the result files keep, groups the rows by
Geekbench's own category (Crypto / Integer / Floating Point), colours each bar by it, and
prints the measured rate after the score, because a score of 1 spans a wide range of rates.
Horizontal bars, so no text is rotated: every renderer agrees on that layout. No
dependencies; the SVG is written by hand so the chart is reproducible from the table alone.
"""
import re, sys

CATS = [("Crypto", "#b5651d", ["AES-XTS"]),
        ("Integer", "#2b6cb0", ["Text Compression", "Image Compression", "Navigation", "HTML5",
                               "SQLite", "PDF Rendering", "Text Rendering", "Clang", "Camera"]),
        ("Floating Point", "#2f855a", ["N-Body Physics", "Rigid Body Physics", "Gaussian Blur",
                                      "Face Detection", "Horizon Detection", "Image Inpainting",
                                      "HDR", "Ray Tracing", "Structure from Motion",
                                      "Speech Recognition", "Machine Learning"])]

def read(path):
    rows = {}
    for line in open(path):
        m = re.match(r"\|\s*([^|]+?)\s*\|\s*(\d+)\s*\|\s*([^|]*?)\s*\|", line)
        if m: rows[m.group(1).strip("* ")] = (int(m.group(2)), m.group(3))
    return rows

def svg(rows, title, subtitle):
    left, bar0, rowh, barmax, w = 150, 160, 18, 260, 640
    ymax = 10
    lines = [f'<text x="12" y="24" font-size="15" font-weight="bold">{title}</text>']
    for i, s in enumerate(subtitle):
        lines.append(f'<text x="12" y="{41+14*i}" fill="#555">{s}</text>')
    y = 46 + 14 * len(subtitle)
    for cat, col, names in CATS:
        lines.append(f'<text x="12" y="{y+12}" font-weight="bold" fill="{col}">{cat}</text>')
        y += rowh
        for name in names:
            score, rate = rows.get(name, (0, "?"))
            bw = score * barmax / ymax
            lines.append(f'<text x="{left-6}" y="{y+13}" text-anchor="end">{name}</text>')
            lines.append(f'<rect x="{bar0}" y="{y+3}" width="{bw:.1f}" height="{rowh-6}" fill="{col}"/>')
            lines.append(f'<text x="{bar0+bw+6:.1f}" y="{y+13}" font-weight="bold">{score}</text>')
            lines.append(f'<text x="{bar0+barmax+30}" y="{y+13}" fill="#666">{rate}</text>')
            y += rowh
        y += 6
    for s in range(0, ymax + 1, 2):
        x = bar0 + s * barmax / ymax
        lines.append(f'<line x1="{x:.1f}" y1="{40+14*len(subtitle)}" x2="{x:.1f}" y2="{y}" stroke="#ddd"/>')
        lines.append(f'<text x="{x:.1f}" y="{y+14}" text-anchor="middle" fill="#555">{s}</text>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{y+22}" '
            f'font-family="Helvetica, Arial, sans-serif" font-size="12">\n'
            f'<rect width="100%" height="100%" fill="white"/>\n' + "\n".join(lines) + "\n</svg>")

if __name__ == "__main__":
    path = sys.argv[1]
    rows = read(path)
    date = re.search(r"(\d{4}-\d{2}-\d{2})", path)
    print(svg(rows, "Geekbench 5.4.1 single-core, score per subtest",
              [f"SmolRV64 OOO2 on the XCKU5P at 166.67 MHz, {date.group(1) if date else path}",
               "single-core 5: Integer 5, Crypto 1, Floating Point 0 (a geometric mean; one 0 zeroes it)"]))
