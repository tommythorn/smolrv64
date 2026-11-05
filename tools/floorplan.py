#!/usr/bin/env python3
"""Draw a placed design coloured by block: floorplan.py <dir> [out.png] [title]

<dir> is what report_floorplan.tcl wrote (`make floorplan`).  Each slice is filled with the
colour of the block holding most of it (a LUT site 1, a flop 1/2), grey where empty; block RAM,
DSP and UltraRAM sites are drawn at their height in CLB rows, a RAMB18 as half a RAMB36.  A
cell's block is the first BLOCKS entry whose pattern matches its hierarchical name.  Each block's
tag sits where it is densest.  The census, printed and drawn, counts LUT sites (a LUT6_2 or a
LUTRAM bit once), flops, RAMB36 equivalents and DSPs.
"""
import re
import sys
from collections import defaultdict
from PIL import Image, ImageDraw, ImageFont

C = 'probe_core/core/'
BLOCKS = [  # label, tag, colour, pattern on the cell's hierarchy
    ('fe: decoupling queue',           'queue',  '#1f4fb4', C + r'fe/u_q/'),
    ('fe: predictor + training queue', 'BP',     '#4f8fe8', C + r'(fe/u_bp|u_tq)/'),
    ('fe: fetch ring',                 'ring',   '#8cc4ff', C + r'fe/u_ring/'),
    ('fe: fetch, aligner, own',        'fe',     '#00b4d8', C + r'fe/'),
    ('I$ + iTLB',                      'I$',     '#0f8b8d', r'probe_core/(u_icache|u_ic_mem|core/u_immu)/'),
    ('D$',                             'D$',     '#8c564b', r'probe_core/u_dcache/'),
    ('L2',                             'L2',     '#00e676', r'probe_core/u_l2/'),
    ('LSU + dTLB + landing buffers',   'LSU',    '#f2a541', C + r'(u_lsu|u_la|u_sa|g_lp\[\d+\]\.u_lbd)/'),
    ('SQ',                             'SQ',     '#ff6f00', C + r'u_sq/'),
    ('LQ',                             'LQ',     '#ffd08a', C + r'u_lq/'),
    ('PRF',                            'PRF',    '#d62728', C + r'u_prf/'),
    ('lanes: ALU, branch, mul, div',   'lanes',  '#ff9e9e', C + r'(ln\[\d+\]\.u_(x|mul)|u_div)/'),
    ('issue queues + pending',         'IQ',     '#e84393', C + r'(ln\[\d+\]\.u_iq|u_iq_f|u_pend)/'),
    ('rename',                         'rename', '#c39bd3', C + r'u_rename/'),
    ('ROB',                            'ROB',    '#8e44ad', C + r'u_rob/'),
    ('FPU',                            'FPU',    '#2ca02c', C + r'u_fpu/'),
    ('CSR',                            'CSR',    '#b5d33d', C + r'u_csr/'),
    ('core: own logic',                'core',   '#ffe135', C),
    ('DDR4 controller',                'DDR4',   '#4d4d4d', r'u_ddr4_0/'),
    ('SoC + devices',                  'SoC',    '#9e9e9e', r''),
]
UNUSED = '#e6e6e6'
PATS = [re.compile(p) for *_, p in BLOCKS]
RH = 9                                                              # a CLB row's height (px)
WIDTH = {'SLICE': 7, 'RAMB36': 9, 'DSP48E2': 9, 'URAM288': 11}     # a column's width (px)
ROWS = {'SLICE': 1, 'RAMB36': 5, 'DSP48E2': 2.5, 'URAM288': 3.75}  # CLB rows a site spans
SITE = re.compile(r'^(SLICE|RAMB36|RAMB18|DSP48E2|URAM288)_X(\d+)Y(\d+)$')
BIN = 80                                                            # tag placement grid (px)


def block(name, memo={}):
    head = re.sub(r'^shell/', '', name).rsplit('/', 1)[0] + '/'   # every board's SoC is `shell`
    if head not in memo:
        memo[head] = next(i for i, p in enumerate(PATS) if p.match(head))
    return memo[head]


def main():
    d = sys.argv[1]
    png = sys.argv[2] if len(sys.argv) > 2 else d + '/floorplan.png'
    title = sys.argv[3] if len(sys.argv) > 3 else 'post-route placement'

    grid = {}
    for ln in open(d + '/cols.tsv'):
        k, x, g = ln.split()
        grid[(k, int(x))] = int(g)
    sites, crx, cry = [], {}, defaultdict(lambda: [1 << 30, -1])
    for ln in open(d + '/sites.tsv'):
        s, cr = ln.split()
        k, x, y = SITE.match(s).groups()
        x, y = int(x), int(y)
        sites.append((k, x, y))
        cx, cy = map(int, re.match(r'X(\d+)Y(\d+)', cr).groups())
        crx[(k, x)] = cx
        if k == 'SLICE':
            r = cry[cy]; r[0] = min(r[0], y); r[1] = max(r[1], y)
    nrows = max(y for k, x, y in sites if k == 'SLICE') + 1

    # columns left to right, flush; a grid jump wider than a column break (3) is hard IP
    xpos, px, prev = {}, 8, None
    for g, col in sorted((g, c) for c, g in grid.items()):
        if prev is not None:
            px += 6 if g - prev > 3 else 0
        xpos[col] = px
        px += WIDTH[col[0]]
        prev = g
    fw, top = px + 8, 40
    fh = top + nrows * RH + 8

    def ypx(row):                       # the bottom edge of CLB row `row`
        return int(round(top + (nrows - row) * RH))

    luts, weight, owner = set(), defaultdict(lambda: defaultdict(float)), {}
    nlut, nff, nbram, ndsp = [defaultdict(int) for _ in range(4)]
    for ln in open(d + '/cells.tsv'):
        name, ref, site, bel = ln.rstrip('\n').split('\t')
        m = SITE.match(site)
        if not m:
            continue
        k, x, y = m.group(1), int(m.group(2)), int(m.group(3))
        b = block(name)
        bel = bel.rsplit('.', 1)[-1]
        if k == 'SLICE':
            if bel[1:] in ('6LUT', '5LUT') and (x, y, bel[0]) not in luts:
                luts.add((x, y, bel[0]))
                nlut[b] += 1
                weight[('SLICE', x, y, None)][b] += 1
            elif 'FF' in bel:
                nff[b] += 1
                weight[('SLICE', x, y, None)][b] += 0.5
        elif k == 'RAMB18':             # the lower or upper half of RAMB36_X<x>Y<y/2>
            if ('RAMB36', x, y // 2, y % 2) not in owner:
                owner[('RAMB36', x, y // 2, y % 2)] = b
                nbram[b] += 0.5
        elif (k, x, y, None) not in owner:
            owner[(k, x, y, None)] = b
            nbram[b] += k == 'RAMB36'
            ndsp[b] += k == 'DSP48E2'
    for key, w in weight.items():
        owner[key] = max(w, key=w.get)

    img = Image.new('RGB', (fw + 560, fh), 'white')
    dr = ImageDraw.Draw(img)
    font = ImageFont.truetype('DejaVuSans.ttf', 15)
    bold = ImageFont.truetype('DejaVuSans-Bold.ttf', 18)
    tagf = ImageFont.truetype('DejaVuSans-Bold.ttf', 14)
    small = ImageFont.truetype('DejaVuSans.ttf', 11)
    col = [c for _, _, c, _ in BLOCKS]
    dens = defaultdict(lambda: defaultdict(lambda: [0.0, 0.0, 0.0]))   # block -> bin -> [w, sx, sy]

    def fill(box, b, w=1.0):
        dr.rectangle(box, fill=UNUSED if b is None else col[b])
        if b is not None:
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            a = dens[b][(int(cx) // BIN, int(cy) // BIN)]
            a[0] += w; a[1] += w * cx; a[2] += w * cy

    for k, x, y in sites:
        x0, x1 = xpos[(k, x)], xpos[(k, x)] + WIDTH[k] - 1
        h = ROWS[k]
        y0, y1 = ypx(y * h + h), ypx(y * h) - 1
        if k != 'RAMB36' or (k, x, y, 0) not in owner and (k, x, y, 1) not in owner:
            fill([x0, y0, x1, y1], owner.get((k, x, y, None)), 8.0 if k != 'SLICE' else 1.0)
        else:
            mid = (y0 + y1) // 2
            fill([x0, mid + 1, x1, y1], owner.get((k, x, y, 0)), 4.0)
            fill([x0, y0, x1, mid], owner.get((k, x, y, 1)), 4.0)

    # clock regions: rules between CR rows and where a column's region changes
    for lo, hi in cry.values():
        for row in (lo, hi + 1):
            dr.line([(4, ypx(row)), (fw - 4, ypx(row))], fill='black', width=2)
    cols = sorted(xpos, key=xpos.get)
    for a, b in zip(cols, cols[1:]):
        if crx[a] != crx[b]:
            xv = (xpos[a] + WIDTH[a[0]] + xpos[b]) // 2
            dr.line([(xv, top), (xv, fh - 8)], fill='black', width=2)
    for cy, (lo, hi) in cry.items():
        for cx in set(crx.values()):
            xs = min(xpos[c] for c in cols if crx[c] == cx)
            dr.text((xs + 3, ypx(lo) - 14), f'X{cx}Y{cy}', fill='black', font=small)

    # each block's tag at the weighted centre of its densest bin
    for b, bins in dens.items():
        w, sx, sy = max(bins.values())
        tx, ty, tag = sx / w, sy / w, BLOCKS[b][1]
        tw = dr.textlength(tag, font=tagf)
        dr.rounded_rectangle([tx - tw / 2 - 4, ty - 10, tx + tw / 2 + 4, ty + 9], 4,
                             fill='white', outline=col[b], width=2)
        dr.text((tx - tw / 2, ty - 9), tag, fill='black', font=tagf)
    dr.text((8, 10), title, fill='black', font=bold)

    # legend
    lx, ly = fw + 16, top
    dr.text((lx, ly), 'block', fill='black', font=bold)
    hdr = [('LUTs', 350), ('FFs', 420), ('BRAM', 475), ('DSP', 520)]

    def row(y, vals):
        for (t, off), v in zip(hdr, vals):
            s = f'{v:g}' if v else ''
            dr.text((lx + off - dr.textlength(s, font=font), y), s, fill='black', font=font)
    for t, off in hdr:
        dr.text((lx + off - dr.textlength(t, font=font), ly + 2), t, fill='black', font=font)
    ly += 30
    census = []
    for i, (label, tag, c, _) in enumerate(BLOCKS):
        census.append((label, [nlut[i], nff[i], nbram[i], ndsp[i]]))
        dr.rectangle([lx, ly + 2, lx + 16, ly + 18], fill=c, outline='black')
        dr.text((lx + 24, ly + 1), label, fill='black', font=font)
        row(ly + 1, census[-1][1])
        ly += 24
    census.append(('total', [sum(n.values()) for n in (nlut, nff, nbram, ndsp)]))
    dr.line([(lx, ly + 2), (lx + 520, ly + 2)], fill='black')
    dr.text((lx + 24, ly + 6), 'total', fill='black', font=font)
    row(ly + 6, census[-1][1])
    ly += 40
    for t in ('A slice takes the colour of the block holding most of it',
              '(a LUT 1, a flop 1/2); grey is empty.  Tall columns:',
              'block RAM, DSP, UltraRAM.  Black rules: clock regions.',
              'A tag marks where its block is densest.',
              'LUTs counts LUT sites (a LUT6_2 once, a LUTRAM bit',
              'once); BRAM counts RAMB36 equivalents.'):
        dr.text((lx, ly), t, fill='black', font=small)
        ly += 15
    img.save(png)

    print(f'{"block":<32}{"LUTs":>8}{"FFs":>8}{"BRAM":>7}{"DSP":>6}')
    for label, v in census:
        print(f'{label:<32}{v[0]:>8}{v[1]:>8}{v[2]:>7g}{v[3]:>6}')
    print(f'wrote {png} ({img.width}x{img.height})')


main()
