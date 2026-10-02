#!/usr/bin/env python3
"""Trace-driven model of the branch predictor: today's direction history against path history
(docs/PLAN-2026-10-01-uniform-lanes.md, step 2).

  tools/bp-model.py trace.cti [--ghl 11,16] [--shift 1,2,3]

The trace is one line per control transfer, `pc len mnemonic next_pc priv rd rs1` (hex pc and
next_pc, len in bytes, rd `-` when none), and a last line `#insns N`, made from a Simmerv `-t` run:

  simmerv-cli -n -t --max-insns 150M ... | mawk 'have { print ppc, plen, pmn, $3, ppr, prd, prs;
      have=0 } $5 ~ /^(c\\.)?(beq|bne|blt|bge|bltu|bgeu|beqz|bnez|blez|bgez|bltz|bgtz|j|jr|jal|jalr|ret)$/
      { ppc=$3; plen=length($4)/2; pmn=$5; ppr=$2; prd=$6; prs=$7; sub(/,/,"",prd);
      if (prd=="") prd="-"; sub(/,/,"",prs); have=1 } END { print "#insns", NR }' > trace.cti

Both predictors share smolrv64_predictor's tables: a 2048-entry BTB keyed by the transfer's last
halfword holding a 2-bit bimodal weight per conditional, and a 2048-entry YAGS corrector with
8-bit tags indexed by that halfword XOR the folded history, allocated when a known conditional's
bimodal guess was wrong. Every resolved transfer trains, in program order, with no delay.

  dir   today: the history is the directions of the conditionals the BTB knew, one bit per
        conditional, and a pair's rows are read with the history a pair late.
  path  the history is a shifted hash of each taken transfer's address, once per fetch block;
        a block ends at its first taken transfer, so every branch in it is predicted with the
        history from before the block.

Fetch pairs: today a 16-byte pair ends at its first known transfer, taken or not; with path
history a block ends only at a taken one. Both also end at the 16-byte boundary. Counted with
perfect prediction, so the difference is the supply a not-taken branch costs.

MPKI is per thousand instructions: `dir` mispredicted conditional directions, `tgt` taken
transfers the BTB named with a wrong target (returns use an 8-entry RAS), `new` taken transfers
the BTB did not know.
"""
import argparse, collections, re

AP = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
AP.add_argument('trace')
AP.add_argument('--ghl', default='11,16')
AP.add_argument('--shift', default='1,2,3')
AP.add_argument('--hash', choices=('pc', 'target'), default='pc')
OPT = AP.parse_args()

COND = re.compile(r'^(c\.)?(beq|bne|blt|bge|bltu|bgeu|beqz|bnez|blez|bgez|bltz|bgtz)$')
LINK = ('ra', 't0')

def kind_of(mn, rd, rs1):
    """As the RTL classifies at resolve: a call links (rd x1/x5), a return is a jalr reading a
    link register without linking."""
    if COND.match(mn): return 'cond'
    direct = mn in ('j', 'jal', 'c.j', 'c.jal')
    if rd in LINK or mn in ('c.jal', 'c.jalr'): return 'call' if direct else 'icall'
    if mn in ('ret', 'jr', 'jalr', 'c.jr') and rs1 in LINK: return 'ret'
    return 'jmp' if direct else 'ijmp'

def load(path):
    rows, insns = [], 0
    for ln in open(path):
        f = ln.split()
        if len(f) < 4: continue
        pc, n, mn, nx = int(f[0], 16), int(f[1]), f[2], int(f[3], 16)
        rows.append((pc, n, kind_of(mn, f[5], f[6]), nx, nx != pc + n))
    return rows

def fold(x, bits):
    r = 0
    while x:
        r ^= x & ((1 << bits) - 1); x >>= bits
    return r

class Pred:
    """The BTB, the corrector and the RAS. `hist` is whatever history the variant feeds in."""
    BTBN, YN, YTAG = 2048, 2048, 8
    def __init__(self):
        self.btb = {}                 # idx -> (last, kind, ctr, target)
        self.y = {}                   # idx -> (tag, ctr)
        self.ras = [0] * 8; self.rp = 0
    def lookup(self, last):
        e = self.btb.get((last >> 1) % self.BTBN)
        return e if e and e[0] == last else None
    def yidx(self, last, hist):
        return ((last >> 1) ^ (fold(hist, 11) << 3)) % self.YN
    def ytag(self, last):
        return fold(last >> 1, self.YTAG)
    def predict(self, pc, n, hist):
        """-> (known, taken, target, yinfo)"""
        last = pc + n - 2
        e = self.lookup(last)
        if not e: return False, False, None, None
        _, kind, ctr, tgt = e
        if kind == 'cond':
            yi = self.yidx(last, hist); ye = self.y.get(yi)
            yhit = ye is not None and ye[0] == self.ytag(last)
            tk = (ye[1] >= 2) if yhit else (ctr >= 2)
            return True, tk, tgt, (yi, yhit, ctr)
        if kind == 'ret':
            return True, True, self.ras[(self.rp - 1) % 8], None
        return True, True, tgt, None
    def train(self, pc, n, kind, nx, taken, known, yinfo):
        last = pc + n - 2
        old = self.lookup(last)
        ctr = old[2] if (old and old[1] == 'cond') else (2 if taken else 1)
        if old and old[1] == 'cond':
            ctr = min(3, ctr + 1) if taken else max(0, ctr - 1)
        tgt = nx if taken else (old[3] if old else 0)
        self.btb[(last >> 1) % self.BTBN] = (last, kind, ctr, tgt)
        if kind == 'cond' and known and yinfo:
            yi, yhit, bctr = yinfo
            if yhit or (bctr >= 2) != taken:
                ye = self.y.get(yi)
                yc = ye[1] if yhit else (2 if taken else 1)
                if yhit: yc = min(3, yc + 1) if taken else max(0, yc - 1)
                self.y[yi] = (self.ytag(last), yc)
        if kind in ('call', 'icall'):
            self.ras[self.rp % 8] = pc + n; self.rp += 1
        elif kind == 'ret':
            self.rp -= 1

def pairs(rows, cut_not_taken):
    """Fetch pairs over the trace, perfect prediction: each run of sequential bytes ends at a
    taken transfer, or at any known transfer when `cut_not_taken`, and at every 16-byte line."""
    seen = set(); n = 0; start = rows[0][0] if rows else 0
    for pc, ln, _, nx, tk in rows:
        last = pc + ln - 2
        cut = tk or (cut_not_taken and last in seen)
        seen.add(last)
        if not 0 <= pc - start < 4096:      # a trap or a return from one: the run restarts here
            start = pc
        if cut:
            n += (last >> 4) - (start >> 4) + 1
            start = nx if tk else pc + ln
    return n

def run(rows, insns, mode, ghl=11, shift=1):
    p = Pred(); mask = (1 << ghl) - 1
    hist = 0; read_hist = 0; blk_hist = 0
    miss = collections.Counter()
    for pc, n, kind, nx, tk in rows:
        h = read_hist if mode == 'dir' else blk_hist
        known, ptk, ptgt, yinfo = p.predict(pc, n, h)
        cond = kind == 'cond'
        if cond and known and ptk != tk: miss['dir'] += 1
        elif tk and not known: miss['new'] += 1; miss['new:' + kind] += 1
        elif tk and known and ptk and ptgt != nx: miss['tgt'] += 1
        p.train(pc, n, kind, nx, tk, known, yinfo)
        if mode == 'dir':
            if cond and known:
                read_hist = hist                      # the next pair reads a pair late
                hist = ((hist << 1) | tk) & mask
        elif tk:
            a = pc if OPT.hash == 'pc' else nx
            blk_hist = ((blk_hist << shift) ^ fold(a >> 1, shift + 2)) & mask
    return {k: 1000.0 * v / insns for k, v in miss.items()}

def main():
    rows = load(OPT.trace)
    insns = None
    for ln in open(OPT.trace):
        if ln.startswith('#insns'): insns = int(ln.split()[1])
    if insns is None:   # approximate: transfers are ~1 in 6 instructions in kernel code
        insns = 6 * len(rows)
    print('%s: %d transfers, %d instructions, %.1f%% of transfers taken' %
          (OPT.trace, len(rows), insns, 100.0 * sum(r[4] for r in rows) / max(1, len(rows))))
    pt, pp = pairs(rows, True), pairs(rows, False)
    print('fetch pairs per 1000 instructions: today %.1f, cut only by taken %.1f (%.1f%% fewer)' %
          (1000.0 * pt / insns, 1000.0 * pp / insns, 100.0 * (pt - pp) / pt))
    def show(name, m):
        print('  %-26s MPKI dir %6.3f  tgt %6.3f  new %6.3f  total %6.3f' %
              (name, m.get('dir', 0), m.get('tgt', 0), m.get('new', 0),
               m.get('dir', 0) + m.get('tgt', 0) + m.get('new', 0)))
        print('  %-26s      new by kind: %s' % ('', '  '.join('%s %.3f' % (k[4:], v)
              for k, v in sorted(m.items()) if k.startswith('new:'))))
    show('dir history, GHL 11', run(rows, insns, 'dir'))
    for g in [int(x) for x in OPT.ghl.split(',')]:
        for s in [int(x) for x in OPT.shift.split(',')]:
            show('path %s, GHL %d, shift %d' % (OPT.hash, g, s), run(rows, insns, 'path', g, s))

main()
