#!/usr/bin/env python3
"""Report on an ooo2 predictor trace (a run built with -DBP_TRACE; the [BP] PRED/RES lines).

  tools/bp-trace-report.py workloads/rvbench/dhrystone.bptrace.log

Three sections, each per PC and sorted by count:

  returns   each resolved return against the target the RAS predicted for its bundle:
            correct, wrong (stale RAS entry), or never predicted (the bundle's BTB lookup
            missed, so the return fell through to decode or resolve)
  misses    resolved CTIs whose carried BTB hit bit is 0 -- the lookup at fetch missed.
            A CTI that misses on EVERY execution in steady state is trained at a base no
            lookup uses: the train/lookup base mismatch this report exists to catch.
  bases     for each always-missing CTI, the base resolve trained (res_base) against the
            bases fetch actually looked up in the cycles before it
"""
import collections, re, sys

PRED = re.compile(r'\[BP\] c=(\d+) PRED base=([0-9a-f]+) tag_hit=(\d) cti=(\d) type=(\d+) .* pred=(\d) tgt=([0-9a-f]+)')
RES = re.compile(r'\[BP\] c=(\d+) RES pc=([0-9a-f]+) base=([0-9a-f]+) cbr=(\d) call=(\d) ret=(\d) taken=(\d) '
                 r'tgt=([0-9a-f]+) \| carried hit=(\d)')
TY_RET, TY_CALL = '110', '101'

def short(h):
    return h.lstrip('0') or '0'

def main(path):
    preds = []                     # (cycle, base, tag_hit, cti, type, tgt)
    res = []                       # (cycle, pc, base, kind, taken, tgt, hit)
    for ln in open(path, errors='ignore'):
        m = PRED.match(ln)
        if m:
            c, base, th, cti, ty, _, tgt = m.groups()
            preds.append((int(c), base, th == '1', cti == '1', ty, tgt))
            continue
        m = RES.match(ln)
        if m:
            c, pc, base, cbr, call, ret, tk, tgt, hit = m.groups()
            kind = 'cbr' if cbr == '1' else 'call' if call == '1' else 'ret' if ret == '1' else 'jump'
            res.append((int(c), pc, base, kind, tk == '1', tgt, hit == '1'))
    if not res:
        sys.exit(f'{path}: no [BP] RES lines -- was the run built with -DBP_TRACE and the window set?')

    by_base = collections.defaultdict(list)
    for c, base, th, cti, ty, tgt in preds:
        by_base[base].append((c, th and cti, ty, tgt))

    def last_pred(base, before):
        best = None
        for c, hit, ty, tgt in by_base.get(base, []):
            if c < before:
                best = (hit, ty, tgt)
        return best

    kinds = collections.Counter(k for _, _, _, k, _, _, _ in res)
    print(f'{path}: {len(res)} resolved CTIs ' + ', '.join(f'{k} {v}' for k, v in kinds.most_common()))

    rets = collections.Counter()
    for c, pc, base, kind, tk, tgt, hit in res:
        if kind != 'ret':
            continue
        p = last_pred(base, c)
        if not hit or p is None or not p[0] or p[1] != TY_RET:
            rets[(short(pc), 'NOT PREDICTED (BTB miss)')] += 1
        elif p[2] == tgt:
            rets[(short(pc), 'correct')] += 1
        else:
            rets[(short(pc), f'WRONG: RAS said {short(p[2])}, went to {short(tgt)}')] += 1
    nret = sum(rets.values())
    good = sum(v for (pc, what), v in rets.items() if what == 'correct')
    print(f'\nreturns: {good}/{nret} predicted correctly')
    for (pc, what), v in sorted(rets.items(), key=lambda kv: -kv[1]):
        if what != 'correct':
            print(f'  {v:6d}  ret at {pc}: {what}')

    execs = collections.Counter((short(pc), kind) for _, pc, _, kind, _, _, _ in res)
    miss = collections.Counter((short(pc), kind) for _, pc, _, kind, _, _, hit in res if not hit)
    print('\nBTB misses at fetch (resolved with carried hit=0):')
    always = []
    for (pc, kind), v in miss.most_common(20):
        tag = '  ALWAYS' if v == execs[(pc, kind)] and v > 2 else ''
        print(f'  {v:6d}/{execs[(pc, kind)]:<6d} {kind:5s} at {pc}{tag}')
        if tag:
            always.append(pc)

    if always:
        print('\nalways-missing CTIs: the base resolve trains vs the bases fetch looked up just before:')
        for pc in always:
            trained = collections.Counter(short(b) for _, p, b, _, _, _, hit in res if short(p) == pc)
            looked = collections.Counter()
            for c, p, b, _, _, _, _ in res:
                if short(p) != pc:
                    continue
                cands = [(cc, bb) for cc, bb, _, cti, _, _ in preds if c - 40 < cc < c and cti
                         and int(bb, 16) <= int(p, 16) < int(bb, 16) + 16]
                if cands:
                    looked[short(cands[-1][1])] += 1
            print(f'  {pc}: trained at {dict(trained)}, looked up at {dict(looked)}')

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
