#!/usr/bin/env python3
"""Replay an FPGA instruction trace through ooo2_predictor's ACTUAL structures.

  tools/trace-bp.py ~/gb5-traces/*.trace
  tools/trace-bp.py --btbb 10 ~/gb5-traces/html5.trace     # what a 4x BTB would buy

Companion to tools/trace-limit.py. That one answers "how much ILP is there"; this one
answers "does the frontend find it", which the CPI counters can only observe as one
lumped FE_BUB. Modelled exactly as ooo2_predictor.v builds them:

  BTB  1<<BTBB entries, bidx = pc[BTBB:1]
       btag = pc[BTBB+TAGW:BTBB+1] ^ pc[BTBB+2*TAGW:BTBB+TAGW+1] ^ pc[63]
       NO valid bit -- the tag carries validity, so a never-written entry misses
  RAS  1<<RASB entries, circular; push the fall-through on a linking jal/jalr,
       pop on a jr/jalr that reads ra without linking

A miss is COLD (this PC has never been seen) or CONFLICT (the index was written by
another PC). Only conflict is bought back by capacity, so the split is the whole point
of the report -- a workload that is 41% cold, like clang, has a branch working set that
no affordable BTB holds.

The trace is a ~19k-instruction window, so cold counts here are an upper bound on a warm
machine's; conflict rates are the durable number. Direction is not modelled at all (no
GHR/YAGS), so `taken-miss/insn` counts only the branches the BTB could not even name.
"""
import re, sys
CTI = re.compile(r'^(beq|bne|blt|bge|bltu|bgeu|beqz|bnez|j|jr|jal|jalr|ret)$')

def rows(p):
    for ln in open(p, errors='ignore'):
        f = ln.split()
        if len(f) < 5 or len(f[2]) != 16: continue
        yield f

def run(path, BTBB=8, TAGW=12, RASB=3):
    tr = list(rows(path)); btb = {}; seen = set()
    RASN = 1 << RASB; ras = [0]*RASN; rp = 0
    cti = hit = cold = conflict = taken_miss = ret = ret_ok = 0
    for i, f in enumerate(tr[:-1]):
        pc = int(f[2], 16); m = f[4]
        mn = m[2:] if m.startswith('c.') else m
        if not CTI.match(mn): continue
        cti += 1
        ln = 2 if m.startswith('c.') else 4
        nxt = int(tr[i+1][2], 16); taken = (nxt != pc + ln)
        ops = ' '.join(f[5:])
        ix = (pc >> 1) & ((1 << BTBB) - 1)
        tg = ((pc >> (BTBB+1)) ^ (pc >> (BTBB+TAGW+1)) ^ (pc >> 63)) & ((1 << TAGW) - 1)
        e = btb.get(ix)
        if e == tg:          hit += 1
        elif pc not in seen: cold += 1;     taken_miss += taken
        else:                conflict += 1; taken_miss += taken
        is_call = mn in ('jal', 'jalr') and ops.startswith('ra,')
        is_ret  = mn in ('jr', 'ret', 'jalr') and ', ra,' in ops and not is_call
        if is_ret:
            ret += 1; ret_ok += (ras[rp % RASN] == nxt); rp -= 1
        elif is_call:
            rp += 1; ras[rp % RASN] = pc + ln
        seen.add(pc); btb[ix] = tg
    return dict(insn=len(tr), cti=cti, hit=hit, cold=cold, conflict=conflict,
                taken_miss=taken_miss, ret=ret, ret_ok=ret_ok)

def main(argv):
    btbb, rasb, paths = 8, 3, []
    it = iter(argv)
    for a in it:
        if   a == '--btbb': btbb = int(next(it))
        elif a == '--rasb': rasb = int(next(it))
        else: paths.append(a)
    print(f"BTB {1<<btbb} entries, RAS {1<<rasb} entries")
    print(f"{'workload':22s} {'CTI/insn':>8s} {'hit':>5s} {'cold':>5s} {'confl':>6s} "
          f"{'tk-miss/insn':>12s} {'rets':>5s} {'RAS ok':>7s}")
    for p in paths:
        r = run(p, btbb, 12, rasb); c = max(r['cti'], 1)
        nm = p.split('/')[-1].replace('.trace', '')
        rok = f"{100*r['ret_ok']/r['ret']:.0f}%" if r['ret'] else "--"
        print(f"{nm:22s} {100*r['cti']/r['insn']:7.1f}% {100*r['hit']/c:4.0f}% "
              f"{100*r['cold']/c:4.0f}% {100*r['conflict']/c:5.0f}% "
              f"{100*r['taken_miss']/r['insn']:11.1f}% {r['ret']:5d} {rok:>7s}")

if __name__ == '__main__':
    main(sys.argv[1:])
