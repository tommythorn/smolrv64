#!/usr/bin/env python3
"""Limit study from an FPGA instruction trace: how much of the gap is window, and how much
is issue width?

  tools/trace-limit.py ~/text-compression.trace ~/clang.trace ...

Answers questions the CPI counters cannot, because it separates constraints that the
counters can only observe together. Traces come from the board; see docs/OOO2-Spec.md
"Workload shapes".

CEILINGS, NOT PREDICTIONS. Perfect branch prediction, perfect caches, no structural
hazards beyond width. A workload sitting far below its own IW1 number here is limited by
something this model does not contain -- redirects, cache misses, or a unit -- and that is
usually the more actionable finding.
"""
import re, sys, collections, argparse
# 2026-09-17 (the memory backend program, B9): knobs for what the backend rewrite changes.
#   --load-lat N     load-use latency (5 today, 4 with the pipelined LSU)
#   --ld-port N      cycles the load port is occupied per load (4 today: one access per FSM
#                    round trip; 1 with the pipelined door)
#   --st-port N      cycles per store at the door (2 today)
#   --miss-rate P    fraction of loads that miss (deterministic per index, so runs compare)
#   --miss-lat N     cycles a miss adds (36 = 28.75 DDR mean + the cache)
#   --mlp N          misses that may overlap (1 today: one MSHR; 4 with the MSHR table)
#   --mispred-mpki M --mispred-pen P   a full-window drain of P cycles every 1000/M insns
# Still ceilings: perfect prediction otherwise, no dTLB, no structural hazard but the ports.
ARGS = argparse.ArgumentParser()
for k, d in (('load_lat',5),('ld_port',1),('st_port',1),('miss_lat',36),('mlp',1),('mispred_pen',12)):
    ARGS.add_argument('--'+k.replace('_','-'), type=int, default=d)
ARGS.add_argument('--miss-rate', type=float, default=0.0)
ARGS.add_argument('--mispred-mpki', type=float, default=0.0)
ARGS.add_argument('--iw', type=str, default='1,2,4')
ARGS.add_argument('--w', type=str, default='16,32,64')
ARGS.add_argument('traces', nargs='+')
OPT = ARGS.parse_args()
# Limit study from a trace: in-order dispatch into a window of W, issue when operands are
# ready (IW per cycle), in-order retire. Latencies measured on this core:
#   ALU 1 | load 5 (ldbench) | mul 3 | div 64 | FP add/mul 8 (fpbench)
# NO mispredict penalty and perfect caches -- so these are CEILINGS, not predictions.
LOADS  = ('ld','lw','lbu','lb','lh','lhu','lwu','c.ld','c.lw','c.ldsp','c.lwsp','flw','fld')
STORES = ('sd','sw','sb','sh','c.sd','c.sw','c.sdsp','c.swsp','fsw','fsd')
def is_load(mn): return mn.startswith(LOADS)
def is_store(mn): return mn.startswith(STORES)
def lat(mn):
    if mn.startswith(('fmul','fadd','fsub','fdiv','fsqrt','fmadd','fmsub','fnm')): return 8
    if mn.startswith(('div','rem')): return 64
    if mn.startswith('mul'): return 3
    if is_load(mn): return OPT.load_lat
    return 1
pat = re.compile(r'^\d+\s+\d+\s+([0-9a-f]{16})\s+(\S+)\s+(\S+)\s*(.*)$')
def load(path):
    out=[]
    for ln in open(path):
        m=pat.match(ln)
        if not m: continue
        toks=[t.strip() for t in m.group(4).split(',')]
        rd = toks[0] if toks and toks[0] and not toks[0].startswith('0') else None
        srcs=[t for t in toks[1:3] if t and t!='x0' and not t.startswith('0')]
        out.append((m.group(3), rd, srcs))
    return out
def sim(ins, W, IW):
    n=len(ins); ready=collections.defaultdict(int); slots=collections.Counter()
    ret=[0]*n; ld_free=0; st_free=0; misses=[]   # port free times; completion times of misses in flight
    mp_every = int(1000.0/OPT.mispred_mpki) if OPT.mispred_mpki > 0 else 0
    for i,(mn,rd,srcs) in enumerate(ins):
        disp = ret[i-W] if i>=W else 0
        if mp_every and i and i % mp_every == 0:      # a mispredict: the window drains, then the penalty
            disp = max(disp, ret[i-1] + OPT.mispred_pen)
        rdy  = max([ready[s] for s in srcs], default=0)
        t = max(disp, rdy)
        if is_load(mn):  t = max(t, ld_free)
        if is_store(mn): t = max(t, st_free)
        while slots[t] >= IW: t += 1
        slots[t]+=1
        l = lat(mn)
        if is_load(mn):
            ld_free = t + OPT.ld_port
            if OPT.miss_rate > 0 and (i * 2654435761) % 1000 < OPT.miss_rate * 1000:
                misses = [m for m in misses if m > t]
                start = t if len(misses) < OPT.mlp else min(misses)
                fin_m = start + OPT.miss_lat; misses.append(fin_m); l = fin_m - t + l
        if is_store(mn): st_free = t + OPT.st_port
        fin = t + l
        if rd and rd!='x0': ready[rd]=fin
        ret[i] = max(fin, ret[i-1] if i else 0)
    return n/ret[n-1]
IWS=[int(x) for x in OPT.iw.split(',')]; WS=[int(x) for x in OPT.w.split(',')]
for path in OPT.traces:
    ins=load(path); name=path.split('/')[-1].replace('.trace','')
    print("%-18s" % name, end="")
    for k, IW in enumerate(IWS):
        for W in WS:
            print("  IW%d/W%-3d %5.2f" % (IW,W,sim(ins,W,IW)), end="")
        print()
        if k != len(IWS)-1: print("%-18s" % "", end="")
