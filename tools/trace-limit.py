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
import re, sys, collections
# Limit study from a trace: in-order dispatch into a window of W, issue when operands are
# ready (IW per cycle), in-order retire. Latencies measured on this core:
#   ALU 1 | load 5 (ldbench) | mul 3 | div 64 | FP add/mul 8 (fpbench)
# NO mispredict penalty and perfect caches -- so these are CEILINGS, not predictions.
def lat(mn):
    if mn.startswith(('fmul','fadd','fsub','fdiv','fsqrt','fmadd','fmsub','fnm')): return 8
    if mn.startswith(('div','rem')): return 64
    if mn.startswith('mul'): return 3
    if mn.startswith(('ld','lw','lbu','lb','lh','lhu','lwu','c.ld','c.lw','c.ldsp','c.lwsp','flw','fld')): return 5
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
    ret=[0]*n
    for i,(mn,rd,srcs) in enumerate(ins):
        disp = ret[i-W] if i>=W else 0
        rdy  = max([ready[s] for s in srcs], default=0)
        t = max(disp, rdy)
        while slots[t] >= IW: t += 1
        slots[t]+=1
        fin = t + lat(mn)
        if rd and rd!='x0': ready[rd]=fin
        ret[i] = max(fin, ret[i-1] if i else 0)
    return n/ret[n-1]
for path in sys.argv[1:]:
    ins=load(path); name=path.split('/')[-1].replace('.trace','')
    print("%-18s" % name, end="")
    for IW in (1,2,4):
        for W in (16,32,64):
            print("  IW%d/W%-3d %5.2f" % (IW,W,sim(ins,W,IW)), end="")
        print()
        if IW!=4: print("%-18s" % "", end="")
