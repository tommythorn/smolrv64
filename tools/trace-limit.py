#!/usr/bin/env python3
"""Limit study from an FPGA instruction trace: how much of the gap is window, and how much
is issue width?

  tools/trace-limit.py ~/text-compression.trace ~/clang.trace ...

Answers questions the CPI counters cannot, because it separates constraints that the
counters can only observe together. Traces come from the board; see docs/SmolRV64-Spec.md
"Workload shapes".

CEILINGS, NOT PREDICTIONS. Perfect branch prediction, perfect caches, no structural
hazards beyond width. A workload sitting far below its own IW1 number here is limited by
something this model does not contain -- redirects, cache misses, or a unit -- and that is
usually the more actionable finding.

--machine today|lanes replaces the width-only machine with a dispatch model (the uniform-lanes
plan, docs/PLAN-2026-10-01-uniform-lanes.md, step 2):

  today  three-wide dispatch in order; a bundle holds at most one memory op and at most one
         FP/control-flow/mul-div op, so the next one waits for the next cycle and so does
         every younger op; two ALUs; memory ops issue in order, one per --mem-every cycles;
         FP, branches, jumps and mul/div share one port; retire three.
  lanes  four-wide dispatch with no class rule; slot k is lane k, and each lane issues one
         op a cycle -- an ALU op, a multiply, a branch or a memory op's address; the memory
         unit takes one load and one store a cycle; FP goes to one FP unit; retire four.

Both serialise CSR and system ops (the window drains first) and run divides one at a time.
--rob-rows R makes the window R rows of IW entries, one row per dispatch group with slot k in
column k (holes where a group is short), retired whole, --retire-rows rows a cycle; without it
the window is W dense entries retiring IW a cycle.
--cut taken|cti ends a dispatch group after a taken control transfer (a fetch block) or
after any control transfer (today's 16-byte pairs); by default the front end is perfect.
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
ARGS.add_argument('--machine', choices=('width', 'today', 'lanes'), default='width')
ARGS.add_argument('--mem-every', type=int, default=1)
ARGS.add_argument('--cut', choices=('none', 'taken', 'cti'), default='none')
ARGS.add_argument('--rob-rows', type=int, default=0)
ARGS.add_argument('--retire-rows', type=int, default=1)
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
CTI = re.compile(r'^(c\.)?(beq|bne|blt|bge|bltu|bgeu|beqz|bnez|blez|bgez|bltz|bgtz|j|jr|jal|jalr|ret)$')
def klass(mn):
    if is_load(mn) or is_store(mn) or mn.startswith(('amo', 'lr.', 'sc.')): return 'mem'
    if mn.startswith(('csr', 'ecall', 'ebreak', 'mret', 'sret', 'sfence', 'fence', 'wfi')): return 'sys'
    if mn.startswith('f'): return 'fp'
    if CTI.match(mn): return 'ctf'
    if mn.startswith(('div', 'rem')): return 'div'
    if mn.startswith('mul'): return 'mul'
    return 'alu'
pat = re.compile(r'^\d+\s+\d+\s+([0-9a-f]{16})\s+(\S+)\s+(\S+)\s*(.*)$')
def load(path):
    out=[]; rows=[]
    for ln in open(path):
        m=pat.match(ln)
        if not m: continue
        toks=[t.strip() for t in m.group(4).split(',')]
        rd = toks[0] if toks and toks[0] and not toks[0].startswith('0') else None
        srcs=[t for t in toks[1:3] if t and t!='x0' and not t.startswith('0')]
        rows.append((int(m.group(1), 16), len(m.group(2)) // 2, m.group(3), rd, srcs))
    for i, (pc, ln, mn, rd, srcs) in enumerate(rows):
        taken = i + 1 < len(rows) and rows[i + 1][0] != pc + ln
        out.append((mn, rd, srcs, taken))
    return out
def sim(ins, W, IW):
    n=len(ins); ready=collections.defaultdict(int); slots=collections.Counter()
    ret=[0]*n; ld_free=0; st_free=0; misses=[]   # port free times; completion times of misses in flight
    mp_every = int(1000.0/OPT.mispred_mpki) if OPT.mispred_mpki > 0 else 0
    for i,(mn,rd,srcs,_) in enumerate(ins):
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
def sim_machine(ins, W, lanes):
    """--machine today/lanes: in-order dispatch groups under the machine's rules, issue when
    the operands and the op's issue slot are ready, in-order retire. Returns IPC and the share
    of dispatch groups each rule closed early."""
    IW = 4 if lanes else 3
    n = len(ins); ready = collections.defaultdict(int); ret = [0]*n
    busy = collections.Counter()        # (resource, cycle) -> ops issued
    mem_free = ld_free = st_free = div_free = 0
    misses = []                         # completion times of the misses in flight (--miss-rate, --mlp)
    closed = collections.Counter()
    d = 0; grp = []                     # current dispatch cycle and its ops' classes
    R, K = OPT.rob_rows, OPT.retire_rows
    rowret = []; gfin = 0               # each closed group's row retire time; this group's latest result
    def close_row():
        rowret.append(max(gfin, rowret[-1] if rowret else 0, rowret[-K] + 1 if len(rowret) >= K else 0))
    for i, (mn, rd, srcs, taken) in enumerate(ins):
        k = klass(mn)
        full = len(grp) == IW
        rule = (not lanes and ((k == 'mem' and 'mem' in grp) or
                (k in ('fp', 'ctf', 'mul', 'div') and any(c in ('fp', 'ctf', 'mul', 'div') for c in grp))))
        if grp and (full or rule or k == 'sys' or grp[-1] == 'sys' or cut):
            closed['full' if full else 'rule' if rule else 'sys' if 'sys' in (k, grp[-1]) else 'cut'] += 1
            d += 1; grp = []
            if R: close_row(); gfin = 0
        if R:
            if not grp and len(rowret) >= R: d = max(d, rowret[len(rowret) - R])
            if k == 'sys' and rowret: d = max(d, rowret[-1], gfin)
        else:
            d = max(d, ret[i-W] if i >= W else 0, ret[i-1] if (k == 'sys' and i) else 0)
        lane = len(grp); grp.append(k)
        cut = (OPT.cut == 'taken' and taken) or (OPT.cut == 'cti' and k == 'ctf')
        t = max([d + 1] + [ready[s] for s in srcs])
        if lanes:
            if k == 'fp':
                while busy['fp', t]: t += 1
                busy['fp', t] += 1
            else:
                while busy[lane, t] or (k == 'div' and t < div_free): t += 1
                busy[lane, t] += 1
                if is_load(mn) or k == 'mem' and not is_store(mn):
                    t = max(t + 1, ld_free); ld_free = t + 1
                elif is_store(mn):
                    t = max(t + 1, st_free); st_free = t + 1
        else:
            res, cap = ('alu', 2) if k in ('alu', 'sys') else ('mem', 1) if k == 'mem' else ('f', 1)
            if k == 'mem': t = max(t, mem_free)
            while busy[res, t] >= cap or (k == 'div' and t < div_free): t += 1
            busy[res, t] += 1
            if k == 'mem': mem_free = t + OPT.mem_every
        if k == 'div': div_free = t + lat(mn)
        l = lat(mn)
        if is_load(mn) and OPT.miss_rate > 0 and (i * 2654435761) % 1000 < OPT.miss_rate * 1000:
            misses = [m for m in misses if m > t]
            start = t if len(misses) < OPT.mlp else min(misses)
            fin_m = start + OPT.miss_lat; misses.append(fin_m); l = fin_m - t + l
        fin = t + l
        if rd and rd != 'x0': ready[rd] = fin
        gfin = max(gfin, fin)
        ret[i] = max(fin, ret[i-1] if i else 0, ret[i-IW] + 1 if i >= IW else 0)
    if R: close_row()
    g = sum(closed.values()) or 1
    return n/(rowret[-1] if R else ret[n-1]), {r: closed[r]/g for r in ('full', 'rule', 'sys', 'cut')}
IWS=[int(x) for x in OPT.iw.split(',')]; WS=[int(x) for x in OPT.w.split(',')]
if OPT.machine != 'width':
    for path in OPT.traces:
        ins = load(path); name = path.split('/')[-1].replace('.trace', '')
        for W in WS:
            ipc, why = sim_machine(ins, W, OPT.machine == 'lanes')
            print("%-22s %-5s W%-3d IPC %5.2f  groups closed: full %3.0f%%  rule %3.0f%%  sys %3.0f%%  cut %3.0f%%" %
                  (name, OPT.machine, W, ipc, *(100*why[r] for r in ('full', 'rule', 'sys', 'cut'))))
    sys.exit(0)
for path in OPT.traces:
    ins=load(path); name=path.split('/')[-1].replace('.trace','')
    print("%-18s" % name, end="")
    for k, IW in enumerate(IWS):
        for W in WS:
            print("  IW%d/W%-3d %5.2f" % (IW,W,sim(ins,W,IW)), end="")
        print()
        if k != len(IWS)-1: print("%-18s" % "", end="")
