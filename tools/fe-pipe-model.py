#!/usr/bin/env python3
"""A cycle model of the two-wide frontend over one straight-line function: the fetch
buffer's request and fill rules (ooo2/rv_soc_top.v), the aligner's bundle rules
(src/aligner.v), an I$ that always hits with the measured request->response latency, and a
backend that never stalls -- the frontend-only ceiling, per design variant, in seconds.

    tools/fe-pipe-model.py <objdump -d output> <function>      # e.g. shabench.dis sha256_blocks

Written 2026-09-05 (plan item 10e) when the second ALU left sha256's kernel at 1.171 IPC with
the F/X queue empty 22.5% of its cycles. Two hypotheses, ranked here before any build: the
aligner's chunk-boundary cap on slot 1 (338 of 934 bundles are singles because of it), and
the fetch buffer's single outstanding request (one 16-byte chunk per request, response two
cycles later, the next request only after the response). The model says the second is the
wall: as built the frontend delivers 1.23 instructions per cycle on this kernel and lifting
the cap alone changes nothing (1.23); a third buffer slot with two requests in flight and a
request three chunks ahead on the slide cycle gives 1.63 with the cap and 1.99 without it,
the same as 32-byte chunks would. `FE_IC` reads ~0 on that kernel because the buffer's idle
cycles reach dispatch two cycles later, through the queue, as `FE_QUE`.

The rules are transcribed from the RTL by hand; validate against an FB_TRACE run before
trusting a variant's number to a tenth.
"""
import re, sys
ins = []; infn = False
DIS, FN = (sys.argv[1], sys.argv[2]) if len(sys.argv) > 2 else sys.exit(__doc__)
for line in open(DIS):
    if re.match(r'^[0-9a-f]+ <%s>:' % re.escape(FN), line): infn = True; continue
    if infn and re.match(r'^[0-9a-f]+ <', line): break
    m = re.match(r'^\s*([0-9a-f]+):\s+([0-9a-f]{4}|[0-9a-f]{8})\s+(\S+)', line)
    if infn and m: ins.append((int(m.group(1), 16), len(m.group(2)) // 2, m.group(3)))
CTI = lambda mn: mn.startswith(('b', 'j', 'c.j', 'c.b', 'ret', 'ecall', 'ebreak', 'fence', 'csr', 'sret', 'mret', 'wfi', 'sfence'))
at = {a: (n, mn) for a, n, mn in ins}
base, end = ins[0][0], ins[-1][0] + ins[-1][1]
CHB, HW = 16, 8                                     # chunk bytes, window halfwords
chunk = lambda a: a & ~(CHB - 1)

def run(cap=True, lat=2, outstanding=1, early=False, wait_slot1=False, third_slot=False, big=False, ic_gap=1):
    global CHB
    CHB = 32 if big else 16
    pc = base; cyc = 0; ninsn = 0; nbund = 0
    pa = None; v0 = v1 = False; w2 = None                     # buffer: chunk0 addr, valid bits, third slot (chunk addr or None)
    reqs = []; last_acc = -99                                   # in flight: (addr, response cycle); last accept cycle
    stat = {'idle': 0, 'single_cap': 0, 'single_bytes': 0, 'single_cti': 0, 'wait': 0, 'realign': 0}
    while pc < end:
        cyc += 1
        c = chunk(pc); lo = pc - c
        in0 = v0 and c == pa; in1 = v1 and pa is not None and c == pa + CHB
        resp = [r for r in reqs if r[1] == cyc]                 # the response landing this cycle
        rip = resp[0][0] if resp else None
        hit = in0 or in1
        if hit:
            off = lo + (CHB if in1 else 0)
            fb_end = 2 * CHB if v1 else CHB
            avail = min((fb_end - off) // 2, HW)
        elif rip is not None and rip == c:                      # arrival bypass
            avail = min((CHB - lo) // 2, HW)
        else:
            avail = 0
        # ---- aligner
        p = pc; slots = 0; why = None
        n0, m0 = at.get(p, (4, 'x'))
        if n0 // 2 <= avail:
            slots = 1; p += n0
            if CTI(m0): why = 'cti'
            else:
                n1, m1 = at.get(p, (4, 'x'))
                to_chunk_end = (CHB - lo) // 2
                av1 = avail if not cap else min(avail, to_chunk_end)
                if (p - pc + n1) // 2 <= av1: slots = 2; p += n1
                elif cap and (p - pc + n1) // 2 <= avail: why = 'cap'
                else:
                    why = 'bytes'
                    if wait_slot1 and avail < HW: slots = 0; p = pc   # the bundle waits for its bytes
        if slots:
            nbund += 1; ninsn += slots
            if slots == 1: stat['single_' + why] += 1
            pc_next = p
        else:
            stat['wait' if why == 'bytes' else 'idle'] += 1
            pc_next = pc
        # ---- fetch buffer request (this cycle's state)
        miss = not hit
        want = None
        if miss: want = c
        elif not v0: want = pa
        elif not v1: want = pa + CHB
        elif in1: want = pa + 2 * CHB                             # the slide cycle
        elif early and v0 and v1 and in0 and (lo >= CHB // 2): want = pa + 2 * CHB
        elif third_slot and v0 and v1 and w2 is None: want = pa + 2 * CHB
        if third_slot >= 2 and not miss and pa is not None:
            want = None
            for k, held in ((0, v0), (1, v1), (2, w2 is not None)):
                a = pa + k * CHB
                if not held and all(r[0] != a for r in reqs): want = a; break
            if want is None and third_slot == 3 and in1 and all(r[0] != pa + 3 * CHB for r in reqs):
                want = pa + 3 * CHB                                # the slide cycle: three ahead, lands in the slot the slide frees
        if want is not None and len(reqs) < outstanding and all(r[0] != want for r in reqs) and cyc - last_acc >= ic_gap:
            reqs.append((want, cyc + lat)); last_acc = cyc
        # ---- buffer update at the edge
        if resp: reqs = [r for r in reqs if r[1] != cyc]
        if miss:
            stat['realign'] += 1
            pa = c; v0 = v1 = False; w2 = None
            if rip == c: v0 = True
        elif in1:
            pa, v0, v1 = pa + CHB, v1, False
            if w2 == pa + CHB: v1 = True; w2 = None
            if rip == pa: v0 = True
            elif rip == pa + CHB: v1 = True
            elif rip == pa + 2 * CHB and third_slot: w2 = rip
        else:
            if rip == pa: v0 = True
            elif rip == pa + CHB: v1 = True
            elif rip == pa + 2 * CHB and third_slot: w2 = rip
        pc = pc_next
    return cyc, ninsn, nbund, stat

variants = [("A  as built (cap, 1 outstanding, lat 2)", dict()),
            ("B  no chunk cap", dict(cap=False)),
            ("C  no cap, slot 1 waits for in-page bytes", dict(cap=False, wait_slot1=True)),
            ("D  as built + 2 outstanding, early request at mid-chunk", dict(outstanding=2, early=True)),
            ("E  D + no cap", dict(cap=False, outstanding=2, early=True)),
            ("F  E + slot 1 waits", dict(cap=False, outstanding=2, early=True, wait_slot1=True)),
            ("G  third slot (2 ahead), 2 outstanding, no cap, wait", dict(cap=False, outstanding=2, third_slot=True, wait_slot1=True)),
            ("K  third slot requested 2 ahead, 2 outstanding, cap", dict(outstanding=2, third_slot=2)),
            ("K2 K + no cap, cut when late", dict(cap=False, outstanding=2, third_slot=2)),
            ("K3 K + no cap, slot 1 waits", dict(cap=False, outstanding=2, third_slot=2, wait_slot1=True)),
            ("K4 K3 with 3 outstanding", dict(cap=False, outstanding=3, third_slot=2, wait_slot1=True)),
            ("K5 K3, I$ accepts one request per 2 cycles", dict(cap=False, outstanding=2, third_slot=2, wait_slot1=True, ic_gap=2)),
            ("K6 K (cap), I$ accepts one per 2 cycles", dict(outstanding=2, third_slot=2, ic_gap=2)),
            ("K7 K3 with lat 3", dict(cap=False, outstanding=2, third_slot=2, wait_slot1=True, lat=3)),
            ("K8 K3 + slide-cycle request three ahead", dict(cap=False, outstanding=2, third_slot=3, wait_slot1=True)),
            ("K9 K8 with the I$ door open every 2nd cycle", dict(cap=False, outstanding=2, third_slot=3, wait_slot1=True, ic_gap=2)),
            ("K10 K9 with the cap kept", dict(outstanding=2, third_slot=3, ic_gap=2)),
            ("K11 K9, 3 outstanding", dict(cap=False, outstanding=3, third_slot=3, wait_slot1=True, ic_gap=2)),
            ("H  32-byte chunks, as built otherwise", dict(big=True)),
            ("I  32-byte chunks, no cap, wait", dict(big=True, cap=False, wait_slot1=True)),
            ("J  lat 1 (a faster I$), as built", dict(lat=1)),
            ]
print("%s: %d insns (%d compressed)" % (FN, len(ins), sum(1 for _, n, _ in ins if n == 2)))
for name, kw in variants:
    cyc, n, nb, st = run(**kw)
    print("%-58s %5d cycles  %.3f insn/cyc  singles cap/bytes/cti %3d/%3d/%d  idle %3d wait %3d realign %3d"
          % (name, cyc, n / cyc, st['single_cap'], st['single_bytes'], st['single_cti'], st['idle'], st['wait'], st['realign']))
