#!/usr/bin/env python3
"""memrand (B4, 2026-09-17): generate ops.S, a straight-line stream of random memory operations
for the lockstep cosim -- every load's value and every store's bytes are judged against simmerv.

The stream runs in S-mode under Sv39 (memrand.c) over a 128 KiB data region reachable through
three aliases (the identity gigapage, W0, W1) so the same bytes are hit by different VAs; the
first 4 KiB of the region is a POINTER PAGE whose entries are random addresses into the region
through random aliases: a pointer load followed by an access through it is a store or load
whose address arrives late (the shape the speculative-load work of C4b must get right). Misaligned
loads and stores (line and page straddles included), AMOs, LR/SC pairs, FP loads/stores, cbo.zero/
clean/flush, fence/fence.i, sfence.vma, and a rare REMAP of W1's megapage to the region's second
half (a mapping change under live accesses) are all in the mix. Register x5-x31 and f0-f31 are the
value pool; sp/gp/tp are never touched. The DMA agent's interrupts land anywhere in the stream.

    ./gen.py --seed N --ops M > ops-N.S
"""
import argparse, random

ap = argparse.ArgumentParser()
ap.add_argument('--seed', type=int, default=1)
ap.add_argument('--ops', type=int, default=200000)
ap.add_argument('--misaligned-megapage', action='store_true',
                help='the first remap installs a 64 KiB-aligned megapage: both models must page-fault there')
ap.add_argument('--fpmix', action='store_true',
                help='FP-dense mix: FP arithmetic chains, real call/return through the stack, unpredictable '
                     'branches -- the 3-wide FP-shard + control-flow pattern of GB6 PDF Renderer (2026-09-21)')
a = ap.parse_args()
R = random.Random(a.seed)

REGION = 0x20000                       # 128 KiB
IDENT, W0, W1 = 0x81000000, 0x1000000000, 0x2000000000
ALIASES = [IDENT, W0, W1]
PTRS = 512                             # the pointer page (region offset 0..4095)
XREGS = list(range(5, 32))             # x5..x31: the integer value pool
BASES = [5, 6, 7]                      # x5-x7: chunk bases; x28 the aligned scratch; x29 the pointer
VALS = [r for r in XREGS if r not in (5, 6, 7, 28, 29)]
FREGS = list(range(32))

out = []
def e(s): out.append('        ' + s)

# ---- the region's initial contents: the pointer page (random addresses into the region past the
# pointer page, any alias, 8-aligned) then random data -- a zero region would let zero loads wipe the
# value pool within a few hundred ops and every compare would be of zeros
ptrs = [R.choice(ALIASES) + R.randrange(4096, REGION - 4096, 8) for _ in range(PTRS)]
e('.section .rodata'); e('.globl region_init'); e('.align 3'); out.append('region_init:')
for p in ptrs: e(f'.quad 0x{p:x}')
for _ in range((REGION - 4096) // 8): e(f'.quad 0x{R.getrandbits(64):x}')
e('.text')
NSUB = 4
FPOPS3 = ['fadd.d', 'fsub.d', 'fmul.d', 'fmin.d', 'fmax.d', 'fsgnj.d', 'fsgnjn.d', 'fsgnjx.d']
def fpalu():
    r = R.random()
    if r < 0.55:
        e(f'{R.choice(FPOPS3)} f{R.choice(FREGS)}, f{R.choice(FREGS)}, f{R.choice(FREGS)}')
    elif r < 0.75:
        e(f'{R.choice(["fmadd.d", "fmsub.d", "fnmadd.d", "fnmsub.d"])} f{R.choice(FREGS)}, f{R.choice(FREGS)}, f{R.choice(FREGS)}, f{R.choice(FREGS)}')
    elif r < 0.85:
        e(f'{R.choice(["fmv.x.d", "fclass.d"])} x{R.choice(VALS)}, f{R.choice(FREGS)}')      # FP -> INT (the CTF/FP-shared pipe)
    elif r < 0.93:
        e(f'{R.choice(["feq.d", "flt.d", "fle.d"])} x{R.choice(VALS)}, f{R.choice(FREGS)}, f{R.choice(FREGS)}')
    elif r < 0.98:
        e(f'fmv.d.x f{R.choice(FREGS)}, x{R.choice(VALS)}')
    else:
        e(f'fdiv.d f{R.choice(FREGS)}, f{R.choice(FREGS)}, f{R.choice(FREGS)}')
if a.fpmix:
    # NSUB subroutines: a real call saves ra on the stack, churns FP and integer state, restores ra, returns.
    # The failure under study is a `ret` whose reloaded ra is stale (2026-09-21).
    for k in range(NSUB):
        e('.align 2'); out.append(f'sub_{k}:')
        e('addi sp, sp, -32'); e('sd x1, 0(sp)'); e(f'sd x{R.choice(VALS)}, 8(sp)')
        for _ in range(R.randint(3, 9)):
            if R.random() < 0.7: fpalu()
            else:
                d, s_, t = R.choice(VALS), R.choice(VALS), R.choice(VALS)
                e(f'{R.choice(["add", "xor", "sub", "or", "sll", "srl"])} x{d}, x{s_}, x{t}')
        e('ld x1, 0(sp)'); e(f'ld x{R.choice(VALS)}, 8(sp)'); e('addi sp, sp, 32'); e('ret')
e('.globl run_ops'); e('.align 2'); out.append('run_ops:')
e('addi sp, sp, -112')
for i, r in enumerate([1, 8, 9, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]): e(f'sd x{r}, {i*8}(sp)')
e('la x29, l1_w1')                      # the remap target: W1's L1 table, through the identity alias
e('sd x29, 104(sp)')

def chunk_base(reg):
    """point a base register at a random 4 KiB chunk of the region (past the pointer page) through a random alias"""
    e(f'li x{reg}, 0x{R.choice(ALIASES) + R.randrange(4096, REGION - 4096, 4096) + 2048:x}')   # +2048: the 12-bit immediate spans the chunk

for b in BASES: chunk_base(b)
for r in VALS: e(f'li x{r}, 0x{R.getrandbits(64):x}')
for f in FREGS: e(f'fmv.d.x f{f}, x{R.choice(VALS)}')

LD = [('lb', 1), ('lh', 2), ('lw', 4), ('ld', 8), ('lbu', 1), ('lhu', 2), ('lwu', 4)]
ST = [('sb', 1), ('sh', 2), ('sw', 4), ('sd', 8)]
AMO = ['amoadd', 'amoswap', 'amoxor', 'amoand', 'amoor', 'amomin', 'amomax', 'amominu', 'amomaxu']

def off_for(size, aligned):
    """an offset inside the 4 KiB chunk (base+2048 +/- 2 KiB), natural alignment or any byte (a straddle stays in the region)"""
    return R.randrange(-2048, 2048 - size + 1, size if aligned else 1)

def load(base):
    op, sz = R.choice(LD)
    e(f'{op} x{R.choice(VALS)}, {off_for(sz, R.random() < 0.7)}(x{base})')
def store(base):
    op, sz = R.choice(ST)
    e(f'{op} x{R.choice(VALS)}, {off_for(sz, R.random() < 0.7)}(x{base})')
def fload(base):
    op, sz = R.choice([('flw', 4), ('fld', 8)])
    e(f'{op} f{R.choice(FREGS)}, {off_for(sz, R.random() < 0.8)}(x{base})')
def fstore(base):
    op, sz = R.choice([('fsw', 4), ('fsd', 8)])
    e(f'{op} f{R.choice(FREGS)}, {off_for(sz, R.random() < 0.8)}(x{base})')
def amo(base):
    w = R.choice(['w', 'd']); sz = 4 if w == 'w' else 8
    e(f'addi x28, x{base}, {off_for(sz, True)}')
    e(f'{R.choice(AMO)}.{w} x{R.choice(VALS)}, x{R.choice(VALS)}, (x28)')
def lrsc(base):
    w = R.choice(['w', 'd']); sz = 4 if w == 'w' else 8
    d, s = R.choice(VALS), R.choice(VALS)
    e(f'addi x28, x{base}, {off_for(sz, True)}')
    e(f'lr.{w} x{d}, (x28)')
    e(f'addi x{d}, x{d}, {R.randrange(-16, 16)}')
    e(f'sc.{w} x{s}, x{d}, (x28)')       # no memory op in between: both models succeed
def cbo(base):
    e(f'addi x28, x{base}, {R.randrange(-2048, 2048, 64)}')
    e(f'{R.choice(["cbo.zero", "cbo.clean", "cbo.flush", "cbo.clean"])} (x28)')
def alu():
    d, s, t = R.choice(VALS), R.choice(VALS), R.choice(VALS)
    e(f'{R.choice(["add", "xor", "sub", "or", "sll", "srl", "mul"])} x{d}, x{s}, x{t}')
def div():
    d, s, t = R.choice(VALS), R.choice(VALS), R.choice(VALS)
    e(f'ori x{t}, x{t}, 1'); e(f'div x{d}, x{s}, x{t}')
def pointer_chase():
    """a pointer load, then 1-3 accesses through it: their addresses arrive when the load lands"""
    # the pointer page sits at region offset 0, so it needs its own base -- through the identity
    # alias or W0 only: after a remap W1 shows another page, and a zero pointer is a fault
    e(f'li x28, 0x{R.choice([IDENT, W0]) + 2048:x}')
    idx = R.randrange(0, PTRS)
    e(f'ld x29, {idx * 8 - 2048}(x28)')
    room = 4096 - (ptrs[idx] & 0xfff)   # a misaligned access may not leave the page: the DUT raises
    for _ in range(R.randint(1, 3)):     # address-misaligned for a page straddle (documented), by design
        op, sz = R.choice(LD + ST)
        off = R.randrange(0, min(2048, room) - sz + 1, sz if R.random() < 0.7 else 1)
        e(f'{op} x{R.choice(VALS)}, {off}(x29)')
def fences():
    e(R.choice(['fence rw,rw', 'fence r,rw', 'fence w,w', 'fence.i', 'sfence.vma']))
def remap(state):
    """W1's megapage -> another 2 MiB page (or back): a mapping change under live accesses. The
    target must be 2 MiB-aligned: a misaligned megapage PTE faults (the DUT did; simmerv let it pass)"""
    state['half'] ^= 1
    pa = IDENT + ((0x10000 if a.misaligned_megapage else 0x400000) if state['half'] else 0)
    e('ld x28, 104(sp)')                 # l1_w1 through the identity alias
    e(f'li x29, 0x{((pa >> 12) << 10) | 0xC7:x}')   # A D V R W
    e('sd x29, 0(x28)'); e('sfence.vma')
    # after a remap the W1 bases point into the other page: plain RAM, zero-initialised, still checkable

def call():
    e(f'call sub_{R.randrange(NSUB)}')          # auipc+jalr through ra: unlimited reach, and the indirect-jump path real calls take
def branch():
    """an unpredictable branch over 1-3 ops: random value registers rarely compare equal, so the mix
    of taken/not-taken is set by the opcode; the skipped ops are FP/ALU, no memory"""
    e(f'{R.choice(["beq", "bne", "blt", "bge", "bltu", "bgeu"])} x{R.choice(VALS)}, x{R.choice(VALS)}, 1f')
    for _ in range(R.randint(1, 3)):
        if R.random() < 0.6: fpalu()
        else: alu()
    out.append('1:')
state = {'half': 0}
if a.fpmix:
    MIX = [(fpalu, 40), (fload, 7), (fstore, 6), (call, 8), (branch, 10), (alu, 12), (load, 8), (store, 6),
           (pointer_chase, 2), (fences, 0.5)]
else:
    MIX = [(load, 30), (store, 22), (fload, 4), (fstore, 4), (amo, 4), (lrsc, 2), (cbo, 2),
           (alu, 18), (div, 1), (pointer_chase, 10), (fences, 2), (remap, 0.3)]
ops_, wts = zip(*MIX)
for i in range(a.ops):
    if i % 16 == 0: chunk_base(R.choice(BASES))
    f = R.choices(ops_, weights=wts)[0]
    if f is remap: remap(state)
    elif f in (load, store, fload, fstore, amo, lrsc, cbo): f(R.choice(BASES))
    else: f()
e('fence rw,rw')
for i, r in enumerate([1, 8, 9, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]): e(f'ld x{r}, {i*8}(sp)')
e('addi sp, sp, 112'); e('ret')
print('\n'.join(out))
