#!/usr/bin/env python3
"""fpref -- the exact reference for the FPU's benches: RISC-V F/D results and flags, by rational arithmetic.

    tools/fpref.py vectors <op> <n> [seed]     n random vectors for op (div, sqrt, min, max) as lines
        "<fmt> <rm> <a> <b> <result> <flags>": fmt 0 = binary32, 1 = binary64; rm 0..4 = RNE RTZ RDN
        RUP RMM; a, b, result 16 hex digits (a binary32 value NaN-boxed: its upper 32 bits ones);
        flags = {NV, DZ, OF, UF, NX} as 2 hex digits.
    tools/fpref.py vectors fma <n> [seed]      n vectors for the FMA unit, "v sf df rm x a b c result flags":
                                               the FMA group (ffma) and the conversions (f2f, i2f, f2i,
                                               fcvtmod)
    tools/fpref.py vectors zfa <n> [seed]      n vectors for the core's Zfa ops, "op fmt rm a b result flags"

Every value is decoded to an exact rational, the operation is done exactly (a square root by an
integer square root of the scaled radicand, its remainder the sticky bit), and the one rounding
step follows IEEE 754-2008 as RISC-V specifies it: tininess is detected AFTER rounding, a NaN
result is the canonical NaN, and a binary32 operand that is not NaN-boxed reads as the canonical
NaN. Nothing here is hardware-shaped: it is the specification, computed.
"""
import random
import sys
from fractions import Fraction
from math import isqrt

FMT = {0: (8, 23), 1: (11, 52), 2: (5, 10)}   # binary32, binary64, binary16: exponent bits, fraction bits
NV, DZ, OF, UF, NX = 16, 8, 4, 2, 1


def params(fmt):
    e, f = FMT[fmt]
    bias = (1 << (e - 1)) - 1
    return e, f, bias, 1 - bias, bias        # ..., emin, emax


def width(fmt):
    e, f = FMT[fmt]
    return 1 + e + f


def unbox(fmt, x):
    """The operand's own bits, or None for a narrower value that is not NaN-boxed."""
    if fmt == 1:
        return x
    w = width(fmt)
    return x & ((1 << w) - 1) if x >> w == (1 << (64 - w)) - 1 else None


def box(fmt, x):
    w = width(fmt)
    return x if fmt == 1 else ((1 << (64 - w)) - 1) << w | x


def canon_nan(fmt):
    e, f, *_ = params(fmt)
    return ((1 << e) - 1) << f | 1 << (f - 1)


def decode(fmt, x):
    """('nan', signaling) | ('inf', sign) | ('num', sign, Fraction)."""
    e, f, bias, emin, _ = params(fmt)
    s = x >> (e + f) & 1
    ex = x >> f & ((1 << e) - 1)
    fr = x & ((1 << f) - 1)
    if ex == (1 << e) - 1:
        return ('nan', fr >> (f - 1) == 0) if fr else ('inf', s)
    m = fr if ex == 0 else fr | 1 << f
    exp = emin if ex == 0 else ex - bias
    return ('num', s, Fraction(m) * Fraction(2) ** (exp - f))


def round_to(fmt, rm, s, v, sticky):
    """Round the positive value v (exact up to the sticky bit, which says a nonzero remainder below
    v's last bit) with sign s. Returns (bits, flags)."""
    e, f, bias, emin, emax = params(fmt)
    if v == 0 and not sticky:
        return s << (e + f), 0
    # the exponent of v's leading bit; v >= 2^E
    E = v.numerator.bit_length() - v.denominator.bit_length()
    if Fraction(2) ** E > v:
        E -= 1
    def rnd(q_exp):
        """v / 2^q_exp rounded to an integer under rm, and whether that was inexact."""
        t = v / Fraction(2) ** q_exp
        n = t.numerator // t.denominator
        rem = t - n
        inexact = rem != 0 or sticky
        half = Fraction(1, 2)
        if rm == 0:   up = rem > half or (rem == half and not sticky and n & 1) or (rem == half and sticky)
        elif rm == 1: up = False
        elif rm == 2: up = s == 1 and inexact
        elif rm == 3: up = s == 0 and inexact
        else:         up = rem > half or rem == half
        return n + up, inexact
    # tininess after rounding: round at the unbounded exponent range, full precision
    n_full, _ = rnd(E - f)
    E_after = E + (1 if n_full >> (f + 1) else 0)
    tiny = E_after < emin
    q_exp = max(E, emin) - f                    # the last bit's weight: a subnormal's is emin - f
    n, inexact = rnd(q_exp)
    flags = NX if inexact else 0
    if n >> (f + 1):                             # carried out to the next binade
        n >>= 1
        q_exp += 1
    exp = q_exp + f
    if n >> f == 0:                              # subnormal (or rounded to zero)
        if tiny and inexact:
            flags |= UF
        return s << (e + f) | n, flags
    if exp > emax:                               # overflow
        flags |= OF | NX
        to_max = rm == 1 or (rm == 2 and s == 0) or (rm == 3 and s == 1)
        mag = ((1 << e) - 2) << f | ((1 << f) - 1) if to_max else ((1 << e) - 1) << f
        return s << (e + f) | mag, flags
    if tiny and inexact:
        flags |= UF
    return s << (e + f) | (exp + bias) << f | (n & ((1 << f) - 1)), flags


def fdiv(fmt, rm, a, b):
    ua, ub = unbox(fmt, a), unbox(fmt, b)
    da = decode(fmt, ua) if ua is not None else ('nan', False)
    db = decode(fmt, ub) if ub is not None else ('nan', False)
    e, f, *_ = params(fmt)
    if da[0] == 'nan' or db[0] == 'nan':
        nv = (da[0] == 'nan' and da[1]) or (db[0] == 'nan' and db[1])
        return box(fmt, canon_nan(fmt)), NV if nv else 0
    s = (da[1] ^ db[1])
    za = da[0] == 'num' and da[2] == 0
    zb = db[0] == 'num' and db[2] == 0
    if (da[0] == 'inf' and db[0] == 'inf') or (za and zb):
        return box(fmt, canon_nan(fmt)), NV
    if da[0] == 'inf':
        return box(fmt, s << (e + f) | ((1 << e) - 1) << f), 0
    if db[0] == 'inf':
        return box(fmt, s << (e + f)), 0
    if zb:
        return box(fmt, s << (e + f) | ((1 << e) - 1) << f), DZ
    r, fl = round_to(fmt, rm, s, da[2] / db[2], False)
    return box(fmt, r), fl


def fsqrt(fmt, rm, a, _b=0):
    ua = unbox(fmt, a)
    da = decode(fmt, ua) if ua is not None else ('nan', False)
    e, f, *_ = params(fmt)
    if da[0] == 'nan':
        return box(fmt, canon_nan(fmt)), NV if da[1] else 0
    if da[0] == 'inf':
        if da[1]:
            return box(fmt, canon_nan(fmt)), NV
        return box(fmt, ((1 << e) - 1) << f), 0
    s, v = da[1], da[2]
    if v == 0:
        return box(fmt, s << (e + f)), 0
    if s:
        return box(fmt, canon_nan(fmt)), NV
    # v = p / 2^k exactly (a binary value); scale so the root has far more bits than the format
    k = v.denominator.bit_length() - 1
    p = v.numerator
    extra = 2 * (f + 70)
    if (k + extra) & 1:
        extra += 1
    N = p << extra                               # v * 2^(k + extra) = N / 2^k ... root = isqrt(N) / 2^((k+extra)/2)
    root = isqrt(N)
    sticky = root * root != N
    r, fl = round_to(fmt, rm, 0, Fraction(root, 1 << ((k + extra) // 2)), sticky)
    return box(fmt, r), fl


# FMA variants: v = 0 FMADD (a*b+c), 1 FMSUB (a*b-c), 2 FNMSUB (-a*b+c),
# 3 FNMADD (-a*b-c), 4 ADD (a+c), 5 SUB (a-c), 6 MUL (a*b). ADD/SUB take a times an exact 1; MUL has
# no addend, so an exact zero product keeps its own sign.
FMA_NAMES = ['fmadd', 'fmsub', 'fnmsub', 'fnmadd', 'add', 'sub', 'mul']


def ffma(fmt, rm, v, a, b, c):
    e, f, *_ = params(fmt)
    one = box(fmt, (((1 << (e - 1)) - 1) << f))
    if v in (4, 5):
        b = one                                       # a + c: a times an exact 1
    used = [a, b] + ([] if v == 6 else [c])
    dec = []
    for x in used:
        ux = unbox(fmt, x)
        dec.append(decode(fmt, ux) if ux is not None else ('nan', False))
    da, db = dec[0], dec[1]
    dc = dec[2] if v != 6 else ('num', 0, Fraction(0))
    negp = v in (2, 3)
    negc = v in (1, 3, 5)
    snan = any(d[0] == 'nan' and d[1] for d in dec)
    zero = lambda d: d[0] == 'num' and d[2] == 0
    inf0 = (da[0] == 'inf' and zero(db)) or (db[0] == 'inf' and zero(da))
    if any(d[0] == 'nan' for d in dec) or inf0:
        return box(fmt, canon_nan(fmt)), NV if (snan or inf0) else 0
    sp = da[1] ^ db[1] ^ negp
    sc = dc[1] ^ negc
    pinf = da[0] == 'inf' or db[0] == 'inf'
    if pinf and dc[0] == 'inf' and sp != sc:
        return box(fmt, canon_nan(fmt)), NV
    if pinf:
        return box(fmt, sp << (e + f) | ((1 << e) - 1) << f), 0
    if dc[0] == 'inf':
        return box(fmt, sc << (e + f) | ((1 << e) - 1) << f), 0
    pv = da[2] * db[2] * (-1 if sp else 1)
    cv = dc[2] * (-1 if sc else 1)
    r = pv + cv
    if r == 0:
        if v == 6:
            s = sp
        elif pv == 0 and cv == 0:
            s = (sp | sc) if rm == 2 else (sp & sc)
        else:
            s = 1 if rm == 2 else 0
        return box(fmt, s << (e + f)), 0
    bits, fl = round_to(fmt, rm, 1 if r < 0 else 0, abs(r), False)
    return box(fmt, bits), fl


# The conversions: F2F (fcvt between binary16, binary32 and binary64), I2F (fcvt.{s,d}.{w,wu,l,lu}), F2I
# (fcvt.{w,wu,l,lu}.{s,d}). il: a 64-bit integer (l/lu), iu: unsigned (wu/lu). A 32-bit integer
# result is sign-extended to 64 bits; an integer source reads its low 32 bits for w/wu.
def f2f(sf, df, rm, a):
    ua = unbox(sf, a)
    da = decode(sf, ua) if ua is not None else ('nan', False)
    e, f, *_ = params(df)
    if da[0] == 'nan':
        return box(df, canon_nan(df)), NV if da[1] else 0
    if da[0] == 'inf':
        return box(df, da[1] << (e + f) | ((1 << e) - 1) << f), 0
    if da[2] == 0:
        return box(df, da[1] << (e + f)), 0
    r, fl = round_to(df, rm, da[1], da[2], False)
    return box(df, r), fl


def i2f(df, rm, x, il, iu):
    v = x & (2**64 - 1) if il else x & (2**32 - 1)
    w = 64 if il else 32
    if not iu and v >> (w - 1):
        v -= 1 << w
    if v == 0:
        return box(df, 0), 0
    r, fl = round_to(df, rm, 1 if v < 0 else 0, Fraction(abs(v)), False)
    return box(df, r), fl


def round_int(v, rm):
    """v (a Fraction) rounded to an integer under rm."""
    n = v.numerator // v.denominator                 # floor
    rem = v - n
    if rem == 0:
        return n
    half = Fraction(1, 2)
    if rm == 0:   up = rem > half or (rem == half and n & 1)
    elif rm == 1: up = v < 0                         # toward zero: floor of a negative goes up
    elif rm == 2: up = False
    elif rm == 3: up = True
    else:         up = rem >= half if v >= 0 else rem > half
    return n + up


def f2i(sf, rm, a, il, iu):
    w = 64 if il else 32
    lo, hi = (0, 2**w - 1) if iu else (-(2**(w - 1)), 2**(w - 1) - 1)
    ext = lambda n: (n & (2**w - 1)) if il else ((n & 0xFFFFFFFF) | (0xFFFFFFFF00000000 if (n >> 31) & 1 else 0))
    ua = unbox(sf, a)
    da = decode(sf, ua) if ua is not None else ('nan', False)
    if da[0] == 'nan':
        return ext(hi), NV
    if da[0] == 'inf':
        return ext(lo if da[1] else hi), NV
    v = -da[2] if da[1] else da[2]
    n = round_int(v, rm)
    if n < lo:
        return ext(lo), NV
    if n > hi:
        return ext(hi), NV
    return ext(n), NX if n != v else 0


def fcvtmod(a):
    """Zfa fcvtmod.w.d: toward zero, the low 32 bits of the unbounded integer, sign-extended; +-inf
    and NaN give 0. The flags are fcvt.w.d's (RTZ) for the same operand."""
    da = decode(1, a)
    _, fl = f2i(1, 1, a, 0, 0)
    if da[0] != 'num':
        return 0, fl
    v = -da[2] if da[1] else da[2]
    n = int(v) & 0xFFFFFFFF                          # int() truncates toward zero
    return n | (0xFFFFFFFF00000000 if n >> 31 else 0), fl


def fround(fmt, rm, a, nx):
    """Zfa fround (nx False) / froundnx: to an integral value in the same format. Zeros and
    infinities pass unchanged; a NaN gives the canonical NaN (NV for a signaling one); a zero result
    keeps the operand's sign; froundnx alone raises NX, when the result differs."""
    ua = unbox(fmt, a)
    da = decode(fmt, ua) if ua is not None else ('nan', False)
    e, f, *_ = params(fmt)
    if da[0] == 'nan':
        return box(fmt, canon_nan(fmt)), NV if da[1] else 0
    if da[0] == 'inf' or da[2] == 0:
        return box(fmt, ua), 0
    v = -da[2] if da[1] else da[2]
    n = round_int(v, rm)
    if n == 0:
        bits = da[1] << (e + f)
    else:
        bits, _ = round_to(fmt, 0, 1 if n < 0 else 0, Fraction(abs(n)), False)   # exact
    return box(fmt, bits), NX if nx and n != v else 0


FLI = [-1, 'min', Fraction(1, 2**16), Fraction(1, 2**15), Fraction(1, 2**8), Fraction(1, 2**7),
       Fraction(1, 16), Fraction(1, 8), Fraction(1, 4), Fraction(5, 16), Fraction(3, 8), Fraction(7, 16),
       Fraction(1, 2), Fraction(5, 8), Fraction(3, 4), Fraction(7, 8), 1, Fraction(5, 4), Fraction(3, 2),
       Fraction(7, 4), 2, Fraction(5, 2), 3, 4, 8, 16, 128, 256, 2**15, 2**16, 'inf', 'nan']


def fli(fmt, i):
    """Zfa fli: entry i of the constant table, in fmt."""
    e, f, bias, emin, _ = params(fmt)
    c = FLI[i]
    if c == 'nan':
        bits = canon_nan(fmt)
    elif c == 'inf':
        bits = ((1 << e) - 1) << f
    elif c == 'min':
        bits = 1 << f
    else:
        bits, _ = round_to(fmt, 0, 1 if c < 0 else 0, Fraction(abs(c)), False)
    return box(fmt, bits), 0


def fminmaxm(fmt, a, b, is_max):
    """Zfa fminm/fmaxm: fmin/fmax, except that a NaN in either operand gives the canonical NaN."""
    ua, ub = unbox(fmt, a), unbox(fmt, b)
    da = decode(fmt, ua) if ua is not None else ('nan', False)
    db = decode(fmt, ub) if ub is not None else ('nan', False)
    if da[0] == 'nan' or db[0] == 'nan':
        nv = (da[0] == 'nan' and da[1]) or (db[0] == 'nan' and db[1])
        return box(fmt, canon_nan(fmt)), NV if nv else 0
    return fminmax(fmt, a, b, is_max)


def fcmpq(fmt, a, b, lt):
    """Zfa fleq (lt False) / fltq: the quiet compares, NV only for a signaling NaN."""
    ua, ub = unbox(fmt, a), unbox(fmt, b)
    da = decode(fmt, ua) if ua is not None else ('nan', False)
    db = decode(fmt, ub) if ub is not None else ('nan', False)
    if da[0] == 'nan' or db[0] == 'nan':
        nv = (da[0] == 'nan' and da[1]) or (db[0] == 'nan' and db[1])
        return 0, NV if nv else 0
    val = lambda d: (Fraction(-1) if d[1] else Fraction(1)) * (Fraction(10) ** 400 if d[0] == 'inf' else d[2])
    return int(val(da) < val(db) if lt else val(da) <= val(db)), 0


def fma_vectors(n, seed):
    """Lines "v sf df rm x a b c result flags" (sf, df: the source and destination formats, 0 binary32,
    1 binary64, 2 binary16; x: {il, iu}); a quarter are conversions (v 7 F2F, 8 I2F, 9 F2I, 10
    fcvtmod.w.d), a quarter of the rest cancel: c is about -(a*b)."""
    rng = random.Random(seed)
    out = sys.stdout
    for i in range(n):
        fmt = rng.getrandbits(1); rm = rng.randrange(5); v = rng.randrange(7)
        a, b, c = operand(rng, fmt), operand(rng, fmt), operand(rng, fmt)
        if i % 4 == 0:
            # near-cancellation: c = the product rounded, nudged by a few ulps, signed to cancel
            pa, pb = decode(fmt, unbox(fmt, a) or 0), decode(fmt, unbox(fmt, b) or 0)
            if pa[0] == 'num' and pb[0] == 'num' and pa[2] * pb[2] != 0:
                bits, _ = round_to(fmt, rng.randrange(5), 0, pa[2] * pb[2], False)
                e, f, *_ = params(fmt)
                bits = max(0, min(bits + rng.randrange(-3, 4), ((1 << e) - 1) << f - 1))
                sp = pa[1] ^ pb[1] ^ (v in (2, 3))
                want_neg = sp ^ 1 ^ (v in (1, 3))           # c's sign that cancels after negc
                c = box(fmt, want_neg << (e + f) | bits)
                if v in (4, 5):
                    c = box(fmt, (pa[1] ^ 1 ^ (v == 5)) << (e + f) | (unbox(fmt, a) & ((1 << (e + f)) - 1)) ^ rng.getrandbits(3))
        sf = df = fmt; x = 0
        if rng.randrange(4) == 0:                    # a conversion
            v = 7 + rng.randrange(4); x = rng.randrange(4); sf = rng.getrandbits(1); df = rng.getrandbits(1)
            if v == 7:                               # between any two of binary16, binary32, binary64
                sf, df = rng.choice([(0, 1), (1, 0), (2, 0), (2, 1), (0, 2), (1, 2)])
                a = operand(rng, sf)
                r, fl = f2f(sf, df, rm, a)
            elif v == 10:                            # fcvtmod.w.d: binary64, RTZ
                sf = df = 1; rm = 1; x = 0
                a = operand(rng, 1)
                if rng.randrange(2):                 # around the 32-bit range and beyond 2^64
                    a = rng.getrandbits(1) << 63 | (1023 + rng.choice([-1, 0, 30, 31, 32, 52, 62, 63, 64, 83, 84, 85, 100])) << 52 | rng.getrandbits(52)
                r, fl = fcvtmod(a)
            elif v == 8:
                k = rng.randrange(6)
                a = [rng.getrandbits(64), rng.getrandbits(rng.randrange(1, 65)), -rng.getrandbits(rng.randrange(1, 64)) & (2**64 - 1),
                     rng.choice([0, 1, 2**31, 2**31 - 1, 2**32 - 1, 2**63, 2**63 - 1, 2**64 - 1]), (1 << rng.randrange(64)) | rng.getrandbits(3),
                     rng.getrandbits(64) & ~((1 << rng.randrange(64)) - 1)][k]
                r, fl = i2f(df, rm, a, x >> 1, x & 1)
            else:
                a = operand(rng, sf)
                if rng.randrange(2):                 # near the integer range and its halves
                    e, f, bias, *_ = params(sf)
                    ex = bias + rng.choice([-1, 0, 1, 30, 31, 32, 62, 63, 64]) + rng.randrange(-1, 2)
                    a = box(sf, rng.getrandbits(1) << (e + f) | (ex & ((1 << e) - 1)) << f | rng.getrandbits(f))
                r, fl = f2i(sf, rm, a, x >> 1, x & 1)
            b = c = 0
        else:
            r, fl = ffma(fmt, rm, v, a, b, c)
        out.write(f"{v} {sf} {df} {rm} {x} {a:016x} {b:016x} {c:016x} {r:016x} {fl:02x}\n")


def fminmax(fmt, a, b, is_max):
    ua, ub = unbox(fmt, a), unbox(fmt, b)
    da = decode(fmt, ua) if ua is not None else ('nan', False)
    db = decode(fmt, ub) if ub is not None else ('nan', False)
    e, f, *_ = params(fmt)
    nv = NV if (da[0] == 'nan' and da[1]) or (db[0] == 'nan' and db[1]) else 0
    if da[0] == 'nan' and db[0] == 'nan':
        return box(fmt, canon_nan(fmt)), nv
    if da[0] == 'nan':
        return box(fmt, ub), nv
    if db[0] == 'nan':
        return box(fmt, ua), nv
    def val(d):
        return (Fraction(-1) if d[1] else Fraction(1)) * (Fraction(10) ** 400 if d[0] == 'inf' else d[2])
    va, vb = val(da), val(db)
    if va == vb == 0:                           # -0 < +0
        sa, sb = ua >> (e + f), ub >> (e + f)
        neg = (sa & sb) if is_max else (sa | sb)
        return box(fmt, neg << (e + f)), nv
    pick_a = (va > vb) if is_max else (va < vb)
    if va == vb:
        pick_a = True
    return box(fmt, ua if pick_a else ub), nv


OPS = {'div': fdiv, 'sqrt': fsqrt,
       'min': lambda fmt, rm, a, b: fminmax(fmt, a, b, False),
       'max': lambda fmt, rm, a, b: fminmax(fmt, a, b, True)}


def operand(rng, fmt):
    """A random operand, biased toward the cases that break units: specials, subnormals, the
    boundaries of the exponent range, ones and all-ones significands."""
    e, f, bias, *_ = params(fmt)
    k = rng.randrange(16)
    s = rng.getrandbits(1)
    if k == 0:   ex, fr = 0, 0                                         # zero
    elif k == 1: ex, fr = 0, rng.getrandbits(f) or 1                   # subnormal
    elif k == 2: ex, fr = 0, 1 << rng.randrange(f)                     # one-bit subnormal
    elif k == 3: ex, fr = (1 << e) - 1, 0                              # inf
    elif k == 4: ex, fr = (1 << e) - 1, rng.getrandbits(f) or 1        # NaN, either kind
    elif k == 5: ex, fr = rng.choice([1, 2, (1 << e) - 2, (1 << e) - 3]), rng.getrandbits(f)
    elif k == 6: ex, fr = rng.randrange(1, (1 << e) - 1), (1 << f) - 1 # all-ones significand
    elif k == 7: ex, fr = rng.randrange(1, (1 << e) - 1), 0            # a power of two
    elif k == 8: ex, fr = bias + rng.randrange(-3, 4), rng.getrandbits(f)
    else:        ex, fr = rng.randrange(1, (1 << e) - 1), rng.getrandbits(f)
    x = s << (e + f) | ex << f | fr
    if fmt != 1:
        return box(fmt, x) if rng.randrange(64) else rng.getrandbits(64)   # 1/64: not NaN-boxed
    return x


def exact_pair(rng, fmt, op):
    """Operands whose result is exact or a tie: a = q * b (division) or a = q^2 (square root)."""
    e, f, bias, *_ = params(fmt)
    h = (f + 2) // 2                    # q has f - h + 1 bits: its square fits the significand
    q = (1 << (f - h)) | rng.getrandbits(f - h)
    qe = bias + rng.randrange(-40, 41)
    if op == 'sqrt':
        r = Fraction(q * q, 1 << (2 * (f - h))) * Fraction(2) ** (2 * rng.randrange(-30, 31))
        bits, _ = round_to(fmt, 1, 0, r, False)
        return box(fmt, bits), 0
    b = operand(rng, fmt)
    db = decode(fmt, unbox(fmt, b) or 0)
    if db[0] != 'num' or db[2] == 0:
        return operand(rng, fmt), b
    v = Fraction(q, 1 << (f - h)) * Fraction(2) ** (qe - bias) * db[2]
    bits, _ = round_to(fmt, 1, rng.getrandbits(1), v, False)
    return box(fmt, bits), b


def zfa_vectors(n, seed):
    """Lines "op fmt rm a b result flags" for the core's own Zfa ops: op 0 fround, 1 froundnx, 2 fminm,
    3 fmaxm, 4 fleq, 5 fltq (an integer result), 6 fli (a is the table index)."""
    rng = random.Random(seed)
    out = sys.stdout
    for i in range(n):
        fmt = rng.getrandbits(1); rm = rng.randrange(5); op = rng.randrange(7)
        a, b = operand(rng, fmt), operand(rng, fmt)
        if op in (0, 1) and rng.randrange(2):        # near the integers: halves, and the edge of 2^f
            e, f, bias, *_ = params(fmt)
            ex = bias + rng.choice([-2, -1, 0, 1, 2, 3, f - 2, f - 1, f, f + 1])
            a = box(fmt, rng.getrandbits(1) << (e + f) | ex << f | (rng.getrandbits(f) & ~((1 << rng.randrange(f)) - 1)))
        if op in (2, 3, 4, 5) and i % 4 == 0:
            b = a ^ (rng.getrandbits(1) << (63 if fmt else 31))
        if op == 6:
            a, b = rng.randrange(32), 0
        r, fl = [lambda: fround(fmt, rm, a, False), lambda: fround(fmt, rm, a, True),
                 lambda: fminmaxm(fmt, a, b, False), lambda: fminmaxm(fmt, a, b, True),
                 lambda: fcmpq(fmt, a, b, False), lambda: fcmpq(fmt, a, b, True), lambda: fli(fmt, a)][op]()
        out.write(f"{op} {fmt} {rm} {a:016x} {b:016x} {r:016x} {fl:02x}\n")


def main():
    if len(sys.argv) >= 4 and sys.argv[1] == 'vectors' and sys.argv[2] in ('fma', 'zfa'):
        (fma_vectors if sys.argv[2] == 'fma' else zfa_vectors)(int(sys.argv[3]), int(sys.argv[4]) if len(sys.argv) > 4 else 1)
        return
    if len(sys.argv) < 4 or sys.argv[1] != 'vectors' or sys.argv[2] not in OPS:
        sys.exit(__doc__)
    op, n = sys.argv[2], int(sys.argv[3])
    rng = random.Random(int(sys.argv[4]) if len(sys.argv) > 4 else 1)
    fn = OPS[op]
    out = sys.stdout
    for i in range(n):
        fmt = rng.getrandbits(1)
        rm = rng.randrange(5)
        if i % 4 == 0 and op in ('div', 'sqrt'):
            a, b = exact_pair(rng, fmt, op)
        elif i % 4 == 0:
            a = operand(rng, fmt); b = a ^ (rng.getrandbits(1) << (63 if fmt else 31))   # equal, or the same magnitude
        else:
            a, b = operand(rng, fmt), operand(rng, fmt)
        if op == 'sqrt':
            b = 0
            if rng.randrange(2) and unbox(fmt, a) is not None:   # mostly positive radicands
                e, f, *_ = params(fmt)
                a &= ~(1 << (e + f))
        r, fl = fn(fmt, rm, a, b)
        out.write(f"{fmt} {rm} {a:016x} {b:016x} {r:016x} {fl:02x}\n")


if __name__ == '__main__':
    main()
