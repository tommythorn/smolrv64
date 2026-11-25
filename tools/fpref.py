#!/usr/bin/env python3
"""fpref -- the exact reference for the FPU's benches: RISC-V F/D results and flags, by rational arithmetic.

    tools/fpref.py vectors <op> <n> [seed]     n random vectors for op (div, sqrt) as lines
        "<fmt> <rm> <a> <b> <result> <flags>": fmt 0 = binary32, 1 = binary64; rm 0..4 = RNE RTZ RDN
        RUP RMM; a, b, result 16 hex digits (a binary32 value NaN-boxed: its upper 32 bits ones);
        flags = {NV, DZ, OF, UF, NX} as 2 hex digits.

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

FMT = {0: (8, 23), 1: (11, 52)}            # exponent bits, fraction bits
NV, DZ, OF, UF, NX = 16, 8, 4, 2, 1


def params(fmt):
    e, f = FMT[fmt]
    bias = (1 << (e - 1)) - 1
    return e, f, bias, 1 - bias, bias        # ..., emin, emax


def unbox(fmt, x):
    """The operand's own bits, or None for a binary32 value that is not NaN-boxed."""
    if fmt == 1:
        return x
    return x & 0xFFFFFFFF if (x >> 32) == 0xFFFFFFFF else None


def box(fmt, x):
    return x if fmt == 1 else (0xFFFFFFFF << 32) | x


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


OPS = {'div': fdiv, 'sqrt': fsqrt}


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
    if fmt == 0:
        return box(0, x) if rng.randrange(64) else rng.getrandbits(64)   # 1/64: not NaN-boxed
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


def main():
    if len(sys.argv) < 4 or sys.argv[1] != 'vectors' or sys.argv[2] not in OPS:
        sys.exit(__doc__)
    op, n = sys.argv[2], int(sys.argv[3])
    rng = random.Random(int(sys.argv[4]) if len(sys.argv) > 4 else 1)
    fn = OPS[op]
    out = sys.stdout
    for i in range(n):
        fmt = rng.getrandbits(1)
        rm = rng.randrange(5)
        if i % 4 == 0:
            a, b = exact_pair(rng, fmt, op)
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
