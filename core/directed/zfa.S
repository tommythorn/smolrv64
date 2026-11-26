# zfa.S -- directed test: Zfa and Zfhmin through the whole core (decode, the F stage's own ops,
# fp_fma's binary16 conversions and fcvtmod.w.d, the LSU's binary16 loads and stores), every
# result and the flags each one raises, against values computed by tools/fpref.py.
#
#   1  flh NaN-boxes, fsh stores the low 16 bits, fmv.x.h sign-extends, fmv.h.x boxes
#   2  fcvt.s.h / fcvt.d.h (exact), fcvt.h.s / fcvt.h.d (rounded, NX/OF), an unboxed source
#   3  fli.s / fli.d
#   4  fround (no NX) / froundnx (NX), a dynamic rounding mode from frm, -0 kept
#   5  fcvtmod.w.d: in range, wrapped out of range (NV), NaN (0, NV)
#   6  fminm / fmaxm (a NaN gives the canonical NaN), fleq / fltq (quiet: no NV for a qNaN)
#   7  the reserved neighbours trap: fcvtmod with rm RNE, fli.h, fadd.h, fcvt.h.h
#
# M-mode, no MMU, mstatus.FS on.   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.option arch, +zfa, +zfhmin, +d
	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	li	t0, 0x2000		# mstatus.FS = Initial
	csrs	mstatus, t0
	li	s3, 0
	li	s5, 0
	la	s2, fail0
	la	a0, area

	#---------------- 1: binary16 loads, stores and moves
	flh	ft0, 0(a0)		# 0xc248 = -3.140625
	fmv.x.d	t2, ft0
	li	t1, 0xffffffffffffc248
	bne	t2, t1, fail1
	fmv.x.h	t2, ft0			# sign-extended
	li	t1, 0xffffffffffffc248
	bne	t2, t1, fail1
	li	t3, 0x12343c00		# fmv.h.x takes the low 16 bits: 1.0
	fmv.h.x	ft1, t3
	fmv.x.d	t2, ft1
	li	t1, 0xffffffffffff3c00
	bne	t2, t1, fail1
	fsh	ft0, 8(a0)		# two bytes only
	ld	t2, 8(a0)
	li	t1, 0x5555555555550000 | 0xc248
	bne	t2, t1, fail1

	#---------------- 2: the conversions
	fcvt.s.h fa0, ft0		# -3.140625f
	fmv.x.d	t2, fa0
	li	t1, 0xffffffffc0490000
	bne	t2, t1, fail2
	fcvt.d.h fa1, ft1		# 1.0
	fmv.x.d	t2, fa1
	li	t1, 0x3ff0000000000000
	bne	t2, t1, fail2
	csrw	fflags, x0
	li	t3, 0x400921fb54442d18	# pi
	fmv.d.x	fa2, t3
	fcvt.h.d fa3, fa2		# RNE: 0x4248, inexact
	fmv.x.d	t2, fa3
	li	t1, 0xffffffffffff4248
	bne	t2, t1, fail2
	csrr	t2, fflags
	li	t1, 1			# NX
	bne	t2, t1, fail2
	csrw	fflags, x0
	li	t3, 0x47800000		# 65536.0f overflows binary16
	fmv.w.x	fa4, t3
	fcvt.h.s fa5, fa4, rtz		# RTZ: the largest finite, 0x7bff
	fmv.x.d	t2, fa5
	li	t1, 0xffffffffffff7bff
	bne	t2, t1, fail2
	csrr	t2, fflags
	li	t1, 5			# OF | NX
	bne	t2, t1, fail2
	li	t3, 0x3c00		# an unboxed binary16 operand reads as the canonical NaN
	fmv.d.x	fa6, t3
	fcvt.s.h fa7, fa6
	fmv.x.d	t2, fa7
	li	t1, 0xffffffff7fc00000
	bne	t2, t1, fail2

	#---------------- 3: fli
	fli.d	fs0, 0x1p-16		# entry 2
	fmv.x.d	t2, fs0
	li	t1, 0x3ef0000000000000
	bne	t2, t1, fail3
	fli.s	fs1, 1.75		# entry 19
	fmv.x.d	t2, fs1
	li	t1, 0xffffffff3fe00000
	bne	t2, t1, fail3
	fli.s	fs1, min		# entry 1
	fmv.x.d	t2, fs1
	li	t1, 0xffffffff00800000
	bne	t2, t1, fail3
	fli.d	fs0, nan		# entry 31
	fmv.x.d	t2, fs0
	li	t1, 0x7ff8000000000000
	bne	t2, t1, fail3

	#---------------- 4: fround / froundnx
	csrw	fflags, x0
	li	t3, 0x4004000000000000	# 2.5
	fmv.d.x	fs2, t3
	fround.d fs3, fs2, rne		# 2.0, no flag
	fmv.x.d	t2, fs3
	li	t1, 0x4000000000000000
	bne	t2, t1, fail4
	csrr	t2, fflags
	bnez	t2, fail4
	froundnx.d fs3, fs2, rmm	# 3.0, NX
	fmv.x.d	t2, fs3
	li	t1, 0x4008000000000000
	bne	t2, t1, fail4
	csrr	t2, fflags
	li	t1, 1
	bne	t2, t1, fail4
	li	t3, 2			# frm = RDN
	csrw	frm, t3
	li	t3, 0xbfd0000000000000	# -0.25
	fmv.d.x	fs2, t3
	fround.d fs3, fs2		# dynamic RDN: -1.0
	fmv.x.d	t2, fs3
	li	t1, 0xbff0000000000000
	bne	t2, t1, fail4
	fround.d fs3, fs2, rtz		# -0.0: the sign kept
	fmv.x.d	t2, fs3
	li	t1, 0x8000000000000000
	bne	t2, t1, fail4
	csrw	frm, x0
	li	t3, 0xffffffff4b7fffff	# 16777215.0f: already integral
	fmv.d.x	fs2, t3
	froundnx.s fs3, fs2, rup
	fmv.x.d	t2, fs3
	bne	t2, t3, fail4

	#---------------- 5: fcvtmod.w.d
	csrw	fflags, x0
	li	t3, 0xc1e0000000200000	# -2147483649.0: wraps to 0x7fffffff, NV
	fmv.d.x	fs4, t3
	fcvtmod.w.d t2, fs4, rtz
	li	t1, 0x7fffffff
	bne	t2, t1, fail5
	csrr	t2, fflags
	li	t1, 16
	bne	t2, t1, fail5
	csrw	fflags, x0
	li	t3, 0xbff8000000000000	# -1.5: -1, NX
	fmv.d.x	fs4, t3
	fcvtmod.w.d t2, fs4, rtz
	li	t1, -1
	bne	t2, t1, fail5
	csrr	t2, fflags
	li	t1, 1
	bne	t2, t1, fail5
	csrw	fflags, x0
	li	t3, 0x7ff8000000000000	# NaN: 0, NV
	fmv.d.x	fs4, t3
	fcvtmod.w.d t2, fs4, rtz
	bnez	t2, fail5
	csrr	t2, fflags
	li	t1, 16
	bne	t2, t1, fail5

	#---------------- 6: fminm / fmaxm, fleq / fltq
	csrw	fflags, x0
	li	t3, 0x7ff8000000000000	# a quiet NaN
	fmv.d.x	fs5, t3
	li	t3, 0x3ff0000000000000	# 1.0
	fmv.d.x	fs6, t3
	fminm.d	fs7, fs5, fs6		# canonical NaN (fmin would give 1.0)
	fmv.x.d	t2, fs7
	li	t1, 0x7ff8000000000000
	bne	t2, t1, fail6
	fmaxm.d	fs7, fs6, fs6
	fmv.x.d	t2, fs7
	bne	t2, t3, fail6
	fleq.d	t2, fs5, fs6		# a qNaN: 0, quiet
	bnez	t2, fail6
	fltq.d	t2, fs6, fs6
	bnez	t2, fail6
	fleq.d	t2, fs6, fs6
	li	t1, 1
	bne	t2, t1, fail6
	csrr	t2, fflags
	bnez	t2, fail6		# no NV from the quiet compares or fminm on a qNaN

	#---------------- 7: reserved neighbours trap as illegal
	la	s2, 71f
	.word	0xc2860553		# fcvtmod.w.d a0, fa2 with rm RNE
	li	s5, 99
71:	li	t1, 1
	bne	s5, t1, fail7
	li	t1, 2
	bne	s3, t1, fail7
	li	s5, 0
	la	s2, 72f
	.word	0xf4108053		# fli.h ft0, min
	li	s5, 99
72:	li	t1, 1
	bne	s5, t1, fail7
	li	s5, 0
	la	s2, 73f
	.word	0x04208053		# fadd.h ft0, ft1, ft2
	li	s5, 99
73:	li	t1, 1
	bne	s5, t1, fail7
	li	s5, 0
	la	s2, 74f
	.word	0x44208053		# fcvt.h.h ft0, ft1
	li	s5, 99
74:	li	t1, 1
	bne	s5, t1, fail7

	li	a0, 1
	j	done
fail0:	li	a0, 1001
	j	done
fail1:	li	a0, 3
	j	done
fail2:	li	a0, 5
	j	done
fail3:	li	a0, 7
	j	done
fail4:	li	a0, 9
	j	done
fail5:	li	a0, 11
	j	done
fail6:	li	a0, 13
	j	done
fail7:	li	a0, 15
done:
	la	t0, tohost
	sd	a0, 0(t0)
99:	j	99b

	.align 2
trap_handler:
	csrr	s3, mcause
	addi	s5, s5, 1
	jr	s2

	.section .data
	.align 3
	.globl area
area:	.dword	0x000000000000c248
	.dword	0x5555555555555555
	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
