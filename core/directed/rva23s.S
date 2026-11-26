# rva23s.S -- directed test: the RVA23 scalar encodings decoded by the expander and the
# decoder (Zcb, Zcmop, Zimop, Zawrs), and the reserved encodings beside them still trapping.
#
# Cases 1-4 check every Zcb operation's value; 5 that c.mop.N, mop.r.N/mop.rr.N and
# wrs.nto/wrs.sto do what they define (nothing, rd <- 0, nothing) and trap nowhere; 6-10 that
# reserved neighbours raise cause 2 (illegal instruction): c.sh with bit 6 set, c.lui x16 with
# a zero immediate, the reserved Zcb unary selector, an H-extension load (SYSTEM funct3 100,
# not a mop) and a funct3-100 word whose bits [29:28] are not 00.
#
# M-mode, no MMU.   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.option arch, +c, +zcb, +zcmop, +zimop, +zawrs, +zba, +zbb, +zmmul
	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0		# direct mode
	li	s3, 0			# captured mcause
	li	s5, 0			# trap count
	la	s2, fail0		# a trap before case 6 lands here

	#---------------- 1: Zcb loads (registers x8-x15 only)
	la	a0, area
	c.lbu	a1, 1(a0)		# 0xf2
	li	t1, 0xf2
	bne	a1, t1, fail1
	c.lhu	a2, 2(a0)		# 0x8483
	li	t1, 0x8483
	bne	a2, t1, fail1
	c.lh	a3, 2(a0)		# sign-extended
	li	t1, -0x7b7d
	bne	a3, t1, fail1
	c.lbu	a4, 0(a0)		# offset bit 0 and bit 1 both reach the address
	li	t1, 0x81
	bne	a4, t1, fail1
	c.lbu	a4, 3(a0)
	li	t1, 0x84
	bne	a4, t1, fail1

	#---------------- 2: Zcb stores
	addi	a5, a0, 8
	c.sb	a1, 0(a5)		# 0xf2 at +8
	c.sb	a1, 3(a5)		# 0xf2 at +11
	c.sh	a2, 2(a5)		# 0x8483 at +10 (overwrites +11)
	ld	t2, 8(a0)
	li	t1, 0x848300f2
	bne	t2, t1, fail2

	#---------------- 3: Zcb unary ops
	li	a1, -2
	c.zext.b a1			# 0xfe
	li	t1, 0xfe
	bne	a1, t1, fail3
	li	a1, 0x180
	c.sext.b a1			# 0x80 -> -128
	li	t1, -128
	bne	a1, t1, fail3
	li	a1, -2
	c.zext.h a1			# 0xfffe
	li	t1, 0xfffe
	bne	a1, t1, fail3
	li	a1, 0x18000
	c.sext.h a1			# 0x8000 -> -32768
	li	t1, -32768
	bne	a1, t1, fail3
	li	a1, -1
	c.zext.w a1			# 0xffffffff
	li	t1, 0xffffffff
	bne	a1, t1, fail3
	li	a1, 0x5a
	c.not	a1
	li	t1, ~0x5a
	bne	a1, t1, fail3

	#---------------- 4: c.mul
	li	a1, -7
	li	a2, 6
	c.mul	a1, a2
	li	t1, -42
	bne	a1, t1, fail4

	#---------------- 5: may-be-operations and Zawrs: no trap, rd <- 0 for the mops
	li	ra, 123
	li	t0, 456
	c.mop.1				# sspush ra under Zicfiss; a nop here
	c.mop.5
	c.mop.15
	li	t1, 123
	bne	ra, t1, fail5
	li	t1, 456
	bne	t0, t1, fail5
	li	t2, 77
	li	t3, 88
	mop.r.0	t2, t3
	bnez	t2, fail5
	li	t2, 77
	mop.r.31 t2, t3
	bnez	t2, fail5
	li	t2, 77
	mop.rr.7 t2, t3, t0
	bnez	t2, fail5
	li	t1, 88
	bne	t3, t1, fail5
	wrs.nto
	wrs.sto
	bnez	s5, fail5		# nothing above trapped

	#---------------- 6: c.sh with bit 6 set is reserved
	la	s2, 6f
	.hword	0x8d4c
	li	s5, 99
6:	li	t1, 1
	bne	s5, t1, fail6
	li	t1, 2
	bne	s3, t1, fail6

	#---------------- 7: c.lui x16, 0 is reserved (only odd rd < 16 are c.mop.N)
	li	s5, 0
	la	s2, 7f
	.hword	0x6801
	li	s5, 99
7:	li	t1, 1
	bne	s5, t1, fail7
	li	t1, 2
	bne	s3, t1, fail7

	#---------------- 8: the reserved Zcb unary selector (110)
	li	s5, 0
	la	s2, 8f
	.hword	0x9d79
	li	s5, 99
8:	li	t1, 1
	bne	s5, t1, fail8
	li	t1, 2
	bne	s3, t1, fail8

	#---------------- 9: hlv.b a0, (a1): SYSTEM funct3 100 but not a mop (no H here)
	li	s5, 0
	la	s2, 9f
	.word	0x6005c573
	li	s5, 99
9:	li	t1, 1
	bne	s5, t1, fail9
	li	t1, 2
	bne	s3, t1, fail9

	#---------------- 10: funct3 100 with bits [29:28] = 01 is not a mop
	li	s5, 0
	la	s2, 10f
	.word	0x91c5c573
	li	s5, 99
10:	li	t1, 1
	bne	s5, t1, fail10
	li	t1, 2
	bne	s3, t1, fail10

	li	a0, 1			# PASS
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
	j	done
fail8:	li	a0, 17
	j	done
fail9:	li	a0, 19
	j	done
fail10:	li	a0, 21
done:
	la	t0, tohost
	sd	a0, 0(t0)
99:	j	99b

	# Records the cause and resumes at s2 rather than mret-ing: the faulting
	# instruction must not be retried, and each case wants the trap observable.
	.align 2
trap_handler:
	csrr	s3, mcause
	addi	s5, s5, 1
	jr	s2

	.section .data
	.align 3
	.globl area
area:	.byte	0x81, 0xf2, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88
	.dword	0
	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
