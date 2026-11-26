# fpill.S -- directed test: an FP-opcode word this core does not implement traps as illegal.
#
# riscv-tests runs only the F and D operations, so nothing in it reaches an unassigned OP-FP
# funct5, the H or Q format, or an FP load or store of a width the core lacks (Q; H is Zfhmin's,
# directed/zfa.S). Before the fix such a word
# decoded as a legal FP op that no unit implements: dispatch classed it a head op AND plain, so it
# could be grouped beside other instructions, and alone it reached M, which holds no FP arithmetic.
#
# Cases 1-5 require cause 2 (illegal instruction); case 6 requires that real D loads, an add and a
# store still work. The neighbouring addi's put each word in a group with others.
#
# M-mode, no MMU, mstatus.FS on. The FP words are .word: the directed build is rv64ima.
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0		# direct mode
	li	t0, 0x2000		# mstatus.FS = Initial
	csrs	mstatus, t0
	li	s3, 0			# captured mcause
	li	s5, 0			# trap count
	la	a0, area

	#---------------- 1: an unassigned OP-FP funct5 (00110, D format)
	li	s5, 0
	la	s2, 1f
	addi	t3, t3, 1
	.word	0x322081d3		# funct7 0011001 ft3, ft1, ft2
	addi	t4, t4, 1
	li	s5, 99			# reached only if it did NOT trap
1:	li	t1, 1
	bne	s5, t1, fail1
	li	t1, 2
	bne	s3, t1, fail1

	#---------------- 2: the H format (fadd.h ft3, ft1, ft2)
	li	s5, 0
	la	s2, 2f
	addi	t3, t3, 1
	.word	0x042081d3
	addi	t4, t4, 1
	li	s5, 99
2:	li	t1, 1
	bne	s5, t1, fail2
	li	t1, 2
	bne	s3, t1, fail2

	#---------------- 3: the Q format (fmadd.q ft3, ft1, ft2, ft4)
	li	s5, 0
	la	s2, 3f
	addi	t3, t3, 1
	.word	0x262081c3
	addi	t4, t4, 1
	li	s5, 99
3:	li	t1, 1
	bne	s5, t1, fail3
	li	t1, 2
	bne	s3, t1, fail3

	#---------------- 4: a quad-word FP load (flq ft3, 0(a0))
	li	s5, 0
	la	s2, 4f
	addi	t3, t3, 1
	.word	0x00054187
	addi	t4, t4, 1
	li	s5, 99
4:	li	t1, 1
	bne	s5, t1, fail4
	li	t1, 2
	bne	s3, t1, fail4

	#---------------- 5: a quad-word FP store (fsq ft3, 0(a0))
	li	s5, 0
	la	s2, 5f
	addi	t3, t3, 1
	.word	0x00354027
	addi	t4, t4, 1
	li	s5, 99
5:	li	t1, 1
	bne	s5, t1, fail5
	li	t1, 2
	bne	s3, t1, fail5

	#---------------- 6: fld, fadd.d and fsd still work: 1.5 + 2.25 = 3.75
	li	s5, 0
	la	s2, fail6		# a trap is a failure
	.word	0x00053087		# fld   ft1, 0(a0)
	.word	0x00853107		# fld   ft2, 8(a0)
	.word	0x022081d3		# fadd.d ft3, ft1, ft2, rne
	.word	0x00353827		# fsd   ft3, 16(a0)
	ld	t2, 16(a0)
	li	t1, 0x400E000000000000	# 3.75
	bne	t2, t1, fail6

	li	a0, 1			# PASS
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
done:
	la	t0, tohost
	sd	a0, 0(t0)
9:	j	9b

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
area:	.dword 0x3FF8000000000000	# 1.5
	.dword 0x4002000000000000	# 2.25
	.dword 0
	.dword 0

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
