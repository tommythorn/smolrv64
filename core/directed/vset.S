# vset.S -- directed test: vsetvli and vsetivli (the cracker's first instructions: a vl uop into the
# VL file, then rd), at VLEN 128.
#
#   1  VS Off: vsetvli is an illegal instruction
#   2  vsetvli rd, rs1: vl = min(AVL, VLMAX) across SEW and LMUL, and csrr vl / vtype agree
#   3  vsetivli: AVL is the immediate; a fractional LMUL
#   4  rs1 = x0, rd != x0: vl = VLMAX; rd = rs1 = x0: vtype changes and vl stays
#   5  an unsupported vtype (e64 at mf8, a reserved bit): vill, vl 0
#   6  60 vsets in a loop, half of them behind a branch that alternates (the vl ring wraps, and
#      the mispredicted ones are squashed): the last one's vl and vtype hold
#   7  vsetvl: vtype from a register, a reserved bit (vill), the keep form, and a vsetvli after it
#
# A trap with mcause 2 is counted in s5 and returns past the instruction.
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.option	arch, +v
#define VL     0xc20
#define VTYPE  0xc21

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	li	s5, 0
	csrw	mstatus, x0

	#---------------- 1: VS Off
	li	a0, 4
	vsetvli	t0, a0, e8, m1, ta, ma
	li	t2, 1
	bne	s5, t2, fail1
	li	t0, 1 << 9		# VS Initial
	csrw	mstatus, t0
	li	s5, 0

	#---------------- 2: vsetvli rd, rs1
	li	a0, 5
	vsetvli	t0, a0, e8, m1, ta, ma	# VLMAX 16
	li	t2, 5
	bne	t0, t2, fail2
	csrr	t1, VL
	bne	t1, t2, fail2
	csrr	t1, VTYPE
	li	t2, 0xc0		# vma vta, e8, m1
	bne	t1, t2, fail2
	li	a0, 100
	vsetvli	t0, a0, e32, m2, tu, mu	# VLMAX 8
	li	t2, 8
	bne	t0, t2, fail2
	csrr	t1, VL
	bne	t1, t2, fail2
	csrr	t1, VTYPE
	li	t2, 0x11		# e32, m2
	bne	t1, t2, fail2
	li	a0, 128
	vsetvli	t0, a0, e8, m8, ta, mu	# VLMAX 128
	li	t2, 128
	bne	t0, t2, fail2

	#---------------- 3: vsetivli
	vsetivli t0, 3, e16, mf2, ta, ma	# VLMAX 4
	li	t2, 3
	bne	t0, t2, fail3
	vsetivli t0, 31, e16, mf2, ta, ma
	li	t2, 4
	bne	t0, t2, fail3
	csrr	t1, VL
	bne	t1, t2, fail3
	csrr	t1, VTYPE
	li	t2, 0xcf		# vma vta, e16, mf2
	bne	t1, t2, fail3

	#---------------- 4: the x0 forms
	vsetvli	t0, x0, e64, m8, ta, ma	# VLMAX 16
	li	t2, 16
	bne	t0, t2, fail4
	vsetvli	x0, x0, e32, m8, ta, ma	# vtype changes, vl stays 16
	csrr	t1, VL
	bne	t1, t2, fail4
	csrr	t1, VTYPE
	li	t2, 0xd3
	bne	t1, t2, fail4

	#---------------- 5: vill
	li	a0, 7
	vsetvli	t0, a0, e64, mf8, ta, ma
	bnez	t0, fail5
	csrr	t1, VL
	bnez	t1, fail5
	csrr	t1, VTYPE
	li	t2, 1
	slli	t2, t2, 63
	bne	t1, t2, fail5
	li	a0, 9
	.word	0x4c0572d7		# vsetvli t0, a0, e8, m1, ta, ma with zimm[10] (reserved) set
	bnez	t0, fail5
	csrr	t1, VTYPE
	bne	t1, t2, fail5
	bnez	s5, fail5

	#---------------- 6: the ring and the squashes
	li	s1, 60			# iterations
	li	a0, 1
1:	andi	t3, s1, 1
	beqz	t3, 2f			# alternates: mispredicted until the predictor learns it
	vsetvli	t0, a0, e8, m2, ta, ma	# VLMAX 32
	addi	a0, a0, 1
2:	vsetvli	t4, a0, e16, m1, ta, ma	# VLMAX 8
	addi	s1, s1, -1
	bnez	s1, 1b
	# the last iteration (s1 = 1 before the decrement) took the first vsetvli too: a0 = 31
	li	t2, 8
	bne	t4, t2, fail6
	csrr	t1, VL
	bne	t1, t2, fail6
	csrr	t1, VTYPE
	li	t2, 0xc8		# vma vta, e16, m1
	bne	t1, t2, fail6
	li	t2, 30
	bne	t0, t2, fail6		# the last first-vsetvli saw a0 = 30
	bnez	s5, fail6

	#---------------- 7: vsetvl
	li	a1, 0x11		# e32, m2: VLMAX 8
	li	a0, 100
	vsetvl	t0, a0, a1
	li	t2, 8
	bne	t0, t2, fail7
	csrr	t1, VL
	bne	t1, t2, fail7
	csrr	t1, VTYPE
	li	t2, 0x11
	bne	t1, t2, fail7
	li	a1, 0x13		# e32, m8: the keep form keeps vl 8
	vsetvl	x0, x0, a1
	csrr	t1, VL
	li	t2, 8
	bne	t1, t2, fail7
	csrr	t1, VTYPE
	li	t2, 0x13
	bne	t1, t2, fail7
	li	a1, 0x100		# a reserved bit: vill, vl 0
	vsetvl	t0, a0, a1
	bnez	t0, fail7
	csrr	t1, VTYPE
	li	t2, 1
	slli	t2, t2, 63
	bne	t1, t2, fail7
	li	a0, 3
	vsetvli	t0, a0, e8, m1, ta, ma	# the version after vsetvl's
	li	t2, 3
	bne	t0, t2, fail7
	csrr	t1, VL
	bne	t1, t2, fail7
	bnez	s5, fail7

	li	a0, 1
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
	csrr	t3, mepc
	addi	t3, t3, 4
	csrw	mepc, t3
	mret

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
