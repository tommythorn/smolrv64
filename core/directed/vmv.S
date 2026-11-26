# vmv.S -- directed test: the vector unit's moves (vmv.v.v/x/i, vmv<nr>r.v, vmv.s.x, vmv.x.s) at
# VLEN 128. vmv.x.s at e64 reads a register's low 8 bytes, so a register's elements 0..7 at e8 are
# checked here; the lockstep compares every register a uop writes, all 16 bytes.
#
#   1  VS Off: a move is an illegal instruction
#   2  vmv.v.x and vmv.x.s at e64
#   3  the tail is undisturbed: vl 5 at e8 over a register of 0xff
#   4  a group (e8, m2, vl 20): the second register's elements 0..3 written, the rest kept
#   5  vmv.v.v at e16, vl 3
#   6  vmv2r.v copies a group whatever vl is, vmv1r.v at vl 0 too
#   7  vmv.s.x writes element 0 alone, and nothing at vl 0
#   8  vmv.x.s sign-extends at e8, e16 and e32
#   9  vstart: the first move after a write of vstart keeps the prestart elements, the next does
#      not, and vstart reads 0 after them
#  10  vill: a move is illegal
#  11  a dependent chain in a loop behind an alternating branch (wakes, squashes)
#
# A trap with mcause 2 is counted in s5 and returns past the instruction.
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.option	arch, +v
#define VSTART 0x008

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	li	s5, 0
	csrw	mstatus, x0

	#---------------- 1: VS Off
	.word	0x5e003057		# vmv.v.i v0, 0 (the assembler wants V on)
	li	t2, 1
	bne	s5, t2, fail1
	li	t0, 1 << 9		# VS Initial
	csrw	mstatus, t0
	li	s5, 0

	#---------------- 2: vmv.v.x, vmv.x.s at e64
	li	a0, 2
	vsetvli	t0, a0, e64, m1, ta, ma
	li	a1, 0x1122334455667788
	vmv.v.x	v1, a1
	vmv.x.s	t1, v1
	bne	t1, a1, fail2

	#---------------- 3: the tail
	li	a0, 16
	vsetvli	t0, a0, e8, m1, tu, mu
	vmv.v.i	v1, -1
	li	a0, 5
	vsetvli	t0, a0, e8, m1, ta, ma
	li	a1, 0x5a
	vmv.v.x	v1, a1
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v1
	li	t2, 0xffffff5a5a5a5a5a
	bne	t1, t2, fail3

	#---------------- 4: a group
	li	a0, 32
	vsetvli	t0, a0, e8, m2, ta, ma
	vmv.v.i	v2, -1
	li	a0, 20
	vsetvli	t0, a0, e8, m2, ta, ma
	li	a1, 0x33
	vmv.v.x	v2, a1
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v3
	li	t2, 0xffffffff33333333
	bne	t1, t2, fail4
	vmv.x.s	t1, v2
	li	t2, 0x3333333333333333
	bne	t1, t2, fail4

	#---------------- 5: vmv.v.v
	vsetivli t0, 4, e16, m1, ta, ma
	li	a1, 0x1234
	vmv.v.x	v4, a1
	vsetivli t0, 8, e16, m1, ta, ma
	vmv.v.i	v5, 0
	vsetivli t0, 3, e16, m1, ta, ma
	vmv.v.v	v5, v4
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v5
	li	t2, 0x0000123412341234
	bne	t1, t2, fail5

	#---------------- 6: whole-register moves
	vsetivli t0, 1, e8, m1, ta, ma
	vmv2r.v	v6, v2
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v7
	li	t2, 0xffffffff33333333
	bne	t1, t2, fail6
	vsetivli t0, 0, e8, m1, ta, ma	# vl 0
	vmv1r.v	v8, v5
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v8
	li	t2, 0x0000123412341234
	bne	t1, t2, fail6

	#---------------- 7: vmv.s.x
	vsetivli t0, 4, e32, m1, ta, ma
	vmv.v.i	v9, 7
	li	a1, 0xabcdef01
	vmv.s.x	v9, a1
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v9
	li	t2, 0x00000007abcdef01
	bne	t1, t2, fail7
	vsetivli t0, 0, e32, m1, ta, ma	# vl 0: no write
	li	a1, 0x55
	vmv.s.x	v9, a1
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v9
	bne	t1, t2, fail7

	#---------------- 8: vmv.x.s sign-extends
	vsetivli t0, 1, e8, m1, ta, ma
	li	a1, 0x80
	vmv.s.x	v10, a1
	vmv.x.s	t1, v10
	li	t2, -128
	bne	t1, t2, fail8
	vsetivli t0, 1, e16, m1, ta, ma
	li	a1, 0x8001
	vmv.s.x	v10, a1
	vmv.x.s	t1, v10
	li	t2, -32767
	bne	t1, t2, fail8
	vsetivli t0, 1, e32, m1, ta, ma
	li	a1, 0x80000002
	vmv.s.x	v10, a1
	vmv.x.s	t1, v10
	li	t2, 0xffffffff80000002
	bne	t1, t2, fail8

	#---------------- 9: vstart
	vsetivli t0, 16, e8, m1, ta, ma
	vmv.v.i	v11, 0
	vmv.v.i	v12, 0
	vsetivli t0, 8, e8, m1, ta, ma
	li	t0, 2
	csrw	VSTART, t0
	li	a1, 0x11
	vmv.v.x	v11, a1			# elements 2..7
	li	a1, 0x22
	vmv.v.x	v12, a1			# elements 0..7: vstart is 0 again
	csrr	t1, VSTART
	bnez	t1, fail9
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v11
	li	t2, 0x1111111111110000
	bne	t1, t2, fail9
	vmv.x.s	t1, v12
	li	t2, 0x2222222222222222
	bne	t1, t2, fail9
	bnez	s5, fail9

	#---------------- 10: vill
	li	a1, 0x100		# a reserved vtype bit
	li	a0, 4
	vsetvl	t0, a0, a1
	vmv.v.i	v13, 1
	vmv.x.s	t1, v1
	li	t2, 2
	bne	s5, t2, fail10
	li	s5, 0

	#---------------- 11: a chain in a loop
	vsetivli t0, 2, e64, m1, ta, ma
	li	s1, 40
	li	s2, 0
1:	vmv.v.x	v14, s1
	andi	t3, s1, 1
	beqz	t3, 2f			# alternates
	vmv.v.v	v15, v14
	vmv1r.v	v16, v15
	vmv.x.s	t1, v16
	add	s2, s2, t1
2:	addi	s1, s1, -1
	bnez	s1, 1b
	li	t2, 400			# 39 + 37 + ... + 1
	bne	s2, t2, fail11
	bnez	s5, fail11

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
	j	done
fail8:	li	a0, 17
	j	done
fail9:	li	a0, 19
	j	done
fail10:	li	a0, 21
	j	done
fail11:	li	a0, 23
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
