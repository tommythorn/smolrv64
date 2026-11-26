# viop.S -- directed test: the vector unit's integer operations (vadd, vsub, vrsub, vand, vor, vxor,
# vminu, vmin, vmaxu, vmax, vsll, vsrl, vsra; .vv, .vx, .vi; unmasked) at VLEN 128. Each result is
# stored whole (vs1r.v) and both doublewords checked; the lockstep compares every register write.
#
#   1  vadd.vv and vadd.vx at e8: carries stay inside an element
#   2  vsub.vx at e16, vrsub.vi at e32: borrows stay inside an element
#   3  vxor.vi -1, vand.vx and vor.vi at e64: simm5 sign-extends
#   4  vmin.vx, vmax.vx, vmaxu.vx and vminu.vx at e8: signed against unsigned
#   5  vsll.vi 29 at e64 (uimm5 zero-extends), vsrl.vx 17 at e16 (the amount mod SEW), vsra.vv at e32
#   6  a group (e8, m2, vl 20): the second register's elements 0..3 written, the rest kept
#   7  vstart 3: the prestart elements keep the old destination's
#   8  in place over a group: vadd.vv v10, v10, v10 at e8, m2, vl 32
#   9  vill: an operation is illegal
#  10  a dependent chain in a loop behind an alternating branch (wakes, squashes)
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
	li	t0, 1 << 9		# VS Initial
	csrw	mstatus, t0
	# A: bytes 1..16, into v1
	la	a2, srca
	li	t0, 0x0807060504030201
	sd	t0, 0(a2)
	li	t0, 0x100f0e0d0c0b0a09
	sd	t0, 8(a2)
	vl1re8.v v1, (a2)
	la	a3, dst

	#---------------- 1: vadd at e8
	vsetivli t0, 16, e8, m1, ta, ma
	vadd.vv	v2, v1, v1
	li	a0, 0x100e0c0a08060402
	li	a1, 0x201e1c1a18161412
	vmv1r.v	v31, v2
	call	chk
	bnez	a0, fail1
	li	t1, -1
	vadd.vx	v2, v1, t1
	li	a0, 0x0706050403020100
	li	a1, 0x0f0e0d0c0b0a0908
	vmv1r.v	v31, v2
	call	chk
	bnez	a0, fail1

	#---------------- 2: vsub.vx at e16, vrsub.vi at e32
	vsetivli t0, 8, e16, m1, ta, ma
	li	t1, 0x0101
	vsub.vx	v3, v1, t1
	li	a0, 0x0706050403020100
	li	a1, 0x0f0e0d0c0b0a0908
	vmv1r.v	v31, v3
	call	chk
	bnez	a0, fail2
	vsetivli t0, 4, e32, m1, ta, ma
	vrsub.vi v3, v1, 5
	li	a0, 0xf7f8fa00fbfcfe04
	li	a1, 0xeff0f1f8f3f4f5fc
	vmv1r.v	v31, v3
	call	chk
	bnez	a0, fail2

	#---------------- 3: logic at e64
	vsetivli t0, 2, e64, m1, ta, ma
	vxor.vi	v4, v1, -1
	li	a0, 0xf7f8f9fafbfcfdfe
	li	a1, 0xeff0f1f2f3f4f5f6
	vmv1r.v	v31, v4
	call	chk
	bnez	a0, fail3
	li	t1, 0x00ff00ff00ff00ff
	vand.vx	v5, v1, t1
	li	a0, 0x0007000500030001
	li	a1, 0x000f000d000b0009
	vmv1r.v	v31, v5
	call	chk
	bnez	a0, fail3
	vor.vi	v5, v1, -16
	li	a0, 0xfffffffffffffff1
	li	a1, 0xfffffffffffffff9
	vmv1r.v	v31, v5
	call	chk
	bnez	a0, fail3

	#---------------- 4: min and max at e8
	vsetivli t0, 16, e8, m1, ta, ma
	li	t1, 5
	vmin.vx	v6, v1, t1
	li	a0, 0x0505050504030201
	li	a1, 0x0505050505050505
	vmv1r.v	v31, v6
	call	chk
	bnez	a0, fail4
	li	t1, 0x80
	vmax.vx	v6, v1, t1		# -128: A
	li	a0, 0x0807060504030201
	li	a1, 0x100f0e0d0c0b0a09
	vmv1r.v	v31, v6
	call	chk
	bnez	a0, fail4
	vmaxu.vx v6, v1, t1		# 128: all 0x80
	li	a0, 0x8080808080808080
	mv	a1, a0
	vmv1r.v	v31, v6
	call	chk
	bnez	a0, fail4
	vminu.vv v6, v6, v1		# A
	li	a0, 0x0807060504030201
	li	a1, 0x100f0e0d0c0b0a09
	vmv1r.v	v31, v6
	call	chk
	bnez	a0, fail4

	#---------------- 5: shifts
	vsetivli t0, 2, e64, m1, ta, ma
	vsll.vi	v7, v1, 29
	li	a0, 0xa080604020000000
	li	a1, 0xa181614120000000
	vmv1r.v	v31, v7
	call	chk
	bnez	a0, fail5
	vsetivli t0, 8, e16, m1, ta, ma
	li	t1, 17
	vsrl.vx	v7, v1, t1
	li	a0, 0x0403030202010100
	li	a1, 0x0807070606050504
	vmv1r.v	v31, v7
	call	chk
	bnez	a0, fail5
	vsetivli t0, 4, e32, m1, ta, ma
	vmv.v.i	v8, 4
	vsra.vv	v7, v4, v8
	li	a0, 0xff7f8f9fffbfcfdf
	li	a1, 0xfeff0f1fff3f4f5f
	vmv1r.v	v31, v7
	call	chk
	bnez	a0, fail5

	#---------------- 6: a group
	li	a0, 32
	vsetvli	t0, a0, e8, m2, ta, ma
	vmv.v.i	v10, -1
	vmv1r.v	v12, v1
	vmv1r.v	v13, v1
	li	a0, 20
	vsetvli	t0, a0, e8, m2, ta, ma
	vadd.vi	v10, v12, 1
	li	a0, 0x0908070605040302
	li	a1, 0x11100f0e0d0c0b0a
	vmv1r.v	v31, v10
	call	chk
	bnez	a0, fail6
	li	a0, 0xffffffff05040302
	li	a1, -1
	vmv1r.v	v31, v11
	call	chk
	bnez	a0, fail6

	#---------------- 7: vstart
	vsetivli t0, 16, e8, m1, ta, ma
	vmv.v.i	v14, 7
	vadd.vi	v14, v14, 0		# 0x07 everywhere (no vstart)
	li	t0, 3
	csrw	VSTART, t0
	vadd.vi	v14, v1, 0		# elements 3..15 from A
	csrr	t1, VSTART
	bnez	t1, fail7
	li	a0, 0x0807060504070707
	li	a1, 0x100f0e0d0c0b0a09
	vmv1r.v	v31, v14
	call	chk
	bnez	a0, fail7
	bnez	s5, fail7

	#---------------- 8: in place over a group (v10, v11 from test 6)
	li	a0, 32
	vsetvli	t0, a0, e8, m2, ta, ma
	vadd.vv	v10, v10, v10
	li	a0, 0x12100e0c0a080604
	li	a1, 0x22201e1c1a181614
	vmv1r.v	v31, v10
	call	chk
	bnez	a0, fail8
	li	a0, 0xfefefefe0a080604
	li	a1, 0xfefefefefefefefe
	vmv1r.v	v31, v11
	call	chk
	bnez	a0, fail8

	#---------------- 9: vill
	li	a1, 0x100
	li	a0, 4
	vsetvl	t0, a0, a1
	vadd.vv	v2, v1, v1
	li	t2, 1
	bne	s5, t2, fail9
	li	s5, 0

	#---------------- 10: a chain in a loop
	vsetivli t0, 2, e64, m1, ta, ma
	vmv.v.i	v16, 0
	li	s1, 40
1:	andi	t3, s1, 1
	beqz	t3, 2f			# alternates
	vadd.vx	v16, v16, s1
	vxor.vi	v17, v16, 0
	vor.vv	v16, v17, v17
2:	addi	s1, s1, -1
	bnez	s1, 1b
	vmv.x.s	t1, v16
	li	t2, 400			# 39 + 37 + ... + 1
	bne	t1, t2, fail10
	bnez	s5, fail10

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
done:
	la	t0, tohost
	sd	a0, 0(t0)
99:	j	99b

	# a0 = 0 when v31 holds {a1, a0}
chk:	vs1r.v	v31, (a3)
	ld	t4, 0(a3)
	ld	t5, 8(a3)
	xor	t4, t4, a0
	xor	t5, t5, a1
	or	a0, t4, t5
	ret

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

	.section .data
	.align 4
srca:	.zero 16
dst:	.zero 16
