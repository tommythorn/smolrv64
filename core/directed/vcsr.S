# vcsr.S -- directed test: mstatus.VS and the vector CSRs (vstart, vxsat, vxrm, vcsr, vl, vtype,
# vlenb).
#
#   1  VS resets Off: a vector CSR access is an illegal instruction
#   2  VS = Initial: vlenb 16, vl 0, vtype vill; SD clear
#   3  vxrm, vxsat and vcsr are one state; a write makes VS (and SD) Dirty, in sstatus too
#   4  vstart is WARL over 7 bits (VLEN 128: elements 0..127)
#   5  vl, vtype and vlenb are read-only: a write is illegal
#   6  VS = Off again: the vector CSRs are illegal again
#
# mstatus is written before it is read: its other fields' reset values are the implementation's.
# The CSRs go by number, so the test assembles without V. A trap with mcause 2 is counted in s5
# (s3 = mcause) and returns past the instruction.
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

#define VSTART 0x008
#define VXSAT  0x009
#define VXRM   0x00a
#define VCSR   0x00f
#define VL     0xc20
#define VTYPE  0xc21
#define VLENB  0xc22

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	li	s5, 0

	#---------------- 1: VS Off at reset
	csrr	t1, VLENB
	csrw	VXRM, x0
	li	t2, 2
	bne	s5, t2, fail1
	li	t2, 2
	bne	s3, t2, fail1

	#---------------- 2: VS Initial
	li	s5, 0
	li	t0, 1 << 9
	csrw	mstatus, t0
	csrr	t1, mstatus
	bltz	t1, fail2		# SD clear
	srli	t1, t1, 9
	andi	t1, t1, 3
	li	t2, 1
	bne	t1, t2, fail2
	csrr	t1, VLENB
	li	t2, 16
	bne	t1, t2, fail2
	csrr	t1, VL
	bnez	t1, fail2
	csrr	t1, VTYPE
	li	t2, 1
	slli	t2, t2, 63
	bne	t1, t2, fail2
	bnez	s5, fail2

	#---------------- 3: vxrm, vxsat, vcsr
	csrwi	VXRM, 3
	csrr	t1, VXRM
	li	t2, 3
	bne	t1, t2, fail3
	csrr	t1, VCSR
	li	t2, 6
	bne	t1, t2, fail3
	csrwi	VXSAT, 1
	csrr	t1, VCSR
	li	t2, 7
	bne	t1, t2, fail3
	csrwi	VCSR, 2			# vxrm 1, vxsat 0
	csrr	t1, VXRM
	li	t2, 1
	bne	t1, t2, fail3
	csrr	t1, VXSAT
	bnez	t1, fail3
	csrr	t1, mstatus
	bgez	t1, fail3		# SD set
	srli	t1, t1, 9
	andi	t1, t1, 3
	li	t2, 3
	bne	t1, t2, fail3		# VS Dirty
	csrr	t1, sstatus
	bgez	t1, fail3
	srli	t1, t1, 9
	andi	t1, t1, 3
	bne	t1, t2, fail3
	bnez	s5, fail3

	#---------------- 4: vstart
	li	t0, 200
	csrw	VSTART, t0
	csrr	t1, VSTART
	li	t2, 200 & 127
	bne	t1, t2, fail4
	csrw	VSTART, x0
	csrr	t1, VSTART
	bnez	t1, fail4
	bnez	s5, fail4

	#---------------- 5: read-only
	li	t0, 5
	csrw	VL, t0
	csrw	VTYPE, t0
	csrw	VLENB, t0
	li	t2, 3
	bne	s5, t2, fail5
	csrr	t1, VLENB
	li	t2, 16
	bne	t1, t2, fail5

	#---------------- 6: VS Off again
	li	s5, 0
	li	t0, 3 << 9
	csrc	mstatus, t0
	csrr	t1, VSTART
	csrr	t1, VCSR
	li	t2, 2
	bne	s5, t2, fail6

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
done:
	la	t0, tohost
	sd	a0, 0(t0)
99:	j	99b

	# Count the trap, return past the (4-byte) instruction.
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
