# sscofpmf.S -- directed test for Sscofpmf: counter overflow, LCOFI, mode filtering, scountovf.
#
# riscv-tests has no Sscofpmf test, and a Linux boot never lets a counter wrap, so neither
# reaches the overflow path that `perf record` sampling stands on. Counter 3 counts cycles
# (event 0x0001) in every case.
#
#   1  a wrap sets mhpmevent3.OF and mip.LCOFIP and takes LCOFI (mcause 13) in M
#   2  scountovf in M shows OF; with OF set a second wrap raises nothing
#   3  MINH stops the counter in M; clearing it lets it run
#   4  STI outranks LCOFI when both are pending
#   5  scountovf in S reads 0 unless mcounteren grants counter 3
#   6  LCOFI delegated by mideleg traps to S (scause 13), the S views carrying bit 13
#   7  SINH stops the counter in S only
#   8  scountovf from U is an illegal instruction
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

#define SCOUNTOVF 0xda0
#define EV_CYC    1
#define LCOF      (1 << 13)

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, m_handler
	csrw	mtvec, t0		# direct mode
	la	t0, s_handler
	csrw	stvec, t0
	csrw	mideleg, zero
	csrw	medeleg, zero
	csrw	mcountinhibit, zero
	li	s3, 0			# captured cause
	li	s5, 0			# trap count

	#---------------- 1: a wrap takes LCOFI in M
	li	t0, EV_CYC
	csrw	mhpmevent3, t0
	li	t0, -64
	csrw	mhpmcounter3, t0
	li	t0, LCOF
	csrw	mie, t0
	li	s5, 0
	la	s2, 1f
	csrsi	mstatus, 8		# MIE
	li	t1, 2000
0:	addi	t1, t1, -1
	bnez	t1, 0b
	j	fail1			# no interrupt
1:	li	t1, 1
	bne	s5, t1, fail1
	li	t1, -1
	srli	t1, t1, 1
	not	t1, t1
	ori	t1, t1, 13		# 0x8000...000d
	bne	s3, t1, fail1
	csrr	t0, mhpmevent3
	bgez	t0, fail1		# OF (bit 63) set
	csrr	t0, mip
	li	t1, LCOF
	and	t0, t0, t1
	beqz	t0, fail1

	#---------------- 2: scountovf in M; OF set silences the next wrap
	csrr	t0, SCOUNTOVF
	li	t1, 8
	bne	t0, t1, fail2
	li	t1, LCOF
	csrc	mip, t1
	li	t0, -64
	csrw	mhpmcounter3, t0
	li	s5, 0
	la	s2, fail2		# any interrupt now is a failure
	csrsi	mstatus, 8
	li	t1, 2000
0:	addi	t1, t1, -1
	bnez	t1, 0b
	csrci	mstatus, 8
	csrr	t0, mhpmcounter3
	bltz	t0, fail2		# it did wrap
	csrr	t0, mip
	li	t1, LCOF
	and	t0, t0, t1
	bnez	t0, fail2

	#---------------- 3: MINH stops the counter in M
	li	t0, (1 << 62) | EV_CYC
	csrw	mhpmevent3, t0
	csrw	mhpmcounter3, zero
	li	t1, 500
0:	addi	t1, t1, -1
	bnez	t1, 0b
	csrr	t0, mhpmcounter3
	bnez	t0, fail3
	li	t0, EV_CYC
	csrw	mhpmevent3, t0
	li	t1, 500
0:	addi	t1, t1, -1
	bnez	t1, 0b
	csrr	t0, mhpmcounter3
	beqz	t0, fail3

	#---------------- 4: STI outranks LCOFI
	li	t0, LCOF | (1 << 5)
	csrw	mie, t0
	csrw	mip, t0
	li	s5, 0
	la	s2, 4f
	csrsi	mstatus, 8
	j	fail4
4:	li	t1, -1
	srli	t1, t1, 1
	not	t1, t1
	ori	t1, t1, 5
	bne	s3, t1, fail4
	csrw	mip, zero
	csrw	mie, zero

	#---------------- 5: scountovf in S, masked by mcounteren
	li	t0, (1 << 63) | EV_CYC	# OF set by software
	csrw	mhpmevent3, t0
	csrw	mcounteren, zero
	li	a2, -1
	la	s2, 5f
	la	t0, 50f
	jal	ra, to_s
50:	csrr	a2, SCOUNTOVF		# S
	ecall
5:	li	t1, 9			# ecall from S
	bne	s3, t1, fail5
	bnez	a2, fail5
	li	t0, 8
	csrw	mcounteren, t0
	li	a2, -1
	la	s2, 51f
	la	t0, 52f
	jal	ra, to_s
52:	csrr	a2, SCOUNTOVF		# S
	ecall
51:	li	t1, 8
	bne	a2, t1, fail5

	#---------------- 6: LCOFI delegated to S
	li	t0, EV_CYC		# OF clear
	csrw	mhpmevent3, t0
	li	t0, -256
	csrw	mhpmcounter3, t0
	li	t0, LCOF
	csrw	mideleg, t0
	csrw	mie, zero
	li	s5, 0
	la	s2, 6f
	la	t0, 60f
	jal	ra, to_s
60:	li	t0, LCOF		# S
	csrs	sie, t0
	csrr	t1, sie
	bne	t1, t0, 61f
	csrsi	sstatus, 2		# SIE
	li	t1, 4000
0:	addi	t1, t1, -1
	bnez	t1, 0b
61:	ecall				# no interrupt, or sie lost bit 13: lands in M, cause 9
6:	li	t1, 1
	bne	s5, t1, fail6
	li	t1, -1
	srli	t1, t1, 1
	not	t1, t1
	ori	t1, t1, 13
	bne	s3, t1, fail6
	csrr	t0, sip			# still S: the S view shows LCOFIP
	li	t1, LCOF
	and	t0, t0, t1
	beqz	t0, fail6
	la	s2, 62f
	ecall
62:	li	t1, 9
	bne	s3, t1, fail6
	csrw	mideleg, zero
	csrw	mie, zero
	csrw	mip, zero

	#---------------- 7: SINH stops the counter in S only
	li	t0, (1 << 61) | EV_CYC
	csrw	mhpmevent3, t0
	csrw	mhpmcounter3, zero
	la	s2, 7f
	la	t0, 70f
	jal	ra, to_s
70:	li	t1, 4000		# S
0:	addi	t1, t1, -1
	bnez	t1, 0b
	ecall
7:	csrr	t0, mhpmcounter3
	li	t1, 1000
	bgeu	t0, t1, fail7		# counted the S loop
	beqz	t0, fail7		# counted nothing in M either

	#---------------- 8: scountovf from U is illegal
	li	s3, 0
	la	s2, 8f
	la	t0, 80f
	csrw	mepc, t0
	li	t0, 3 << 11
	csrc	mstatus, t0		# MPP = U
	mret
80:	csrr	a2, SCOUNTOVF		# U
	j	fail8
8:	li	t1, 2
	bne	s3, t1, fail8

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
	j	done
fail7:	li	a0, 15
	j	done
fail8:	li	a0, 17
done:
	la	t0, tohost
	sd	a0, 0(t0)
9:	j	9b

	# mret to S at t0.
to_s:	csrw	mepc, t0
	li	t0, 3 << 11
	csrc	mstatus, t0
	li	t0, 1 << 11
	csrs	mstatus, t0		# MPP = S
	mret

	# Both handlers record the cause and resume at s2 in the mode they trapped to: each case
	# wants the trap observable and nothing retried.
	.align 2
m_handler:
	csrr	s3, mcause
	addi	s5, s5, 1
	jr	s2
	.align 2
s_handler:
	csrr	s3, scause
	addi	s5, s5, 1
	jr	s2

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
