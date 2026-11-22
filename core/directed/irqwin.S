# irqwin.S -- an interrupt pending when its enable rises is taken, however short the enabled window.
#
# A kernel's idle loop without a waiting WFI is `csrsi sstatus,SIE; a few instructions; csrci
# sstatus,SIE` forever, perfectly predicted. The interrupt pseudo-op is injected at fetch, behind
# everything already fetched; if that includes the loop's next csrci, the pseudo-op reaches the
# head with the enable off and does nothing, every iteration, and the machine is deaf.
# riscv-tests and a Linux boot only enable interrupts in front of long, mispredicting code.
#
# The privileged spec has the trap conditions evaluated immediately after an explicit write to
# mstatus, so mepc is the instruction behind the first csrsi.
# Counter 3 wraps with MIE off, so LCOFI is pending before each loop starts.
#   1  window of 0 instructions (csrsi; csrci)
#   2  window of 4
#   3  window of 16
#   4  window of 40
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

#define EV_CYC    1
#define LCOF      (1 << 13)

	.macro	pend
	csrci	mstatus, 8
	li	t0, LCOF
	csrc	mip, t0
	csrw	mhpmevent3, zero
	li	t0, EV_CYC
	csrw	mhpmevent3, t0
	li	t0, -16
	csrw	mhpmcounter3, t0
	li	t1, 64
9:	addi	t1, t1, -1
	bnez	t1, 9b
	csrr	t0, mip			# pending now, and not taken: MIE is off
	li	t1, LCOF
	and	t0, t0, t1
	.endm

	.macro	window n
	li	s4, 4000
	la	s7, 7f			# where the interrupt must be taken: right behind the first csrsi
8:	csrsi	mstatus, 8
7:	.rept	\n
	addi	s6, s6, 1
	.endr
	csrci	mstatus, 8
	.rept	60			# a long stretch with the enable off, as an idle loop has
	addi	s8, s8, 1
	.endr
	addi	s4, s4, -1
	bnez	s4, 8b
	.endm

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, m_handler
	csrw	mtvec, t0
	csrw	mideleg, zero
	csrw	medeleg, zero
	csrw	mcountinhibit, zero
	li	t0, LCOF
	csrw	mie, t0

	pend
	beqz	t0, fail1
	la	s2, 1f
	window	0
	j	fail1
1:
	pend
	beqz	t0, fail2
	la	s2, 2f
	window	4
	j	fail2
2:
	pend
	beqz	t0, fail3
	la	s2, 3f
	window	16
	j	fail3
3:
	pend
	beqz	t0, fail4
	la	s2, 4f
	window	40
	j	fail4
4:
	li	a0, 1
	j	done
fail1:	li	a0, 3
	j	done
fail2:	li	a0, 5
	j	done
fail3:	li	a0, 7
	j	done
fail4:	li	a0, 9
done:	la	t0, tohost
	sd	a0, 0(t0)
0:	j	0b

	.align	2
m_handler:
	csrr	t0, mcause
	bgez	t0, fail1		# an exception: not what this test raises
	li	t0, LCOF
	csrc	mip, t0
	csrr	t0, mepc
	bne	t0, s7, late
	csrw	mepc, s2
	mret
late:	li	a0, 4000		# FAIL: test# = 1000 + the iterations it went untaken
	sub	a0, a0, s4
	addi	a0, a0, 1000
	slli	a0, a0, 1
	ori	a0, a0, 1
	j	done

	.section .tohost,"aw",@progbits
	.align	6
	.globl	tohost
tohost:	.dword	0
