# irqwarm.S -- an interrupt that becomes pending in the middle of a warm, perfectly predicted
# csrsi/csrci loop is taken (irqwin.S has it pending before the loop, where the first iterations'
# mispredicts empty the frontend and let it through).
#
# The enabled window is 0, 4, 16 and 40 instructions, each loop followed by 60 with the enable off.
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1: the loop with that window ran out untaken
#define EV_CYC    1
#define LCOF      (1 << 13)
	.macro	window n, tail=60
	csrci	mstatus, 8
	li	t0, LCOF
	csrc	mip, t0
	csrw	mhpmevent3, zero
	li	t0, EV_CYC
	csrw	mhpmevent3, t0
	li	t0, -6000
	csrw	mhpmcounter3, t0
	li	s4, 3000
8:	csrsi	mstatus, 8
	.rept	\n
	addi	s6, s6, 1
	.endr
	csrci	mstatus, 8
	.rept	\tail
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
	la	s2, 1f
	li	a0, 3
	window	0
	j	done
1:	la	s2, 2f
	li	a0, 5
	window	4
	j	done
2:	la	s2, 3f
	li	a0, 7
	window	16
	j	done
3:	la	s2, 4f
	li	a0, 9
	window	40
	j	done
4:	li	a0, 1
done:	la	t0, tohost
	sd	a0, 0(t0)
0:	j	0b
	.align	2
m_handler:
	li	t0, LCOF
	csrc	mip, t0
	csrw	mepc, s2
	mret
	.section .tohost,"aw",@progbits
	.align	6
	.globl	tohost
tohost:	.dword	0
