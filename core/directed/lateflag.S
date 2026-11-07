# lateflag.S -- directed test: a load flagged for replay is never behind the irrevocable pointer.
#
# A store whose address waits on a divide shares a ROB row with a younger load from the same
# address. The load reads memory first; the store's address then arrives and the overlap, which
# the page bits qualify, flags the load a cycle later. The store's data is already in (the store
# queue snooped it from an add in the same row), so the store can commit in that cycle, and the
# ROB's irrevocable pointer must not cross the load then: past it, a younger store is irrevocable
# ahead of the replay that squashes it (smolrv64_rob asserts this). A store whose data is still to
# be read from the register file commits later and never reaches that cycle. Each case is a fresh
# PC, so the dependence predictor never blocks the load, and puts the add, the store and the load
# at different slots of an aligned row.
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	s6, area
	li	s8, 3
	mul	s7, s6, s8		# divu s7/s8 = the area's address, 30+ cycles late

# case n: memory holds 0; an add in the store's row produces the store's data, which the store queue
# snoops long before the divide delivers the address, so the store can commit the cycle after its
# address arrives -- the cycle its overlap flags the load (a, s, l: the add's, the store's and the
# load's slots in the row). The load must return n+0x100 after its replay.
.macro LATE n, a, s, l
	sd	zero, 0(s6)
	li	t4, \n + 0x100
	fence
	divu	t0, s7, s8
	.balign 16
	.rept \a
	nop
	.endr
	addi	t1, t4, 0
	.rept \s - \a - 1
	nop
	.endr
	sd	t1, 0(t0)
	.rept \l - \s - 1
	nop
	.endr
	ld	t2, 0(s6)
	.rept 3 - \l
	nop
	.endr
	sd	t1, 8(s6)		# a younger store, committable as the replay redirects
	li	gp, \n
	bne	t2, t1, fail
	ld	t3, 0(s6)
	bne	t3, t1, fail
.endm

	LATE 1, 0, 1, 2
	LATE 2, 0, 1, 3
	LATE 3, 0, 2, 3
	LATE 4, 1, 2, 3
	LATE 5, 0, 1, 2
	LATE 6, 0, 1, 3
	LATE 7, 0, 2, 3
	LATE 8, 1, 2, 3

	li	a0, 1
	la	t0, tohost
	sd	a0, 0(t0)
1:	j	1b

fail:	slli	a0, gp, 1
	ori	a0, a0, 1
	la	t0, tohost
	sd	a0, 0(t0)
2:	j	2b

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.align 6
	.globl fromhost
fromhost: .dword 0

	.section .data
	.align 6
area:	.dword 0, 0
