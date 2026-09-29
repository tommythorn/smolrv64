# amomis.S -- directed test for AMO/LR natural-alignment trapping.
#
# riscv-tests never issues a misaligned atomic, so rv64ua passing says nothing about
# this path. The in-order LSU aligns the AMO address down to the containing 8-byte
# word (pa_q <= t_paddr & ~56'd7) and builds a_wmask relative to that word, so before
# the check an unaligned AMO silently read-modify-wrote its neighbours.
#
# Positive cases 1-3 require the trap; negative cases 4-5 require that ordinary
# aligned atomics still work, which is what guards against an over-eager check.
#
# M-mode, no MMU -- the alignment test sits ahead of translation, so bare mode is
# the right place to exercise it.
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0		# direct mode
	li	s3, 0			# captured mcause
	li	s4, 0			# captured mtval
	li	s5, 0			# trap count
	la	s6, area		# 8-byte-aligned base

	#---------------- 1: amoadd.d at +4 must trap, cause 6, mtval = vaddr
	addi	a0, s6, 4
	li	a1, 1
	li	s5, 0
	la	s2, 1f
	amoadd.d a2, a1, (a0)
	li	s5, 99			# reached only if it did NOT trap
1:	li	t1, 1
	bne	s5, t1, fail1
	li	t1, 6			# store/AMO address misaligned
	bne	s3, t1, fail1
	bne	s4, a0, fail1

	#---------------- 2: lr.d at +4 must trap, cause 4 (LR is load-class)
	addi	a0, s6, 4
	li	s5, 0
	la	s2, 2f
	lr.d	a2, (a0)
	li	s5, 99
2:	li	t1, 1
	bne	s5, t1, fail2
	li	t1, 4			# load address misaligned
	bne	s3, t1, fail2

	#---------------- 3: amoadd.w at +2 must trap (word needs 4-byte alignment)
	addi	a0, s6, 2
	li	a1, 1
	li	s5, 0
	la	s2, 3f
	amoadd.w a2, a1, (a0)
	li	s5, 99
3:	li	t1, 1
	bne	s5, t1, fail3
	li	t1, 6
	bne	s3, t1, fail3

	#---------------- 4: aligned amoadd.d must NOT trap, and must actually work
	li	t2, 0x100
	sd	t2, 0(s6)
	li	s5, 0
	la	s2, 4f			# a trap would land here with s5=1
	li	a1, 5
	mv	a0, s6
	amoadd.d a2, a1, (a0)
4:	bnez	s5, fail4		# trapped -> over-eager check
	li	t1, 0x100
	bne	a2, t1, fail4		# rd = old memory value
	ld	t3, 0(s6)
	li	t1, 0x105
	bne	t3, t1, fail4		# memory updated

	#---------------- 5: aligned lr.d/sc.d round trip still succeeds
	li	s5, 0
	la	s2, 5f
	lr.d	a2, (s6)
	sc.d	a3, a2, (s6)
5:	bnez	s5, fail5
	bnez	a3, fail5		# sc.d must report success

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
done:
	la	t0, tohost
	sd	a0, 0(t0)
9:	j	9b

	# Records the cause and resumes at s2 rather than mret-ing: the faulting
	# instruction must not be retried, and each case wants the trap observable.
	.align 2
trap_handler:
	csrr	s3, mcause
	csrr	s4, mtval
	addi	s5, s5, 1
	jr	s2

	.section .data
	.align 3
	.globl area
area:	.dword 0x1111111111111111
	.dword 0x2222222222222222
	.dword 0x3333333333333333
	.dword 0x4444444444444444

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
