# cboperm.S -- directed test for the Zicbom permission checks under Sv39.
#
# cbo.clean, cbo.flush and cbo.inval are permitted wherever a load or a store would be (the CMO
# specification), so a read-only page takes them; only where neither is permitted do they raise
# a store page fault. cbo.zero writes, so a read-only page faults it. Linux cleans the kernel's
# read-only zero page before a virtio-blk device reads it (blkdev_issue_zeroout, ext4's lazy
# inode-table init on an SD-card root): a CBO checked as a store traps there.
#
# Sv39 with three 1 GiB leaves: 0x8000_0000 -> itself, RWX; 0xC000_0000 -> 0x8000_0000, read-only;
# 0x4000_0000 unmapped. The cases run in S-mode; the M-mode handler records each trap and
# returns to the case's resume point, and an ecall ends the run with the verdict in a0.
#
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	la	t0, pt
	li	t1, (0x80000 << 10) | 0xcf	# V R W X A D
	sd	t1, 16(t0)			# VPN[2] = 2: 0x8000_0000
	li	t1, (0x80000 << 10) | 0x43	# V R A: read-only
	sd	t1, 24(t0)			# VPN[2] = 3: 0xC000_0000
	srli	t0, t0, 12
	li	t1, 8 << 60			# Sv39
	or	t0, t0, t1
	csrw	satp, t0
	sfence.vma
	li	t0, 0xf0			# menvcfg: CBIE = invalidate, CBCFE, CBZE
	csrs	menvcfg, t0
	li	t0, 3 << 11
	csrc	mstatus, t0
	li	t0, 1 << 11			# MPP = S
	csrs	mstatus, t0
	la	t0, s_start
	csrw	mepc, t0
	mret

s_start:
	la	s6, area			# read-write
	li	t0, 0x40000000
	add	s7, s6, t0			# the same line, read-only
	li	s8, 0x40000000			# unmapped

	#---------------- 1: cbo.clean on a read-only page must NOT trap
	li	s5, 0
	la	s2, 1f
	.insn	i 0x0f, 2, x0, s7, 1		# cbo.clean (s7)
1:	bnez	s5, fail1

	#---------------- 2: cbo.flush on a read-only page must NOT trap
	li	s5, 0
	la	s2, 2f
	.insn	i 0x0f, 2, x0, s7, 2		# cbo.flush (s7)
2:	bnez	s5, fail2

	#---------------- 3: cbo.inval on a read-only page must NOT trap
	li	s5, 0
	la	s2, 3f
	.insn	i 0x0f, 2, x0, s7, 0		# cbo.inval (s7)
3:	bnez	s5, fail3

	#---------------- 4: cbo.zero on a read-only page must trap, store page fault
	li	s5, 0
	la	s2, 4f
	.insn	i 0x0f, 2, x0, s7, 4		# cbo.zero (s7)
4:	li	t1, 1
	bne	s5, t1, fail4
	li	t1, 15
	bne	s3, t1, fail4
	bne	s4, s7, fail4

	#---------------- 5: cbo.clean on an unmapped page must trap, store page fault
	li	s5, 0
	la	s2, 5f
	.insn	i 0x0f, 2, x0, s8, 1		# cbo.clean (s8)
5:	li	t1, 1
	bne	s5, t1, fail5
	li	t1, 15
	bne	s3, t1, fail5
	bne	s4, s8, fail5

	#---------------- 6: cbo.clean on a read-write page must NOT trap
	li	s5, 0
	la	s2, 6f
	.insn	i 0x0f, 2, x0, s6, 1		# cbo.clean (s6)
6:	bnez	s5, fail6

	li	a0, 1				# PASS
	ecall
fail1:	li	a0, 3
	ecall
fail2:	li	a0, 5
	ecall
fail3:	li	a0, 7
	ecall
fail4:	li	a0, 9
	ecall
fail5:	li	a0, 11
	ecall
fail6:	li	a0, 13
	ecall

	# M-mode: an ecall from S ends the run; any other trap is recorded and the case resumes
	# at s2, back in S-mode (MPP is S after a trap from S).
	.align 2
trap_handler:
	csrr	s3, mcause
	csrr	s4, mtval
	li	t6, 9				# environment call from S
	beq	s3, t6, done
	addi	s5, s5, 1
	csrw	mepc, s2
	mret
done:
	la	t0, tohost
	sd	a0, 0(t0)
9:	j	9b

	.section .data
	.align 12
pt:	.zero 4096
	.align 6
	.globl area
area:	.zero 64

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
