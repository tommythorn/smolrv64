# supm.S -- directed test: pointer masking for U-mode (Ssnpm's senvcfg.PMM = PMLEN 7, Supm).
#
#   1  senvcfg.PMM is WARL: 10 (PMLEN 7) reads back, 01 and 11 read back as 00
#   2  U-mode, Bare: a pointer with junk in bits 63:57 loads and stores (zero-extended); with PMM
#      off the same pointer is an access fault
#   3  M-mode with MPRV and MPP = U is masked too; plain M-mode never is
#   4  U-mode, Sv39 (one identity gigapage): a tagged pointer loads (sign-extended from bit 56);
#      a fault on a tagged pointer reports the TRANSFORMED address in mtval
#   5  MXR: a U-mode load is not masked while MXR is set (non-canonical: a load page fault), a
#      store still is
#
# Each U-mode step is `call user`: it runs a0 = the step's code in U-mode and returns to M by
# ecall; a trap in U-mode returns too, with s3 = mcause, s4 = mtval, s5 = traps taken.
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	li	s5, 0

	#---------------- 1: the WARL field
	li	t0, 1 << 33		# PMM = 10
	csrw	senvcfg, t0
	csrr	t1, senvcfg
	srli	t1, t1, 32
	li	t2, 2
	bne	t1, t2, fail1
	li	t0, 3 << 32		# 11: PMLEN 16, not implemented
	csrw	senvcfg, t0
	csrr	t1, senvcfg
	srli	t1, t1, 32
	bnez	t1, fail1
	li	t0, 1 << 32		# 01: reserved
	csrw	senvcfg, t0
	csrr	t1, senvcfg
	srli	t1, t1, 32
	bnez	t1, fail1

	#---------------- 2: U-mode, Bare
	li	t0, 1 << 33
	csrw	senvcfg, t0
	la	a1, area
	li	t0, 0x5a << 57
	or	a1, a1, t0		# a tagged pointer to area
	la	a0, u_ldst
	call	user
	bnez	s5, fail2		# no trap
	li	t1, 0x1122334455667788
	bne	a2, t1, fail2		# the load's value
	ld	t2, 8 + area
	li	t1, 0x0123456789abcdef
	bne	t2, t1, fail2		# the store landed at area + 8
	csrw	senvcfg, x0		# PMM off: the same pointer faults
	li	s5, 0
	la	a0, u_ldst
	call	user
	li	t1, 1
	bne	s5, t1, fail2
	li	t1, 5			# load access fault
	bne	s3, t1, fail2

	#---------------- 3: MPRV with MPP = U is masked; M-mode is not
	li	t0, 1 << 33
	csrw	senvcfg, t0
	li	s5, 0
	li	t0, 0x1800		# MPP = U
	csrc	mstatus, t0
	li	t0, 1 << 17		# MPRV
	csrs	mstatus, t0
	ld	a2, 0(a1)
	csrc	mstatus, t0
	bnez	s5, fail3
	li	t1, 0x1122334455667788
	bne	a2, t1, fail3
	la	s2, 3f
	ld	a2, 0(a1)		# plain M-mode: an access fault
3:	li	t1, 1
	bne	s5, t1, fail3
	li	t1, 5
	bne	s3, t1, fail3

	#---------------- 4: U-mode under Sv39
	la	t0, pt			# root: entry 2 maps VA 0x8000_0000 - 0xbfff_ffff to itself (1 GiB, U RWX AD)
	li	t1, (0x80000000 >> 12 << 10) | 0xdf
	sd	t1, 16(t0)
	srli	t0, t0, 12
	li	t1, 8 << 60
	or	t0, t0, t1
	csrw	satp, t0
	sfence.vma
	li	s5, 0
	la	a0, u_ldst
	call	user			# a1: 0x5a in bits 63:57, bits 56:39 zero -> sign-extended to a canonical VA
	bnez	s5, fail4
	li	t1, 0x1122334455667788
	bne	a2, t1, fail4
	li	s5, 0
	li	a1, (0x2b << 57) | 0x40001000	# VA 0x4000_1000: unmapped (root entry 1)
	la	a0, u_ldst
	call	user
	li	t1, 1
	bne	s5, t1, fail4
	li	t1, 13			# load page fault
	bne	s3, t1, fail4
	li	t1, 0x40001000		# mtval: the transformed address
	bne	s4, t1, fail4

	#---------------- 5: MXR
	li	t0, 1 << 19		# MXR
	csrs	mstatus, t0
	li	s5, 0
	la	a1, area
	li	t0, 0x5a << 57
	or	a1, a1, t0
	la	a0, u_ld		# the load is not masked: a non-canonical VA
	call	user
	li	t1, 1
	bne	s5, t1, fail5
	li	t1, 13
	bne	s3, t1, fail5
	li	s5, 0
	la	a0, u_st		# a store still is
	call	user
	bnez	s5, fail5
	li	t0, 1 << 19
	csrc	mstatus, t0
	csrw	satp, x0

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
done:
	la	t0, tohost
	sd	a0, 0(t0)
99:	j	99b

	# Run a0 in U-mode; return here (to ra) on its ecall or any trap.
user:
	la	s2, user_back
	mv	s6, ra
	csrw	mepc, a0
	li	t0, 0x1800		# MPP = U
	csrc	mstatus, t0
	mret
user_back:
	mv	ra, s6
	ret

	# U-mode code: a1 is the (tagged) pointer
u_ldst:	ld	a2, 0(a1)
	li	t1, 0x0123456789abcdef
	sd	t1, 8(a1)
	ecall
u_ld:	ld	a2, 0(a1)
	ecall
u_st:	li	t1, 0x0123456789abcdef
	sd	t1, 8(a1)
	ecall

	# An ecall from U returns to s2 (user_back) without counting; any other trap records it.
	.align 2
trap_handler:
	csrr	t3, mcause
	li	t4, 8
	beq	t3, t4, 1f
	mv	s3, t3
	csrr	s4, mtval
	addi	s5, s5, 1
1:	li	t4, 0x1800		# back in M
	csrs	mstatus, t4
	csrw	mepc, s2
	mret

	.section .data
	.align 12
pt:	.zero	4096
	.align 3
	.globl area
area:	.dword	0x1122334455667788
	.dword	0
	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0
