# vwr.S -- directed test: the whole-register loads and stores (vl<nf>re<eew>.v, vs<nf>r.v), each
# register two 8-byte chunks through VH0, at VLEN 128.
#
#   1  VS Off: vl1re8.v is an illegal instruction
#   2  vl1re8.v and vs1r.v: 16 bytes round trip
#   3  vl2re16.v, vl4re32.v, vl8re64.v and vs2r.v, vs4r.v, vs8r.v: 32, 64 and 128 bytes
#   4  vstart 3 on a load (e8): bytes 0..2 keep the old ones, and vstart reads 0 after
#   5  vstart 8 on a store: bytes 8..15 alone are stored
#   6  vstart past the end: a load and a store change nothing
#   7  vill: a whole-register load does not read vtype
#   8  S-mode, Sv39 with 4 KiB pages: vl2re8.v across into an unmapped page takes a load page fault
#      at its second register, vstart 16 and the first register loaded; the handler maps the page
#      and the retry loads the second
#   9  vs2r.v the same way, a store page fault: the first register stored before it, the second
#      after the retry
#
# The M-mode handler counts a trap in s5 and keeps mcause in s3 and mtval in s4; with s6 nonzero it
# writes s6 to the PTE at s7 and retries the instruction, else it returns past it. An ecall from S
# ends the run with the verdict in a0.
#   tohost = 1 PASS, else FAIL with test# = tohost>>1

	.option	arch, +v
#define VSTART 0x008

	.section .text.init,"ax",@progbits
	.globl _start
_start:
	la	t0, trap_handler
	csrw	mtvec, t0
	li	s5, 0
	li	s6, 0
	csrw	mstatus, x0
	# src: 128 bytes, dword k = 0x0807060504030201 + k * 0x0808080808080808
	la	a2, src
	li	t0, 0x0807060504030201
	li	t1, 0x0808080808080808
	li	t2, 16
1:	sd	t0, 0(a2)
	add	t0, t0, t1
	addi	a2, a2, 8
	addi	t2, t2, -1
	bnez	t2, 1b

	#---------------- 1: VS Off
	la	a2, src
	vl1re8.v v1, (a2)
	li	t2, 1
	bne	s5, t2, fail1
	li	t0, 1 << 9		# VS Initial
	csrw	mstatus, t0
	li	s5, 0

	#---------------- 2: one register
	la	a2, src
	la	a3, dst
	vl1re8.v v1, (a2)
	vs1r.v	v1, (a3)
	li	a4, 2
	call	cmp
	bnez	a0, fail2
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v1
	ld	t2, 0(a2)
	bne	t1, t2, fail2

	#---------------- 3: groups of 2, 4 and 8 registers
	la	a2, src
	la	a3, dst
	vl2re16.v v2, (a2)
	vs2r.v	v2, (a3)
	li	a4, 4
	call	cmp
	bnez	a0, fail3
	vl4re32.v v4, (a2)
	vs4r.v	v4, (a3)
	li	a4, 8
	call	cmp
	bnez	a0, fail3
	vl8re64.v v8, (a2)
	vs8r.v	v8, (a3)
	li	a4, 16
	call	cmp
	bnez	a0, fail3

	#---------------- 4: vstart on a load
	vsetivli t0, 16, e8, m1, ta, ma
	vmv.v.i	v20, 7
	li	t0, 3
	csrw	VSTART, t0
	la	a2, src
	vl1re8.v v20, (a2)
	csrr	t1, VSTART
	bnez	t1, fail4
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v20
	ld	t2, 0(a2)
	li	t3, -1 << 24
	and	t2, t2, t3
	li	t3, 0x070707
	or	t2, t2, t3
	bne	t1, t2, fail4

	#---------------- 5: vstart on a store
	la	a3, dst
	li	t0, -1
	sd	t0, 0(a3)
	sd	t0, 8(a3)
	li	t0, 8
	csrw	VSTART, t0
	vs1r.v	v1, (a3)
	ld	t1, 0(a3)
	li	t2, -1
	bne	t1, t2, fail5
	ld	t1, 8(a3)
	la	a2, src
	ld	t2, 8(a2)
	bne	t1, t2, fail5

	#---------------- 6: vstart past the end
	li	t0, 100
	csrw	VSTART, t0
	vl1re8.v v20, (a2)		# changes nothing
	csrr	t1, VSTART
	bnez	t1, fail6
	vmv.x.s	t1, v20
	ld	t2, 0(a2)
	li	t3, -1 << 24
	and	t2, t2, t3
	li	t3, 0x070707
	or	t2, t2, t3
	bne	t1, t2, fail6
	li	t0, 100
	csrw	VSTART, t0
	la	a3, dst
	vs1r.v	v20, (a3)		# stores nothing
	ld	t1, 0(a3)
	li	t2, -1
	bne	t1, t2, fail6

	#---------------- 7: vill
	li	a1, 0x100
	li	a0, 4
	vsetvl	t0, a0, a1
	la	a2, src
	vl1re8.v v21, (a2)
	vs1r.v	v21, (a3)
	li	a4, 2
	call	cmp
	bnez	a0, fail7
	bnez	s5, fail7

	#---------------- 8, 9: S-mode, page faults
	# Sv39: 0x8000_0000 -> itself as a gigapage (RWX); 0x4000_0000 -> pagea, 0x4000_1000 -> pageb
	# (4 KiB pages, pt0[1] invalid until the handler writes it)
	la	t0, pt2
	la	t1, pt1
	srli	t1, t1, 12
	slli	t1, t1, 10
	ori	t1, t1, 1
	sd	t1, 8(t0)		# VPN[2] = 1: pt1
	li	t1, (0x80000 << 10) | 0xcf
	sd	t1, 16(t0)		# VPN[2] = 2: the gigapage
	la	t0, pt1
	la	t1, pt0
	srli	t1, t1, 12
	slli	t1, t1, 10
	ori	t1, t1, 1
	sd	t1, 0(t0)		# VPN[1] = 0: pt0
	la	t0, pt0
	la	t1, pagea
	srli	t1, t1, 12
	slli	t1, t1, 10
	ori	t1, t1, 0xc7		# V R W A D
	sd	t1, 0(t0)
	la	t0, pt2
	srli	t0, t0, 12
	li	t1, 8 << 60
	or	t0, t0, t1
	csrw	satp, t0
	sfence.vma
	li	t0, 3 << 11
	csrc	mstatus, t0
	li	t0, 1 << 11		# MPP = S
	csrs	mstatus, t0
	la	t0, s_start
	csrw	mepc, t0
	mret

s_start:
	# the source's last 16 bytes at 0x4000_0ff0, the rest at 0x4000_1000
	la	a2, src
	li	a3, 0x40000ff0
	ld	t0, 0(a2)
	sd	t0, 0(a3)
	ld	t0, 8(a2)
	sd	t0, 8(a3)
	la	t1, pageb
	ld	t0, 16(a2)
	sd	t0, 0(t1)
	ld	t0, 24(a2)
	sd	t0, 8(t1)
	vsetivli t0, 16, e8, m1, ta, ma
	vmv.v.i	v22, 5
	vmv.v.i	v23, 5
	la	s7, pt0
	addi	s7, s7, 8		# pt0[1]
	la	s6, pageb
	srli	s6, s6, 12
	slli	s6, s6, 10
	ori	s6, s6, 0xc7
	li	s5, 0
	vl2re8.v v22, (a3)		# faults at v23, the handler maps the page, the retry finishes
	li	t1, 1
	bne	s5, t1, fail8
	li	t1, 13			# load page fault
	bne	s3, t1, fail8
	li	t1, 0x40001000
	bne	s4, t1, fail8
	bne	s8, s9, fail8		# what the handler saw of v22 and vstart
	vsetivli t0, 1, e64, m1, ta, ma
	vmv.x.s	t1, v23
	ld	t2, 16(a2)
	bne	t1, t2, fail8
	vmv.x.s	t1, v22
	ld	t2, 0(a2)
	bne	t1, t2, fail8

	#---------------- 9: a store fault
	sd	x0, 0(s7)		# pt0[1] invalid again
	sfence.vma
	li	a4, 0x40000ff0
	sd	x0, 0(a4)
	sd	x0, 8(a4)
	li	s5, 0
	vs2r.v	v22, (a4)		# faults at v23's first chunk
	li	t1, 1
	bne	s5, t1, fail9
	li	t1, 15			# store page fault
	bne	s3, t1, fail9
	li	t1, 0x40001000
	bne	s4, t1, fail9
	ld	t1, 0(a4)
	ld	t2, 0(a2)
	bne	t1, t2, fail9
	la	t1, pageb
	ld	t1, 8(t1)
	ld	t2, 24(a2)
	bne	t1, t2, fail9

	li	a0, 1
	ecall
fail1:	li	a0, 3
	j	done_m
fail2:	li	a0, 5
	j	done_m
fail3:	li	a0, 7
	j	done_m
fail4:	li	a0, 9
	j	done_m
fail5:	li	a0, 11
	j	done_m
fail6:	li	a0, 13
	j	done_m
fail7:	li	a0, 15
	j	done_m
fail8:	li	a0, 17
	ecall
fail9:	li	a0, 19
	ecall

	# a0 = 0 when a4 dwords at a2 and a3 agree
cmp:	mv	t4, a2
	mv	t5, a3
	li	a0, 0
1:	ld	t0, 0(t4)
	ld	t1, 0(t5)
	beq	t0, t1, 2f
	li	a0, 1
2:	addi	t4, t4, 8
	addi	t5, t5, 8
	addi	a4, a4, -1
	bnez	a4, 1b
	ret

	.align 2
trap_handler:
	csrr	s3, mcause
	csrr	s4, mtval
	li	t6, 9			# environment call from S
	beq	s3, t6, done_m
	addi	s5, s5, 1
	beqz	s6, 1f
	# a page fault to retry: vstart must be the second register's first element, 16
	csrr	s9, VSTART
	li	s8, 16
	sd	s6, 0(s7)
	sfence.vma
	mret
1:	csrr	t6, mepc
	addi	t6, t6, 4
	csrw	mepc, t6
	mret
done_m:
	la	t0, tohost
	sd	a0, 0(t0)
9:	j	9b

	.section .tohost,"aw",@progbits
	.align 6
	.globl tohost
tohost:	.dword 0
	.globl fromhost
fromhost:
	.dword 0

	.section .data
	.align 12
pt2:	.zero 4096
pt1:	.zero 4096
pt0:	.zero 4096
pagea:	.zero 4096
pageb:	.zero 4096
src:	.zero 128
dst:	.zero 128
