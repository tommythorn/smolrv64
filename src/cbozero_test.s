# Directed test for Zicboz cbo.zero (whole-line zero).
#
# Bare-metal M-mode (satp=0): exercises the cacheable cbo.zero data path
# (all-8-bank zero + dirty, hit case) with no translation. Signals the result
# via the riscv-tests tohost convention: store 1 to tohost = "Test Passed",
# any other non-zero value = "Test Failed with N".
#
# Build + run:
#   make -C src smolrv64-tester cbozero_test.even cbozero_test.odd
#   ./smolrv64-tester +even=cbozero_test.even +odd=cbozero_test.odd
#
        .option arch, +zicboz
        .section .text
        .globl _start
_start:
        li      s0, 0x80010000          # block A base (64-byte aligned)
        li      s1, 0x80010040          # block B base (adjacent; must stay intact)
        li      t1, 0xA5A5A5A5A5A5A5A5   # block A fill pattern
        li      t2, 0x5A5A5A5A5A5A5A5A   # block B fill pattern (distinct)

        # Fill all 8 doublewords of block A with the pattern.
        sd      t1, 0(s0)
        sd      t1, 8(s0)
        sd      t1, 16(s0)
        sd      t1, 24(s0)
        sd      t1, 32(s0)
        sd      t1, 40(s0)
        sd      t1, 48(s0)
        sd      t1, 56(s0)

        # Fill block B with a different pattern (guards against over-zeroing).
        sd      t2, 0(s1)
        sd      t2, 8(s1)
        sd      t2, 16(s1)
        sd      t2, 24(s1)
        sd      t2, 32(s1)
        sd      t2, 40(s1)
        sd      t2, 48(s1)
        sd      t2, 56(s1)

        # Zero block A (resident & dirty -> hit path, all 8 banks in one cycle).
        cbo.zero (s0)

        # All 8 doublewords of A must now read as zero.
        ld      a0, 0(s0);  bnez a0, fail
        ld      a0, 8(s0);  bnez a0, fail
        ld      a0, 16(s0); bnez a0, fail
        ld      a0, 24(s0); bnez a0, fail
        ld      a0, 32(s0); bnez a0, fail
        ld      a0, 40(s0); bnez a0, fail
        ld      a0, 48(s0); bnez a0, fail
        ld      a0, 56(s0); bnez a0, fail

        # Block B must be untouched (still its pattern) - cbo.zero must not
        # spill past the 64-byte line.
        ld      a0, 0(s1);  bne a0, t2, fail
        ld      a0, 8(s1);  bne a0, t2, fail
        ld      a0, 16(s1); bne a0, t2, fail
        ld      a0, 24(s1); bne a0, t2, fail
        ld      a0, 32(s1); bne a0, t2, fail
        ld      a0, 40(s1); bne a0, t2, fail
        ld      a0, 48(s1); bne a0, t2, fail
        ld      a0, 56(s1); bne a0, t2, fail

        # cbo.zero operates on the block *containing* the address: re-fill A and
        # zero it via an unaligned pointer into the block; the whole line clears.
        sd      t1, 0(s0)
        sd      t1, 56(s0)
        addi    t3, s0, 37              # unaligned pointer inside block A
        cbo.zero (t3)
        ld      a0, 0(s0);  bnez a0, fail
        ld      a0, 56(s0); bnez a0, fail

pass:
        li      t0, 1
        li      t1, 0x80001000          # default tohost
        sd      t0, 0(t1)
0:      j       0b

fail:
        li      t0, 3                   # fail code (>1 -> "Test Failed with 3")
        li      t1, 0x80001000
        sd      t0, 0(t1)
0:      j       0b
