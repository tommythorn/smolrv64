# Directed test: Svpbmt NC (PBMT=01) pages bypass the L1 cache.
#
# Proof by HPM VHPR_FILLS counter: repeatedly reading one NC page must miss+fill
# on EVERY access (never cached), while reading a PMA page fills only once.
#
# M-mode test using MPRV (MPP=S) to perform S-mode-translated data accesses
# without changing execution privilege. Two 1 GiB superpages map the same
# physical region twice: VA 0x40000000.. as NC, VA 0x80000000.. as PMA.
#
# Build + run:
#   make -C src smolrv64-tester svpbmt_nc_test.even svpbmt_nc_test.odd
#   ./smolrv64-tester +even=svpbmt_nc_test.even +odd=svpbmt_nc_test.odd
#
        .option arch, +zicbom
        .section .text
        .globl _start
_start:
        # --- build root page table at 0x80100000 (1 GiB leaf PTEs) ---
        li      t0, 0x80100008          # &root_pt[1]  (covers VA 0x40000000+)
        li      t1, 0x20000000200000CF  # NC leaf: PBMT=01(bit61), PA 0x80000000, RWXAD V
        sd      t1, 0(t0)
        li      t0, 0x80100010          # &root_pt[2]  (covers VA 0x80000000+)
        li      t1, 0x200000CF          # PMA leaf: PA 0x80000000, RWXAD V
        sd      t1, 0(t0)
        # Flush the PT line to memory: the hardware PTW reads page tables on a
        # cache-bypassing path, so the entries must be resident in DRAM first.
        li      t0, 0x80100000
        cbo.flush (t0)
        fence

        # --- enable Svpbmt and Sv39 translation ---
        li      t0, 0x4000000000000000  # menvcfg.PBMTE (bit 62)
        csrs    0x30a, t0               # menvcfg
        li      t0, 0x8000000000080100  # satp = MODE=8 (Sv39) | PPN(0x80100000)
        csrw    0x180, t0               # satp
        sfence.vma
        li      t0, 0x20800             # mstatus: MPRV(17)=1, MPP[12:11]=01 (S)
        csrs    0x300, t0

        # --- HPM counter 3 := VHPR fills ---
        li      t0, 0x406               # HPM_EVENT_VHPR_FILLS
        csrw    0x323, t0               # mhpmevent3
        li      t0, 8                   # counter 3 bit
        csrc    0x320, t0               # mcountinhibit: run counter 3

        # --- NC: 16 reads of one page; expect ~16 fills (never cached) ---
        csrr    s0, 0xb03               # mhpmcounter3 before
        li      t0, 0x40200000          # NC VA
        li      t2, 16
1:      ld      zero, 0(t0)
        addi    t2, t2, -1
        bnez    t2, 1b
        csrr    s1, 0xb03
        sub     s2, s1, s0              # NC fills delta

        # --- PMA: 16 reads of the same page; expect ~1 fill (cached) ---
        csrr    s0, 0xb03
        li      t0, 0x80200000          # PMA VA (same physical, cacheable)
        li      t2, 16
2:      ld      zero, 0(t0)
        addi    t2, t2, -1
        bnez    t2, 2b
        csrr    s1, 0xb03
        sub     s3, s1, s0              # PMA fills delta

        li      t0, 0x20000             # clear MPRV: tohost store is M-mode physical
        csrc    0x300, t0

        li      t3, 12
        blt     s2, t3, fail            # NC fills too few -> not bypassing
        li      t3, 4
        bgt     s3, t3, fail            # PMA fills too many -> regression
pass:
        li      t0, 1
        li      t1, 0x80001000
        sd      t0, 0(t1)
0:      j       0b
fail:
        li      t0, 3                   # "Test Failed with 3"
        li      t1, 0x80001000
        sd      t0, 0(t1)
0:      j       0b
