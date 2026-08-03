    # SoC end-to-end test: program the real CLINT timer over MMIO (read mtime, write
    # mtimecmp), then take a machine timer interrupt delivered through the address-
    # decoded bus. Proves LSU store->CLINT and LSU load<-CLINT routing + mtip->trap.
    # CLINT @ 0x0200_0000: mtime @ +0xBFF8, mtimecmp @ +0x4000 (64-bit aligned sd/ld).
    .section .text
    .globl _start
_start:
    la   t0, handler
    csrw mtvec, t0

    li   t2, 0x02000000          # CLINT base
    li   t3, 0xBFF8
    add  t3, t2, t3              # &mtime
    ld   t4, 0(t3)               # t4 = current mtime  (MMIO load)
    addi t4, t4, 10              # fire 10 ticks from now
    li   t5, 0x4000
    add  t5, t2, t5              # &mtimecmp
    sd   t4, 0(t5)               # mtimecmp = mtime + 10  (MMIO store)

    li   t0, 0x80                # MTIE
    csrw mie, t0
    li   t0, 0x8                 # MIE
    csrs mstatus, t0
spin:
    j    spin

    # keep the handler off the spin loop's fall-through speculation path.
    .balign 256
handler:
    csrr t0, mcause
    li   t1, 0x8000000000000007  # machine timer interrupt (MSB | cause 7)
    bne  t0, t1, fail
    li   t2, 0x02000000          # disarm: mtimecmp = -1 (clears MTIP next cycle)
    li   t5, 0x4000
    add  t5, t2, t5
    li   t4, -1
    sd   t4, 0(t5)
    li   t0, 1                   # PASS
    la   t1, tohost
    sd   t0, 0(t1)
hdone:
    j    hdone
fail:
    li   t0, 0xbad
    la   t1, tohost
    sd   t0, 0(t1)
fdone:
    j    fdone

    .section .tohost,"aw"
    .align 3
    .globl tohost
tohost: .dword 0
