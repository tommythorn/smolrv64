# store->load ordering torture for the sharded-OoO probe.
#
# Reproduces (fast) the class of bug caught in the Ubuntu cosim at 6.58B insn: a load
# returned STALE data instead of a prior committed same-address store's value. The LSU has
# no memory-order replay -- correctness rests on the SELECT-stage order gate + byte-forward,
# and a store can drain out of the 4-deep SB in the 1-cycle window between a same-address
# load's SELECT (mem read issued) and MERGE (s_use recomputed). This hammers that window.
#
# Method: each iteration bursts 8 unique values into 8 aligned 8-byte slots (SBDEPTH=4 -> the
# burst forces drains mid-flight), sweeps the store->load spacing (0..N filler ops keyed by
# the iteration counter, so we cover every drain-vs-SELECT/MERGE alignment), then loads all 8
# back and checks each == what it stored. Monotonic values -> any stale byte is detected.
#   tohost = 1  -> PASS (ran the whole count clean)
#   tohost = 3  -> FAIL (a load != its store: the ordering bug)   [test# = 1]

    .section .text
    .globl _start
_start:
    li      s0, 0x80002000          # working set: 8 x 8-byte aligned slots
    li      s2, 0                   # iteration counter
    li      s3, 0x7fffffff          # cap (run is really bounded by +cycles)
    li      t0, 0x1000              # monotonic value seed (nonzero: stale-0 is caught)

.loop:
    addi    a0, t0, 0               # 8 unique values this iteration
    addi    a1, t0, 1
    addi    a2, t0, 2
    addi    a3, t0, 3
    addi    a4, t0, 4
    addi    a5, t0, 5
    addi    a6, t0, 6
    addi    a7, t0, 7

    sd      a0, 0(s0)               # burst: 8 stores into a 4-deep SB -> drains mid-burst
    sd      a1, 8(s0)
    sd      a2, 16(s0)
    sd      a3, 24(s0)
    sd      a4, 32(s0)
    sd      a5, 40(s0)
    sd      a6, 48(s0)
    sd      a7, 56(s0)

    andi    t4, s2, 31              # sweep the store->load window: 0..31 filler beats
.fill:
    beqz    t4, .chk
    addi    t4, t4, -1
    j       .fill
.chk:
    ld      t3, 0(s0)               # load each slot back; the later slots race their drain
    bne     t3, a0, .fail
    ld      t3, 8(s0)
    bne     t3, a1, .fail
    ld      t3, 16(s0)
    bne     t3, a2, .fail
    ld      t3, 24(s0)
    bne     t3, a3, .fail
    ld      t3, 32(s0)
    bne     t3, a4, .fail
    ld      t3, 40(s0)
    bne     t3, a5, .fail
    ld      t3, 48(s0)
    bne     t3, a6, .fail
    ld      t3, 56(s0)
    bne     t3, a7, .fail

    addi    t0, t0, 8               # advance the monotonic seed
    addi    s2, s2, 1
    bltu    s2, s3, .loop

.pass:
    li      t0, 1
    la      t1, tohost
    sd      t0, 0(t1)
1:  j       1b

.fail:
    li      t0, 3                   # (test# 1) << 1 | 1
    la      t1, tohost
    sd      t0, 0(t1)
1:  j       1b

    .section .data
    .globl  tohost
    .align  3
tohost:
    .dword  0
