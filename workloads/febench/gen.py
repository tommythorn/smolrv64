#!/usr/bin/env python3
"""febench: what the FRONTEND delivers on straight-line code, nothing else in the way.

Three blocks of 4096 independent 32-bit ALU instructions each (no loads, no stores, no
branches inside), run 64 times, cycles and instructions read around each:
  aligned   : every instruction 32-bit, the block starts 4-byte aligned
  shifted   : one c.nop first, so every 32-bit instruction sits at a 2-byte offset
              and straddles the 8-byte fetch window at every other window
  compressed: c.add/c.and/c.xor/c.or only, two per 4 bytes
IPC = the fetch rate, since dispatch is one per cycle and nothing stalls. The CPI stack of
sha256sum on the board (2026-09-05) put 42% of its cycles in the fetch buckets; this is
the bench that measures the window and the straddle in isolation, before and after HW=8.
"""
regs = ["t0","t1","t2","t3","t4","t5","t6","a0","a1","a2","a3","a4","a5","a6","a7","s2","s3","s4","s5","s6","s7","s8","s9","s10","s11"]
def block(kind, n=4096):
    out = []
    if kind == "shifted":
        out += ["    .option push", "    .option rvc", "    c.nop", "    .option pop"]
    if kind == "compressed":
        out += ["    .option push", "    .option rvc"]
    for i in range(n):
        d, a, b = regs[i % len(regs)], regs[(i + 7) % len(regs)], regs[(i + 13) % len(regs)]
        if kind == "compressed":
            # c.and/c.or/c.xor need rd=rs1 in x8..x15: use a-registers
            r1, r2 = ["a0","a1","a2","a3","a4","a5"][i % 6], ["a0","a1","a2","a3","a4","a5"][(i + 3) % 6]
            out.append("    c.%s %s, %s" % (["and","or","xor"][i % 3], r1, r2))
        else:
            out.append("    %s %s, %s, %s" % (["addw","xor","or","and","subw"][i % 5], d, a, b))
    if kind == "compressed":
        out.append("    .option pop")
    return "\n".join(out)
print(""".option norvc
.text
.globl main
main:
    addi sp, sp, -16
    sd ra, 8(sp)
""")
for kind in ["aligned", "shifted", "compressed"]:
    print("    .balign 4")
    print("    rdcycle s0")
    print("    rdinstret s1")
    print("    li t0, 64")
    print("1:")
    print("    .balign 4")
    print("    addi t0, t0, -1")   # keep t0 the loop counter: the block must not touch t0
    print("    beqz t0, 2f")
    print(block(kind).replace("t0,", "s6,").replace(" t0 ", " s6 "))
    print("    j 1b")
    print("2:")
    print("    rdcycle a0")
    print("    rdinstret a1")
    print("    sub a0, a0, s0")
    print("    sub a1, a1, s1")
    print("    la a2, name_%s" % kind)
    print("    call report")
print("""    ld ra, 8(sp)
    addi sp, sp, 16
3:  j 3b
.section .rodata
name_aligned:    .asciz "aligned   "
name_shifted:    .asciz "shifted   "
name_compressed: .asciz "compressed"
""")
