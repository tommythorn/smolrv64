    # SoC UART test: print a string over the NS16550A UART @ 0x1000_0000 (poll LSR
    # THRE, write THR per char), exercising sub-word (byte) MMIO loads+stores through
    # tb_soc.v's byte-addressed device routing. PASS once the string is emitted.
    .section .text
    .globl _start
_start:
    li   s0, 0x10000000          # UART base
    la   s1, msg
1:  lbu  s2, 0(s1)               # next char
    beqz s2, 2f
3:  lbu  s3, 5(s0)               # LSR
    andi s3, s3, 0x20            # THRE (transmit-hold empty)?
    beqz s3, 3b
    sb   s2, 0(s0)               # THR <- char
    addi s1, s1, 1
    j    1b
2:  li   t0, 1                   # PASS
    la   t1, tohost
    sd   t0, 0(t1)
done:
    j    done

msg: .string "Hello from the OoO SoC\n"

    .section .tohost,"aw"
    .align 3
    .globl tohost
tohost: .dword 0
