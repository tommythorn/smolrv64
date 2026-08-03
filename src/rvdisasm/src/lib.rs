// Standalone RISC-V disassembler (RV64GC), std-only, no dependencies.
//
// The probe pipeline carries the RVC-*expanded* 32-bit instruction in its payload, so
// this decodes 32-bit encodings only (no compressed decode needed). It emits objdump-
// flavoured text with ABI register names and the common pseudo-instructions, and
// resolves branch/jump targets to absolute addresses using the instruction's PC.
//
// This is the seed of the "beautiful disassembly" sub-project: integer (RV64IMA),
// Zicsr/system, fences, and FP load/store are covered; remaining FP/vector ops fall
// back to a raw `.insn 0x........`. Extend per opcode.

const X: [&str; 32] = [
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4",
    "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4",
    "t5", "t6",
];
const F: [&str; 32] = [
    "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "fs0", "fs1", "fa0", "fa1", "fa2",
    "fa3", "fa4", "fa5", "fa6", "fa7", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9",
    "fs10", "fs11", "ft8", "ft9", "ft10", "ft11",
];

#[inline]
fn rd(i: u32) -> usize {
    ((i >> 7) & 0x1f) as usize
}
#[inline]
fn rs1(i: u32) -> usize {
    ((i >> 15) & 0x1f) as usize
}
#[inline]
fn rs2(i: u32) -> usize {
    ((i >> 20) & 0x1f) as usize
}
#[inline]
fn f3(i: u32) -> u32 {
    (i >> 12) & 0x7
}
#[inline]
fn f7(i: u32) -> u32 {
    (i >> 25) & 0x7f
}
#[inline]
fn sext(v: u32, bits: u32) -> i64 {
    let shift = 64 - bits;
    ((v as i64) << shift) >> shift
}
fn imm_i(i: u32) -> i64 {
    sext(i >> 20, 12)
}
fn imm_s(i: u32) -> i64 {
    sext(((i >> 25) << 5) | ((i >> 7) & 0x1f), 12)
}
fn imm_b(i: u32) -> i64 {
    let v = (((i >> 31) & 1) << 12)
        | (((i >> 7) & 1) << 11)
        | (((i >> 25) & 0x3f) << 5)
        | (((i >> 8) & 0xf) << 1);
    sext(v, 13)
}
fn imm_u(i: u32) -> i64 {
    (i & 0xffff_f000) as i32 as i64
}
fn imm_j(i: u32) -> i64 {
    let v = (((i >> 31) & 1) << 20)
        | (((i >> 12) & 0xff) << 12)
        | (((i >> 20) & 1) << 11)
        | (((i >> 21) & 0x3ff) << 1);
    sext(v, 21)
}

fn csr_name(c: u32) -> String {
    let n = match c {
        0x001 => "fflags",
        0x002 => "frm",
        0x003 => "fcsr",
        0x100 => "sstatus",
        0x104 => "sie",
        0x105 => "stvec",
        0x106 => "scounteren",
        0x10a => "senvcfg",
        0x140 => "sscratch",
        0x141 => "sepc",
        0x142 => "scause",
        0x143 => "stval",
        0x144 => "sip",
        0x14d => "stimecmp",
        0x180 => "satp",
        0x300 => "mstatus",
        0x301 => "misa",
        0x302 => "medeleg",
        0x303 => "mideleg",
        0x304 => "mie",
        0x305 => "mtvec",
        0x306 => "mcounteren",
        0x30a => "menvcfg",
        0x340 => "mscratch",
        0x341 => "mepc",
        0x342 => "mcause",
        0x343 => "mtval",
        0x344 => "mip",
        0xc00 => "cycle",
        0xc01 => "time",
        0xc02 => "instret",
        0xf11 => "mvendorid",
        0xf12 => "marchid",
        0xf13 => "mimpid",
        0xf14 => "mhartid",
        _ => return format!("0x{c:x}"),
    };
    n.to_string()
}

/// Disassemble one 32-bit instruction located at `pc`.
pub fn disasm(insn: u32, pc: u64) -> String {
    let op = insn & 0x7f;
    let (d, a, b) = (rd(insn), rs1(insn), rs2(insn));
    let tgt = |imm: i64| format!("{:x}", pc.wrapping_add(imm as u64));
    match op {
        0x37 => format!("lui     {},0x{:x}", X[d], (imm_u(insn) >> 12) & 0xfffff),
        0x17 => format!("auipc   {},0x{:x}", X[d], (imm_u(insn) >> 12) & 0xfffff),
        0x6f => {
            // JAL
            if d == 0 {
                format!("j       {}", tgt(imm_j(insn)))
            } else if d == 1 {
                format!("jal     {}", tgt(imm_j(insn)))
            } else {
                format!("jal     {},{}", X[d], tgt(imm_j(insn)))
            }
        }
        0x67 => {
            // JALR
            let im = imm_i(insn);
            if d == 0 && a == 1 && im == 0 {
                "ret".to_string()
            } else if d == 0 && im == 0 {
                format!("jr      {}", X[a])
            } else if d == 0 {
                format!("jalr    {},{}({})", X[d], im, X[a])
            } else {
                format!("jalr    {},{}({})", X[d], im, X[a])
            }
        }
        0x63 => {
            // BRANCH
            let m = ["beq", "bne", "?", "?", "blt", "bge", "bltu", "bgeu"][f3(insn) as usize];
            let t = tgt(imm_b(insn));
            if b == 0 && (f3(insn) == 0 || f3(insn) == 1) {
                let p = if f3(insn) == 0 { "beqz" } else { "bnez" };
                format!("{:<7} {},{}", p, X[a], t)
            } else {
                format!("{:<7} {},{},{}", m, X[a], X[b], t)
            }
        }
        0x03 => {
            // LOAD
            let m = ["lb", "lh", "lw", "ld", "lbu", "lhu", "lwu", "?"][f3(insn) as usize];
            format!("{:<7} {},{}({})", m, X[d], imm_i(insn), X[a])
        }
        0x23 => {
            // STORE
            let m = ["sb", "sh", "sw", "sd", "?", "?", "?", "?"][f3(insn) as usize];
            format!("{:<7} {},{}({})", m, X[b], imm_s(insn), X[a])
        }
        0x13 => {
            // OP-IMM
            let im = imm_i(insn);
            match f3(insn) {
                0 if d == 0 && a == 0 && im == 0 => "nop".to_string(),
                0 if im == 0 => format!("mv      {},{}", X[d], X[a]),
                0 if a == 0 => format!("li      {},{}", X[d], im),
                0 => format!("addi    {},{},{}", X[d], X[a], im),
                1 => format!("slli    {},{},{}", X[d], X[a], (insn >> 20) & 0x3f),
                2 => format!("slti    {},{},{}", X[d], X[a], im),
                3 => format!("sltiu   {},{},{}", X[d], X[a], im),
                4 if im == -1 => format!("not     {},{}", X[d], X[a]),
                4 => format!("xori    {},{},{}", X[d], X[a], im),
                5 if f7(insn) & 0x20 != 0 => format!("srai    {},{},{}", X[d], X[a], (insn >> 20) & 0x3f),
                5 => format!("srli    {},{},{}", X[d], X[a], (insn >> 20) & 0x3f),
                6 => format!("ori     {},{},{}", X[d], X[a], im),
                _ => format!("andi    {},{},{}", X[d], X[a], im),
            }
        }
        0x1b => {
            // OP-IMM-32
            let im = imm_i(insn);
            match f3(insn) {
                0 if a == 0 => format!("li      {},{}  # sext.w", X[d], im),
                0 => format!("addiw   {},{},{}", X[d], X[a], im),
                1 => format!("slliw   {},{},{}", X[d], X[a], (insn >> 20) & 0x1f),
                5 if f7(insn) & 0x20 != 0 => format!("sraiw   {},{},{}", X[d], X[a], (insn >> 20) & 0x1f),
                _ => format!("srliw   {},{},{}", X[d], X[a], (insn >> 20) & 0x1f),
            }
        }
        0x33 => {
            // OP
            if f7(insn) == 1 {
                let m = ["mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"][f3(insn) as usize];
                return format!("{:<7} {},{},{}", m, X[d], X[a], X[b]);
            }
            let alt = f7(insn) & 0x20 != 0;
            let m = match f3(insn) {
                0 => {
                    if alt {
                        "sub"
                    } else {
                        "add"
                    }
                }
                1 => "sll",
                2 => "slt",
                3 => "sltu",
                4 => "xor",
                5 => {
                    if alt {
                        "sra"
                    } else {
                        "srl"
                    }
                }
                6 => "or",
                _ => "and",
            };
            if f3(insn) == 0 && alt && a == 0 {
                format!("neg     {},{}", X[d], X[b])
            } else {
                format!("{:<7} {},{},{}", m, X[d], X[a], X[b])
            }
        }
        0x3b => {
            // OP-32
            if f7(insn) == 1 {
                let m = ["mulw", "?", "?", "?", "divw", "divuw", "remw", "remuw"][f3(insn) as usize];
                return format!("{:<7} {},{},{}", m, X[d], X[a], X[b]);
            }
            let alt = f7(insn) & 0x20 != 0;
            let m = match f3(insn) {
                0 => {
                    if alt {
                        "subw"
                    } else {
                        "addw"
                    }
                }
                1 => "sllw",
                5 => {
                    if alt {
                        "sraw"
                    } else {
                        "srlw"
                    }
                }
                _ => "?",
            };
            format!("{:<7} {},{},{}", m, X[d], X[a], X[b])
        }
        0x0f => {
            // MISC-MEM
            if f3(insn) == 1 {
                "fence.i".to_string()
            } else {
                "fence".to_string()
            }
        }
        0x73 => {
            // SYSTEM
            match f3(insn) {
                0 => match insn {
                    0x0000_0073 => "ecall".to_string(),
                    0x0010_0073 => "ebreak".to_string(),
                    0x3020_0073 => "mret".to_string(),
                    0x1020_0073 => "sret".to_string(),
                    0x1050_0073 => "wfi".to_string(),
                    _ if f7(insn) == 0x09 => format!("sfence.vma {},{}", X[a], X[b]),
                    _ => format!(".insn   0x{insn:08x}"),
                },
                1 if d == 0 => format!("csrw    {},{}", csr_name(insn >> 20), X[a]),
                1 => format!("csrrw   {},{},{}", X[d], csr_name(insn >> 20), X[a]),
                2 if a == 0 => format!("csrr    {},{}", X[d], csr_name(insn >> 20)),
                2 => format!("csrrs   {},{},{}", X[d], csr_name(insn >> 20), X[a]),
                3 => format!("csrrc   {},{},{}", X[d], csr_name(insn >> 20), X[a]),
                5 if d == 0 => format!("csrwi   {},{}", csr_name(insn >> 20), a),
                5 => format!("csrrwi  {},{},{}", X[d], csr_name(insn >> 20), a),
                6 => format!("csrrsi  {},{},{}", X[d], csr_name(insn >> 20), a),
                7 => format!("csrrci  {},{},{}", X[d], csr_name(insn >> 20), a),
                _ => format!(".insn   0x{insn:08x}"),
            }
        }
        0x2f => {
            // AMO
            let w = if f3(insn) == 3 { "d" } else { "w" };
            let m = match f7(insn) >> 2 {
                0x00 => "amoadd",
                0x01 => "amoswap",
                0x02 => "lr",
                0x03 => "sc",
                0x04 => "amoxor",
                0x08 => "amoor",
                0x0c => "amoand",
                0x10 => "amomin",
                0x14 => "amomax",
                0x18 => "amominu",
                0x1c => "amomaxu",
                _ => "amo?",
            };
            if (f7(insn) >> 2) == 0x02 {
                format!("{m}.{w}  {},({})", X[d], X[a])
            } else {
                format!("{m}.{w}  {},{},({})", X[d], X[b], X[a])
            }
        }
        0x07 => {
            // LOAD-FP
            let m = if f3(insn) == 3 { "fld" } else { "flw" };
            format!("{:<7} {},{}({})", m, F[d], imm_i(insn), X[a])
        }
        0x27 => {
            // STORE-FP
            let m = if f3(insn) == 3 { "fsd" } else { "fsw" };
            format!("{:<7} {},{}({})", m, F[b], imm_s(insn), X[a])
        }
        _ => format!(".insn   0x{insn:08x}"),
    }
}
