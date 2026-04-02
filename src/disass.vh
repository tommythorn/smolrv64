           // We disassemble the *previous* instruction so we can read
           // the value written to rd (but when csr_mcycle == 0 we
           // have no previous instruction)

           if (csr_mcycle) begin
           if ((insn & 3) == 3)
             $write("%05d   %1d %x %x ", $time, prv, pc, insn);
           else
             $write("%05d   %1d %x     %x ", $time, prv, pc, insn[15:0]);

           if ((insn & 'hffff) == 'h0000)
             $write("c.unimp");
              // Quadrant 0
           else if ((insn & 'he003) == 'h0000)
             $write("c.addi4spn x%1d,%1d", write_back_register, c_nzuimm107_1211_5_6_x4);
           else if ((insn & 'he003) == 'h2000)
             $write("c.fld   x%1d,%1d(x%1d)            UNTESTED", write_back_register, rs1, c_uimm65_1210_x8);
           else if ((insn & 'he003) == 'h4000)
             $write("c.lw    x%1d,%1d(x%1d)", write_back_register, c_uimm5_1210_6_x4, rs1);
           else if ((insn & 'he003) == 'h6000)
             $write("c.ld    x%1d,%1d(x%1d)", write_back_register, c_uimm65_1210_x8, rs1);
           else if ((insn & 'he003) == 'ha000)
             $write("c.fsd   x%1d,%1d(x%1d)    UNTESTED", rs2, c_uimm65_1210_x8, rs1);
           else if ((insn & 'he003) == 'hc000)
             $write("c.sw    x%1d,%1d(x%1d)", rs2, c_uimm5_1210_6_x4, rs1);
           else if ((insn & 'he003) == 'he000)
             $write("c.sd    x%1d,%1d(x%1d)", rs2, c_uimm65_1210_x8, rs1);

              // Quadrant 1
           else if ((insn & 'hffff) == 'h0001)
             $write("c.nop");
           else if ((insn & 'he003) == 'h0001)
             $write("c.addi  x%1d,%1d", write_back_register, $signed(c_imm12_62));
           else if ((insn & 'he003) == 'h2001)
             $write("c.addiw x%1d,%1d", write_back_register, $signed(c_imm12_62));
           else if ((insn & 'he003) == 'h4001)
             $write("c.li    x%1d,%1d", write_back_register, $signed(c_imm12_62));
           else if ((insn & 'hef83) == 'h6101)
             $write("c.addi16sp x%1d,%x", write_back_register, c_imm12_43_5_2_6_x16);
           else if ((insn & 'he003) == 'h6001)
             $write("c.lui   x%1d,%1d", write_back_register, $signed(c_imm12_62)<<12);
           else if ((insn & 'hec03) == 'h8001)
             $write("c.srli  x%1d,%1d", write_back_register, c_imm12_62[5:0]);
           else if ((insn & 'hec03) == 'h8401)
             $write("c.srai  x%1d,%1d", write_back_register, c_imm12_62[5:0]);
           else if ((insn & 'hec03) == 'h8801)
             $write("c.andi  x%1d,%x", write_back_register, c_imm12_62);
           else if ((insn & 'hfc63) == 'h8c01)
             $write("c.sub   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h8c21)
             $write("c.xor   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h8c41)
             $write("c.or    x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h8c61)
             $write("c.and   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h9c01)
             $write("c.subw  x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h9c21)
             $write("c.addw  x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'he003) == 'ha001)
             $write("c.j     %8x", pc + $signed(c_imm12_8_109_6_7_2_11_53_x2));
           else if ((insn & 'he003) == 'hc001)
             $write("c.beqz  x%1d,%8x", rs1, pc + $signed(c_imm12_65_2_1110_43_x2));
           else if ((insn & 'he003) == 'he001)
             $write("c.bnez  x%1d,%8x", rs1, pc + $signed(c_imm12_65_2_1110_43_x2));

              // Quadrant 2
           else if ((insn & 'he003) == 'h0002)
             $write("c.slli  x%1d,%1d", write_back_register, c_imm12_62[5:0]);
           else if ((insn & 'he003) == 'h2002)
             $write("c.fldsp x%1d,%1d(x2)       UNTESTED", write_back_register, c_uimm42_12_65_x8);
           else if ((insn & 'he003) == 'h4002)
             $write("c.lwsp  x%1d,%1d(x2)", write_back_register, c_uimm32_12_64_x4);
           else if ((insn & 'he003) == 'h6002)
             $write("c.ldsp  x%1d,%1d(x2)", write_back_register, c_uimm42_12_65_x8);
           else if ((insn & 'hf07f) == 'h8002)
             $write("c.jr    x%1d", rs1);
           else if ((insn & 'hf003) == 'h8002)
             $write("c.mv    x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hffff) == 'h9002)
             $write("c.ebreak");
           else if ((insn & 'hf07f) == 'h9002)
             $write("c.jalr  x1,0(x%1d)", rs1);
           else if ((insn & 'hf003) == 'h9002)
             $write("c.add   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'he003) == 'ha002)
             $write("c.fsdsp x%1d,%1d(x2)       UNTESTED", rs2, c_uimm97_1210_x8);
           else if ((insn & 'he003) == 'hc002)
             $write("c.swsp  x%1d,%1d(x2)", rs2, c_uimm87_129_x4);
           else if ((insn & 'he003) == 'he002)
             $write("c.sdsp  x%1d,%1d(x2)", rs2, c_uimm97_1210_x8);



           else if ((insn & 'h0000007f) == 'h00000037) // LUI
             $write("lui     x%1d,0x%x", rd, imm_u);
           else if ((insn & 'h0000007f) == 'h00000017) // AUIPC
             $write("auipc   x%1d,0x%x", rd, imm_u);
           else if ((insn & 'h0000007f) == 'h0000006f) // JAL
             $write("jal     x%1d,%x", rd, pc + imm_j);
           else if ((insn & 'h0000707f) == 'h00000067) // JALR
             $write("jalr    x%1d,%x", rd, rs1);
           else if ((insn & 'h0000707f) == 'h00000063) // BEQ
             $write("beq     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00001063) // BNE
             $write("bne     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00004063) // BLT
             $write("blt     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00005063) // BGE
             $write("bge     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00006063) // BLTU
             $write("bltu    x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00007063) // BGEU
             $write("bgeu    x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00000003) // LB
             $write("lb      x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00001003) // LH
             $write("lh      x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00002003) // LW
             $write("lw      x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00004003) // LBU
             $write("lbu     x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00005003) // LHU
             $write("lhu     x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00006003) // LWU
             $write("lwu     x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00003003) // LD
             $write("ld      x%1d,%1d(x%1d)", rd, $signed(imm_i), rs1);
           else if ((insn & 'h0000707f) == 'h00000023) // SB
             $write("sb      x%1d,%1d(x%1d)", rs2, $signed(imm_s), rs1);
           else if ((insn & 'h0000707f) == 'h00001023) // SH
             $write("sh      x%1d,%1d(x%1d)", rs2, $signed(imm_s), rs1);
           else if ((insn & 'h0000707f) == 'h00002023) // SW
             $write("sw      x%1d,%1d(x%1d)", rs2, $signed(imm_s), rs1);
           else if ((insn & 'h0000707f) == 'h00003023) // SD
             $write("sd      x%1d,%1d(x%1d)", rs2, $signed(imm_s), rs1);
           else if ((insn & 'h0000707f) == 'h00000013 && rs1 == 0) // LI (ADDI)
             $write("li      x%1d,0x%x", rd, imm_i);
           else if ((insn & 'h0000707f) == 'h00000013) // ADDI
             $write("addi    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00002013) // SLTI
             $write("slti    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00003013) // SLTIU
             $write("sltiu   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00004013) // XORI
             $write("xori    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00006013) // ORI
             $write("ori     x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00007013) // ANDI
             $write("andi    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h00000033) // ADD
             $write("add     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h40000033) // SUB
             $write("sub     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00001033) // SLL
             $write("sll     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00002033) // SLT
             $write("slt     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00003033) // SLTU
             $write("sltu    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00004033) // XOR
             $write("xor     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00005033) // SRL
             $write("srl     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h40005033) // SRA
             $write("sra     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00006033) // OR
             $write("or      x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00007033) // AND
             $write("and     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hf000707f) == 'h0000000f) // FENCE
             $write("fence");
           else if ((insn & 'hf000707f) == 'h8000000f) // FENCE.TSO
             $write("fence.tso");
           else if ((insn & 'hffffffff) == 'h00000073) // ECALL
             $write("ecall");
           else if ((insn & 'hffffffff) == 'h00100073) // EBREAK
             $write("ebreak");
           else if ((insn & 'hfc00707f) == 'h00001013) // SLLI
             $write("slli    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfc00707f) == 'h00005013) // SRLI
             $write("srli    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfc00707f) == 'h40005013) // SRAI
             $write("srai    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h0000001b) // ADDIW
             $write("addiw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h0000101b) // SLLIW
             $write("slliw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h0000501b) // SRLIW
             $write("srliw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h4000501b) // SRAIW
             $write("sraiw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h0000003b) // ADDW
             $write("addw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h4000003b) // SUBW
             $write("subw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0000103b) // SLLW
             $write("sllw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0000503b) // SRLW
             $write("srlw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h4000503b) // SRAW
             $write("sraw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hffffffff) == 'h0000100f) // FENCE.I
             $write("fence.i");
           else if ((insn & 'h0000707f) == 'h00001073) // CSRRW
             $write("csrrw   x%1d,csr[%3x],x%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00002073) // CSRRS
             $write("csrrs   x%1d,csr[%3x],x%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00003073) // CSRRC
             $write("csrrc   x%1d,csr[%3x],x%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00005073) // CSRRWI
             $write("csrrwi  x%1d,csr[%3x],%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00006073) // CSRRSI
             $write("csrrsi  x%1d,csr[%3x],%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00007073) // CSRRCI
             $write("csrrci  x%1d,csr[%3x],%1d", rd, csrno, rs1);
           else if ((insn & 'hfe00707f) == 'h02000033) // MUL
             $write("mul     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02001033) // MULH
             $write("mulh    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02002033) // MULHSU
             $write("mulhsu  x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02003033) // MULHU
             $write("mulhu   x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02004033) // DIV
             $write("div     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02005033) // DIVU
             $write("divu    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02006033) // REM
             $write("rem     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02007033) // REMU
             $write("remu    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200003b) // MULW
             $write("mulw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200403b) // DIVW
             $write("divw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200503b) // DIVUW
             $write("divuw   x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200603b) // REMW
             $write("remw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200703b) // REMUW
             $write("remuw   x%1d,x%1d,x%1d", rd, rs1, rs2);

           else if ((insn & 'hf9f0707f) == 'h1000202f) // LR.W
             $write("lr.w    x%1d,(x%1d)", rd, rs1);
           else if ((insn & 'hf800707f) == 'h1800202f) // SC.W
             $write("sc.w    x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0800202f) // AMOSWAP.W
             $write("amoswap.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0000202f) // AMOADD.W
             $write("amoadd.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h2000202f) // AMOXOR.W
             $write("amoxor.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h6000202f) // AMOAND.W
             $write("amoand.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h4000202f) // AMOOR.W
             $write("amoor.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h8000202f) // AMOMIN.W
             $write("amomin.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'ha000202f) // AMOMAX.W
             $write("amomax.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'hc000202f) // AMOMINU.W
             $write("amominu.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'he000202f) // AMOMAXU.W
             $write("amomaxu.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);

           else if ((insn & 'hf9f0707f) == 'h1000302f) // LR.D
             $write("lr.d    x%1d,(x%1d)", rd, rs1);
           else if ((insn & 'hf800707f) == 'h1800302f) // SC.D
             $write("sc.d    x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0800302f) // AMOSWAP.D
             $write("amoswap.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0000302f) // AMOADD.D
             $write("amoadd.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h2000302f) // AMOXOR.D
             $write("amoxor.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h6000302f) // AMOAND.D
             $write("amoand.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h4000302f) // AMOOR.D
             $write("amoor.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h8000302f) // AMOMIN.D
             $write("amomin.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'ha000302f) // AMOMAX.D
             $write("amomax.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'hc000302f) // AMOMINU.D
             $write("amominu.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'he000302f) // AMOMAXU.D
             $write("amomaxu.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hffffffff) == 'h30200073) // MRET
             $write("mret");
           else if ((insn & 'hffffffff) == 'h10200073) // SRET
             $write("sret");
           else if ((insn & 'hfe007fff) == 'h12000073) // SFENCE.VMA
             $write("sfence.vma");
           else if ((insn & 'hffffffff) == 'h10500073) // WFI
             $write("wfi");
           else
             $write("illegal or unsupported instruction");

           if (write_back_register != 0)
             $display("     x%1d = %x", write_back_register, write_back_register == 0 ? 0 : write_back_value);
           else
             $display("");
           end
