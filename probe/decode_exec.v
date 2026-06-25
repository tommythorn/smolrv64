`include "alu_ops.vh"
`default_nettype none

// Execution control decode ("ctl blob") for the sharded-OoO backend, sibling to
// decode_operands (which produces the rename-facing rs/rd/imm). Input is the
// RVC-EXPANDED 32-bit instruction, so only the 32-bit encodings exist here -- the
// whole compressed half of smolrv64.v's pre-decode disappears. The ALU op map is
// ported verbatim from that pre-decode (the hand-written, non-generated part);
// alu.v is reused unchanged.
//
// Result routing folds the old EXOP_OPB pseudo-op away: LUI = ADD with op1=0,
// AUIPC = ADD with op1=PC, JAL/JALR link = next_pc (res_link). Loads/stores use
// alu.sum (rs1+imm) as the address; a store's data (rs2) is a separate dependency
// so the store can occupy its LSU slot without waiting for data. CSR-write and
// other serializing ops are flagged so the steering can pin them to shard 0.
module decode_exec
  (input  wire [31:0] insn,        // RVC-expanded 32-bit instruction
   output reg  [5:0]  alu_op,      // ALU_* (alu.v)
   output reg         alu_w,       // *W 32-bit word op
   output reg         alu_uw,      // Zba .uw / slli.uw
   output reg  [1:0]  op1_sel,     // OP1_RS1 / OP1_PC / OP1_ZERO
   output reg         op2_imm,     // 1 = op2 is imm, 0 = op2 is rs2
   output reg         res_link,    // 1 = result is next_pc (JAL/JALR link)
   output reg         is_mem,
   output reg         is_store,
   output reg  [1:0]  mem_size,    // 0=B 1=H 2=W 3=D
   output reg         mem_signed,  // load sign-extends (funct3[2]==0)
   output reg         is_branch,
   output reg  [2:0]  br_func,     // BRANCH funct3
   output reg         is_jump,     // JAL/JALR -> redirect
   output reg         is_csr,
   output reg  [2:0]  csr_func,    // SYSTEM funct3 (csrrw/s/c + imm variants)
   output reg         is_serialize,// CSR-write / system / fence -> pin to shard 0
   output reg         is_mul,      // M ext (deferred unit)
   output reg         is_amo,      // A ext (serialized RMW in LSU)
   output reg  [4:0]  amo_func,    // AMO funct5 (insn[31:27]): LR/SC/swap/add/and/or/xor/min/max
   output reg         is_fp,       // F/D ext (deferred unit)
   output reg         is_fencei,   // FENCE.I -> serialize + redirect to refetch (I/D coherence)
   output reg         illegal);

   localparam [1:0] OP1_RS1 = 2'd0, OP1_PC = 2'd1, OP1_ZERO = 2'd2;

   wire [4:0] opc = insn[6:2];
   wire [2:0] f3  = insn[14:12];
   wire [6:0] f7  = insn[31:25];
   wire [5:0] f6  = insn[31:26];

   always @* begin
      // defaults: a harmless ADD rs1,rs2
      alu_op=`ALU_ADD; alu_w=0; alu_uw=0; op1_sel=OP1_RS1; op2_imm=0; res_link=0;
      is_mem=0; is_store=0; mem_size=2'd0; mem_signed=0;
      is_branch=0; br_func=f3; is_jump=0;
      is_csr=0; csr_func=f3; is_serialize=0;
      is_mul=0; is_amo=0; amo_func=insn[31:27]; is_fp=0; is_fencei=0; illegal=0;

      if (insn[1:0] != 2'b11) illegal = 1'b1;   // not a 32-bit insn (should be expanded)
      else case (opc)
        5'b01101: begin op1_sel=OP1_ZERO; op2_imm=1; end                 // LUI  (ADD 0,imm)
        5'b00101: begin op1_sel=OP1_PC;   op2_imm=1; end                 // AUIPC(ADD pc,imm)
        5'b11011: begin res_link=1; is_jump=1; end                       // JAL
        5'b11001: begin res_link=1; is_jump=1; op2_imm=1; end            // JALR (addr=rs1+imm)

        5'b00100: begin // OP-IMM (+ Zbb/Zbs immediate forms)
           op2_imm=1;
           case (f3)
             3'b000: alu_op=`ALU_ADD;  3'b010: alu_op=`ALU_SLT;
             3'b011: alu_op=`ALU_SLTU; 3'b100: alu_op=`ALU_XOR;
             3'b110: alu_op=`ALU_OR;   3'b111: alu_op=`ALU_AND;
             3'b001: case (f6)                                            // SLLI / Zbs / Zbb-unary
                       6'b010010: alu_op=`ALU_BCLR; 6'b011010: alu_op=`ALU_BINV;
                       6'b001010: alu_op=`ALU_BSET;
                       6'b011000: case (insn[24:20])
                                    5'b00001: alu_op=`ALU_CTZ;  5'b00010: alu_op=`ALU_CPOP;
                                    5'b00100: alu_op=`ALU_SEXTB; 5'b00101: alu_op=`ALU_SEXTH;
                                    default:  alu_op=`ALU_CLZ;
                                  endcase
                       default:   alu_op=`ALU_SLL;
                     endcase
             3'b101: case (f6)                                            // SRLI/SRAI / Zbs / Zbb
                       6'b010000: alu_op=`ALU_SRA; 6'b010010: alu_op=`ALU_BEXT;
                       6'b011000: alu_op=`ALU_ROR; 6'b001010: alu_op=`ALU_ORCB;
                       6'b011010: alu_op=`ALU_REV8; default: alu_op=`ALU_SRL;
                     endcase
           endcase
        end

        5'b01100: begin // OP-REG (+ Zba/Zbb/Zbs/Zicond, M)
           if (f7==7'b0000001) is_mul=1;                                 // MUL/DIV (deferred)
           else case (f3)
             3'b000: alu_op = (f7==7'b0100000)? `ALU_SUB : `ALU_ADD;
             3'b001: case (f7) 7'b0110000:alu_op=`ALU_ROL; 7'b0100100:alu_op=`ALU_BCLR;
                               7'b0110100:alu_op=`ALU_BINV; 7'b0010100:alu_op=`ALU_BSET;
                               default:alu_op=`ALU_SLL; endcase
             3'b010: alu_op = (f7==7'b0010000)? `ALU_SH1ADD : `ALU_SLT;
             3'b011: alu_op = `ALU_SLTU;
             3'b100: case (f7) 7'b0010000:alu_op=`ALU_SH2ADD; 7'b0100000:alu_op=`ALU_XNOR;
                               7'b0000101:alu_op=`ALU_MIN; default:alu_op=`ALU_XOR; endcase
             3'b101: case (f7) 7'b0100000:alu_op=`ALU_SRA; 7'b0110000:alu_op=`ALU_ROR;
                               7'b0100100:alu_op=`ALU_BEXT; 7'b0000101:alu_op=`ALU_MINU;
                               7'b0000111:alu_op=`ALU_CZEQZ; default:alu_op=`ALU_SRL; endcase
             3'b110: case (f7) 7'b0010000:alu_op=`ALU_SH3ADD; 7'b0100000:alu_op=`ALU_ORN;
                               7'b0000101:alu_op=`ALU_MAX; default:alu_op=`ALU_OR; endcase
             3'b111: case (f7) 7'b0100000:alu_op=`ALU_ANDN; 7'b0000101:alu_op=`ALU_MAXU;
                               7'b0000111:alu_op=`ALU_CZNEZ; default:alu_op=`ALU_AND; endcase
           endcase
        end

        5'b00110: begin // OP-IMM-32 (+ Zba slli.uw / Zbb clzw..)
           op2_imm=1; alu_w=1;
           case (f3)
             3'b000: alu_op=`ALU_ADD;                                    // ADDIW
             3'b001: case (f6)
                       6'b000010: begin alu_op=`ALU_SLL; alu_uw=1; alu_w=0; end // SLLI.UW (64b)
                       6'b011000: case (insn[24:20])
                                    5'b00001: alu_op=`ALU_CTZ; 5'b00010: alu_op=`ALU_CPOP;
                                    default:  alu_op=`ALU_CLZ; endcase
                       default:   alu_op=`ALU_SLL;                       // SLLIW
                     endcase
             3'b101: case (f6) 6'b011000:alu_op=`ALU_ROR; 6'b010000:alu_op=`ALU_SRA;
                               default:alu_op=`ALU_SRL; endcase           // RORIW/SRAIW/SRLIW
             default: illegal=1;
           endcase
        end

        5'b01110: begin // OP-REG-32 (+ Zba add.uw/sh*add.uw / Zbb / M-W)
           alu_w=1;
           if (f7==7'b0000001) begin is_mul=1; end                      // MULW/DIVW (deferred)
           else case (f3)
             3'b000: case (f7) 7'b0100000:alu_op=`ALU_SUB;
                               7'b0000100: begin alu_op=`ALU_ADD; alu_uw=1; alu_w=0; end // ADD.UW
                               default:alu_op=`ALU_ADD; endcase
             3'b001: case (f7) 7'b0110000:alu_op=`ALU_ROL; default:alu_op=`ALU_SLL; endcase
             3'b010: begin alu_op=`ALU_SH1ADD; alu_uw=1; alu_w=0; end    // SH1ADD.UW
             3'b100: case (f7) 7'b0000100: begin alu_op=`ALU_ZEXTH; alu_w=0; end // ZEXT.H
                               default: begin alu_op=`ALU_SH2ADD; alu_uw=1; alu_w=0; end endcase
             3'b101: case (f7) 7'b0110000:alu_op=`ALU_ROR; 7'b0100000:alu_op=`ALU_SRA;
                               default:alu_op=`ALU_SRL; endcase
             3'b110: begin alu_op=`ALU_SH3ADD; alu_uw=1; alu_w=0; end    // SH3ADD.UW
             default: illegal=1;
           endcase
        end

        5'b00000: begin is_mem=1; op2_imm=1; alu_op=`ALU_ADD;            // LOAD
                        mem_size=f3[1:0]; mem_signed=~f3[2];
                        if (f3==3'b111) illegal=1; end                   // funct3=111 reserved
        5'b01000: begin is_mem=1; is_store=1; op2_imm=1; alu_op=`ALU_ADD;// STORE
                        mem_size=f3[1:0]; if (f3[2]) illegal=1; end
        5'b11000: begin is_branch=1; br_func=f3;                         // BRANCH
                        if (f3==3'b010||f3==3'b011) illegal=1; end

        5'b11100: begin // SYSTEM
           if (f3==3'b000) is_serialize=1;                              // ECALL/EBREAK/xRET/SFENCE/WFI
           else if (f3==3'b100) illegal=1;                              // reserved
           else begin is_csr=1; csr_func=f3; is_serialize=1; end        // CSRRW/S/C (+imm)
                      // every CSR op serializes (issues only when oldest) -- simplest
                      // correct rule; a read-only CSR is rare enough that the drain is free
        end
        5'b00011: if (f3==3'b001) begin is_serialize=1; is_fencei=1; end // FENCE.I: serialize +
                      // refetch (the store-to-instruction must be visible to the refetch). Plain
                      // FENCE (f3=000) stays a NOP (single hart, in-order commit -> barrier free).

        5'b01011: begin                                                // AMO (A ext)
           is_amo=1; is_serialize=1;            // serialized + solo (gate to oldest)
           op2_imm=1; alu_op=`ALU_ADD;          // AGU = rs1 + 0 (addr = rs1)
           mem_size=f3[1:0]; mem_signed=1'b1;   // .W=2/.D=3; rd sign-extends (.W)
           if (f3!=3'b010 && f3!=3'b011) illegal=1;
        end
        5'b10100: is_fp=1;                                              // OP-FP (deferred)
        5'b00001, 5'b01001: begin is_fp=1; is_mem=1; is_store=opc[3];   // F/D LOAD/STORE-FP
                                  op2_imm=1; alu_op=`ALU_ADD;
                                  mem_size=f3[1:0]; mem_signed=1'b0; end // FLW=W FLD=D; no sign-ext
        5'b10000,5'b10001,5'b10010,5'b10011: is_fp=1;                   // FMADD/FMSUB/FNMSUB/FNMADD
        default: illegal=1;
      endcase

      // An illegal instruction raises a precise trap (cause 2) instead of executing;
      // neutralize every op-type flag so it carries no side effect (mem/branch/csr/...)
      // -- it issues as a harmless ADD whose only role is to flag the trap downstream.
      if (illegal) begin
         is_mem=0; is_store=0; is_branch=0; is_jump=0; res_link=0;
         is_csr=0; is_serialize=0; is_mul=0; is_amo=0; is_fp=0; is_fencei=0;
      end
   end
endmodule

`default_nettype wire
