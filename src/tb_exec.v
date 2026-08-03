`timescale 1ns/1ps
`default_nettype none

// End-to-end execute test: insn -> decode_operands (imm) + decode_exec (ctl)
// -> exec_alu, on real 32-bit encodings with hand-computed expectations.
module tb;
   reg  [31:0] insn;
   reg  [63:0] rs1v, rs2v, pc;
   wire [63:0] imm; wire dummy_himm, dummy_legal, dummy_rdv, dummy_r1v, dummy_r2v;
   wire [5:0]  dummy_rd, dummy_rs1, dummy_rs2;
   // decode_exec ctl
   wire [5:0]  alu_op; wire alu_w, alu_uw; wire [1:0] op1_sel; wire op2_imm, res_link;
   wire is_mem, is_store; wire [1:0] mem_size; wire mem_signed;
   wire is_branch; wire [2:0] br_func; wire is_jump;
   wire is_csr; wire [2:0] csr_func; wire is_serialize, is_mul, is_amo, is_fp, illegal;
   // exec result
   wire [63:0] result, addr; wire cmp_eq, cmp_lt, cmp_ltu;
   integer errs=0;

   decode_operands uo (.insn(insn), .rd(dummy_rd), .rd_v(dummy_rdv),
      .rs1(dummy_rs1), .rs1_v(dummy_r1v), .rs2(dummy_rs2), .rs2_v(dummy_r2v),
      .imm(imm), .has_imm(dummy_himm), .legal(dummy_legal));

   decode_exec ux (.insn(insn), .alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw),
      .op1_sel(op1_sel), .op2_imm(op2_imm), .res_link(res_link),
      .is_mem(is_mem), .is_store(is_store), .mem_size(mem_size), .mem_signed(mem_signed),
      .is_branch(is_branch), .br_func(br_func), .is_jump(is_jump),
      .is_csr(is_csr), .csr_func(csr_func), .is_serialize(is_serialize),
      .is_mul(is_mul), .is_amo(is_amo), .is_fp(is_fp), .illegal(illegal));

   exec_alu ue (.alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw), .op1_sel(op1_sel),
      .op2_imm(op2_imm), .res_link(res_link), .is_rvc(1'b0),
      .rs1_val(rs1v), .rs2_val(rs2v), .imm(imm), .pc(pc),
      .result(result), .addr(addr), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   task drive(input [31:0] ins, input [63:0] a, input [63:0] b, input [63:0] p);
      begin insn=ins; rs1v=a; rs2v=b; pc=p; #1; end
   endtask
   task ckr(input [127:0] nm, input [63:0] e);
      begin if (result!==e) begin $display("FAIL %0s result=%h exp %h",nm,result,e); errs=errs+1; end end
   endtask
   task cka(input [127:0] nm, input [63:0] e);
      begin if (addr!==e) begin $display("FAIL %0s addr=%h exp %h",nm,addr,e); errs=errs+1; end end
   endtask
   task ck1(input [127:0] nm, input got, input exp);
      begin if (got!==exp) begin $display("FAIL %0s flag=%b exp %b",nm,got,exp); errs=errs+1; end end
   endtask

   initial begin
      // ALU / immediate / upper-imm
      drive(32'h00310093, 64'd5, 64'd0, 0);              ckr("ADDI", 64'd8);
      drive(32'h123450B7, 0, 0, 0);                      ckr("LUI", 64'h0000_0000_1234_5000);
      drive(32'h00001097, 0, 0, 64'h8000_0000);         ckr("AUIPC", 64'h8000_1000);
      drive(32'h003100B3, 64'd10, 64'd20, 0);            ckr("ADD", 64'd30);
      drive(32'h403100B3, 64'd10, 64'd20, 0);            ckr("SUB", -64'd10);
      drive(32'h003120B3, -64'd1, 64'd1, 0);             ckr("SLT", 64'd1);
      drive(32'h4031F0B3, 64'hFF, 64'h0F, 0);            ckr("ANDN", 64'hF0);   // Zbb
      drive(32'h00411093, 64'd1, 64'd0, 0);              ckr("SLLI", 64'd16);

      // ADDIW (*W sign-extend): (0x7FFFFFFF + 1) -> 0x80000000 sign-extended
      drive(32'h0010809B, 64'h7FFF_FFFF, 0, 0);          ckr("ADDIW", 64'hFFFF_FFFF_8000_0000);

      // loads/stores: address only (= rs1 + imm), no value yet
      drive(32'h00812083, 64'h1000, 0, 0);
        ck1("LW.mem",is_mem,1); ck1("LW.store",is_store,0); ck1("LW.signed",mem_signed,1);
        cka("LW.addr", 64'h1008);
        if (mem_size!==2'd2) begin $display("FAIL LW.size"); errs=errs+1; end
      drive(32'h00312623, 64'h2000, 64'hDEAD, 0);
        ck1("SW.store",is_store,1); cka("SW.addr", 64'h200C);
        if (mem_size!==2'd2) begin $display("FAIL SW.size"); errs=errs+1; end

      // branch compare primitives (no redirect here)
      drive(32'h00208063, 64'd7, 64'd7, 0);
        ck1("BEQ.is_branch",is_branch,1); ck1("BEQ.eq",cmp_eq,1);
      drive(32'h00208063, 64'd7, 64'd8, 0); ck1("BNE.eq",cmp_eq,0);

      // JALR link = next_pc
      drive(32'h000100E7, 64'h40, 0, 64'h8000_0100);
        ck1("JALR.jump",is_jump,1); ckr("JALR.link", 64'h8000_0104); cka("JALR.tgt", 64'h40);

      // CSRRW -> csr + serialize; MUL -> deferred unit flag
      drive(32'h30011073, 64'd0, 0, 0);
        ck1("CSRRW.csr",is_csr,1); ck1("CSRRW.serialize",is_serialize,1);
        if (csr_func!==3'b001) begin $display("FAIL CSRRW.func"); errs=errs+1; end
      drive(32'h023100B3, 0, 0, 0);  ck1("MUL.is_mul",is_mul,1);

      if (errs==0) $display("exec: ALL TESTS PASSED");
      else         $display("exec: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
