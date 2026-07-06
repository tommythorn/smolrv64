`timescale 1ns/1ps
`default_nettype none

// branch_unit resolution tests: each branch condition (taken/not), JAL, JALR.
module tb;
   reg        is_branch, is_jump, is_jalr;
   reg  [2:0] br_func;
   reg        cmp_eq, cmp_lt, cmp_ltu;
   reg  [63:0] pc, imm, agu_addr;
   wire       redirect;
   wire [63:0] target;
   integer errs=0;

   // pred_npc = fall-through (a never-predicting frontend): the redirect/target
   // expectations below are exactly the old (is_branch & taken) | is_jump ones.
   // mis_taken/mis_nt arrive precomputed (RR-time in the real pipe).
   wire [63:0] pred_npc  = pc + 64'd4;
   wire        mis_taken = (pc + imm) != pred_npc;
   wire        mis_nt    = (pc + 64'd4) != pred_npc;
   branch_unit dut
     (.is_branch(is_branch), .is_jump(is_jump), .is_jalr(is_jalr), .is_rvc(1'b0),
      .br_func(br_func),
      .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu),
      .pc(pc), .imm(imm), .agu_addr(agu_addr),
      .mis_taken(mis_taken), .mis_nt(mis_nt), .pred_npc(pred_npc),
      .redirect(redirect), .target(target), .taken_o(), .taken_tgt());

   task chk(input [127:0] nm, input er, input [63:0] et);
      begin #1;
        if (redirect!==er) begin $display("FAIL %0s redirect=%b exp %b",nm,redirect,er); errs=errs+1; end
        else if (er && target!==et) begin $display("FAIL %0s target=%h exp %h",nm,target,et); errs=errs+1; end
      end
   endtask
   task clr; begin is_branch=0;is_jump=0;is_jalr=0;br_func=0;cmp_eq=0;cmp_lt=0;cmp_ltu=0;
                   pc=64'h1000; imm=64'h40; agu_addr=64'h2223; end endtask

   initial begin
      // BEQ taken / not
      clr; is_branch=1; br_func=3'b000; cmp_eq=1; chk("BEQ-taken",1,64'h1040);
      clr; is_branch=1; br_func=3'b000; cmp_eq=0; chk("BEQ-nottaken",0,0);
      // BNE
      clr; is_branch=1; br_func=3'b001; cmp_eq=0; chk("BNE-taken",1,64'h1040);
      clr; is_branch=1; br_func=3'b001; cmp_eq=1; chk("BNE-nottaken",0,0);
      // BLT / BGE (signed)
      clr; is_branch=1; br_func=3'b100; cmp_lt=1; chk("BLT-taken",1,64'h1040);
      clr; is_branch=1; br_func=3'b101; cmp_lt=0; chk("BGE-taken",1,64'h1040);
      // BLTU / BGEU
      clr; is_branch=1; br_func=3'b110; cmp_ltu=1; chk("BLTU-taken",1,64'h1040);
      clr; is_branch=1; br_func=3'b111; cmp_ltu=0; chk("BGEU-taken",1,64'h1040);
      // JAL: always redirect, target = pc+imm
      clr; is_jump=1; chk("JAL",1,64'h1040);
      // JALR: always redirect, target = (rs1+imm)&~1 = agu_addr&~1
      clr; is_jump=1; is_jalr=1; chk("JALR",1,64'h2222);   // 0x2223 & ~1

      if (errs==0) $display("branch_unit: ALL TESTS PASSED");
      else         $display("branch_unit: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
