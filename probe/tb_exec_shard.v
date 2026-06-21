`timescale 1ns/1ps
`default_nettype none

// exec_shard: RF read -> exec_alu -> writeback, with write-before-read forwarding.
// This shard is lane 0 (owns pr%4==0); its wb_out loops back to wb_in[0]. Sibling
// lane 1 is driven directly to model a cross-shard producer (pr%4==1).
module tb;
   localparam SHARDS=4, SBITS=2, NPHYS=128, PBITS=7, POOL=32, IDXB=5;

   reg                clk=0; always #5 clk=~clk;
   reg                iss_valid, iss_pdst_v;
   reg  [PBITS-1:0]   iss_pdst, iss_ps1, iss_ps2;
   reg  [5:0]         alu_op; reg alu_w, alu_uw; reg [1:0] op1_sel; reg op2_imm, res_link, is_rvc, is_mem;
   reg  [63:0]        imm, pc;
   reg                sib1_v; reg [PBITS-1:0] sib1_pr; reg [63:0] sib1_val;
   wire               wb_valid; wire [PBITS-1:0] wb_pr; wire [63:0] wb_val;
   wire [63:0]        agu_addr; wire cmp_eq, cmp_lt, cmp_ltu;
   integer errs=0;

   wire [SHARDS-1:0]       wb_valid_in = {2'b00, sib1_v,  wb_valid};
   wire [SHARDS*PBITS-1:0] wb_pr_in    = {{2*PBITS{1'b0}}, sib1_pr,  wb_pr};
   wire [SHARDS*64-1:0]    wb_val_in   = {{2*64{1'b0}},    sib1_val, wb_val};

   exec_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
                .POOL(POOL), .IDXB(IDXB)) dut
     (.clk(clk), .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2),
      .alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw), .op1_sel(op1_sel),
      .op2_imm(op2_imm), .res_link(res_link), .is_rvc(is_rvc), .is_mem(is_mem),
      .imm(imm), .pc(pc),
      .wb_valid_in(wb_valid_in), .wb_pr_in(wb_pr_in), .wb_val_in(wb_val_in),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .agu_addr(agu_addr), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   // issue an op (defaults: ADD, op2=imm)
   task iss(input [PBITS-1:0] dst, input dv, input [PBITS-1:0] s1, input [PBITS-1:0] s2,
            input [1:0] o1s, input o2i, input [63:0] im);
      begin iss_valid=1; iss_pdst=dst; iss_pdst_v=dv; iss_ps1=s1; iss_ps2=s2;
            alu_op=6'd0; alu_w=0; alu_uw=0; op1_sel=o1s; op2_imm=o2i; res_link=0;
            is_rvc=0; is_mem=0; imm=im; pc=0; end
   endtask
   task noiss; begin iss_valid=0; iss_pdst_v=0; end endtask
   task ckv(input [127:0] nm, input [63:0] e);
      begin #1; if (wb_val!==e) begin $display("FAIL %0s wb_val=%h exp %h",nm,wb_val,e); errs=errs+1; end end
   endtask

   initial begin
      noiss; sib1_v=0; sib1_pr=0; sib1_val=0;
      @(negedge clk);

      // T0: producer  pr8 <- 100  (op1=ZERO, imm=100)
      @(negedge clk); iss(7'd8, 1'b1, 7'd0, 7'd0, 2'd2, 1'b1, 64'd100);
      ckv("prod", 64'd100);
      // T1: consumer  pr12 <- RF[8] + 5  -> 105 (write-before-read forwarding)
      @(negedge clk); iss(7'd12, 1'b1, 7'd8, 7'd0, 2'd0, 1'b1, 64'd5);
      ckv("self-fwd", 64'd105);

      // T2: sibling (shard1) writes pr9 <- 200 ; no local issue
      @(negedge clk); noiss; sib1_v=1; sib1_pr=7'd9; sib1_val=64'd200;
      // T3: consumer reads pr9 -> 201
      @(negedge clk); sib1_v=0; iss(7'd16, 1'b1, 7'd9, 7'd0, 2'd0, 1'b1, 64'd1);
      ckv("xshard-fwd", 64'd201);

      // T4: read pr8 again (still 100 from T0) -> +7 = 107  (persistence)
      @(negedge clk); iss(7'd20, 1'b1, 7'd8, 7'd0, 2'd0, 1'b1, 64'd7);
      ckv("persist", 64'd107);

      // T5: x0 reads as 0 : ADD x0-as-src + 9 -> 9
      @(negedge clk); iss(7'd24, 1'b1, 7'd0, 7'd0, 2'd0, 1'b1, 64'd9);
      ckv("ps0-zero", 64'd9);

      @(negedge clk); noiss;
      if (errs==0) $display("exec_shard: ALL TESTS PASSED");
      else         $display("exec_shard: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
