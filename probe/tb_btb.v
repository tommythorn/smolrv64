// tb_btb.v -- self-checking testbench for btb.v (branch-predictor step 1).
// Exercises: reset/unseen default, allocate-on-taken-miss, no-allocate-on-not-taken-miss,
// confirm climb (deterministic weak->strong, probabilistic strong->stronger->definite),
// asymmetric demote + flip-at-weak, unconditional always-taken, and field refresh.
`default_nettype none
`timescale 1ns/1ps

module tb;
   localparam PCW = 40, BBW = 3, TYPEW = 3;
   localparam [TYPEW-1:0] COND = 0, JMP = 1, CALL = 2, RET = 3;
   localparam WEAK = 0, STRONG = 1, STRONGER = 2, DEFINITE = 3;

   reg               clk = 0, reset = 1;
   reg  [PCW-1:0]    p_pc = 0;
   wire              p_hit, p_taken;
   wire [PCW-1:0]    p_target;
   wire [BBW-1:0]    p_bb_len;
   wire [TYPEW-1:0]  p_type;
   wire [1:0]        p_conf;

   reg               t_valid = 0;
   reg  [PCW-1:0]    t_pc = 0, t_target = 0;
   reg               t_taken = 0;
   reg  [BBW-1:0]    t_bb_len = 0;
   reg  [TYPEW-1:0]  t_type = 0;
   reg               climb_en = 0;

   btb #(.PCW(PCW), .NENTRY(256), .IDXW(8), .BBW(BBW), .TYPEW(TYPEW)) dut (
      .clk(clk), .reset(reset),
      .p_pc(p_pc), .p_hit(p_hit), .p_taken(p_taken), .p_target(p_target),
      .p_bb_len(p_bb_len), .p_type(p_type), .p_conf(p_conf),
      .t_valid(t_valid), .t_pc(t_pc), .t_taken(t_taken), .t_target(t_target),
      .t_bb_len(t_bb_len), .t_type(t_type), .climb_en(climb_en));

   always #5 clk = ~clk;

   integer pass = 0, fail = 0;
   task chk(input cond, input [255:0] msg);
      begin
         if (cond) begin pass = pass + 1; end
         else      begin fail = fail + 1; $display("  FAIL: %0s", msg); end
      end
   endtask

   // one clocked training event
   task train(input [PCW-1:0] pc, input tk, input [PCW-1:0] tgt,
              input [BBW-1:0] bb, input [TYPEW-1:0] ty, input ce);
      begin
         @(negedge clk);
         t_valid = 1; t_pc = pc; t_taken = tk; t_target = tgt;
         t_bb_len = bb; t_type = ty; climb_en = ce;
         @(negedge clk);
         t_valid = 0;
      end
   endtask

   // read the predictor combinationally for a given PC
   task predict(input [PCW-1:0] pc);
      begin p_pc = pc; #1; end
   endtask

   localparam [PCW-1:0] A = 40'h0002, B = 40'h0004, C = 40'h0006, D = 40'h0008;

   integer k;
   initial begin
      repeat (3) @(negedge clk);
      reset = 0;
      @(negedge clk);

      // 1) unseen default everywhere: miss, fall-through, definite
      predict(A);
      chk(p_hit == 0 && p_taken == 0 && p_conf == DEFINITE, "unseen A != (miss,NT,definite)");
      predict(D);
      chk(p_hit == 0 && p_taken == 0, "unseen D not a miss");

      // 2) allocate on TAKEN miss: a taken cond at A
      train(A, 1, 40'h1234, 3'd5, COND, 0);
      predict(A);
      chk(p_hit && p_taken && p_target == 40'h1234 && p_bb_len == 3'd5 &&
          p_type == COND && p_conf == WEAK, "A after taken-alloc != (hit,T,tgt,len,cond,weak)");

      // 3) NOT-taken miss allocates nothing: a not-taken cond at B stays unseen
      train(B, 0, 40'hBEEF, 3'd2, COND, 0);
      predict(B);
      chk(p_hit == 0 && p_conf == DEFINITE, "B not-taken-miss wrongly allocated");

      // 4) confirm climb. weak->strong is deterministic even with climb_en=0
      train(A, 1, 40'h1234, 3'd5, COND, 0);
      predict(A); chk(p_conf == STRONG, "A confirm weak->strong (det) failed");
      // strong stays without climb_en
      train(A, 1, 40'h1234, 3'd5, COND, 0);
      predict(A); chk(p_conf == STRONG, "A strong climbed without climb_en");
      // strong->stronger->definite require climb_en
      train(A, 1, 40'h1234, 3'd5, COND, 1);
      predict(A); chk(p_conf == STRONGER, "A strong->stronger w/ climb_en failed");
      train(A, 1, 40'h1234, 3'd5, COND, 1);
      predict(A); chk(p_conf == DEFINITE, "A stronger->definite w/ climb_en failed");
      // definite saturates
      train(A, 1, 40'h1234, 3'd5, COND, 1);
      predict(A); chk(p_conf == DEFINITE, "A definite did not saturate");

      // 5) asymmetric demote: at definite, a single miss -> strong; miss -> weak; miss(weak) -> flip
      train(A, 0, 40'h1234, 3'd5, COND, 0);     // miss (predicted T, was NT)
      predict(A); chk(p_conf == STRONG && p_taken == 1, "A definite+miss != strong (still T)");
      train(A, 0, 40'h1234, 3'd5, COND, 0);     // miss -> weak
      predict(A); chk(p_conf == WEAK && p_taken == 1, "A strong+miss != weak (still T)");
      train(A, 0, 40'h0AA0, 3'd1, COND, 0);     // miss at weak -> FLIP to NT
      predict(A);
      chk(p_hit && p_taken == 0 && p_conf == WEAK, "A weak+miss did not flip to NT/weak");
      // now confirming NT climbs again
      train(A, 0, 40'h0AA0, 3'd1, COND, 0);
      predict(A); chk(p_taken == 0 && p_conf == STRONG, "A NT confirm weak->strong failed");

      // 6) unconditional (JMP) is always taken and climbs on every confirm
      train(C, 1, 40'h7777, 3'd4, JMP, 1);      // allocate
      predict(C); chk(p_hit && p_taken && p_type == JMP && p_conf == WEAK, "C jmp alloc failed");
      train(C, 1, 40'h7777, 3'd4, JMP, 0);
      predict(C); chk(p_taken && p_conf == STRONG, "C jmp weak->strong failed");
      train(C, 1, 40'h8888, 3'd4, JMP, 1);      // target refresh + climb
      predict(C); chk(p_taken && p_target == 40'h8888 && p_conf == STRONGER, "C jmp refresh/climb failed");

      // 7) field refresh on a cond hit (bb_len + target track the latest resolve)
      train(A, 0, 40'h0BB0, 3'd7, COND, 0);
      predict(A); chk(p_target == 40'h0BB0 && p_bb_len == 3'd7, "A field refresh failed");

      $display("\n=== tb_btb: %0d passed, %0d failed ===", pass, fail);
      if (fail == 0) $display("ALL TESTS PASSED"); else $display("Test FAILED");
      $finish;
   end

   initial begin
      #100000 $display("TIMEOUT"); $finish;
   end
endmodule

`default_nettype wire
