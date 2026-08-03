`timescale 1ns/1ps
`default_nettype none

// CPR reclamation milestone: a long dependent chain that allocates far more
// physical registers than the pool holds, so it can only complete if committed
// checkpoints reclaim their dead registers and free their ring slots.
//
// Program: N x  `addi x1,x1,1`  (0x00108093). x1 is arch reg 1 -> owned by shard
// 1, so shard 1 allocates a NEW physreg every instruction and the prior mapping
// dies immediately (a pure pold-reclamation stress). Within each 4-wide bundle the
// four addis form a RAW+WAW chain, exercising the cross-slot src bypass (SLOT) and
// the intra-bundle pold (d_is_slot). The chain serialises issue (1/cycle), so with
// NCHK=4 checkpoints the ring fills and the frontend back-pressures -- the run only
// finishes (x1 reaches N) if commit frees both registers and checkpoints.
//
// shard 1 pool = 64 regs, 16 reserved (arch) -> 48 free. N=64 >> 48, so the run is
// impossible without reclamation. Correctness: the writeback value stream must be
// exactly {1,2,...,N} (each instruction's result, each in a distinct physreg).
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=9, PBW=4, N=64;

   reg                clk=0; always #5 clk=~clk;
   reg                reset;
   wire               redirect;
   wire [PCW-1:0]     redirect_target, imem_addr;
   reg  [HW*16-1:0]   imem_data;
   wire [PBW-1:0]     imem_avail = 4'd8;
   wire [IW-1:0]      wb_valid;
   wire [IW*PBITS-1:0] wb_pr;
   wire [IW*64-1:0]   wb_val;
   wire               commit;
   wire [1:0]         commit_idx;
   integer errs=0, i, k, c, ncommit=0, maxv=0;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .hw_ip(12'd0), .mtime(64'd0), .dmem_rdata(64'd0), .dmem_rvalid(1'b1), .dmem_wready(1'b1), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target),
      .commit(commit), .commit_idx(commit_idx));

   reg [15:0] mem [0:511];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg seen [1:N];

   initial begin
      for (i=0;i<512;i=i+1) mem[i]=16'h0001;          // c.nop fill
      for (i=0;i<N;i=i+1) begin
         mem[2*i]   = 16'h8093;                       // addi x1,x1,1  (low half)
         mem[2*i+1] = 16'h0010;                       //               (high half)
      end
      for (i=1;i<=N;i=i+1) seen[i]=1'b0;

      reset=1; @(negedge clk); @(negedge clk); reset=0;

      // run; capture writeback values + count commits
      for (c=0; c<400; c=c+1) begin
         @(negedge clk);
         if (commit) ncommit = ncommit + 1;
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) begin
            if (wb_val[k*64+:64] >= 1 && wb_val[k*64+:64] <= N) begin
               seen[wb_val[k*64+:64]] = 1'b1;
               if (wb_val[k*64+:64] > maxv) maxv = wb_val[k*64+:64];
            end
         end
      end

      for (i=1;i<=N;i=i+1)
         if (!seen[i]) begin $display("FAIL: missing result x1=%0d", i); errs=errs+1; end

      $display("reclaim: max result=%0d (expect %0d), commits observed=%0d", maxv, N, ncommit);
      if (errs==0 && maxv==N && ncommit>0)
         $display("reclaim: ALL TESTS PASSED (ran %0d allocs through a %0d-deep pool)", N, 48);
      else begin
         if (ncommit==0) $display("FAIL: no commits -- CPR never reclaimed");
         $display("reclaim: %0d FAILURES", errs);
      end
      $finish;
   end
endmodule

`default_nettype wire
