`timescale 1ns/1ps
`default_nettype none
// tb_hpm -- csr_file's Zihpm/Sscofpmf counters against a cycle-exact model: 13 general counters
// (3..15, any event) and 16 fixed ones (16..31, counter 16+k counts one td_k cause), swept from
// accumulators into a LUTRAM.
//   1. RANDOM: random events, top-down causes, event writes, counter writes and mcountinhibit
//      writes; every cycle with no CSR op a random counter is read and must equal the model
//      exactly (a read is the swept value plus the accumulator).
//   2. WRAP: each counter preloaded just under 2^64 wraps and must set OF (scountovf) and LCOFIP,
//      also when it is written within a sweep of the wrap; a wrap with OF already set raises nothing.
//   3. SAMPLING: perf's overflow handler on a general and a fixed counter (stop it, read, preload
//      -P, clear OF, start): the samples must sum to every event counted while it ran -- none lost.
//      Then the same without stopping: the loss is reported (the read-to-write window).
module tb_hpm;
   reg clk = 1'b0, reset = 1'b1;
   always #5 clk = ~clk;

   localparam integer HPMG = 13, HPMN = 29;
   localparam [11:0] MHPMEVENT3 = 12'h323, MHPMCOUNTER3 = 12'hB03, MCOUNTINHIBIT = 12'h320,
                     SCOUNTOVF = 12'hDA0, MIP = 12'h344;

   reg  [11:0] raddr = 12'h000;
   wire [63:0] rdata;
   reg  [43:0] hpm_ev = 44'd0;
   reg  [5:0]  ret = 6'd0, lqocc = 6'd0, sqocc = 6'd0, ldinfl = 6'd0;
   reg  [2:0]  disp = 3'd0;
   reg  [4:0]  tdk = 5'd0;
   reg         upd_valid = 1'b0;
   reg  [11:0] upd_addr = 12'd0;
   reg  [63:0] upd_src = 64'd0;

   wire [39:0] w40;  wire w1a, w1b, w1c, w1d, w1e, w1f, w1g, w1h, w1i, w1j, w1k;
   wire [63:0] w64a, w64b, w64c, w64d;  wire [1:0] w2a, w2b;  wire [2:0] w3;  wire [3:0] w4;
   csr_file dut (
      .clk(clk), .reset(reset), .raddr(raddr), .rdata(rdata),
      .redir_target(w40), .redir_valid(w1a), .redir_is_trap(w1b), .csr_illegal(w1c),
      .o_satp(w64a), .o_priv(w2a), .o_ifx_d(w1d), .o_dpriv(w2b), .o_sum(w1e), .o_mxr(w1f),
      .o_frm(w3), .o_fs_off(w1g), .fp_fflags_we(1'b0), .fp_fflags(5'd0), .fp_dirty_commit(1'b0),
      .o_tlb_flush(w1h), .xtrap_v(1'b0), .xtrap_intr(1'b0), .xtrap_cause(4'd0), .xtrap_epc(40'd0),
      .xtrap_tval(64'd0), .hw_ip(12'd0), .mtime(64'd0), .retire_cnt(ret), .hpm_retire_cnt(ret),
      .hpm_ev(hpm_ev), .hpm_lqocc(lqocc), .hpm_sqocc(sqocc), .hpm_ldinfl(ldinfl), .hpm_disp(disp),
      .hpm_tdk(tdk), .dbg_timer(w64b), .dbg_mtvec_we(w1i), .dbg_csrop_v(w1j), .dbg_csrop(w64c),
      .dbg_mtvec(w64d), .irq_v(w1k), .irq_cause(w4),
      .upd_valid(upd_valid), .upd_is_csr(1'b1), .upd_func(3'b001), .upd_addr(upd_addr),
      .upd_src(upd_src), .upd_pc(40'd0));

   // ---------------- the model ----------------
   reg  [15:0] code [0:HPMN-1];
   reg         minh [0:HPMN-1];
   reg         hold [0:HPMN-1];            // a general counter holds the cycle after its event write
   reg         wct  [0:HPMN-1];            // a counter write's load cycle (the op was last cycle)
   reg  [63:0] wval [0:HPMN-1];
   reg  [64:0] gold [0:HPMN-1];            // the count; [64] unused
   reg         ofm  [0:HPMN-1];            // a wrap happened since OF was last written
   reg  [63:0] tally [0:HPMN-1];           // every event counted while running, writes or not
   reg  [63:0] inhibit = 64'd0;
   reg         tdk_rand = 1'b0;            // a background driver keeps td_k random
   always @(negedge clk) if (tdk_rand) tdk = 5'($urandom_range(0, 22));
   integer     i, errors = 0, checks = 0;

   localparam [79:0] HPM_TDK = {5'd20, 5'd19, 5'd18, 5'd17, 5'd16, 5'd15, 5'd14, 5'd13,
                                5'd12, 5'd10, 5'd8,  5'd7,  5'd6,  5'd5,  5'd4,  5'd3};
   function [4:0] fixk(input integer n); fixk = HPM_TDK[5*(n-HPMG) +: 5]; endfunction
   function [15:0] fixc(input integer n); fixc = {11'h028, fixk(n)}; endfunction
   function [5:0] val(input integer n);    // counter n's event this cycle, if it runs
      reg [15:0] c;
      reg [4:0]  t;
      begin
         c = code[n];  t = (n >= HPMG) ? HPM_TDK[5*(n-HPMG) +: 5] : 5'd0;
         if (n >= HPMG) val = (c == {11'h028, t}) ? {5'd0, tdk == t} : 6'd0;
         else case (c)
            16'h0001: val = 6'd1;
            16'h0002: val = ret;
            16'h0003: val = {5'd0, hpm_ev[0]};
            16'h0004: val = {5'd0, hpm_ev[1]};
            16'h0321: val = lqocc;
            16'h0408: val = {3'd0, disp};
            default:  val = (c[15:8] == 8'h05 && c[7:0] < 8'd23) ? {5'd0, tdk == c[4:0]} : 6'd0;
         endcase
      end
   endfunction
   function run(input integer n);
      run = ~inhibit[n + 3] & ~minh[n] & ~hold[n];
   endfunction

   // The model steps at each edge with the inputs and the op of the cycle that edge closes.
   reg  [64:0] mnx;
   reg  [5:0]  minc;
   reg  [11:0] eva, cta;
   always @(posedge clk) if (!reset) begin
      for (i = 0; i < HPMN; i = i + 1) begin
         minc = run(i) ? val(i) : 6'd0;
         mnx  = (wct[i] ? {1'b0, wval[i]} : gold[i]) + {59'd0, minc};
         if (mnx[64]) ofm[i] <= 1'b1;
         gold[i]  <= {1'b0, mnx[63:0]};
         tally[i] <= tally[i] + {58'd0, minc};
         eva = MHPMEVENT3 + i[11:0];  cta = MHPMCOUNTER3 + i[11:0];
         wct[i]  <= upd_valid && upd_addr == cta;
         if (upd_valid && upd_addr == cta) wval[i] <= upd_src;
         hold[i] <= (i < HPMG) && upd_valid && upd_addr == eva;
         if (upd_valid && upd_addr == eva) begin
            code[i] <= (i < HPMG) ? upd_src[15:0] : (upd_src[15:0] == fixc(i) ? upd_src[15:0] : 16'd0);
            minh[i] <= upd_src[62];
            ofm[i]  <= upd_src[63];
         end
      end
      if (upd_valid && upd_addr == MCOUNTINHIBIT) inhibit <= upd_src & 64'hFFFF_FFFF_FFFF_FFFD;
   end

   // ---------------- drivers ----------------
   task automatic op(input [11:0] a, input [63:0] v);
      begin
         @(negedge clk); upd_valid = 1'b1; upd_addr = a; upd_src = v; raddr = a;
         @(negedge clk); upd_valid = 1'b0;
      end
   endtask
   task automatic idle(input integer n);
      repeat (n) @(negedge clk);
   endtask
   task automatic rnd_inputs;
      begin
         hpm_ev = {$urandom, $urandom};  ret = 6'($urandom_range(0, 4));
         lqocc = 6'($urandom_range(0, 63));  sqocc = 6'($urandom_range(0, 40));
         disp = 3'($urandom_range(0, 4));  tdk = 5'($urandom_range(0, 22));
      end
   endtask
   // read counter n now (combinational) and compare with the model
   task automatic check_ctr(input integer n, input [255:0] what);
      begin
         raddr = MHPMCOUNTER3 + 12'(n);  #1;
         checks = checks + 1;
         if (rdata !== gold[n][63:0]) begin
            errors = errors + 1;
            if (errors < 20) $display("FAIL %0s: counter %0d reads %h, model %h", what, n + 3, rdata, gold[n][63:0]);
         end
      end
   endtask
   reg [63:0] rd;                               // what read() saw
   task automatic read(input [11:0] a);
      begin raddr = a; #1; rd = rdata; end
   endtask

   function [15:0] gen_ev(input integer j);    // the general counters' events (0x0099: none)
      case (j % 8)
         0: gen_ev = 16'h0001;  1: gen_ev = 16'h0002;  2: gen_ev = 16'h0003;  3: gen_ev = 16'h0004;
         4: gen_ev = 16'h0321;  5: gen_ev = 16'h0408;  6: gen_ev = 16'h050f;  default: gen_ev = 16'h0099;
      endcase
   endfunction
   integer cyc, n, k, s, u1, u2;
   reg [63:0] v, r, P, sw_sum, truth, ovf;
   reg        fixed;

   initial begin
      for (i = 0; i < HPMN; i = i + 1) begin
         code[i] = 16'd0; minh[i] = 1'b0; hold[i] = 1'b0; wct[i] = 1'b0; wval[i] = 64'd0;
         gold[i] = 65'd0; ofm[i] = 1'b0; tally[i] = 64'd0;
      end
      repeat (4) @(negedge clk);
      reset = 1'b0;
      idle(40);                                   // the zeroing round
      for (n = 0; n < HPMN; n = n + 1) check_ctr(n, "after reset");

      // ---- 1. RANDOM ----
      for (n = 0; n < HPMN; n = n + 1)
         op(MHPMEVENT3 + 12'(n), {48'd0, (n < HPMG) ? gen_ev(n) : fixc(n)});
      for (cyc = 0; cyc < 200000; cyc = cyc + 1) begin
         @(negedge clk);
         rnd_inputs;
         upd_valid = 1'b0;
         k = $urandom_range(0, 99);
         if (k < 3) begin                          // a counter write: small, random, or near the top
            n = $urandom_range(0, HPMN - 1);
            s = $urandom_range(0, 2);
            u1 = $urandom_range(0, 1000);  u2 = $urandom_range(0, 300);
            v = {$urandom, $urandom};
            if (s == 0) v = 64'(u1);
            else if (s == 2) v = 64'hFFFF_FFFF_FFFF_FFFF - 64'(u2);
            upd_valid = 1'b1; upd_addr = MHPMCOUNTER3 + 12'(n); upd_src = v; raddr = upd_addr;
         end else if (k < 5) begin                 // an event write: a code, sometimes MINH or a wrong code
            n = $urandom_range(0, HPMN - 1);
            u1 = $urandom_range(0, 9);  u2 = $urandom_range(0, 7);  s = $urandom_range(0, 4);
            v = {1'b0, (u1 == 0), 46'd0, (n < HPMG) ? gen_ev(u2) : (s == 0 ? 16'h0001 : fixc(n))};
            upd_valid = 1'b1; upd_addr = MHPMEVENT3 + 12'(n); upd_src = v; raddr = upd_addr;
         end else if (k < 6) begin                 // mcountinhibit: a few counters stopped
            v = {$urandom, $urandom};  r = {$urandom, $urandom};  v = v & r;  r = {$urandom, $urandom};  v = v & r;
            upd_valid = 1'b1; upd_addr = MCOUNTINHIBIT; upd_src = v; raddr = upd_addr;
         end else
         begin n = $urandom_range(0, HPMN - 1); check_ctr(n, "random"); end
      end
      @(negedge clk); upd_valid = 1'b0;
      idle(2);
      for (n = 0; n < HPMN; n = n + 1) check_ctr(n, "end of random");
      $display("random: %0d reads checked, %0d errors", checks, errors);

      // ---- 2. WRAP ----
      op(MIP, 64'd0);
      op(MCOUNTINHIBIT, 64'd0);
      tdk = 5'd0; hpm_ev = 44'd0; ret = 6'd0; lqocc = 6'd0; disp = 3'd0;
      for (n = 0; n < HPMN; n = n + 1) begin
         fixed = (n >= HPMG);
         op(MHPMEVENT3 + 12'(n), {48'd0, fixed ? fixc(n) : 16'h0001});   // OF clear; cycles or its cause
         op(MHPMCOUNTER3 + 12'(n), 64'hFFFF_FFFF_FFFF_FFF0);
         op(MIP, 64'd0);
         // feed its event until it wraps; for one in three, overwrite it right after the wrap
         for (cyc = 0; cyc < 40; cyc = cyc + 1) begin @(negedge clk); tdk = fixed ? HPM_TDK[5*(n-HPMG) +: 5] : 5'd0; end
         if (n % 3 == 0) op(MHPMCOUNTER3 + 12'(n), 64'd5);
         tdk = 5'd0;
         idle(80);                                 // a sweep and more: OF must be in
         read(SCOUNTOVF); ovf = rd;
         if (!ovf[n + 3] || !ofm[n]) begin errors = errors + 1; $display("FAIL wrap: counter %0d OF %b (model %b)", n + 3, ovf[n + 3], ofm[n]); end
         read(MIP); r = rd;
         if (!r[13]) begin errors = errors + 1; $display("FAIL wrap: counter %0d raised no LCOFIP", n + 3); end
         // with OF still set a second wrap raises nothing
         op(MIP, 64'd0);
         op(MHPMCOUNTER3 + 12'(n), 64'hFFFF_FFFF_FFFF_FFF0);
         for (cyc = 0; cyc < 40; cyc = cyc + 1) begin @(negedge clk); tdk = fixed ? HPM_TDK[5*(n-HPMG) +: 5] : 5'd0; end
         tdk = 5'd0;
         idle(80);
         read(MIP); r = rd;
         if (r[13]) begin errors = errors + 1; $display("FAIL wrap: counter %0d raised LCOFIP with OF set", n + 3); end
         op(MHPMEVENT3 + 12'(n), 64'd0);
      end
      $display("wrap: %0d counters, %0d errors so far", HPMN, errors);

      // ---- 3. SAMPLING ----
      for (s = 0; s < 4; s = s + 1) begin : samp
         reg stop;
         stop  = (s < 2);                          // 0,1: perf's handler stops the counter; 2,3: it does not
         n     = (s % 2 == 0) ? 2 : HPMG + 10;      // a general counter (counting td_k == 15), a fixed one
         P     = 64'd0 - 64'd997;                  // the sampling period
         op(MCOUNTINHIBIT, 64'd0);
         op(MIP, 64'd0);
         op(MHPMEVENT3 + 12'(n), {48'd0, (n < HPMG) ? 16'h050f : fixc(n)});
         op(MHPMCOUNTER3 + 12'(n), P);
         sw_sum = 64'd0;  truth = tally[n];
         tdk_rand = 1'b1;
         for (k = 0; k < 60; ) begin
            @(negedge clk);
            read(SCOUNTOVF); ovf = rd;
            if (ovf[n + 3]) begin                  // the interrupt: the handler
               k = k + 1;
               if (stop) op(MCOUNTINHIBIT, 64'd1 << (n + 3));
               read(MHPMCOUNTER3 + 12'(n)); r = rd;
               sw_sum = sw_sum + (r - P);
               op(MHPMCOUNTER3 + 12'(n), P);
               op(MHPMEVENT3 + 12'(n), {48'd0, (n < HPMG) ? 16'h050f : fixc(n)});   // OF cleared
               op(MIP, 64'd0);
               if (stop) op(MCOUNTINHIBIT, 64'd0);
            end
         end
         tdk_rand = 1'b0;
         @(negedge clk); tdk = 5'd0;
         idle(2);
         truth = tally[n] - truth;
         // what the counter holds now has not been sampled yet
         read(MHPMCOUNTER3 + 12'(n)); r = rd;
         sw_sum = sw_sum + (r - P);
         if (stop) begin
            if (sw_sum != truth) begin errors = errors + 1; $display("FAIL sampling (stopped) counter %0d: samples sum %0d, events %0d", n + 3, sw_sum, truth); end
            else $display("sampling (stopped) counter %0d: %0d samples, %0d events, none lost", n + 3, k, truth);
         end else
            $display("sampling (running) counter %0d: %0d samples, %0d events, %0d lost (the read-to-write windows)",
                     n + 3, k, truth, $signed(truth - sw_sum));
      end

      if (errors == 0) $display("HPM-TB PASS: %0d reads checked", checks);
      else             $display("HPM-TB FAIL: %0d errors", errors);
      $finish;
   end
endmodule
`default_nettype wire
