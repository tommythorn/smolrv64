`timescale 1ns/1ps
// tb_rv_dcache: rv_dcache against a golden memory and a reference translation (stage 1: loads).
//
// MEMORY. A memory-port model with up to NOUT reads in flight, each with its own latency (a draw
// of LATMIN..LATMAX cycles), served in request order unless +reorder, in which case any response
// whose latency has passed may go first. A read returns its line in four 128-bit beats, in order.
// The request channel is back-pressured at random. Memory holds a fixed pattern of its address,
// so every answer is checkable.
//
// TRANSLATION. NVP virtual pages map onto NPP physical pages at random, so several virtual pages
// -- of different colours, VA[15:12] -- share one physical page (synonyms). Every REMAP cycles
// one mapping changes and the cache is told (ep_bump), as an sfence.vma would; wraps included.
//
// REQUESTERS. 16 tags; each has at most one read outstanding. A read is 1, 2, 4 or 8 bytes,
// naturally aligned, at a random offset of a random virtual page, with the PA of the mapping at
// issue. The answer must be that PA's bytes, under that tag, within TIMEOUT cycles.
`include "tb_rand.vh"
module tb;
   reg clk = 0, reset = 1;
   always #5 clk = ~clk;
   localparam NTAG = 16, NVP = 48, NPP = 24, NOUT = 8;
   localparam [63:0] PBASE = 64'h8000_0000;
   integer seed, cycles, reorder, latmin, latmax, remap, timeout;
   `TB_RAND(rnd, rs)

   // ---- the DUT
   reg         rd_req;  reg [63:0] rd_va, rd_pa;  reg [3:0] rd_tag;
   wire        rd_ack, rd_valid;  wire [63:0] rd_data, rd_resp_addr;  wire [3:0] rd_resp_tag;
   reg         ep_bump;  wire inv_busy;
   wire        cq_valid, cq_we;  reg cq_ready;  wire [3:0] cq_slot;  wire [57:0] cq_addr;
   wire [63:0] cq_wmask;  wire [511:0] cq_wdata;
   reg         cr_valid, cr_last;  reg [3:0] cr_slot;  reg [1:0] cr_beat;  reg [127:0] cr_data;
   reg         cw_valid;  reg [3:0] cw_slot;
   wire        perf_access, perf_miss;  wire [15:0] err;
   rv_dcache dut (.clk(clk), .reset(reset),
      .rd_req(rd_req), .rd_va(rd_va), .rd_pa(rd_pa), .rd_tag(rd_tag), .rd_ack(rd_ack),
      .rd_valid(rd_valid), .rd_data(rd_data), .rd_resp_tag(rd_resp_tag), .rd_resp_addr(rd_resp_addr),
      .ep_bump(ep_bump), .inv_busy(inv_busy),
      .cq_valid(cq_valid), .cq_ready(cq_ready), .cq_slot(cq_slot), .cq_we(cq_we), .cq_addr(cq_addr),
      .cq_wmask(cq_wmask), .cq_wdata(cq_wdata),
      .cr_valid(cr_valid), .cr_slot(cr_slot), .cr_beat(cr_beat), .cr_last(cr_last), .cr_data(cr_data),
      .cw_valid(cw_valid), .cw_slot(cw_slot),
      .perf_access(perf_access), .perf_miss(perf_miss), .err(err));

   // ---- the golden memory: 8 bytes per chunk, a function of the chunk's PA
   function [63:0] mem_chunk; input [63:0] pa; reg [63:0] a; begin
      a = {pa[63:3], 3'b000};
      mem_chunk = (a * 64'h9E37_79B9_7F4A_7C15) ^ {a[31:0], a[63:32]} ^ 64'h0123_4567_89AB_CDEF;
   end endfunction

   // ---- the translation
   reg [63:0] vp_va [0:NVP-1];                 // each virtual page's VA (distinct pages, all colours)
   reg [5:0]  vp_pp [0:NVP-1];                 // its physical page
   integer k;
   function [63:0] pp_pa; input [5:0] p; pp_pa = PBASE + {52'd0, p, 12'd0} * 64'd1; endfunction

   // ---- the memory model: outstanding reads, then their beats
   reg [57:0] mo_line [0:NOUT-1];  reg [3:0] mo_slot [0:NOUT-1];  reg [31:0] mo_due [0:NOUT-1];
   reg        mo_v [0:NOUT-1];  integer mo_seq [0:NOUT-1];  integer seqn;
   integer    beating;                          // the outstanding read whose beats are going out, or -1
   integer    bi;                               // its next beat
   reg [31:0] now;

   // ---- the requesters
   reg        out_v [0:NTAG-1];  reg [63:0] out_pa [0:NTAG-1];  reg [1:0] out_sz [0:NTAG-1];
   reg [31:0] out_t [0:NTAG-1];
   integer    n_req, n_resp, n_remap, n_miss, n_access, errors;
   integer    hp [0:7], ho [0:7], hw;     // the last 8 requests' page and offset: locality
   integer    c_rst, c_drop, c_merge, c_alloc, c_wait, c_wrap, n_out;   // coverage: every miss outcome occurs
   reg [63:0] expv, mask;  integer t, p, off, sz, pick, best;
   // acceptance is decided at the edge, by the pre-edge rd_ack
   reg        took;  reg [3:0] took_tag;
   always @(posedge clk) begin took <= rd_req & rd_ack & ~reset; took_tag <= rd_tag; end
   // ...and so is a memory request's
   reg        mq_took, mq_we;  reg [57:0] mq_addr;  reg [3:0] mq_slot;
   always @(posedge clk) begin
      mq_took <= cq_valid & cq_ready & ~reset;  mq_we <= cq_we;  mq_addr <= cq_addr;  mq_slot <= cq_slot;
   end

   initial begin
      if (!$value$plusargs("seed=%d", seed)) seed = 1;
      rs = `TB_SEED(seed);
      if (!$value$plusargs("cycles=%d", cycles)) cycles = 200000;
      reorder = $test$plusargs("reorder");
      if (!$value$plusargs("latmin=%d", latmin)) latmin = 20;
      if (!$value$plusargs("latmax=%d", latmax)) latmax = 60;
      if (!$value$plusargs("remap=%d", remap)) remap = 3000;
      if (!$value$plusargs("timeout=%d", timeout)) timeout = 5000;
      // pages: virtual pages spread over many colours; physical pages fewer, so synonyms abound
      for (k = 0; k < NVP; k = k + 1) begin
         vp_va[k] = 64'h0000_0010_0000_0000 + (64'(k * 37 + 5) << 12);   // distinct VPNs, every colour
         vp_pp[k] = rnd(0) % NPP;
      end
      for (k = 0; k < NTAG; k = k + 1) out_v[k] = 1'b0;
      for (k = 0; k < NOUT; k = k + 1) mo_v[k] = 1'b0;
      rd_req = 0; rd_va = 0; rd_pa = 0; rd_tag = 0; ep_bump = 0; cq_ready = 0;
      cr_valid = 0; cr_last = 0; cr_slot = 0; cr_beat = 0; cr_data = 0; cw_valid = 0; cw_slot = 0;
      n_req = 0; n_resp = 0; n_remap = 0; n_miss = 0; n_access = 0; errors = 0;
      for (k = 0; k < 8; k = k + 1) begin hp[k] = 0; ho[k] = 0; end
      hw = 0;
      c_rst = 0; c_drop = 0; c_merge = 0; c_alloc = 0; c_wait = 0; c_wrap = 0; seqn = 0; beating = -1; bi = 0; now = 0;
      repeat (3) @(posedge clk);
      reset = 0;
      while (dut.inv_busy) @(posedge clk);
      // run, then drain: every request taken is answered within TIMEOUT of the last one
      n_out = 1;
      for (now = 0; now < cycles || (n_out != 0 && now < cycles + timeout + 1); now = now + 1) begin
         @(negedge clk);
         // ---- check this cycle's response (registered by the DUT at the last edge)
         if (rd_valid) begin
            t = rd_resp_tag;
            if (!out_v[t]) begin $display("FAIL c=%0d: a response for tag %0d, which has nothing outstanding", now, t); errors = errors + 1; end
            else begin
               expv = mem_chunk(out_pa[t]) >> (8 * out_pa[t][2:0]);
               mask = (out_sz[t] == 3) ? ~64'd0 : ((64'd1 << (8 << out_sz[t])) - 1);
               if (((rd_data ^ expv) & mask) != 0) begin
                  $display("FAIL c=%0d tag %0d pa %h size %0d: got %h expected %h", now, t, out_pa[t], 1 << out_sz[t], rd_data & mask, expv & mask);
                  errors = errors + 1;
               end
               if (rd_resp_addr != out_pa[t]) begin $display("FAIL c=%0d tag %0d: response addr %h, issued %h", now, t, rd_resp_addr, out_pa[t]); errors = errors + 1; end
               out_v[t] = 1'b0;  n_resp = n_resp + 1;
            end
         end
         if (perf_access) n_access = n_access + 1;
         if (perf_miss)   n_miss = n_miss + 1;
         c_rst = c_rst + dut.m_restamp;  c_drop = c_drop + dut.m_drop;  c_merge = c_merge + dut.m_merge;
         c_alloc = c_alloc + dut.m_alloc;  c_wait = c_wait + dut.m_wait;  c_wrap = c_wrap + dut.ep_wrap;
         // ---- the request taken at the last edge
         if (took) begin out_v[took_tag] = 1'b1; out_t[took_tag] = now; n_req = n_req + 1; end
         // ---- a new request: a free tag, a random page and offset
         rd_req = 1'b0;
         t = rnd(0) % NTAG;
         if (now < cycles && !out_v[t] && !(took && took_tag == t[3:0]) && (rnd(0) % 4 != 0)) begin
            p   = rnd(0) % NVP;
            sz  = rnd(0) % 4;
            off = (rnd(0) % 4096) & ~((1 << sz) - 1);
            if (rnd(0) % 2 == 0) begin           // half the time, a recent request's line
               k = rnd(0) % 8;  p = hp[k];  off = (ho[k] & ~63) | (off & 63);
            end
            hp[hw] = p;  ho[hw] = off;  hw = (hw + 1) % 8;
            rd_req = 1'b1;  rd_tag = t[3:0];
            rd_va = vp_va[p] + off;  rd_pa = pp_pa(vp_pp[p]) + off;
            out_pa[t] = rd_pa;  out_sz[t] = sz[1:0];
         end
         // ---- a remap: one mapping changes, and the cache hears of it
         ep_bump = 1'b0;
         if ((now % remap) == remap - 1) begin
            p = rnd(0) % NVP;  vp_pp[p] = rnd(0) % NPP;
            ep_bump = 1'b1;  n_remap = n_remap + 1;
         end
         // ---- the memory model's response side (one beat per cycle)
         cr_valid = 1'b0;  cr_last = 1'b0;
         if (beating < 0) begin
            best = -1;
            for (k = 0; k < NOUT; k = k + 1)
               if (mo_v[k] && (mo_due[k] <= now) && (best < 0 || (!reorder && mo_seq[k] < mo_seq[best]) || (reorder && (rnd(0) & 1))))
                  best = k;
            // in order: only the oldest may answer, and only once due
            if (!reorder) begin
               pick = -1;
               for (k = 0; k < NOUT; k = k + 1) if (mo_v[k] && (pick < 0 || mo_seq[k] < mo_seq[pick])) pick = k;
               best = (pick >= 0 && mo_due[pick] <= now) ? pick : -1;
            end
            if (best >= 0) begin beating = best; bi = 0; end
         end
         if (beating >= 0) begin
            cr_valid = 1'b1;  cr_slot = mo_slot[beating];  cr_beat = bi[1:0];  cr_last = (bi == 3);
            cr_data = {mem_chunk({mo_line[beating], 6'd0} + 16 * bi + 8), mem_chunk({mo_line[beating], 6'd0} + 16 * bi)};
            bi = bi + 1;
            if (bi == 4) begin mo_v[beating] = 1'b0; beating = -1; end
         end
         // ---- the request side: back-pressured at random; a taken read gets a slot and a latency
         if (mq_took) begin
            if (mq_we) begin $display("FAIL c=%0d: a write request in stage 1", now); errors = errors + 1; end
            pick = -1;
            for (k = NOUT - 1; k >= 0; k = k - 1) if (!mo_v[k]) pick = k;
            if (pick < 0) begin $display("FAIL c=%0d: more than %0d reads outstanding", now, NOUT); errors = errors + 1; end
            else begin
               mo_v[pick] = 1'b1;  mo_line[pick] = mq_addr;  mo_slot[pick] = mq_slot;  mo_seq[pick] = seqn;  seqn = seqn + 1;
               mo_due[pick] = now + latmin + (rnd(0) % (latmax - latmin + 1));
            end
         end
         pick = 0;  for (k = 0; k < NOUT; k = k + 1) if (mo_v[k]) pick = pick + 1;
         cq_ready = (pick < NOUT) && (rnd(0) % 8 != 0);
         // ---- nobody waits forever
         for (k = 0; k < NTAG; k = k + 1)
            if (out_v[k] && (now - out_t[k] > timeout)) begin
               $display("FAIL c=%0d: tag %0d (pa %h) unanswered for %0d cycles", now, k, out_pa[k], timeout);
               errors = errors + 1;  out_v[k] = 1'b0;
            end
         if (errors > 10) begin $display("DCACHE-TB FAIL: too many errors"); $finish; end
         n_out = 0;
         for (k = 0; k < NTAG; k = k + 1) n_out = n_out + out_v[k];
         n_out = n_out + took + rd_req;
      end
      if (n_req != n_resp) begin $display("FAIL: %0d requests, %0d answered after the drain", n_req, n_resp); errors = errors + 1; end
      if (c_rst == 0 || c_drop == 0 || c_merge == 0 || c_alloc == 0 || c_wait == 0 || c_wrap == 0) begin
         $display("FAIL: a miss outcome never occurred"); errors = errors + 1;
      end
      $display("DCACHE-TB %s seed=%0d %s: %0d requests answered, %0d lookups, %0d misses (%0d re-stamp, %0d drop, %0d merge, %0d alloc, %0d wait), %0d remaps, %0d wraps",
               errors ? "FAIL" : "PASS", seed, reorder ? "reorder" : "in-order", n_resp, n_access, n_miss,
               c_rst, c_drop, c_merge, c_alloc, c_wait, n_remap, c_wrap);
      $finish;
   end
endmodule
