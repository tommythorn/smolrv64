`timescale 1ns/1ps
// tb_rv_l2: rv_l2 between a random requester and a memory model, against a golden memory.
//
// ABOVE. The arbiter's request register: one request presented, held until taken. Ids are 32,
// each with at most one transaction outstanding. A request is a read, a whole-line write (a D$
// write-back) or a partial write (an NC store), of a line of a pool -- half the time a recent one
// or one of its set -- in the DRAM window; now and then a read of the same lines' alias below the
// window (a wrong-path fetch of PA 0, which the port below serves from the window's wrap). As the
// caches guarantee, no write of a line is presented while anything else of that line is
// outstanding, and no read of a line while a write of it is. So `gold` is updated when a write is
// presented, and a read's beats must be gold's, in order, under its id, within TIMEOUT cycles.
//
// BELOW. Up to NOUT transactions in flight, each with its own latency (LATMIN..LATMAX), in request
// order unless +reorder. A read's line is the memory as it was when the read was taken, its four
// beats in order with gaps at random (+gaps percent); a write reaches the memory only when it
// completes. The request channel is back-pressured at random.
//
// DMA. Now and then a line of the pool is snooped (held until s_ack), as a DMA master's is once the
// D$ has answered for it: cleaned for a DMA read or flushed for a DMA write. While a snoop is
// pending no write of its line is presented. At the answer DRAM must hold gold's line. A clean's
// DMA read is then done; a flush's DMA write (random bytes) goes into DRAM and gold some cycles
// later, with s_rel, and until then no write of the line is presented either. A request for a
// flushed line is not taken until its DMA write is in; one taken before the snoop is answered
// returns the line as it was.
//
// AT THE END, after the drain, every line of the pool is read back through the cache.
`include "tb_rand.vh"
module tb #(parameter integer SB = 4);
   reg clk = 0, reset = 1;
   always #5 clk = ~clk;
   localparam integer IDW = 5, NID = 32, NM = 8, NWB = 4, NOUT = 12;
   localparam [25:0] ALIAS = 26'h200_0000;      // the DRAM index of PA 0's line (dram_axi)
   integer seed, cycles, reorder, latmin, latmax, timeout, nl, wrpct, ptpct, gaps, maxout;
   `TB_RAND(rnd, rs)

   // ---- the DUT
   reg            u_q_valid, u_q_we;  reg [IDW-1:0] u_q_id;  reg [57:0] u_q_addr;  reg [63:0] u_q_wmask;  reg [511:0] u_q_wdata;
   wire           u_q_ready, u_r_valid, u_r_last, u_w_valid;
   wire [IDW-1:0] u_r_id, u_w_id;  wire [1:0] u_r_beat;  wire [127:0] u_r_data;
   wire           d_q_valid, d_q_we;  reg d_q_ready;  wire [IDW-1:0] d_q_id;  wire [57:0] d_q_addr;
   wire [63:0]    d_q_wmask;  wire [511:0] d_q_wdata;
   reg            d_r_valid, d_r_last, d_w_valid;  reg [IDW-1:0] d_r_id, d_w_id;  reg [1:0] d_r_beat;  reg [127:0] d_r_data;
   wire           d_r_ready, d_w_ready;
   reg            s_valid, s_keep, s_rel;  reg [25:0] s_line;  wire s_ack;
   rv_l2 #(.IDW(IDW), .SB(SB), .NM(NM), .NWB(NWB)) dut (.clk(clk), .reset(reset),
      .u_q_valid(u_q_valid), .u_q_ready(u_q_ready), .u_q_id(u_q_id), .u_q_we(u_q_we), .u_q_addr(u_q_addr),
      .u_q_wmask(u_q_wmask), .u_q_full(&u_q_wmask), .u_q_wdata(u_q_wdata),
      .u_r_valid(u_r_valid), .u_r_ready(1'b1), .u_r_id(u_r_id), .u_r_beat(u_r_beat), .u_r_last(u_r_last), .u_r_data(u_r_data),
      .u_w_valid(u_w_valid), .u_w_ready(1'b1), .u_w_id(u_w_id),
      .d_q_valid(d_q_valid), .d_q_ready(d_q_ready), .d_q_id(d_q_id), .d_q_we(d_q_we), .d_q_addr(d_q_addr),
      .d_q_wmask(d_q_wmask), .d_q_wdata(d_q_wdata),
      .d_r_valid(d_r_valid), .d_r_ready(d_r_ready), .d_r_id(d_r_id), .d_r_beat(d_r_beat), .d_r_last(d_r_last), .d_r_data(d_r_data),
      .d_w_valid(d_w_valid), .d_w_ready(d_w_ready), .d_w_id(d_w_id),
      .s_valid(s_valid), .s_line(s_line), .s_keep(s_keep), .s_ack(s_ack), .s_rel(s_rel));

   // ---- the images: lines [0, nl) at PA 0x8000_0000 are k; their aliases below the window nl + k
   localparam integer NLMAX = 65536;
   reg [511:0] gold [0:2*NLMAX-1];
   reg [511:0] dram [0:2*NLMAX-1];
   function integer ix_of(input [25:0] dl);
      ix_of = (dl >= ALIAS) ? nl + (dl - ALIAS) : dl;
   endfunction
   function [25:0] dl_of(input [57:0] line);
      dl_of = {line[25] ^ 1'b1, line[24:0]};
   endfunction
   function [511:0] pat(input integer k);
      integer b;
      for (b = 0; b < 8; b = b + 1) pat[b*64 +: 64] = (64'(k) * 64'h9E37_79B9_7F4A_7C15) ^ (64'(b) << 56) ^ 64'h0123_4567_89AB_CDEF;
   endfunction

   // ---- above: what is outstanding
   reg         o_v [0:NID-1];  reg o_we [0:NID-1];  integer o_ix [0:NID-1];  integer o_t [0:NID-1];  integer o_beat [0:NID-1];
   reg [511:0] o_exp [0:NID-1];                   // a read's line as it was when taken
   integer     l_rd [0:2*NLMAX-1];  integer l_wr [0:2*NLMAX-1];   // per line: reads, writes outstanding
   reg         q_took, s_took;
   always @(posedge clk) begin q_took <= u_q_valid & u_q_ready & ~reset;  s_took <= s_valid & s_ack & ~reset; end
   // DMA: the snoop pending, the flushed line waiting for its DMA write
   integer     sn_ix, blk_ix, blk_due, snrate;  reg blk_v;

   // ---- below: the memory model
   reg         mo_v [0:NOUT-1];  reg mo_we [0:NOUT-1];  integer mo_ix [0:NOUT-1];  reg [IDW-1:0] mo_id [0:NOUT-1];
   integer     mo_due [0:NOUT-1];  integer mo_seq [0:NOUT-1];  reg [511:0] mo_data [0:NOUT-1];
   reg         d_took, d_twe;  reg [IDW-1:0] d_tid;  reg [57:0] d_taddr;  reg [63:0] d_twmask;  reg [511:0] d_twdata;
   always @(posedge clk) begin
      d_took <= d_q_valid & d_q_ready & ~reset;  d_twe <= d_q_we;  d_tid <= d_q_id;  d_taddr <= d_q_addr;
      d_twmask <= d_q_wmask;  d_twdata <= d_q_wdata;
   end

   integer now, k, j, pick, best, beating, bi, seqn, errors, n_out, ix, hix [0:7], hw;
   integer n_rd, n_wr, n_pw, n_wp, n_drd, n_dwr, n_beats, sweep, sw_ix;
   integer c_hit, c_rmiss, c_whit, c_walloc, c_pmiss, c_dvic, c_cf, c_push, c_byp, c_mfull, c_wfull, c_ffmax;
   integer n_scl, n_sfl, c_sdty, c_xb;
   reg [511:0] wd;  reg [63:0] wm;  reg [57:0] line;  reg [25:0] dl;
   initial begin
      if (!$value$plusargs("seed=%d", seed)) seed = 1;
      rs = `TB_SEED(seed);
      if (!$value$plusargs("cycles=%d", cycles)) cycles = 200000;
      reorder = $test$plusargs("reorder");
      if (!$value$plusargs("latmin=%d", latmin)) latmin = 20;
      if (!$value$plusargs("latmax=%d", latmax)) latmax = 60;
      if (!$value$plusargs("timeout=%d", timeout)) timeout = 20000;
      if (!$value$plusargs("lines=%d", nl)) nl = 512;
      if (!$value$plusargs("writes=%d", wrpct)) wrpct = 35;    // percent of requests that write
      if (!$value$plusargs("partial=%d", ptpct)) ptpct = 10;   // percent of writes that are partial
      if (!$value$plusargs("gaps=%d", gaps)) gaps = 30;        // percent of beat cycles left empty
      if (!$value$plusargs("maxout=%d", maxout)) maxout = 31;  // requests outstanding above
      if (!$value$plusargs("snoop=%d", snrate)) snrate = 100;   // a snoop one cycle in this many (0: none)
      if (nl > NLMAX) begin $display("L2-TB FAIL: +lines=%0d above %0d", nl, NLMAX); $finish; end
      for (k = 0; k < 2 * nl; k = k + 1) begin gold[k] = pat(k); dram[k] = gold[k]; l_rd[k] = 0; l_wr[k] = 0; end
      for (k = 0; k < NID; k = k + 1) o_v[k] = 1'b0;
      for (k = 0; k < NOUT; k = k + 1) mo_v[k] = 1'b0;
      for (k = 0; k < 8; k = k + 1) hix[k] = 0;
      u_q_valid = 0; u_q_we = 0; u_q_id = 0; u_q_addr = 0; u_q_wmask = 0; u_q_wdata = 0; d_q_ready = 0;
      d_r_valid = 0; d_r_last = 0; d_r_id = 0; d_r_beat = 0; d_r_data = 0; d_w_valid = 0; d_w_id = 0;
      s_valid = 0; s_keep = 0; s_rel = 0; s_line = 0; blk_v = 0; sn_ix = 0; blk_ix = 0; blk_due = 0;
      n_scl = 0; n_sfl = 0; c_sdty = 0; c_xb = 0;
      errors = 0; seqn = 0; beating = -1; bi = 0; hw = 0; sweep = 0; sw_ix = 0;
      n_rd = 0; n_wr = 0; n_pw = 0; n_wp = 0; n_drd = 0; n_dwr = 0; n_beats = 0;
      c_hit = 0; c_rmiss = 0; c_whit = 0; c_walloc = 0; c_pmiss = 0; c_dvic = 0; c_cf = 0; c_push = 0; c_byp = 0;
      c_mfull = 0; c_wfull = 0; c_ffmax = 0;
      repeat (3) @(posedge clk);
      reset = 0;
      n_out = 1;
      for (now = 0; now < cycles || ((n_out != 0 || sweep < 2) && now < cycles + 8 * timeout + 40 * nl); now = now + 1) begin
         @(negedge clk);
         if (q_took) begin
            u_q_valid = 1'b0;
            k = u_q_id;  o_v[k] = 1'b1;  o_we[k] = u_q_we;  o_t[k] = now;  o_beat[k] = 0;
            o_ix[k] = ix_of(dl_of(u_q_addr));
            o_exp[k] = gold[o_ix[k]];
            if (blk_v && blk_ix == o_ix[k]) begin
               $display("FAIL c=%0d id %0d: line %0d taken while it waits for its DMA write", now, k, o_ix[k]);
               errors = errors + 1;
            end
         end
         // ---- answers above (registered by the DUT at the last edge)
         if (u_r_valid) begin
            k = u_r_id;  n_beats = n_beats + 1;
            if (!o_v[k] || o_we[k]) begin $display("FAIL c=%0d: a read beat for id %0d, no read outstanding", now, k); errors = errors + 1; end
            else begin
               if (u_r_beat != o_beat[k][1:0] || u_r_last != (o_beat[k] == 3)) begin
                  $display("FAIL c=%0d id %0d: beat %0d last %0d, expected beat %0d", now, k, u_r_beat, u_r_last, o_beat[k]);
                  errors = errors + 1;
               end
               wd = o_exp[k];
               if (u_r_data != wd[u_r_beat*128 +: 128]) begin
                  $display("FAIL c=%0d id %0d line %0d beat %0d: got %h expected %h", now, k, o_ix[k], u_r_beat, u_r_data, wd[u_r_beat*128 +: 128]);
                  errors = errors + 1;
               end
               o_beat[k] = o_beat[k] + 1;
               if (u_r_last) begin o_v[k] = 1'b0;  l_rd[o_ix[k]] = l_rd[o_ix[k]] - 1; end
            end
         end
         if (u_w_valid) begin
            k = u_w_id;
            if (!o_v[k] || !o_we[k]) begin $display("FAIL c=%0d: a write done for id %0d, no write outstanding", now, k); errors = errors + 1; end
            else begin o_v[k] = 1'b0;  l_wr[o_ix[k]] = l_wr[o_ix[k]] - 1; end
         end
         // ---- outcomes inside
         c_hit = c_hit + dut.b_hrd;  c_rmiss = c_rmiss + (dut.m_new & dut.a_rd);  c_whit = c_whit + dut.b_wr;
         c_walloc = c_walloc + (dut.a_go & ~dut.k_hit & dut.a_whole);  c_pmiss = c_pmiss + (dut.m_new & ~dut.a_rd);
         c_dvic = c_dvic + dut.b_vrd;  c_cf = c_cf + (dut.st == 2'd3 && dut.k_cf);
         c_push = c_push + dut.ff_push;  c_byp = c_byp + dut.f_byp;
         c_mfull = c_mfull + (dut.st == 2'd3 && !dut.k_cf && !dut.k_hit && !dut.a_whole && !dut.mf_any);
         c_wfull = c_wfull + (dut.st == 2'd3 && !dut.k_cf && !dut.k_hit && dut.k_vd && !dut.wf_any);
         if (dut.ff_n > c_ffmax) c_ffmax = dut.ff_n;
         c_sdty = c_sdty + (dut.s_go & dut.a_sdty);  c_xb = c_xb + (dut.st == 2'd2 && dut.q_xb);

         // ---- DMA: a snoop answered at the last edge; a flushed line's DMA write; a new snoop
         s_rel = 1'b0;
         if (s_took) begin
            s_valid = 1'b0;
            if (dram[sn_ix] != gold[sn_ix]) begin
               $display("FAIL c=%0d: line %0d %s, but DRAM does not hold it", now, sn_ix, s_keep ? "cleaned" : "flushed");
               errors = errors + 1;
            end
            if (!s_keep) begin blk_v = 1'b1;  blk_ix = sn_ix;  blk_due = now + rnd(0) % 40; end
         end
         if (blk_v && now >= blk_due) begin
            wm = {rnd(0), rnd(0)};  if (wm == 64'd0) wm = 64'h1;
            for (k = 0; k < 16; k = k + 1) wd[k*32 +: 32] = rnd(0);
            for (k = 0; k < 64; k = k + 1) if (wm[k]) begin gold[blk_ix][k*8 +: 8] = wd[k*8 +: 8];  dram[blk_ix][k*8 +: 8] = wd[k*8 +: 8]; end
            s_rel = 1'b1;  blk_v = 1'b0;
         end
         if (!s_valid && !s_took && snrate > 0 && now < cycles && rnd(0) % snrate == 0) begin
            ix = (rnd(0) % 2) ? hix[rnd(0) % 8] : rnd(0) % nl;
            s_keep = (rnd(0) % 2) || blk_v;
            if (l_wr[ix] == 0 && !(blk_v && blk_ix == ix)) begin
               s_valid = 1'b1;  s_line = 26'(ix);  sn_ix = ix;
               if (s_keep) n_scl = n_scl + 1; else n_sfl = n_sfl + 1;
            end
         end

         // ---- above: present the next request
         n_out = 0;
         for (k = 0; k < NID; k = k + 1) n_out = n_out + o_v[k];
         n_out = 0;
         for (k = 0; k < NID; k = k + 1) n_out = n_out + o_v[k];
         if (!u_q_valid && n_out < maxout && (now < cycles || sweep == 1) && rnd(0) % 4 != 0) begin
            pick = -1;
            for (k = 0; k < NID; k = k + 1) if (!o_v[k] && (pick < 0 || rnd(0) % 2)) pick = k;
            if (sweep == 1) begin ix = sw_ix;  j = 100; end   // the final read-back, line by line
            else begin
               ix = (rnd(0) % 2) ? hix[rnd(0) % 8] : rnd(0) % nl;
               if (rnd(0) % 4 == 0 && (nl >> SB) > 0) ix = (ix % (1 << SB)) + (1 << SB) * (rnd(0) % (nl >> SB));   // one of its set
               j = rnd(0) % 100;
            end
            if (j < wrpct) begin
               if (l_rd[ix] == 0 && l_wr[ix] == 0 && !(s_valid && sn_ix == ix) && !(blk_v && blk_ix == ix)) begin
                  if (rnd(0) % 100 < ptpct) begin
                     wm = {rnd(0), rnd(0)} & {rnd(0), rnd(0)};   // a quarter of the bytes
                     if (wm == 64'd0 || wm == ~64'd0) wm = 64'h0000_0000_0000_00F0;
                     n_pw = n_pw + 1;
                  end else wm = ~64'd0;
                  for (k = 0; k < 16; k = k + 1) wd[k*32 +: 32] = rnd(0);
                  for (k = 0; k < 64; k = k + 1) if (wm[k]) gold[ix][k*8 +: 8] = wd[k*8 +: 8];
                  u_q_valid = 1'b1;  u_q_we = 1'b1;  u_q_id = pick;  u_q_addr = 58'h200_0000 + 58'(ix);
                  u_q_wmask = wm;  u_q_wdata = wd;  l_wr[ix] = l_wr[ix] + 1;  n_wr = n_wr + 1;
               end
            end else if (l_wr[ix] == 0) begin
               if (sweep == 0 && rnd(0) % 50 == 0) begin                 // the alias below the window
                  if (l_wr[nl + ix] == 0) begin
                     u_q_valid = 1'b1;  u_q_we = 1'b0;  u_q_id = pick;  u_q_addr = 58'(ix);
                     l_rd[nl + ix] = l_rd[nl + ix] + 1;  n_rd = n_rd + 1;  n_wp = n_wp + 1;
                  end
               end else begin
                  u_q_valid = 1'b1;  u_q_we = 1'b0;  u_q_id = pick;  u_q_addr = 58'h200_0000 + 58'(ix);
                  l_rd[ix] = l_rd[ix] + 1;  n_rd = n_rd + 1;
                  if (sweep == 1) begin sw_ix = sw_ix + 1;  if (sw_ix == nl) sweep = 2; end
               end
            end
            if (u_q_valid && sweep == 0) begin hix[hw] = ix;  hw = (hw + 1) % 8; end
         end
         if (now >= cycles && n_out == 0 && !u_q_valid && sweep == 0) sweep = 1;

         // ---- below: one beat and one write completion a cycle
         d_r_valid = 1'b0;  d_r_last = 1'b0;  d_w_valid = 1'b0;
         if (beating < 0) begin
            best = -1;
            if (reorder) begin
               for (k = 0; k < NOUT; k = k + 1)
                  if (mo_v[k] && !mo_we[k] && mo_due[k] <= now && (best < 0 || (rnd(0) & 1))) best = k;
            end else begin
               pick = -1;
               for (k = 0; k < NOUT; k = k + 1) if (mo_v[k] && (pick < 0 || mo_seq[k] < mo_seq[pick])) pick = k;
               if (pick >= 0 && !mo_we[pick] && mo_due[pick] <= now) best = pick;
            end
            if (best >= 0) begin beating = best; bi = 0; end
         end
         if (beating >= 0 && rnd(0) % 100 >= gaps) begin
            d_r_valid = 1'b1;  d_r_id = mo_id[beating];  d_r_beat = bi[1:0];  d_r_last = (bi == 3);
            d_r_data = mo_data[beating][bi*128 +: 128];
            bi = bi + 1;
            if (bi == 4) begin mo_v[beating] = 1'b0; beating = -1; end
         end
         best = -1;
         if (reorder) begin
            for (k = 0; k < NOUT; k = k + 1)
               if (mo_v[k] && mo_we[k] && mo_due[k] <= now && (best < 0 || (rnd(0) & 1))) best = k;
         end else if (beating < 0) begin
            pick = -1;
            for (k = 0; k < NOUT; k = k + 1) if (mo_v[k] && (pick < 0 || mo_seq[k] < mo_seq[pick])) pick = k;
            if (pick >= 0 && mo_we[pick] && mo_due[pick] <= now) best = pick;
         end
         if (best >= 0) begin                     // a write completes: now it is in the memory
            dram[mo_ix[best]] = mo_data[best];
            d_w_valid = 1'b1;  d_w_id = mo_id[best];  mo_v[best] = 1'b0;
         end
         if (d_took) begin
            pick = -1;
            for (k = NOUT - 1; k >= 0; k = k - 1) if (!mo_v[k]) pick = k;
            dl = dl_of(d_taddr);
            if (pick < 0) begin $display("FAIL c=%0d: more than %0d transactions below", now, NOUT); errors = errors + 1; end
            else if (d_taddr[57:27] != 0 || !(d_taddr[26] ^ d_taddr[25])) begin
               $display("FAIL c=%0d: a request below for line %h, outside the window", now, d_taddr); errors = errors + 1;
            end else if (!(dl < nl || (dl >= ALIAS && dl < ALIAS + nl))) begin
               $display("FAIL c=%0d: a request below for line %h, not one of the pool's", now, d_taddr); errors = errors + 1;
            end else begin
               if (d_twe && (d_tid < NM || d_tid >= NM + NWB || d_twmask != ~64'd0)) begin
                  $display("FAIL c=%0d: a write below with id %0d mask %h", now, d_tid, d_twmask); errors = errors + 1;
               end
               if (!d_twe && d_tid >= NM) begin $display("FAIL c=%0d: a read below with id %0d", now, d_tid); errors = errors + 1; end
               for (k = 0; k < NOUT; k = k + 1) if (mo_v[k] && mo_id[k] == d_tid) begin
                  $display("FAIL c=%0d: id %0d below is already in flight", now, d_tid); errors = errors + 1;
               end
               mo_v[pick] = 1'b1;  mo_we[pick] = d_twe;  mo_ix[pick] = ix_of(dl);  mo_id[pick] = d_tid;
               mo_seq[pick] = seqn;  seqn = seqn + 1;
               mo_due[pick] = now + latmin + (rnd(0) % (latmax - latmin + 1));
               if (d_twe) begin mo_data[pick] = d_twdata;  n_dwr = n_dwr + 1; end
               else begin mo_data[pick] = dram[ix_of(dl)];  n_drd = n_drd + 1; end
            end
         end
         pick = 0;  for (k = 0; k < NOUT; k = k + 1) if (mo_v[k]) pick = pick + 1;
         d_q_ready = (pick < NOUT) && (rnd(0) % 8 != 0);

         // ---- nobody waits forever
         for (k = 0; k < NID; k = k + 1)
            if (o_v[k] && (now - o_t[k] > timeout)) begin
               $display("FAIL c=%0d: id %0d (line %0d, we %0d) unanswered for %0d cycles", now, k, o_ix[k], o_we[k], timeout);
               errors = errors + 1;  o_v[k] = 1'b0;
            end
         if (errors > 10) begin $display("L2-TB FAIL: too many errors"); $finish; end
         n_out = 0;
         for (k = 0; k < NID; k = k + 1) n_out = n_out + o_v[k];
         n_out = n_out + u_q_valid + q_took + s_valid + s_took + blk_v;
      end
      if (n_out != 0 || sweep != 2) begin $display("FAIL: %0d outstanding after the drain (sweep %0d)", n_out, sweep); errors = errors + 1; end
      // what every shape must reach; a pool over the capacity evicts; more requests than MSHRs fill them
      if (c_hit == 0 || c_rmiss == 0 || c_whit == 0 || c_cf == 0 || c_byp == 0 || n_wp == 0 ||
          (nl > (8 << SB) && (c_dvic == 0 || c_walloc == 0 || c_pmiss == 0)) ||
          (nl > (8 << SB) && maxout > NM && SB < 8 && (c_push == 0 || c_mfull == 0)) ||
          (snrate > 0 && snrate <= 100 && (n_scl == 0 || n_sfl == 0 || c_sdty == 0 || c_xb == 0))) begin
         $display("FAIL: an outcome never occurred"); errors = errors + 1;
      end
      $display("L2-TB %s seed=%0d SB=%0d %s lat %0d-%0d: %0d reads (%0d below the window), %0d writes (%0d partial), %0d beats; %0d read hits, %0d read misses, %0d write hits, %0d whole-line allocations, %0d partial misses, %0d dirty victims, %0d lookups repeated, %0d waits for an MSHR, %0d for a write-back; fill beats %0d straight up, %0d queued (deepest %0d); below %0d reads %0d writes; DMA %0d cleans %0d flushes (%0d found dirty), %0d lookups blocked behind a flush",
               errors ? "FAIL" : "PASS", seed, SB, reorder ? "reorder" : "in-order", latmin, latmax, n_rd, n_wp, n_wr, n_pw, n_beats,
               c_hit, c_rmiss, c_whit, c_walloc, c_pmiss, c_dvic, c_cf, c_mfull, c_wfull, c_byp, c_push, c_ffmax, n_drd, n_dwr,
               n_scl, n_sfl, c_sdty, c_xb);
      $finish;
   end
endmodule
