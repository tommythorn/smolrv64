`timescale 1ns/1ps
// tb_rv_dcache: rv_dcache against a golden memory image and a reference translation.
//
// MEMORY. A memory-port model with up to NOUT transactions in flight, each with its own latency (a
// draw of LATMIN..LATMAX cycles), served in request order unless +reorder, in which case any one
// whose latency has passed may go first. A read returns its line in four 128-bit beats, in order;
// a write completes with one cw pulse. The model is as late as the port allows: a read returns
// the memory as it was when the read was taken, and a write reaches the memory only when it
// completes. The request channel is back-pressured at random.
//
// TWO IMAGES. `dram` is the memory behind the port. `gold` is the architectural memory: every
// store the cache took is in it from the cycle it was taken. Every load answer is checked against
// gold. Both start as a pattern of the address.
//
// TRANSLATION. NVP virtual pages map onto NPP physical pages at random, so several virtual pages
// -- of different colours, VA[15:12] -- share one physical page (synonyms). Every REMAP cycles
// one mapping changes -- half the time a page among the last 8 accessed -- and the cache is told
// (ep_bump), as an sfence.vma would; wraps included.
// As in the core, where sfence.vma waits for the store queue to drain, a remap waits until no
// store is presented and the cache's store queue is empty: a request's translation holds in the
// epoch it is taken in.
//
// REQUESTERS. 16 load tags, each with at most one load outstanding -- a cached load, a page-table
// walk (by PA) or an NC load -- and the store port: committed stores, one a cycle whenever wr_room
// is set (each presented for that cycle only, and taken then), or one NC store or cbo.clean,
// cbo.flush or cbo.zero, held until its wr_cpl with nothing else presented meanwhile. An access
// is 1, 2, 4 or 8 bytes, naturally aligned, at a random offset of a random virtual page -- half
// the time in the line of one of the last 8 accesses -- with the PA of the mapping at issue. As
// the core's store queue guarantees, no store is presented to a chunk a load in flight reads (a
// CBO: its line), and no load to the chunk (the line) of an NC store or CBO not yet done; a load
// to the line of a committed store the cache has queued is presented, and the cache must order
// it. A load's answer must be gold's bytes, under its tag, within TIMEOUT cycles. A plain store
// is in gold from the cycle it is taken; an NC store and a CBO act at wr_cpl.
// As in the core, which performs NC accesses at the head of the ROB, one NC access is in flight at
// a time.
//
// WHAT MEMORY MUST HOLD. When a cbo.clean or cbo.flush completes, memory holds gold's line; half
// the flushes are followed by a DMA write of the line, straight into both images, which later
// loads must see. Every FENCE cycles, and after the drain, inv_req cleans the cache -- as fence.i
// does, with nothing in flight and nothing presented until it is done: then memory must equal
// gold, every word.
`include "tb_rand.vh"
`ifndef DC_VIRT
 `define DC_VIRT 1
`endif
module tb;
   reg clk = 0, reset = 1;
   always #5 clk = ~clk;
   localparam NTAG = 16, NVP = 48, NPP = 24, NOUT = 8, NMSHR = 8;
   localparam [63:0] PBASE = 64'h8000_0000;
   integer seed, cycles, reorder, latmin, latmax, remap, timeout, stpct, fence;
   localparam NCS = 10;                         // the NC slot's memory-port id (NMSHR + NWB)
   localparam FN_BUDGET = 1000000;              // a clean writes back up to every line, NWB at a time
   localparam K_ST = 0, K_NC = 1, K_CLN = 2, K_FL = 3, K_Z = 4;
   `TB_RAND(rnd, rs)

   // ---- the DUT
   reg         rd_req;  reg [63:0] rd_va, rd_pa;  reg [3:0] rd_tag;  reg rd_phys, rd_nc;
   wire        rd_ack, rd_valid;  wire [63:0] rd_data, rd_resp_addr;  wire [3:0] rd_resp_tag;
   reg         wr_req;  reg [63:0] wr_va, wr_pa, wr_data;  reg [7:0] wr_mask;  wire wr_room, wr_acc, wr_cpl;
   reg         wr_nc, cbo_req, cbo_zero, cbo_keep;
   reg         ep_bump, inv_req;  wire inv_busy;
   wire        cq_valid, cq_we;  reg cq_ready;  wire [3:0] cq_slot;  wire [57:0] cq_addr;
   wire [63:0] cq_wmask;  wire [511:0] cq_wdata;
   reg         cr_valid, cr_last;  reg [3:0] cr_slot;  reg [1:0] cr_beat;  reg [127:0] cr_data;
   reg         cw_valid;  reg [3:0] cw_slot;
   wire        perf_access, perf_miss;  wire [15:0] err;
   rv_dcache #(.VIRT(`DC_VIRT)) dut (.clk(clk), .reset(reset),
      .rd_req(rd_req), .rd_va(rd_va), .rd_pa(rd_pa), .rd_tag(rd_tag), .rd_phys(rd_phys), .rd_nc(rd_nc), .rd_ack(rd_ack),
      .rd_valid(rd_valid), .rd_data(rd_data), .rd_resp_tag(rd_resp_tag), .rd_resp_addr(rd_resp_addr),
      .wr_req(wr_req), .wr_va(wr_va), .wr_pa(wr_pa), .wr_data(wr_data), .wr_mask(wr_mask),
      .wr_nc(wr_nc), .cbo_req(cbo_req), .cbo_zero(cbo_zero), .cbo_keep(cbo_keep),
      .wr_room(wr_room), .wr_acc(wr_acc), .wr_cpl(wr_cpl),
      .ep_bump(ep_bump), .inv_req(inv_req), .inv_busy(inv_busy),
      .cq_valid(cq_valid), .cq_ready(cq_ready), .cq_slot(cq_slot), .cq_we(cq_we), .cq_addr(cq_addr),
      .cq_wmask(cq_wmask), .cq_wdata(cq_wdata),
      .cr_valid(cr_valid), .cr_slot(cr_slot), .cr_beat(cr_beat), .cr_last(cr_last), .cr_data(cr_data),
      .cw_valid(cw_valid), .cw_slot(cw_slot),
      .perf_access(perf_access), .perf_miss(perf_miss), .err(err));

   // ---- the two images, 8 bytes a word, over the NPP physical pages
   localparam NWD = NPP * 512;
   reg [63:0] gold [0:NWD-1], dram [0:NWD-1];
   function integer wi; input [63:0] pa; wi = (pa - PBASE) >> 3; endfunction
   function [63:0] bytes; input [7:0] m; integer b; begin for (b = 0; b < 8; b = b + 1) bytes[b*8 +: 8] = {8{m[b]}}; end endfunction

   // ---- the translation
   reg [63:0] vp_va [0:NVP-1];                 // each virtual page's VA (distinct pages, all colours)
   reg [5:0]  vp_pp [0:NVP-1];                 // its physical page
   integer k, b;
   function [63:0] pp_pa; input [5:0] p; pp_pa = PBASE + {52'd0, p, 12'd0} * 64'd1; endfunction

   // ---- the memory model: transactions in flight, then their beats or completion
   reg        mo_v [0:NOUT-1];  reg mo_we [0:NOUT-1];  reg [57:0] mo_line [0:NOUT-1];  reg [3:0] mo_slot [0:NOUT-1];
   reg [31:0] mo_due [0:NOUT-1];  integer mo_seq [0:NOUT-1];  reg [511:0] mo_data [0:NOUT-1];  reg [63:0] mo_mask [0:NOUT-1];
   integer    seqn, beating, bi, n_rd, n_wr;
   reg [31:0] now;

   // ---- the requesters
   reg        out_v [0:NTAG-1];  reg [63:0] out_pa [0:NTAG-1];  reg [1:0] out_sz [0:NTAG-1];
   reg [31:0] out_t [0:NTAG-1];
   reg        st_pend;  reg [31:0] st_t;         // an NC store or a CBO presented, not yet done
   reg        st_acc;  integer st_kind;          // ...taken, awaiting wr_cpl; its kind
   reg        rm_due;                            // a remap waits for the store to be taken
   reg        fn_due, fn_wait, fn_final, fn_done;  reg [31:0] fn_t;   // the clean (inv_req)
   integer    n_ncst, n_cln, n_fl, n_z, n_dma, n_fence, n_walk, n_ncld, kind;
   integer    n_req, n_resp, n_st, n_remap, n_miss, n_access, errors;
   integer    hp [0:7], ho [0:7], hw;           // the last 8 accesses' page and offset: locality
   // coverage: every outcome the cache has occurs
   integer    c_own, c_drop, c_merge, c_alloc, c_wait, c_wrap, c_swr, c_smg, c_sal, c_ro, c_blk, c_def, n_out;
   integer    c_nc, c_cln, c_cdone, c_zero, c_sq1, c_sqbk;
   reg [1:0]  out_kind [0:NTAG-1];              // 0 a cached load, 1 a walk, 2 NC
   reg [63:0] expv, mask, pa;  integer t, p, off, sz, pick, best, w;
   // acceptance is decided at the edge, by the pre-edge handshake
   reg        took;  reg [3:0] took_tag;
   always @(posedge clk) begin took <= rd_req & rd_ack & ~reset; took_tag <= rd_tag; end
   reg        pq_plain;                          // a committed store was presented with wr_room: it must be taken
   always @(posedge clk) pq_plain <= wr_req & ~cbo_req & ~wr_nc & wr_room & ~reset;
   reg        mq_took, mq_we;  reg [57:0] mq_addr;  reg [3:0] mq_slot;  reg [511:0] mq_wdata;  reg [63:0] mq_wmask;
   always @(posedge clk) begin
      mq_took <= cq_valid & cq_ready & ~reset;  mq_we <= cq_we;  mq_addr <= cq_addr;  mq_slot <= cq_slot;
      mq_wdata <= cq_wdata;  mq_wmask <= cq_wmask;
   end

   // a load in flight reads the chunk of pa (line: its line)
   function ld_busy; input [63:0] a; input line; integer q; reg [63:0] m; begin
      m = line ? ~64'd63 : ~64'd7;
      ld_busy = took && ((out_pa[took_tag] & m) == (a & m));
      for (q = 0; q < NTAG; q = q + 1) if (out_v[q] && ((out_pa[q] & m) == (a & m))) ld_busy = 1'b1;
   end endfunction
   // the store-port op not yet done covers pa: its chunk, a CBO its line
   // an NC access is in flight: a load, or the store-port op
   function nc_busy; input dummy; integer q; begin
      nc_busy = st_pend && (st_kind == K_NC);
      if (took && out_kind[took_tag] == 2) nc_busy = 1'b1;
      for (q = 0; q < NTAG; q = q + 1) if (out_v[q] && out_kind[q] == 2) nc_busy = 1'b1;
   end endfunction
   function st_covers; input [63:0] a; reg [63:0] m; begin
      m = (st_kind >= K_CLN) ? ~64'd63 : ~64'd7;
      st_covers = st_pend && ((wr_pa & m) == (a & m));
   end endfunction
   task check_line; input [63:0] a; input [8*16-1:0] what; integer q; begin
      for (q = 0; q < 8; q = q + 1)
         if (dram[wi({a[63:6], 6'd0}) + q] !== gold[wi({a[63:6], 6'd0}) + q]) begin
            $display("FAIL c=%0d: after %0s of %h, memory word %0d is %h, expected %h", now, what, a, q,
                     dram[wi({a[63:6], 6'd0}) + q], gold[wi({a[63:6], 6'd0}) + q]);
            errors = errors + 1;
         end
   end endtask
   // an access: a page and an offset, half the time in a recent line
   task pick_access; begin
      p   = rnd(0) % NVP;
      sz  = rnd(0) % 4;
      off = (rnd(0) % 4096) & ~((1 << sz) - 1);
      if (rnd(0) % 2 == 0) begin k = rnd(0) % 8;  p = hp[k];  off = (ho[k] & ~63) | (off & 63); end
      hp[hw] = p;  ho[hw] = off;  hw = (hw + 1) % 8;
   end endtask

   initial begin
      if (!$value$plusargs("seed=%d", seed)) seed = 1;
      rs = `TB_SEED(seed);
      if (!$value$plusargs("cycles=%d", cycles)) cycles = 200000;
      reorder = $test$plusargs("reorder");
      if (!$value$plusargs("latmin=%d", latmin)) latmin = 20;
      if (!$value$plusargs("latmax=%d", latmax)) latmax = 60;
      if (!$value$plusargs("remap=%d", remap)) remap = 3000;
      if (!$value$plusargs("timeout=%d", timeout)) timeout = 5000;
      if (!$value$plusargs("stores=%d", stpct)) stpct = 30;     // percent of cycles a store is offered
      if (!$value$plusargs("fence=%d", fence)) fence = 20000;
      for (k = 0; k < NWD; k = k + 1) begin
         pa = PBASE + 64'(k) * 8;
         gold[k] = (pa * 64'h9E37_79B9_7F4A_7C15) ^ {pa[31:0], pa[63:32]} ^ 64'h0123_4567_89AB_CDEF;
         dram[k] = gold[k];
      end
      // pages: virtual pages spread over many colours; physical pages fewer, so synonyms abound
      for (k = 0; k < NVP; k = k + 1) begin
         vp_va[k] = 64'h0000_0010_0000_0000 + (64'(k * 37 + 5) << 12);   // distinct VPNs, every colour
         vp_pp[k] = rnd(0) % NPP;
      end
      for (k = 0; k < NTAG; k = k + 1) out_v[k] = 1'b0;
      for (k = 0; k < NOUT; k = k + 1) mo_v[k] = 1'b0;
      rd_req = 0; rd_va = 0; rd_pa = 0; rd_tag = 0; rd_phys = 0; rd_nc = 0; ep_bump = 0; inv_req = 0; cq_ready = 0;
      wr_req = 0; wr_va = 0; wr_pa = 0; wr_data = 0; wr_mask = 0; st_pend = 0; st_t = 0; rm_due = 0;
      wr_nc = 0; cbo_req = 0; cbo_zero = 0; cbo_keep = 0; st_acc = 0; st_kind = K_ST;
      fn_due = 0; fn_wait = 0; fn_final = 0; fn_done = 0; fn_t = 0;
      n_ncst = 0; n_cln = 0; n_fl = 0; n_z = 0; n_dma = 0; n_fence = 0; n_walk = 0; n_ncld = 0;
      c_nc = 0; c_cln = 0; c_cdone = 0; c_zero = 0; c_sq1 = 0; c_sqbk = 0;
      cr_valid = 0; cr_last = 0; cr_slot = 0; cr_beat = 0; cr_data = 0; cw_valid = 0; cw_slot = 0;
      n_req = 0; n_resp = 0; n_st = 0; n_remap = 0; n_miss = 0; n_access = 0; errors = 0; n_rd = 0; n_wr = 0;
      for (k = 0; k < 8; k = k + 1) begin hp[k] = 0; ho[k] = 0; end
      hw = 0;  seqn = 0;  beating = -1;  bi = 0;  now = 0;
      c_own = 0; c_drop = 0; c_merge = 0; c_alloc = 0; c_wait = 0; c_wrap = 0;
      c_swr = 0; c_smg = 0; c_sal = 0; c_ro = 0; c_blk = 0; c_def = 0;
      repeat (3) @(posedge clk);
      reset = 0;
      while (dut.inv_busy) @(posedge clk);
      // run, then drain -- every request taken is answered within TIMEOUT of the last one -- then clean
      n_out = 1;
      for (now = 0; now < cycles || ((n_out != 0 || !fn_done) && now < cycles + 4 * timeout + FN_BUDGET); now = now + 1) begin
         @(negedge clk);
         // ---- this cycle's load answer (registered by the DUT at the last edge), against gold
         if (rd_valid) begin
            t = rd_resp_tag;
            if (!out_v[t]) begin $display("FAIL c=%0d: a response for tag %0d, which has nothing outstanding", now, t); errors = errors + 1; end
            else begin
               expv = gold[wi(out_pa[t])] >> (8 * out_pa[t][2:0]);
               mask = (out_sz[t] == 3) ? ~64'd0 : ((64'd1 << (8 << out_sz[t])) - 1);
               if (((rd_data ^ expv) & mask) != 0) begin
                  $display("FAIL c=%0d tag %0d pa %h size %0d: got %h expected %h", now, t, out_pa[t], 1 << out_sz[t], rd_data & mask, expv & mask);
                  errors = errors + 1;
               end
               if (rd_resp_addr != out_pa[t]) begin $display("FAIL c=%0d tag %0d: response addr %h, issued %h", now, t, rd_resp_addr, out_pa[t]); errors = errors + 1; end
               out_v[t] = 1'b0;  n_resp = n_resp + 1;
               if (out_kind[t] == 1) n_walk = n_walk + 1;
               if (out_kind[t] == 2) n_ncld = n_ncld + 1;
            end
         end
         if (perf_access) n_access = n_access + 1;
         if (perf_miss)   n_miss = n_miss + 1;
         c_own = c_own + dut.m_own;  c_drop = c_drop + dut.m_drop;  c_merge = c_merge + dut.m_merge;
         c_alloc = c_alloc + dut.m_alloc;  c_wait = c_wait + dut.m_wait;  c_wrap = c_wrap + dut.ep_wrap;
         c_swr = c_swr + dut.s_write;  c_smg = c_smg + dut.m_smerge;  c_sal = c_sal + (dut.m_alloc & dut.s1_st);
         c_ro = c_ro + (dut.ro_need & ~dut.ro_block & ~dut.wb_block);  c_blk = c_blk + dut.blk_st;  c_def = c_def + dut.m_defer;
         c_nc = c_nc + dut.m_nc;  c_cln = c_cln + dut.m_clean;  c_cdone = c_cdone + dut.m_cdone;
         c_zero = c_zero + (dut.k_z & (dut.m_alloc | dut.m_smerge));
         c_sq1 = c_sq1 + (dut.st_go & dut.si);  c_sqbk = c_sqbk + dut.sq_bk;
         // ---- the load taken at the last edge
         if (took) begin out_v[took_tag] = 1'b1; out_t[took_tag] = now; n_req = n_req + 1; end
         // ---- the store port: a committed store is presented for one cycle when wr_room says the
         // cache takes it, and is in gold from then; an NC store or a CBO is held until its wr_cpl
         if (pq_plain && !wr_acc) begin $display("FAIL c=%0d: a store presented with wr_room was not taken", now); errors = errors + 1; end
         if (wr_acc && !pq_plain) begin
            if (!st_pend || st_acc) begin $display("FAIL c=%0d: wr_acc with no store-port op presented", now); errors = errors + 1; end
            else st_acc = 1;
         end
         if (wr_cpl) begin
            if (!st_acc) begin $display("FAIL c=%0d: wr_cpl with no NC store or CBO taken", now); errors = errors + 1; end
            else begin
               case (st_kind)
                  K_NC:  begin w = wi(wr_pa);  gold[w] = (gold[w] & ~bytes(wr_mask)) | (wr_data & bytes(wr_mask));  n_ncst = n_ncst + 1; end
                  K_CLN: begin check_line(wr_pa, "cbo.clean");  n_cln = n_cln + 1; end
                  K_FL:  begin
                     check_line(wr_pa, "cbo.flush");  n_fl = n_fl + 1;
                     if (rnd(0) % 2) begin               // a device writes the line behind the cache
                        w = wi({wr_pa[63:6], 6'd0}) + rnd(0) % 8;  gold[w] = {rnd(0), rnd(0)};  dram[w] = gold[w];
                        n_dma = n_dma + 1;
                     end
                  end
                  default: begin for (w = 0; w < 8; w = w + 1) gold[wi({wr_pa[63:6], 6'd0}) + w] = 64'd0;  n_z = n_z + 1; end
               endcase
               st_pend = 0;  st_acc = 0;  wr_req = 0;
            end
         end
         if (st_pend && (now - st_t > timeout)) begin
            $display("FAIL c=%0d: a store-port op (kind %0d) at %h not done for %0d cycles", now, st_kind, wr_pa, timeout);
            errors = errors + 1;  st_pend = 0;  st_acc = 0;  wr_req = 0;
         end
         if (!st_pend) wr_req = 0;
         if (!st_pend && !fn_due && !fn_wait && !rm_due && now < cycles && (rnd(0) % 100) < stpct) begin
            pick_access;
            pa = pp_pa(vp_pp[p]) + off;
            kind = rnd(0) % 100;
            kind = (kind < 72) ? K_ST : (kind < 80) ? K_NC : (kind < 86) ? K_CLN : (kind < 93) ? K_FL : K_Z;
            if (kind == K_NC && nc_busy(0)) kind = K_ST;
            if (!ld_busy(pa, kind >= K_CLN) && (kind != K_ST || wr_room)) begin
               wr_req = 1;
               wr_va = vp_va[p] + off;  wr_pa = pa;
               wr_mask = (sz == 3) ? 8'hFF : (((8'd1 << (1 << sz)) - 8'd1) << off[2:0]);
               wr_data = {rnd(0), rnd(0)};           // the lanes outside the mask are noise
               wr_nc = (kind == K_NC);  cbo_req = (kind >= K_CLN);  cbo_zero = (kind == K_Z);  cbo_keep = (kind == K_CLN);
               if (kind == K_ST) begin
                  w = wi(pa);  gold[w] = (gold[w] & ~bytes(wr_mask)) | (wr_data & bytes(wr_mask));  n_st = n_st + 1;
               end else begin st_pend = 1;  st_t = now;  st_kind = kind; end
            end
         end
         // ---- a new load: a free tag, not covered by the store-port op not yet done
         rd_req = 1'b0;
         t = rnd(0) % NTAG;
         if (now < cycles && !fn_due && !fn_wait && !out_v[t] && !(took && took_tag == t[3:0]) && (rnd(0) % 4 != 0)) begin
            pick_access;
            pa = pp_pa(vp_pp[p]) + off;
            if (!st_covers(pa)) begin
               kind = rnd(0) % 10;
               if (kind == 1 && nc_busy(0)) kind = 2;
               rd_req = 1'b1;  rd_tag = t[3:0];  rd_phys = (kind == 0);  rd_nc = (kind == 1);
               rd_va = (kind == 0) ? {rnd(0), rnd(0)} : vp_va[p] + off;  rd_pa = pa;   // a walk's VA is noise
               out_pa[t] = rd_pa;  out_sz[t] = sz[1:0];  out_kind[t] = (kind == 0) ? 2'd1 : (kind == 1) ? 2'd2 : 2'd0;
            end
         end
         // ---- the clean: every FENCE cycles and after the drain, as fence.i issues it -- with nothing
         // in flight, and nothing presented until it is done; then memory must equal gold
         inv_req = 1'b0;
         n_out = 0;
         for (k = 0; k < NTAG; k = k + 1) n_out = n_out + out_v[k];
         if (now > 0 && fence > 0 && (now % fence) == 0 && now < cycles) fn_due = 1;
         if (now >= cycles && n_out == 0 && !fn_final) begin fn_due = 1;  fn_final = 1; end
         if (fn_due && !st_pend && !fn_wait && n_out == 0 && !took && !rd_req) begin fn_due = 0;  fn_wait = 1;  fn_t = now;  inv_req = 1'b1;  n_fence = n_fence + 1; end
         else if (fn_wait && now > fn_t + 1 && !inv_busy) begin
            fn_wait = 0;
            for (w = 0; w < NWD; w = w + 8) check_line(PBASE + 64'(w) * 8, "the clean");
            if (fn_final) fn_done = 1;
         end
         if (fn_wait && now - fn_t > FN_BUDGET) begin $display("FAIL c=%0d: the clean not done", now); errors = errors + 1; fn_wait = 0; fn_done = 1; end
         // ---- a remap: one mapping changes, and the cache hears of it
         ep_bump = 1'b0;
         if ((now % remap) == remap - 1) rm_due = 1;
         if (rm_due && !st_pend && !wr_req && dut.sc == 2'd0) begin
            rm_due = 0;
            p = (rnd(0) % 2) ? hp[rnd(0) % 8] : rnd(0) % NVP;   // half the time a page in use
            vp_pp[p] = rnd(0) % NPP;
            ep_bump = 1'b1;  n_remap = n_remap + 1;
         end
         // ---- the memory model's response side: one beat and one write completion per cycle
         cr_valid = 1'b0;  cr_last = 1'b0;  cw_valid = 1'b0;
         if (beating < 0) begin
            best = -1;
            if (reorder) begin
               for (k = 0; k < NOUT; k = k + 1)
                  if (mo_v[k] && !mo_we[k] && mo_due[k] <= now && (best < 0 || (rnd(0) & 1))) best = k;
            end else begin                        // in order: only the oldest may go, once due
               pick = -1;
               for (k = 0; k < NOUT; k = k + 1) if (mo_v[k] && (pick < 0 || mo_seq[k] < mo_seq[pick])) pick = k;
               if (pick >= 0 && !mo_we[pick] && mo_due[pick] <= now) best = pick;
            end
            if (best >= 0) begin beating = best; bi = 0; end
         end
         if (beating >= 0) begin
            cr_valid = 1'b1;  cr_slot = mo_slot[beating];  cr_beat = bi[1:0];  cr_last = (bi == 3);
            cr_data = mo_data[beating][bi*128 +: 128];
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
            for (w = 0; w < 8; w = w + 1)
               dram[wi({mo_line[best], 6'd0}) + w] = (dram[wi({mo_line[best], 6'd0}) + w] & ~bytes(mo_mask[best][w*8 +: 8]))
                                                    | (mo_data[best][w*64 +: 64] & bytes(mo_mask[best][w*8 +: 8]));
            cw_valid = 1'b1;  cw_slot = mo_slot[best];  mo_v[best] = 1'b0;
         end
         // ---- the request side: back-pressured at random; a taken request gets a latency, a read its data now
         if (mq_took) begin
            pick = -1;
            for (k = NOUT - 1; k >= 0; k = k - 1) if (!mo_v[k]) pick = k;
            if (pick < 0) begin $display("FAIL c=%0d: more than %0d transactions outstanding", now, NOUT); errors = errors + 1; end
            else begin
               if (!mq_we && mq_slot >= NMSHR && mq_slot != NCS) begin $display("FAIL c=%0d: a read on write slot %0d", now, mq_slot); errors = errors + 1; end
               if ( mq_we && mq_slot <  NMSHR) begin $display("FAIL c=%0d: a write on read slot %0d", now, mq_slot); errors = errors + 1; end
               mo_v[pick] = 1'b1;  mo_we[pick] = mq_we;  mo_line[pick] = mq_addr;  mo_slot[pick] = mq_slot;
               mo_seq[pick] = seqn;  seqn = seqn + 1;
               mo_due[pick] = now + latmin + (rnd(0) % (latmax - latmin + 1));
               mo_mask[pick] = mq_wmask;
               if (mq_we) begin mo_data[pick] = mq_wdata;  n_wr = n_wr + 1; end
               else begin
                  for (w = 0; w < 8; w = w + 1) mo_data[pick][w*64 +: 64] = dram[wi({mq_addr, 6'd0}) + w];
                  n_rd = n_rd + 1;
               end
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
         n_out = n_out + took + rd_req + st_pend + fn_wait + fn_due;
      end
      if (n_req != n_resp) begin $display("FAIL: %0d loads, %0d answered after the drain", n_req, n_resp); errors = errors + 1; end
      if (!fn_done) begin $display("FAIL: the final clean never finished"); errors = errors + 1; end
      if (c_own == 0 || c_drop == 0 || c_merge == 0 || c_alloc == 0 || c_wait == 0 || c_wrap == 0 ||
          c_swr == 0 || c_smg == 0 || c_sal == 0 || c_ro == 0 || c_blk == 0 || c_def == 0 || n_wr == 0 ||
          c_nc == 0 || c_cln == 0 || c_cdone == 0 || c_zero == 0 || n_dma == 0 || n_walk == 0 || n_ncld == 0 || n_ncst == 0 ||
          c_sq1 == 0 || c_sqbk == 0) begin
         $display("FAIL: an outcome never occurred"); errors = errors + 1;
      end
      $display("DCACHE-TB %s seed=%0d %s: %0d loads (%0d walks, %0d NC), %0d stores, %0d NC stores, CBOs %0d clean %0d flush (%0d DMA) %0d zero, %0d cleans; %0d requests: %0d own-set, %0d drop, %0d merge, %0d alloc, %0d wait (%0d behind the store), %0d defer; stores %0d written, %0d merged, %0d allocated; %0d read-outs, %0d reads, %0d writes; %0d remaps, %0d wraps; store queue %0d issued behind the head, %0d sent back",
               errors ? "FAIL" : "PASS", seed, reorder ? "reorder" : "in-order", n_resp, n_walk, n_ncld, n_st, n_ncst, n_cln, n_fl, n_dma, n_z, n_fence, n_access,
               c_own, c_drop, c_merge, c_alloc, c_wait, c_blk, c_def, c_swr, c_smg, c_sal, c_ro, n_rd, n_wr, n_remap, c_wrap, c_sq1, c_sqbk);
      $finish;
   end
endmodule
