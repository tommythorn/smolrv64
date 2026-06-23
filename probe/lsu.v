`default_nettype none

// Unified load/store unit (M1) for the sharded-OoO core. Execution/RF/scheduler
// shard; *memory does not* — one store buffer + one load queue serve all shards,
// because memory disambiguation is inherently global (a load must see every older
// store regardless of shard). See docs/sharded-ooo-plan.md "## LSU / store buffer".
//
// M1 scope (correctness-first, against a flat byte-addressable memory port; the
// real physically-tagged D$ + dTLB drop on later):
//   * Store buffer + load queue are POOLS keyed by program-order seqno (same shape
//     as the scoreboard IQ): allocated in-order at dispatch (back-pressure if full),
//     filled at execute, squashed on rollback by seqno (the branch's redirect_seq).
//   * Non-speculative ordering: a load executes only once ALL older stores have
//     resolved (the "resolved store" gate). No memory-order replay path.
//   * COMPLETE byte-granular forwarding: each load byte takes the youngest older
//     store covering it, else memory. Arbitrary alignment (no misalign trap); two
//     older stores overlapping each other and the load, with some bytes from memory,
//     just work. The address arithmetic is done at FILL (sequential, off the critical
//     path): each entry is reduced to a 2-word representation -- {w0,w1=w0+1} word
//     addresses (8-byte words), a per-word byte-enable mask, and the data laid into the
//     word lanes (so a misaligned/word-spanning access spills into w1). The merge is
//     then pure word-EQUALITY (XNOR, no carry chain) + mask lookup + youngest-select,
//     which is what keeps it off the critical path (the old per-byte range compares
//     were carry chains). Combinational here, result flopped -> fixed 1-cycle load.
//   * Stores issue-on-both (rs1 & rs2): a store entry is filled (addr+data+size) in
//     one execute step, so `rdy` covers both — the addr/data split is a later opt.
//     Therefore a store completes at execute and is counted at issue like an ALU op;
//     only LOADS defer completion to the LSU (ld_done -> commit_ctl decrement).
//   * The store buffer is a CPR structure: stores never hit memory until commit
//     (commit sets per-entry `committed`); a drain engine writes <=1/cycle; rollback
//     squashes wrong-path entries by seqno. Driven by commit_ctl's commit/rollback.
//
// Addresses here are PHYSICAL (in M1 the dTLB is identity, so the AGU output is the
// physical address). Memory port: combinational 8-byte read at any byte address;
// one masked 8-byte write/cycle for drain.
module lsu
  #(parameter IW      = 4,
    parameter SBITS   = 2,        // clog2(IW) — owner-shard id width
    parameter PBITS   = 8,
    parameter SEQW    = 8,
    parameter CBITS   = 2,
    parameter AW      = 64,
    parameter PAW     = 34,        // physical address width: only these bits disambiguate
    parameter SBDEPTH = 8,
    parameter SBI     = 3,        // clog2(SBDEPTH)
    parameter LQDEPTH = 8,
    parameter LQI     = 3)        // clog2(LQDEPTH)
   (input  wire                   clk,
    input  wire                   reset,

    // ---- dispatch: in-order allocation (one bundle/cycle) ----
    input  wire                   disp_fire,          // bundle dispatches this cycle
    input  wire [IW-1:0]          disp_is_load,
    input  wire [IW-1:0]          disp_is_store,
    input  wire [IW*SEQW-1:0]     disp_seq,
    input  wire [IW*CBITS-1:0]    disp_ckpt,
    input  wire [IW*PBITS-1:0]    disp_pdst,          // load destination phys reg
    output wire [IW*SBI-1:0]      disp_sb_idx,        // assigned SB slot per store slot
    output wire [IW*LQI-1:0]      disp_lq_idx,        // assigned LQ slot per load slot
    output wire                   sb_full,            // not enough room for the bundle's stores
    output wire                   lq_full,            // ... or its loads  (-> back-pressure)

    // ---- execute: out-of-order fills from the shards' AGUs ----
    input  wire [IW-1:0]          exe_st_v,
    input  wire [IW*SBI-1:0]      exe_st_idx,
    input  wire [IW*AW-1:0]       exe_st_addr,
    input  wire [IW*64-1:0]       exe_st_data,
    input  wire [IW*4-1:0]        exe_st_nb,          // store size in bytes (1..8)
    input  wire [IW-1:0]          exe_ld_v,
    input  wire [IW*LQI-1:0]      exe_ld_idx,
    input  wire [IW*AW-1:0]       exe_ld_addr,
    input  wire [IW*4-1:0]        exe_ld_nb,          // load size in bytes (1..8)
    input  wire [IW-1:0]          exe_ld_sgn,         // sign-extend the result

    // ---- flat memory port (stub; real D$ later) ----
    output wire [AW-1:0]          mem_raddr,
    input  wire [63:0]            mem_rdata,          // 8 bytes @ mem_raddr (little-endian)
    output reg                    mem_wen,
    output reg  [AW-1:0]          mem_waddr,
    output reg  [63:0]            mem_wdata,
    output reg  [7:0]             mem_wmask,

    // ---- load writeback (to the owner shard's WB lane) + completion ----
    // Combinational: a load completes the cycle it is selected. `wb_busy[s]` marks
    // shards whose WB lane is taken by an ALU writeback this cycle; the LSU simply
    // does not select a load owned by a busy shard (it defers — loads are already
    // variable-latency), so the shared lane never collides (no WB reservation yet).
    input  wire [IW-1:0]          wb_busy,            // NEXT-cycle wb per lane (lane reservation)
    output wire                   ld_wb_v,            // registered (byte-merge is its own stage)
    output wire [PBITS-1:0]       ld_wb_pdst,
    output wire [SBITS-1:0]       ld_wb_owner,
    output wire [63:0]            ld_wb_val,
    output wire                   ld_done,            // -> commit_ctl decrement
    output wire [CBITS-1:0]       ld_done_ckpt,

    // ---- commit / rollback (CPR) ----
    input  wire                   commit,
    input  wire [CBITS-1:0]       commit_idx,
    input  wire                   rollback,
    input  wire [SEQW-1:0]        rollback_seq);      // squash entries newer than this

   integer i, j, b;

   localparam WW = PAW - 3;        // word-address width (8-byte words)

   // ============================ store buffer ============================
   reg              sb_v   [0:SBDEPTH-1];
   reg              sb_rdy [0:SBDEPTH-1];   // filled (addr+data) — M1: addr==data ready
   reg              sb_cmt [0:SBDEPTH-1];   // its checkpoint has committed (drainable)
   reg [SEQW-1:0]   sb_seq [0:SBDEPTH-1];
   reg [CBITS-1:0]  sb_ck  [0:SBDEPTH-1];
   reg [AW-1:0]     sb_addr[0:SBDEPTH-1];   // byte address (drain only)
   reg [63:0]       sb_data[0:SBDEPTH-1];   // raw data    (drain only)
   reg [3:0]        sb_nb  [0:SBDEPTH-1];   // size 1..8   (drain only)
   // forwarding view (computed at fill, off the critical path): 2-word representation
   reg [WW-1:0]     sb_w0  [0:SBDEPTH-1];   // low word address  = addr[PAW-1:3]
   reg [WW-1:0]     sb_w1  [0:SBDEPTH-1];   // high word address = w0 + 1 (spill word)
   reg [7:0]        sb_be0 [0:SBDEPTH-1];   // byte-enables in word w0
   reg [7:0]        sb_be1 [0:SBDEPTH-1];   // byte-enables in word w1
   reg [63:0]       sb_d0  [0:SBDEPTH-1];   // data laid into word-w0 byte lanes
   reg [63:0]       sb_d1  [0:SBDEPTH-1];   // data laid into word-w1 byte lanes

   // ============================ load queue =============================
   reg              lq_v   [0:LQDEPTH-1];
   reg              lq_rdy [0:LQDEPTH-1];   // address resolved
   reg [SEQW-1:0]   lq_seq [0:LQDEPTH-1];
   reg [CBITS-1:0]  lq_ck  [0:LQDEPTH-1];
   reg [AW-1:0]     lq_addr[0:LQDEPTH-1];   // byte address (mem_raddr)
   reg [3:0]        lq_nb  [0:LQDEPTH-1];
   reg              lq_sgn [0:LQDEPTH-1];
   reg [PBITS-1:0]  lq_pd  [0:LQDEPTH-1];
   reg [SBITS-1:0]  lq_own [0:LQDEPTH-1];
   reg [WW-1:0]     lq_w0  [0:LQDEPTH-1];   // low word  (precomputed at fill)
   reg [WW-1:0]     lq_w1  [0:LQDEPTH-1];   // high word = w0 + 1
   reg [2:0]        lq_lb  [0:LQDEPTH-1];   // byte offset within word

   initial begin
      for (i = 0; i < SBDEPTH; i = i + 1) begin sb_v[i]=0; sb_rdy[i]=0; sb_cmt[i]=0; end
      for (i = 0; i < LQDEPTH; i = i + 1) begin lq_v[i]=0; lq_rdy[i]=0; end
   end

   // ----------------------- dispatch allocation -------------------------
   // assign each dispatching store/load the lowest free pool slot not already
   // taken this cycle (combinational, IW x DEPTH); raise *_full if short of room.
   reg [SBDEPTH-1:0] sb_take;
   reg [LQDEPTH-1:0] lq_take;
   reg [SBI-1:0]     sb_as [0:IW-1];
   reg [LQI-1:0]     lq_as [0:IW-1];
   reg               sb_ok, lq_ok, fnd;
   always @* begin
      sb_take = {SBDEPTH{1'b0}}; lq_take = {LQDEPTH{1'b0}};
      sb_ok = 1'b1; lq_ok = 1'b1;
      for (i = 0; i < IW; i = i + 1) begin
         sb_as[i] = {SBI{1'b0}}; lq_as[i] = {LQI{1'b0}};
         if (disp_is_store[i]) begin
            fnd = 1'b0;
            for (j = 0; j < SBDEPTH; j = j + 1)
               if (!fnd && !sb_v[j] && !sb_take[j]) begin
                  sb_as[i] = j[SBI-1:0]; sb_take[j] = 1'b1; fnd = 1'b1;
               end
            if (!fnd) sb_ok = 1'b0;
         end
         if (disp_is_load[i]) begin
            fnd = 1'b0;
            for (j = 0; j < LQDEPTH; j = j + 1)
               if (!fnd && !lq_v[j] && !lq_take[j]) begin
                  lq_as[i] = j[LQI-1:0]; lq_take[j] = 1'b1; fnd = 1'b1;
               end
            if (!fnd) lq_ok = 1'b0;
         end
      end
   end
   assign sb_full = !sb_ok;
   assign lq_full = !lq_ok;
   genvar g;
   generate for (g = 0; g < IW; g = g + 1) begin : pk
      assign disp_sb_idx[g*SBI +: SBI] = sb_as[g];
      assign disp_lq_idx[g*LQI +: LQI] = lq_as[g];
   end endgenerate

   // ---------------------- store "resolved" gate ------------------------
   // a load is order-safe once no older store (seq < load.seq) is still unfilled.
   function automatic older;            // a strictly older than b (wrap-safe)
      input [SEQW-1:0] a, bb;
      older = ($signed(a - bb) < 0);
   endfunction

   // -------------------- pick the load to execute -----------------------
   // oldest valid, address-resolved, order-safe load (combinational scan).
   reg            ld_sel_v;
   reg [LQI-1:0]  ld_sel;
   reg [SEQW-1:0] ld_best;
   reg            blocked;
   always @* begin
      ld_sel_v = 1'b0; ld_sel = {LQI{1'b0}}; ld_best = {SEQW{1'b0}};
      for (i = 0; i < LQDEPTH; i = i + 1) begin
         // owner lane free, AND not squashed this cycle: a wrong-path load (seq newer
         // than the branch's rollback_seq) must not complete even in the squash cycle
         // itself -- the lq_v clear only lands at the next edge, so without this gate a
         // squashed load could combinationally assert ld_wb_v (RF write / wake) and
         // ld_done (a spurious commit_ctl decrement). A correct-path older load is not
         // gated and still completes normally.
         if (lq_v[i] && lq_rdy[i] && !wb_busy[lq_own[i]]
             && !(rollback && older(rollback_seq, lq_seq[i]))) begin
            // order-safe? no older unfilled store
            blocked = 1'b0;
            for (j = 0; j < SBDEPTH; j = j + 1)
               if (sb_v[j] && !sb_rdy[j] && older(sb_seq[j], lq_seq[i])) blocked = 1'b1;
            if (!blocked && (!ld_sel_v || older(lq_seq[i], ld_best))) begin
               ld_sel_v = 1'b1; ld_sel = i[LQI-1:0]; ld_best = lq_seq[i];
            end
         end
      end
   end

   // -------------------- byte-granular forward merge --------------------
   wire [AW-1:0] la = lq_addr[ld_sel];
   assign mem_raddr = la;

   // combinational byte-merge (memory ∪ youngest-older-store/byte) -> the c_* result,
   // which is FLOPPED below. The byte-merge is thus its own pipeline stage; nothing
   // combinational from the LSU reaches the writeback/RF-write/forwarding path.
   //
   // The selected load is reduced (at fill) to {w0,w1,lb}. For load byte mb the absolute
   // position is lb+mb, which falls in word lwb (= w0 or w1) at byte lane `posw`. A store
   // covers that byte iff one of its two words equals lwb and the matching per-word mask
   // bit is set -- both are EQUALITY tests (no carry chain). Youngest older store wins.
   reg [63:0]     m_mrg;
   reg [7:0]      m_byt;
   reg            m_fwd, m_c0, m_c1;
   reg [SEQW-1:0] m_bseq, m_lsq;
   reg [3:0]      m_nb, m_lp;
   reg [2:0]      m_posw;
   reg [WW-1:0]   m_lwb;
   reg [SBDEPTH-1:0] s_use;        // store is valid+ready+older-than-load (byte-independent)
   integer        mb, mj;
   reg            c_v;
   reg [63:0]     c_val;
   wire [WW-1:0]  lw0 = lq_w0[ld_sel];
   wire [WW-1:0]  lw1 = lq_w1[ld_sel];
   wire [2:0]     llb = lq_lb[ld_sel];
   always @* begin
      m_lsq = lq_seq[ld_sel];
      m_nb  = lq_nb [ld_sel];
      m_mrg = 64'd0;
      // hoist the per-store "older than this load" seqno compare out of the byte loop
      // (it does not depend on the byte) -- one 8-bit compare/store, not 8.
      for (mj = 0; mj < SBDEPTH; mj = mj + 1)
         s_use[mj] = sb_v[mj] && sb_rdy[mj] && ($signed(sb_seq[mj] - m_lsq) < 0);
      for (mb = 0; mb < 8; mb = mb + 1) begin
         m_lp   = {1'b0, llb} + mb[3:0];               // 0..14 (no big carry: 3b + const)
         m_posw = m_lp[2:0];
         m_lwb  = m_lp[3] ? lw1 : lw0;                 // which word this load byte is in
         m_fwd  = 1'b0; m_bseq = {SEQW{1'b0}};
         m_byt  = mem_rdata[mb*8 +: 8];                // default: memory (read @ la, byte mb)
         for (mj = 0; mj < SBDEPTH; mj = mj + 1) begin
            m_c0 = s_use[mj] && (sb_w0[mj] == m_lwb) && sb_be0[mj][m_posw];
            m_c1 = s_use[mj] && (sb_w1[mj] == m_lwb) && sb_be1[mj][m_posw];
            if ((m_c0 || m_c1)
                && (!m_fwd || ($signed(m_bseq - sb_seq[mj]) < 0))) begin   // youngest wins
               m_fwd  = 1'b1; m_bseq = sb_seq[mj];
               m_byt  = m_c0 ? sb_d0[mj][m_posw*8 +: 8] : sb_d1[mj][m_posw*8 +: 8];
            end
         end
         m_mrg[mb*8 +: 8] = m_byt;
      end
      c_v   = ld_sel_v;
      c_val = (m_nb==4'd1) ? (lq_sgn[ld_sel] ? {{56{m_mrg[7]}},  m_mrg[7:0]}  : {56'd0, m_mrg[7:0]})
            : (m_nb==4'd2) ? (lq_sgn[ld_sel] ? {{48{m_mrg[15]}}, m_mrg[15:0]} : {48'd0, m_mrg[15:0]})
            : (m_nb==4'd4) ? (lq_sgn[ld_sel] ? {{32{m_mrg[31]}}, m_mrg[31:0]} : {32'd0, m_mrg[31:0]})
            : m_mrg;
   end

   // ---- writeback register (the byte-merge result, flopped) ----
   reg            r_v;
   reg [PBITS-1:0] r_pdst;
   reg [SBITS-1:0] r_owner;
   reg [63:0]     r_val;
   reg [SEQW-1:0] r_seq;
   reg [CBITS-1:0] r_ck;
   initial r_v = 1'b0;
   always @(posedge clk) begin
      if (reset) r_v <= 1'b0;
      else begin
         r_v     <= c_v;
         r_pdst  <= lq_pd [ld_sel];
         r_owner <= lq_own[ld_sel];
         r_val   <= c_val;
         r_seq   <= lq_seq[ld_sel];
         r_ck    <= lq_ck [ld_sel];
      end
   end
   // present the registered result; a rollback that squashes this load (now out of the
   // LQ) the cycle it would write back suppresses it.
   wire r_kill = rollback & older(rollback_seq, r_seq);
   assign ld_wb_v      = r_v & ~r_kill;
   assign ld_wb_pdst   = r_pdst;
   assign ld_wb_owner  = r_owner;
   assign ld_wb_val    = r_val;
   assign ld_done      = ld_wb_v;
   assign ld_done_ckpt = r_ck;

   // ------------------------------ drain --------------------------------
   // oldest committed+filled store -> one masked write/cycle.
   reg            dr_v;
   reg [SBI-1:0]  dr_sel;
   reg [SEQW-1:0] dr_best;
   always @* begin
      dr_v = 1'b0; dr_sel = {SBI{1'b0}}; dr_best = {SEQW{1'b0}};
      for (i = 0; i < SBDEPTH; i = i + 1)
         if (sb_v[i] && sb_rdy[i] && sb_cmt[i] && (!dr_v || older(sb_seq[i], dr_best))) begin
            dr_v = 1'b1; dr_sel = i[SBI-1:0]; dr_best = sb_seq[i];
         end
   end
   reg [7:0] dr_mask;
   always @* begin
      dr_mask = 8'd0;
      for (b = 0; b < 8; b = b + 1) if (b < sb_nb[dr_sel]) dr_mask[b] = 1'b1;
   end
   always @* begin
      mem_wen   = dr_v;
      mem_waddr = sb_addr[dr_sel];
      mem_wdata = sb_data[dr_sel];
      mem_wmask = dr_mask;
   end

   // ----------------------------- sequential ----------------------------
   reg [SBI-1:0]  eidx;
   reg [LQI-1:0]  lidx;
   reg [AW-1:0]   f_addr;          // fill temps (off the critical path)
   reg [2:0]      f_off;
   reg [8:0]      f_be9;
   reg [127:0]    f_wd;
   reg [15:0]     f_wbe;
   always @(posedge clk) begin
      if (reset) begin
         for (i = 0; i < SBDEPTH; i = i + 1) begin sb_v[i]<=0; sb_rdy[i]<=0; sb_cmt[i]<=0; end
         for (i = 0; i < LQDEPTH; i = i + 1) begin lq_v[i]<=0; lq_rdy[i]<=0; end
      end else begin
         // (1) dispatch allocation
         if (disp_fire) begin
            for (i = 0; i < IW; i = i + 1) begin
               if (disp_is_store[i]) begin
                  eidx = sb_as[i];
                  sb_v[eidx]   <= 1'b1; sb_rdy[eidx] <= 1'b0; sb_cmt[eidx] <= 1'b0;
                  sb_seq[eidx] <= disp_seq[i*SEQW +: SEQW];
                  sb_ck[eidx]  <= disp_ckpt[i*CBITS +: CBITS];
               end
               if (disp_is_load[i]) begin
                  lidx = lq_as[i];
                  lq_v[lidx]   <= 1'b1; lq_rdy[lidx] <= 1'b0;
                  lq_seq[lidx] <= disp_seq[i*SEQW +: SEQW];
                  lq_ck[lidx]  <= disp_ckpt[i*CBITS +: CBITS];
                  lq_pd[lidx]  <= disp_pdst[i*PBITS +: PBITS];
                  lq_own[lidx] <= i[SBITS-1:0];     // fixed steering: slot i -> shard i
               end
            end
         end

         // (2) execute fills (out of order)
         for (i = 0; i < IW; i = i + 1) begin
            if (exe_st_v[i]) begin
               eidx   = exe_st_idx[i*SBI +: SBI];
               f_addr = exe_st_addr[i*AW +: AW];
               f_off  = f_addr[2:0];
               f_be9  = (9'd1 << exe_st_nb[i*4 +: 4]) - 9'd1;       // 1..8 -> byte mask
               f_wd   = {64'd0, exe_st_data[i*64 +: 64]} << {f_off, 3'd0}; // data into lanes
               f_wbe  = {8'd0, f_be9[7:0]} << f_off;                // mask into lanes
               sb_addr[eidx] <= f_addr;
               sb_data[eidx] <= exe_st_data[i*64 +: 64];
               sb_nb[eidx]   <= exe_st_nb[i*4 +: 4];
               sb_w0[eidx]   <= f_addr[PAW-1:3];
               sb_w1[eidx]   <= f_addr[PAW-1:3] + 1'b1;
               sb_d0[eidx]   <= f_wd[63:0];
               sb_d1[eidx]   <= f_wd[127:64];
               sb_be0[eidx]  <= f_wbe[7:0];
               sb_be1[eidx]  <= f_wbe[15:8];
               sb_rdy[eidx]  <= 1'b1;
            end
            if (exe_ld_v[i]) begin
               lidx   = exe_ld_idx[i*LQI +: LQI];
               f_addr = exe_ld_addr[i*AW +: AW];
               lq_addr[lidx] <= f_addr;
               lq_nb[lidx]   <= exe_ld_nb[i*4 +: 4];
               lq_sgn[lidx]  <= exe_ld_sgn[i];
               lq_w0[lidx]   <= f_addr[PAW-1:3];
               lq_w1[lidx]   <= f_addr[PAW-1:3] + 1'b1;
               lq_lb[lidx]   <= f_addr[2:0];
               lq_rdy[lidx]  <= 1'b1;
            end
         end

         // (3) load completion is combinational (ld_wb_*/ld_done above); here we
         //     just free the LQ entry of the load that completed this cycle.
         if (ld_sel_v) lq_v[ld_sel] <= 1'b0;

         // (4) commit: mark this checkpoint's stores drainable
         if (commit)
            for (i = 0; i < SBDEPTH; i = i + 1)
               if (sb_v[i] && (sb_ck[i] == commit_idx)) sb_cmt[i] <= 1'b1;

         // (5) drain: retire the selected store from the buffer
         if (dr_v) sb_v[dr_sel] <= 1'b0;

         // (6) rollback: squash wrong-path entries (newer than the branch)
         if (rollback) begin
            for (i = 0; i < SBDEPTH; i = i + 1)
               if (sb_v[i] && older(rollback_seq, sb_seq[i])) sb_v[i] <= 1'b0;
            for (i = 0; i < LQDEPTH; i = i + 1)
               if (lq_v[i] && older(rollback_seq, lq_seq[i])) lq_v[i] <= 1'b0;
         end
      end
   end
endmodule

`default_nettype wire
