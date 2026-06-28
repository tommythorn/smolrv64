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
    parameter LQI     = 3,        // clog2(LQDEPTH)
    parameter [63:0] DEV_TOP = 64'h7000_0000,  // PA < DEV_TOP == MMIO (device) space; ==LBASE
    // valid-DRAM window for the load/store MMUs' unbacked-PA access-fault check (see mmu.v);
    // default permissive -> no new faults for the unit TB.
    parameter [63:0] DRAM_BASE = 64'd0,
    parameter [63:0] DRAM_TOP  = 64'hFFFF_FFFF_FFFF_FFFF)
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
    input  wire [IW-1:0]          exe_st_cbo,         // Zicbom/Zicboz CBO (rides this store entry)
    input  wire [IW-1:0]          exe_st_cbo_zero,    // cbo.zero (else clean/flush/inval)
    input  wire [IW-1:0]          exe_st_cbo_keep,    // cbo.clean keep-valid (else invalidate)
    input  wire [IW-1:0]          exe_ld_v,
    input  wire [IW*LQI-1:0]      exe_ld_idx,
    input  wire [IW*AW-1:0]       exe_ld_addr,
    input  wire [IW*4-1:0]        exe_ld_nb,          // load size in bytes (1..8)
    input  wire [IW-1:0]          exe_ld_sgn,         // sign-extend the result
    input  wire [IW-1:0]          exe_ld_fp,          // FP load -> NaN-box a 4-byte (FLW) result

    // ---- atomic (A ext) execute port (single: atomics are serialized + solo) ----
    input  wire                   amo_v,              // an atomic at EX (oldest, non-spec)
    input  wire [4:0]             amo_func,           // funct5
    input  wire [AW-1:0]          amo_addr,
    input  wire [63:0]            amo_data,           // rs2
    input  wire [1:0]             amo_sz,             // .W=2 / .D=3
    input  wire [PBITS-1:0]       amo_pdst,
    input  wire [SBITS-1:0]       amo_owner,
    input  wire [CBITS-1:0]       amo_ckpt,
    input  wire [SEQW-1:0]        amo_seq,

    // ---- address translation (dTLB); Bare (satp.MODE=0) = identity bypass ----
    // Translation happens at the memory-access boundaries (load SELECT, store drain,
    // AMO), not at fill -- one access per port/cycle, each with a stall hook. Ordering
    // + forwarding stay in VA space (no aliasing in tests); only mem_raddr/mem_waddr
    // become physical. Two single-port walkers (load path; store+amo path -- those two
    // never overlap, since an AMO's A_WAIT waits for all stores to drain first).
    input  wire [63:0]            xl_satp,
    input  wire [1:0]             xl_priv,     // effective data priv (MPRV-resolved)
    input  wire                   xl_sum,
    input  wire                   xl_mxr,
    input  wire                   xl_flush,
    output wire [55:0]            ldp_addr,    // load-path PTW memory port
    output wire                   ldp_read,
    input  wire [63:0]            ldp_rdata,
    input  wire                   ldp_rvalid,
    output wire [55:0]            stp_addr,    // store/amo-path PTW memory port
    output wire                   stp_read,
    input  wire [63:0]            stp_rdata,
    input  wire                   stp_rvalid,
    // ---- data page-fault report (precise: rolls back to the faulting op's ckpt) ----
    output wire                   dfault_v,    // a load/store/amo page-faults this cycle
    output wire [SEQW-1:0]        dfault_seq,
    output wire [CBITS-1:0]       dfault_ckpt,
    output wire [3:0]             dfault_cause,
    output wire [AW-1:0]          dfault_tval, // faulting virtual address
    // ---- store completion (deferred decrement, like ld_done): a store retires from
    //      commit_ctl's count only once its translation has been checked fault-free.
    //      Used only under Sv39 (backend defers store completion to the LSU then). ----
    output wire                   st_done,
    output wire [CBITS-1:0]       st_done_ckpt,
    output wire                   sb_empty,           // no live stores + AMO idle (mem is current) -- for fence.i ordering

    // ---- data memory READ port: request/response handshake (real D$ can stall) ----
    // mem_ren pulses for one cycle when a fresh address is registered on mem_raddr
    // (a load entering MERGE, or an atomic entering its RMW read). mem_raddr then
    // HOLDS until the access completes. mem_rvalid signals mem_rdata is valid for the
    // currently-presented mem_raddr -- it may arrive 1+ cycles later (cache miss/fill).
    // The merge/RMW stall (hold p_*/mem_raddr) until mem_rvalid. Single-outstanding:
    // a new mem_ren supersedes any prior unfinished read, and mem_rdata must always
    // reflect the address held at the cycle mem_rvalid asserts (no stale-data hazard).
    // Tie mem_rvalid high for a zero-latency (combinational) memory -> 1-cycle loads.
    output reg  [AW-1:0]          mem_raddr,          // registered: the selected load's addr
    output reg                    mem_ren,            // read-request pulse (fresh mem_raddr)
    output reg                    mem_runcached,      // Svpbmt: the read addr is NC/IO (don't cache)
    input  wire [63:0]            mem_rdata,          // 8 bytes @ mem_raddr (little-endian)
    input  wire                   mem_rvalid,         // mem_rdata valid for mem_raddr this cycle
    output reg                    mem_wen,            // held until mem_wready (drain + AMO write)
    output reg  [AW-1:0]          mem_waddr,
    output reg  [63:0]            mem_wdata,
    output reg  [7:0]             mem_wmask,
    output reg                    mem_wuncached,      // Svpbmt: the write addr is NC/IO (flush-around)
    output reg                    mem_cbo,            // Zicbom/Zicboz: drain a cache-maintenance op (no data write)
    output reg                    mem_cbo_zero,       // cbo.zero: install a zero line
    output reg                    mem_cbo_keep,       // cbo.clean: writeback but keep line valid
    input  wire                   mem_wready,         // write accepted/done; tie 1 for 1-cycle writes

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
    output wire [SEQW-1:0]        ld_wb_seq,          // seqno of this writeback (cosim capture)
    output wire                   ld_done,            // -> commit_ctl decrement
    output wire [CBITS-1:0]       ld_done_ckpt,

    // ---- commit / rollback (CPR) ----
    input  wire                   commit,
    input  wire [CBITS-1:0]       commit_idx,
    input  wire [CBITS-1:0]       committed,          // oldest-live checkpoint (== commit_ctl committed_idx)
    input  wire                   rollback,
    input  wire [SEQW-1:0]        rollback_seq,       // squash entries newer than this
    input  wire                   dfault_taken);      // our data-fault trap fired this cycle

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
   reg [AW-1:0]     sb_pa  [0:SBDEPTH-1];   // physical drain address (Bare: ==VA; Sv39: filled at check)
   reg              sb_nc  [0:SBDEPTH-1];   // Svpbmt: NC/IO store -> flush-around at drain
   reg              sb_cbo [0:SBDEPTH-1];   // Zicbom/Zicboz: this entry is a CBO maintenance op
   reg              sb_cboz[0:SBDEPTH-1];   // cbo.zero (else clean/flush/inval)
   reg              sb_cbok[0:SBDEPTH-1];   // cbo.clean keep-valid (else invalidate)
   reg              sb_xck [0:SBDEPTH-1];   // translation checked (drainable; Bare: set at fill)
   reg              sb_xflt[0:SBDEPTH-1];   // store page-faults (reported via dfault, never drains)

   // ============================ load queue =============================
   reg              lq_v   [0:LQDEPTH-1];
   reg              lq_rdy [0:LQDEPTH-1];   // address resolved
   reg [SEQW-1:0]   lq_seq [0:LQDEPTH-1];
   reg [CBITS-1:0]  lq_ck  [0:LQDEPTH-1];
   reg [AW-1:0]     lq_addr[0:LQDEPTH-1];   // byte address (mem_raddr)
   reg [3:0]        lq_nb  [0:LQDEPTH-1];
   reg              lq_sgn [0:LQDEPTH-1];
   reg              lq_fp  [0:LQDEPTH-1];   // FLW -> NaN-box a 4-byte result
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

   // ====================== load pipeline: SELECT | MERGE ======================
   // SELECT (combinational scan -> p_* register): pick the oldest order-safe load and
   //   latch its {attrs, address}; issue mem_raddr (registered). The WB-lane is NOT
   //   checked here -- it is checked one cycle later (MERGE), where wb_busy lines up with
   //   the cycle this load actually writes back. So SELECT just picks the global oldest.
   // MERGE (combinational from p_* + the now-valid mem_rdata -> r_* register): byte-merge
   //   memory with the store buffer; commit to the WB register only when the owner lane is
   //   free next cycle. If busy, STALL (hold p_*/mem_raddr, retry) -- correct because
   //   wb_busy is always next-cycle and a fire always writes back next cycle.
   // Net: load-use latency +1 vs the single-stage merge; this is the shape a synchronous
   //   D$ wants (address out -> 1 cycle -> data back).

   // -------------------------- SELECT (scan) --------------------------
   reg            ld_sel_v;
   reg [LQI-1:0]  ld_sel;
   reg [SEQW-1:0] ld_best;
   reg            blocked;
   always @* begin
      ld_sel_v = 1'b0; ld_sel = {LQI{1'b0}}; ld_best = {SEQW{1'b0}};
      for (i = 0; i < LQDEPTH; i = i + 1) begin
         // not squashed this cycle: a wrong-path load (seq newer than the branch's
         // rollback_seq) must not be selected even in the squash cycle itself.
         if (lq_v[i] && lq_rdy[i] && !(rollback && older(rollback_seq, lq_seq[i]))) begin
            blocked = 1'b0;          // order-safe? no older unfilled store
            for (j = 0; j < SBDEPTH; j = j + 1)
               if (sb_v[j] && !sb_rdy[j] && older(sb_seq[j], lq_seq[i])) blocked = 1'b1;
            if (!blocked && (!ld_sel_v || older(lq_seq[i], ld_best))) begin
               ld_sel_v = 1'b1; ld_sel = i[LQI-1:0]; ld_best = lq_seq[i];
            end
         end
      end
   end

   // ------------------- MERGE-stage register (p_*) --------------------
   reg            p_v;
   reg [PBITS-1:0] p_pdst;
   reg [SBITS-1:0] p_owner;
   reg [SEQW-1:0] p_seq;
   reg [CBITS-1:0] p_ck;
   reg [3:0]      p_nb;
   reg            p_sgn;
   reg            p_fp;
   reg [WW-1:0]   p_w0, p_w1;
   reg [2:0]      p_lb;
   initial p_v = 1'b0;

`ifdef LSU_ASSERT
   // In-module assertions (local signals -> no hierarchical-observation problem).
   integer az;
   always @(posedge clk) if (!reset) begin
      // ORDER-SAFETY: a load held in MERGE must have NO older UNFILLED store in the SB
      // (else it read memory/forwarded before the store's data existed -> stale load).
      if (p_v)
         for (az = 0; az < SBDEPTH; az = az + 1)
            if (sb_v[az] && !sb_rdy[az] && ($signed(sb_seq[az] - p_seq) < 0))
               $display("[%0t] *** LSU-ORD: load p_seq=%0d w0=%h merges w/ older UNFILLED store sb[%0d] seq=%0d addr=%h",
                  $time, p_seq, p_w0, az, sb_seq[az], sb_addr[az]);
      // FORWARD: an aligned 8-byte load whose word is fully covered by an older filled
      // 8-byte store MUST get that store's data. If c_val != sb_d0 -> byte-merge dropout.
      if (p_v && (p_nb == 4'd8) && (p_lb == 3'd0))
         for (az = 0; az < SBDEPTH; az = az + 1)
            if (sb_v[az] && sb_rdy[az] && ($signed(sb_seq[az] - p_seq) < 0)
                && (sb_w0[az] == p_w0) && (sb_be0[az] == 8'hff) && (c_val != sb_d0[az]))
               $display("[%0t] *** LSU-FWD-MISS: load p_seq=%0d w0=%h c_val=%h != store sb[%0d] seq=%0d d0=%h (mem_rdata=%h s_use=%b)",
                  $time, p_seq, p_w0, c_val, az, sb_seq[az], sb_d0[az], mem_rdata, s_use);
      // DIAGNOSTIC: a load that returns 0 while an older filled NONZERO store sits in the
      // SB -> dump words to see if the store's sb_w0 matches the load's p_w0 (it should).
      if (p_v && (c_val == 64'd0) && (p_nb == 4'd8))
         for (az = 0; az < SBDEPTH; az = az + 1)
            if (sb_v[az] && sb_rdy[az] && ($signed(sb_seq[az] - p_seq) < 0) && (sb_d0[az] != 64'd0))
               $display("[%0t] LD0 p_seq=%0d p_w0=%h p_w1=%h | sb[%0d] seq=%0d w0=%h w1=%h d0=%h be0=%b",
                  $time, p_seq, p_w0, p_w1, az, sb_seq[az], sb_w0[az], sb_w1[az], sb_d0[az], sb_be0[az]);
   end
`endif

   // ---- atomic (A ext) FSM state (declared early: used by sel_fire below) ----
   localparam A_IDLE=3'd0, A_WAIT=3'd1, A_RD=3'd2, A_WR=3'd4, A_WB=3'd3;
   reg [2:0]        ast;
   reg [AW-1:0]     a_addr;  reg [63:0] a_data;  reg [4:0] a_func;  reg [1:0] a_sz;
   reg [PBITS-1:0]  a_pdst;  reg [SBITS-1:0] a_own;  reg [CBITS-1:0] a_ck;  reg [SEQW-1:0] a_seq;
   reg [63:0]       a_rdval_q;
   reg [AW-1:0]     a_wpa;                             // translated (aligned) AMO phys addr
   reg              a_wnc;                             // Svpbmt: the AMO target is NC/IO
   reg              rsv_v;   reg [WW-1:0] rsv_w;       // LR/SC reservation (word granularity)
   // registered AMO writeback (1-cycle pulse) -- aligns with wb_busy like a load's r_*
   reg              amo_wbv;
   reg [PBITS-1:0]  amo_wbpd;  reg [SBITS-1:0] amo_wbow;  reg [63:0] amo_wbvl;  reg [CBITS-1:0] amo_wbck;
   reg [SEQW-1:0]   amo_wbsq;
   initial begin ast = A_IDLE; rsv_v = 1'b0; amo_wbv = 1'b0; end

   // ===================== address translation (dTLB) =====================
   wire xlate = (xl_satp[63:60] == 4'd8);   // Sv39 on; else Bare identity bypass

   // load-path walker: translate the selected load's VA. A miss holds sel_fire off
   // (the load stays in the LQ) while the PTW walks; the fill makes it hit next cycle.
   wire        ldx_ready, ldx_fault, ldx_uncached;
   wire [55:0] ldx_pa;
   wire [3:0]  ldx_cause;
   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_ldmmu
     (.clk(clk), .reset(reset),
      .req_valid(ld_sel_v), .req_vaddr(lq_addr[ld_sel]), .req_access(2'd1),
      .priv(xl_priv), .sum(xl_sum), .mxr(xl_mxr), .satp(xl_satp), .flush(xl_flush),
      .ptw_addr(ldp_addr), .ptw_read(ldp_read), .ptw_rdata(ldp_rdata), .ptw_rvalid(ldp_rvalid),
      .t_ready(ldx_ready), .t_paddr(ldx_pa), .t_fault(ldx_fault), .t_cause(ldx_cause),
      .t_uncached(ldx_uncached));
   // mmu resolves combinationally in Bare mode (no walk): a noncanon/out-of-range load
   // there yields ldx_fault with an ACCESS-fault cause (5), surfaced like any page fault.
   wire          ld_xok = ldx_ready & ~ldx_fault;
   wire          ld_xflt = ldx_ready & ldx_fault;                  // selected load page/access-faults
   wire [AW-1:0] ld_pa  = {{(AW-56){1'b0}}, ldx_pa};

   // MERGE fire/stall: the held load writes back next cycle iff its owner lane is free
   // next cycle (wb_busy) and it is not squashed; squash drops it; otherwise stall.
   wire merge_squash = rollback & older(rollback_seq, p_seq);
   // a held load merges only once its memory read returns (mem_rvalid) AND its owner
   // WB lane is free next cycle; otherwise STALL (hold p_*/mem_raddr). A squash drops it
   // even mid-miss (the abandoned read's response is simply never consumed).
   wire merge_fire   = p_v & mem_rvalid & ~wb_busy[p_owner] & ~merge_squash;
   wire merge_drop   = p_v & merge_squash;
   wire merge_adv    = ~p_v | merge_fire | merge_drop;   // MERGE stage empties next cycle
   // a younger load must not bypass an in-flight (older) atomic's RMW write -> hold load
   // selection while the atomic FSM is busy (ast != A_IDLE) OR an atomic is arriving this
   // cycle (~amo_v). The latter keeps any load out of the MERGE stage during the atomic, so
   // a load's ld_done never collides with the atomic's (single completion port).
   // Read-side-effecting DEVICE loads (MMIO: anything below the local-SRAM/DRAM base, i.e.
   // CLINT/PLIC/UART) must NOT execute speculatively: a UART RBR / PLIC-claim read pops state
   // at request time, so a branch-mispredict squash (or a replay after an older branch
   // resolves) would consume a byte/claim that the program never receives. Gate on the
   // already-translated PHYSICAL address (correct under Bare and Sv39) and only let such a
   // load fire once its checkpoint is the oldest live one (== committed): then no older branch
   // can squash it. (Residual: an older sibling in the SAME oldest checkpoint page-faulting
   // could still annul it -- needs replay-to-solo; harmless for M-mode/Bare which never data-
   // faults, i.e. the monitor. Stores are already commit-gated, so only loads need this.)
   wire ld_is_dev = ld_pa < DEV_TOP;                     // DEV_TOP is a module parameter
   wire ld_dev_ok = ~ld_is_dev | (lq_ck[ld_sel] == committed);
   wire sel_fire     = merge_adv & ld_sel_v & (ast == A_IDLE) & ~amo_v & ld_xok & ld_dev_ok;

   // ====================== atomic (A ext) FSM ======================
   // Atomics are serialized + solo (issue only when oldest), so when one executes every
   // older store has committed and (after A_WAIT) drained, and nothing younger is live --
   // the RMW is non-speculative and sees coherent memory. As cheap as in-order. States:
   //   IDLE -> WAIT(older stores drain) -> RD(read+compute+write) -> WB(write rd back).
   wire [AW-1:0]    a_waddr = a_addr & ~{{(AW-3){1'b0}}, 3'b111};   // 8-byte aligned
   wire [WW-1:0]    a_word  = a_addr[PAW-1:3];
   wire             a_islr  = (a_func == 5'b00010);
   wire             a_issc  = (a_func == 5'b00011);
   wire             a_isw   = (a_sz == 2'd2);
   wire             a_half  = a_addr[2];
   wire [31:0]      a_old32 = a_half ? mem_rdata[63:32] : mem_rdata[31:0];
   wire [31:0]      a_d32   = a_data[31:0];
   wire             a_scok  = rsv_v && (rsv_w == a_word);
   // committed stores still in the buffer must drain before the RMW reads memory
   reg              amo_pend;
   integer pp;
   always @* begin amo_pend = 1'b0;
      for (pp = 0; pp < SBDEPTH; pp = pp + 1) if (sb_v[pp] && sb_cmt[pp]) amo_pend = 1'b1; end
   // RMW result (value to store; for .W only low 32 used)
   reg [63:0] a_resv;
   always @* begin
      case (a_func)
        5'b00001: a_resv = a_data;                                                    // swap
        5'b00000: a_resv = a_isw ? {32'b0, a_old32 + a_d32}            : mem_rdata + a_data;
        5'b00100: a_resv = a_isw ? {32'b0, a_old32 ^ a_d32}            : mem_rdata ^ a_data;
        5'b01100: a_resv = a_isw ? {32'b0, a_old32 & a_d32}            : mem_rdata & a_data;
        5'b01000: a_resv = a_isw ? {32'b0, a_old32 | a_d32}            : mem_rdata | a_data;
        5'b10000: a_resv = a_isw ? {32'b0, ($signed(a_old32)<$signed(a_d32))?a_old32:a_d32}
                                 : ($signed(mem_rdata)<$signed(a_data))?mem_rdata:a_data;   // min
        5'b10100: a_resv = a_isw ? {32'b0, ($signed(a_old32)>$signed(a_d32))?a_old32:a_d32}
                                 : ($signed(mem_rdata)>$signed(a_data))?mem_rdata:a_data;   // max
        5'b11000: a_resv = a_isw ? {32'b0, (a_old32<a_d32)?a_old32:a_d32}
                                 : (mem_rdata<a_data)?mem_rdata:a_data;                      // minu
        5'b11100: a_resv = a_isw ? {32'b0, (a_old32>a_d32)?a_old32:a_d32}
                                 : (mem_rdata>a_data)?mem_rdata:a_data;                      // maxu
        5'b00011: a_resv = a_data;                                                    // SC stores rs2
        default:  a_resv = mem_rdata;                                                 // LR: no write
      endcase
   end
   wire [63:0] a_oldv   = a_isw ? {{32{a_old32[31]}}, a_old32} : mem_rdata;
   wire [63:0] a_rdval  = a_issc ? (a_scok ? 64'd0 : 64'd1) : a_oldv;     // SC: 0=ok 1=fail
   wire        a_dowr   = a_islr ? 1'b0 : a_issc ? a_scok : 1'b1;         // who writes memory
   wire        amo_wr_now = (ast == A_WR);   // RMW write held through A_WR until mem_wready
   wire        amo_wb_ok  = (ast == A_WB) & ~wb_busy[a_own];   // reserve owner lane (next cycle)
   wire [63:0] a_wdata  = a_isw ? (a_half ? {a_resv[31:0],32'b0} : {32'b0,a_resv[31:0]}) : a_resv;
   wire [7:0]  a_wmask  = a_isw ? (a_half ? 8'hF0 : 8'h0F) : 8'hFF;

   // drain select (oldest committed+filled store -> one masked write/cycle); declared here
   // (before the FSM) because the FSM's reservation-clear references dr_v/dr_sel.
   reg            dr_v;
   reg [SBI-1:0]  dr_sel;
   reg [SEQW-1:0] dr_best;
   always @* begin
      dr_v = 1'b0; dr_sel = {SBI{1'b0}}; dr_best = {SEQW{1'b0}};
      for (i = 0; i < SBDEPTH; i = i + 1)
         if (sb_v[i] && sb_rdy[i] && sb_cmt[i] && sb_xck[i] && !sb_xflt[i]
             && (!dr_v || older(sb_seq[i], dr_best))) begin
            dr_v = 1'b1; dr_sel = i[SBI-1:0]; dr_best = sb_seq[i];
         end
   end
   reg [7:0] dr_mask;
   always @* begin
      dr_mask = 8'd0;
      for (b = 0; b < 8; b = b + 1) if (b < sb_nb[dr_sel]) dr_mask[b] = 1'b1;
   end
   // debug-only: the VIRTUAL address of the store driving mem this cycle (cosim store log)
   wire [AW-1:0] dbg_st_va = amo_wr_now ? a_addr : sb_addr[dr_sel];

   // store/amo-path walker. PRE-COMMIT store check: translate the OLDEST unchecked store
   // (sb_xck=0) so a store page fault is discovered while the store is still speculative
   // (its checkpoint open) -> a precise trap can roll it back. (The drain-time translation
   // it replaces ran post-commit -> a store fault could never be delivered precisely.) The
   // architectural write still happens later at drain, using the PA stashed here (sb_pa).
   // One store checked per cycle (single MMU port) -- matches the single st_done port. AMO
   // (solo + oldest) shares the port and wins: when it needs translation no younger store
   // can be pending a check (older stores already drained, nothing younger in flight).
   wire amo_need_xl = (ast == A_WAIT) & ~amo_pend & ~p_v;   // about to read the RMW location

   // oldest store still needing a translation check
   reg            ck_v;
   reg [SBI-1:0]  ck_sel;
   reg [SEQW-1:0] ck_best;
   always @* begin
      ck_v = 1'b0; ck_sel = {SBI{1'b0}}; ck_best = {SEQW{1'b0}};
      for (i = 0; i < SBDEPTH; i = i + 1)
         if (sb_v[i] && sb_rdy[i] && !sb_xck[i] && (!ck_v || older(sb_seq[i], ck_best))) begin
            ck_v = 1'b1; ck_sel = i[SBI-1:0]; ck_best = sb_seq[i];
         end
   end
   wire        st_need_xl = xlate & ck_v & ~amo_need_xl;    // checking a store this cycle

   wire        stx_ready, stx_fault, stx_uncached;
   wire [55:0] stx_pa;
   wire [3:0]  stx_cause;
   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_stmmu
     (.clk(clk), .reset(reset),
      .req_valid(xlate & (amo_need_xl | ck_v)),
      .req_vaddr(amo_need_xl ? a_addr : sb_addr[ck_sel]),
      .req_access(amo_need_xl ? 2'd3 : 2'd2),
      .priv(xl_priv), .sum(xl_sum), .mxr(xl_mxr), .satp(xl_satp), .flush(xl_flush),
      .ptw_addr(stp_addr), .ptw_read(stp_read), .ptw_rdata(stp_rdata), .ptw_rvalid(stp_rvalid),
      .t_ready(stx_ready), .t_paddr(stx_pa), .t_fault(stx_fault), .t_cause(stx_cause),
      .t_uncached(stx_uncached));
   wire          amo_xok   = ~xlate | (stx_ready & ~stx_fault);   // amo ok (valid when amo_need_xl)
   wire          amo_xflt  = xlate & amo_need_xl & stx_ready & stx_fault;
   wire [AW-1:0] amo_pa_al = xlate ? (({{(AW-56){1'b0}}, stx_pa}) & ~{{(AW-3){1'b0}}, 3'b111})
                                   : a_waddr;
   // store-check outcome this cycle (valid when st_need_xl)
   wire          st_ck_done = st_need_xl & stx_ready & ~stx_fault;   // translated OK -> completes
   wire          st_ck_flt  = st_need_xl & stx_ready &  stx_fault;   // page-faults -> precise trap

   // fence.i ordering: the store buffer is empty (all stores drained+written-through to memory,
   // since an entry frees on mem_wready which the D$ asserts only after its L2 write completes)
   // and no atomic is mid-RMW -> memory is current and safe to refetch from.
   reg sb_any; integer se;
   always @* begin sb_any = 1'b0; for (se=0;se<SBDEPTH;se=se+1) sb_any = sb_any | sb_v[se]; end
   assign sb_empty = ~sb_any & (ast == A_IDLE);

   // store completion: retire from commit_ctl's count once checked fault-free.
   assign st_done      = st_ck_done;
   assign st_done_ckpt = sb_ck[ck_sel];

   // data page-fault report -> backend_top injects a precise trap (rolls back to the
   // faulting op's checkpoint). Precise exceptions require the OLDEST faulting memory op:
   // loads translate eagerly/out-of-order while stores are checked in-order one/cycle, so a
   // younger load can fault before an older store -- if we latched the younger one, commit
   // could never reach its checkpoint (the older faulting op never completes) -> deadlock.
   // So pick the older of a concurrent load/store fault, and (below) let an older fault
   // preempt a younger one already latched.
   // an AMO faulting is solo + oldest by construction (issues only when oldest, holds the
   // pipe), so it is always the oldest fault when present -> highest priority.
   wire             df_now   = ld_xflt | st_ck_flt | amo_xflt;
   wire             pick_am  = amo_xflt;
   wire             pick_ld  = ~pick_am & ld_xflt & (~st_ck_flt | older(lq_seq[ld_sel], sb_seq[ck_sel]));
   wire [SEQW-1:0]  df_nseq  = pick_am ? a_seq     : pick_ld ? lq_seq [ld_sel] : sb_seq [ck_sel];
   wire [CBITS-1:0] df_nck   = pick_am ? a_ck      : pick_ld ? lq_ck  [ld_sel] : sb_ck  [ck_sel];
   wire [3:0]       df_ncau  = pick_am ? stx_cause : pick_ld ? ldx_cause       : stx_cause;
   wire [AW-1:0]    df_ntval = pick_am ? a_addr    : pick_ld ? lq_addr[ld_sel] : sb_addr[ck_sel];

   // Register the report. dfault_v feeds backend_top's roll_v, which feeds our own
   // `rollback`; but the load/store select that produces df_now is itself combinationally
   // gated by `rollback` (the squash-this-cycle guards at the select loops). So a faulting
   // op would chase its own precise-trap rollback in a zero-delay loop (deselect -> fault
   // drops -> roll_v drops -> reselect -> ...). Latching the report breaks the cycle: the
   // output no longer depends combinationally on rollback. Cleared when the faulting op is
   // squashed -- its own precise trap rolls back to its ckpt (rollback_seq < df_seq), and a
   // branch redirect that kills it does the same.
   reg              df_v;
   reg [SEQW-1:0]   df_seq_r;
   reg [CBITS-1:0]  df_ck_r;
   reg [3:0]        df_cau_r;
   reg [AW-1:0]     df_tval_r;
   initial df_v = 1'b0;
   always @(posedge clk) begin
      if (reset) df_v <= 1'b0;
      // clear when our own trap is taken (one-cycle pulse, robust to checkpoint-index reuse
      // corrupting the seqno compare) OR when a branch rollback squashes the faulting op.
      else if (df_v && (dfault_taken || (rollback && older(rollback_seq, df_seq_r)))) df_v <= 1'b0;
      else if (df_now && (!df_v || older(df_nseq, df_seq_r))) begin
         df_v <= 1'b1; df_seq_r <= df_nseq; df_ck_r <= df_nck;
         df_cau_r <= df_ncau; df_tval_r <= df_ntval;
      end
   end
   assign dfault_v     = df_v;
   assign dfault_seq   = df_seq_r;
   assign dfault_ckpt  = df_ck_r;
   assign dfault_cause = df_cau_r;
   assign dfault_tval  = df_tval_r;

   always @(posedge clk) begin
      if (reset) begin p_v <= 1'b0; ast <= A_IDLE; rsv_v <= 1'b0; amo_wbv <= 1'b0; mem_ren <= 1'b0; mem_runcached <= 1'b0; end
      else begin
         amo_wbv <= 1'b0;                       // 1-cycle pulse unless A_WB sets it
         mem_ren <= 1'b0;                        // 1-cycle read-request pulse (set on a fresh mem_raddr)
         if (sel_fire) begin
            p_v <= 1'b1;
            p_pdst  <= lq_pd [ld_sel]; p_owner <= lq_own[ld_sel];
            p_seq   <= lq_seq[ld_sel]; p_ck    <= lq_ck [ld_sel];
            p_nb    <= lq_nb [ld_sel]; p_sgn   <= lq_sgn[ld_sel]; p_fp <= lq_fp[ld_sel];
            p_w0    <= lq_w0 [ld_sel]; p_w1    <= lq_w1 [ld_sel]; p_lb <= lq_lb[ld_sel];
            mem_raddr <= ld_pa;                 // physical address (Bare: == VA)
            mem_runcached <= ldx_uncached;      // Svpbmt: NC/IO load -> don't cache
            mem_ren   <= 1'b1;                  // request the read (mem_raddr valid next cycle)
         end else if (merge_adv) p_v <= 1'b0;   // MERGE emptied, nothing to load (else: stall)

         // ---- atomic FSM (solo: never overlaps a load's mem_raddr/p_*) ----
         case (ast)
           A_IDLE: if (amo_v) begin
                      a_addr<=amo_addr; a_data<=amo_data; a_func<=amo_func; a_sz<=amo_sz;
                      a_pdst<=amo_pdst; a_own<=amo_owner; a_ck<=amo_ckpt; a_seq<=amo_seq; ast<=A_WAIT;
                   end
           A_WAIT: if (!amo_pend && !p_v && amo_xok) begin   // stores drained, load pipe empty, xlate ok
                      mem_raddr <= amo_pa_al; a_wpa <= amo_pa_al; mem_ren <= 1'b1; ast<=A_RD;
                      mem_runcached <= stx_uncached; a_wnc <= stx_uncached;   // Svpbmt: NC/IO AMO
                   end
           A_RD:   if (mem_rvalid) begin a_rdval_q <= a_rdval;  // RMW read data returned
                      if (a_islr) begin rsv_v<=1'b1; rsv_w<=a_word; end
                      if (a_issc) rsv_v<=1'b0;
                      ast <= a_dowr ? A_WR : A_WB;   // write phase only if this AMO writes memory
                   end
           A_WR:   if (mem_wready) ast <= A_WB;      // hold the RMW write until accepted
           A_WB:   if (amo_wb_ok) begin           // owner lane free next cycle -> register wb
                      amo_wbv<=1'b1; amo_wbpd<=a_pdst; amo_wbow<=a_own;
                      amo_wbvl<=a_rdval_q; amo_wbck<=a_ck; amo_wbsq<=a_seq; ast<=A_IDLE;
                   end
         endcase
         // an intervening store to the reserved word breaks the reservation
         if (dr_v && mem_wready && rsv_v && (sb_addr[dr_sel][PAW-1:3] == rsv_w)) rsv_v <= 1'b0;
         // an in-flight AMO squashed by a rollback (its own page-fault trap rolls back to
         // a_ck) must reset the FSM -- else it sticks mid-RMW for a dead atomic. Driven here
         // (priority-last in the FSM's own block) so ast/rsv_v have a SINGLE driver.
         if (rollback && ast != A_IDLE && older(rollback_seq, a_seq)) begin ast <= A_IDLE; rsv_v <= 1'b0; end
      end
   end

   // ----------------------- MERGE (byte merge) -----------------------
   // The held load (p_*) is reduced (at fill) to {w0,w1,lb}. For load byte mb the absolute
   // position is lb+mb, which falls in word lwb (= w0 or w1) at byte lane `posw`. A store
   // covers that byte iff one of its two words equals lwb and the matching per-word mask
   // bit is set -- both are EQUALITY tests (no carry chain). Youngest older store wins.
   reg [63:0]     m_mrg;
   reg [7:0]      m_byt;
   reg            m_fwd, m_c0, m_c1;
   reg [SEQW-1:0] m_bseq;
   reg [3:0]      m_lp;
   reg [2:0]      m_posw;
   reg [WW-1:0]   m_lwb;
   reg [SBDEPTH-1:0] s_use;        // store is valid+ready+older-than-load (byte-independent)
   integer        mb, mj;
   reg [63:0]     c_val;
   always @* begin
      m_mrg = 64'd0;
      // hoist the per-store "older than this load" seqno compare out of the byte loop
      // (it does not depend on the byte) -- one 8-bit compare/store, not 8.
      for (mj = 0; mj < SBDEPTH; mj = mj + 1)
         s_use[mj] = sb_v[mj] && sb_rdy[mj] && ($signed(sb_seq[mj] - p_seq) < 0);
      for (mb = 0; mb < 8; mb = mb + 1) begin
         m_lp   = {1'b0, p_lb} + mb[3:0];              // 0..14 (no big carry: 3b + const)
         m_posw = m_lp[2:0];
         m_lwb  = m_lp[3] ? p_w1 : p_w0;               // which word this load byte is in
         m_fwd  = 1'b0; m_bseq = {SEQW{1'b0}};
         m_byt  = mem_rdata[mb*8 +: 8];                // default: memory (read @ mem_raddr)
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
      c_val = (p_nb==4'd1) ? (p_sgn ? {{56{m_mrg[7]}},  m_mrg[7:0]}  : {56'd0, m_mrg[7:0]})
            : (p_nb==4'd2) ? (p_sgn ? {{48{m_mrg[15]}}, m_mrg[15:0]} : {48'd0, m_mrg[15:0]})
            : (p_nb==4'd4) ? (p_fp  ? {32'hffffffff,    m_mrg[31:0]}      // FLW: NaN-box
                            : p_sgn ? {{32{m_mrg[31]}}, m_mrg[31:0]} : {32'd0, m_mrg[31:0]})
            : m_mrg;
   end

   // ---- writeback register (the byte-merge result, flopped; fires when lane free) ----
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
         r_v     <= merge_fire;
         r_pdst  <= p_pdst;  r_owner <= p_owner;
         r_val   <= c_val;   r_seq   <= p_seq;  r_ck <= p_ck;
      end
   end
   // present the registered result; a rollback that squashes this load the cycle it would
   // write back suppresses it (the MERGE-stage squash covers the cycle before).
   wire r_kill = rollback & older(rollback_seq, r_seq);
   // the atomic FSM writes rd back (and signals completion) in A_WB; it is solo so it
   // never collides with a normal load writeback.
   assign ld_wb_v      = amo_wbv ? 1'b1     : (r_v & ~r_kill);
   assign ld_wb_pdst   = amo_wbv ? amo_wbpd : r_pdst;
   assign ld_wb_owner  = amo_wbv ? amo_wbow : r_owner;
   assign ld_wb_val    = amo_wbv ? amo_wbvl : r_val;
   assign ld_wb_seq    = amo_wbv ? amo_wbsq : r_seq;
   assign ld_done      = amo_wbv ? 1'b1     : (r_v & ~r_kill);
   assign ld_done_ckpt = amo_wbv ? amo_wbck : r_ck;

   // ------------------------------ drain --------------------------------
   // (dr_v/dr_sel/dr_mask are declared+computed above, before the atomic FSM, since the
   //  FSM's reservation-clear references them.)
   always @* begin
      mem_cbo = 1'b0; mem_cbo_zero = 1'b0; mem_cbo_keep = 1'b0;
      if (amo_wr_now) begin                 // atomic RMW write (solo -> no drain conflict)
         mem_wen   = 1'b1;
         mem_waddr = a_wpa;                  // translated (aligned) physical address
         mem_wdata = a_wdata;
         mem_wmask = a_wmask;
         mem_wuncached = a_wnc;
      end else begin
         mem_wen   = dr_v;                   // dr_v already requires the store be xck'd (translated)
         mem_waddr = sb_pa[dr_sel];          // physical address (Bare: == VA, filled at fill-time)
         mem_wdata = sb_data[dr_sel];
         // a CBO carries no store data: drive a maintenance command (wmask=0 so the cache's
         // combinational store-write touches nothing) and let the cache act on its line.
         mem_wmask = sb_cbo[dr_sel] ? 8'd0 : dr_mask;
         mem_wuncached = sb_nc[dr_sel] & ~sb_cbo[dr_sel];
         mem_cbo      = dr_v & sb_cbo[dr_sel];
         mem_cbo_zero = sb_cboz[dr_sel];
         mem_cbo_keep = sb_cbok[dr_sel];
      end
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
               sb_pa[eidx]   <= f_addr;          // default PA==VA (Bare); overwritten by the Sv39 check
               sb_nc[eidx]   <= 1'b0;            // default cacheable (Bare); overwritten by the Sv39 check
               sb_xck[eidx]  <= ~xlate;          // Bare: drainable now; Sv39: await pre-commit check
               sb_xflt[eidx] <= 1'b0;
               sb_data[eidx] <= exe_st_data[i*64 +: 64];
               sb_nb[eidx]   <= exe_st_nb[i*4 +: 4];
               sb_cbo[eidx]  <= exe_st_cbo[i];
               sb_cboz[eidx] <= exe_st_cbo_zero[i];
               sb_cbok[eidx] <= exe_st_cbo_keep[i];
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
               lq_fp[lidx]   <= exe_ld_fp[i];
               lq_w0[lidx]   <= f_addr[PAW-1:3];
               lq_w1[lidx]   <= f_addr[PAW-1:3] + 1'b1;
               lq_lb[lidx]   <= f_addr[2:0];
               lq_rdy[lidx]  <= 1'b1;
            end
         end

         // (2b) pre-commit store-check result (one store/cycle, Sv39): mark the checked
         //      store drainable (stash its PA) or faulting (-> dfault drives a precise trap).
         if (st_ck_done) begin sb_xck[ck_sel] <= 1'b1; sb_pa[ck_sel] <= {{(AW-56){1'b0}}, stx_pa};
                               sb_nc[ck_sel] <= stx_uncached; end
         if (st_ck_flt)  begin sb_xck[ck_sel] <= 1'b1; sb_xflt[ck_sel] <= 1'b1; end

         // (3) the selected load advances into the MERGE stage (p_*) -- free its LQ entry
         //     when SELECT fires (it then lives in the pipeline, not the queue).
         if (sel_fire) lq_v[ld_sel] <= 1'b0;

         // (4) commit: mark this checkpoint's stores drainable
         if (commit)
            for (i = 0; i < SBDEPTH; i = i + 1)
               if (sb_v[i] && (sb_ck[i] == commit_idx)) sb_cmt[i] <= 1'b1;

         // (5) drain: retire the selected store from the buffer once the write is ACCEPTED
         //     (mem_wready). mem_wen=dr_v stays asserted, re-selecting the same store, until
         //     a multi-cycle D$ accepts it. (Tie mem_wready=1 -> frees next cycle, as before.)
         if (dr_v && mem_wready) sb_v[dr_sel] <= 1'b0;

         // (6) rollback: squash wrong-path entries (newer than the branch)
         if (rollback) begin
            for (i = 0; i < SBDEPTH; i = i + 1)
               if (sb_v[i] && older(rollback_seq, sb_seq[i])) sb_v[i] <= 1'b0;
            for (i = 0; i < LQDEPTH; i = i + 1)
               if (lq_v[i] && older(rollback_seq, lq_seq[i])) lq_v[i] <= 1'b0;
            // (the in-flight-AMO squash of ast/rsv_v lives in the AMO FSM block above, so
            //  those regs have a single driver -- avoids a multi-driven net.)
         end
      end
   end
endmodule

`default_nettype wire
