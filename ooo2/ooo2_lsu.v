`default_nettype none

// Blocking load/store unit for ooo2_core -- stage M's memory engine.
//
// The OoO LSU exists to let loads and stores execute out of order against each
// other: a store buffer, a load queue, byte-granular store-to-load forwarding,
// seqno-keyed squash, two page-table walkers, commit-gated drain. NONE of that is
// needed here. One memory operation is in flight at a time and it is already the
// oldest instruction in the machine, so:
//
//   * ordering is program order, for free -- no disambiguation, no forwarding;
//   * a store is non-speculative by construction (nothing older can still trap,
//     nothing younger has passed M), so it writes the D$ directly -- no buffer;
//   * one MMU/walker serves loads, stores and atomics.
//
// The request is held stable by ooo2_core until `done` pulses. `done` is
// combinational in the completing cycle and `rd_val` is valid with it.
//
// Semantics kept bit-identical to the retired sharded core's lsu.v so the Simmerv cosim agrees:
//   * the D$ port is BYTE-ADDRESS-RELATIVE both ways -- rd_data[7:0] is the byte at
//     rd_addr, and wr_data[7:0] is written to wr_addr+0 under wr_mask -- so a
//     misaligned access needs no lane rotation here; cache.v resolves any
//     line-crossing internally via its two-phase lookup.
//   * a misaligned access that crosses a PAGE boundary is not translatable with one
//     walk, so it raises address-misaligned (cause 4 load / 6 store-AMO), exactly as
//     the OoO LSU does -- software emulates it.
//   * atomics read the containing 8-byte word, compute, and write it back under a
//     0x0F/0xF0/0xFF mask; `.W` selects its half with addr[2].
module ooo2_lsu
  #(parameter AW = 64,
    parameter [63:0] DRAM_TOP  = 64'd1 << 56,   // the MMU's physical-address cap (ooo2_core)
    parameter [63:0] LRAM_BASE = 64'h7000_0000,  // the local SRAM: memory behind the D$, aligned like DRAM
    parameter        LRAM_LG2  = 18,
    parameter        LDTW      = 2)     // the load-queue index width: a fast read's tag; 1<<LDTW loads outstanding
   (input  wire            clk,
    input  wire            reset,

    // ---- request (held stable until `done`) ----
    input  wire            req_valid,
    input  wire            req_store,      // plain store (not AMO)
    input  wire            req_amo,
    input  wire [4:0]      req_amo_func,   // funct5
    input  wire            req_cbo,        // Zicbom/Zicboz: maintenance, no data write
    input  wire            req_cbo_zero,
    input  wire            req_cbo_keep,
    input  wire [63:0]     req_vaddr,
    input  wire [1:0]      req_size,       // 0=B 1=H 2=W 3=D
    input  wire            req_signed,     // load sign-extends
    input  wire            req_fp,         // FLW -> NaN-box the 32-bit result
    input  wire [63:0]     req_st_data,    // rs2

    // ---- PRE-TRANSLATED ACCESS PORT ------------------------------------------------
    // Routed through this FSM rather than given its own path to the cache (rule C2): the
    // straddle beat, mem_wready handshaking, device decode from mem_wabase and the LR/SC
    // reservation clear all live below, and a second writer would have to duplicate every
    // one of them. What the commit needs instead is a mux on the TRANSLATION RESULT --
    // the PA and the uncached bit, which ooo2_sq carries -- and everything downstream is
    // the existing code, untouched.
    //
    // Holds priority over an FSM-STARTING req_* when both want an idle LSU: the committing
    // store is at the ROB head, so it is unconditionally older, and M's op is free to wait a
    // cycle. A translate-only req_* does not compete: it never takes the FSM (see xl_x).
    input  wire            pt_v,
    input  wire            pt_store,       // 1 = store draining from ooo2_sq, 0 = load from ooo2_lq
    input  wire [55:0]     pt_pa,          // physical: translated when the op executed
    input  wire [1:0]      pt_size,
    input  wire [63:0]     pt_data,        // store data
    input  wire            pt_signed,      // load formatting
    input  wire            pt_fp,
    input  wire            pt_unc,         // the uncached bit decided at translate time
    output wire            pt_done,
    output wire            pt_ack,         // accepted this cycle -- the requester may advance
    output wire            pt_is_store,    // direction of the access IN FLIGHT
    output wire            pt_ld_done,     // pt_done for a LOAD, from the load terms alone (see acc_done)
    output wire            pt_ld_kill,     // ...and that landing is wrong-path (squashed): do not write it back

    // ---- TRANSLATE-ONLY: the address pass of a buffered store or a queued load ------
    // Neither accesses memory when it executes: it translates, hands the PA to ooo2_sq or
    // ooo2_lq and completes in that cycle. Translation and its faults are exactly what
    // S_IDLE already does, so this is a completion arm, not a state: the FSM stays idle.
    // The access happens later, through the pre-translated port above.
    input  wire            req_xlate,
    // ...and this translate-only LOAD may also ISSUE its access in the same pass. Set by the
    // core when no store older than it is live, so there is no ordering test to wait for.
    // What it saves is exactly ooo2_lq's SELECT cycle: the entry is still filled and M is
    // still released on xo_v, and ooo2_lq still lands the data.
    input  wire            req_early,
    output wire [55:0]     xo_pa,
    output wire            xo_unc,
    output wire            xo_mem,         // the translated PA is DRAM/LRAM (idempotent, speculatable)
    output wire            xo_v,           // translation landed THIS cycle -> fill the entry
    output wire            xo_early,       // ...and the access started here too (req_early
                                           // honoured: the FSM was idle and the port free)


    // ---- translation context (from csr_file) ----
    input  wire [63:0]     xl_satp,
    input  wire [1:0]      xl_priv,
    input  wire            xl_sum,
    input  wire            xl_mxr,
    input  wire            xl_flush,
    input  wire            flush,          // a backend redirect: squash a speculative LOAD in flight
    input  wire [LDTW-1:0] pt_tag,         // the load-queue entry the port's candidate is (C4a)
    input  wire [LDTW-1:0] req_tag,        // ...and the one M's own load (req_early) is
    output wire [LDTW-1:0] pt_rtag,        // the entry whose data lands this cycle (fast path)
    output wire            pt_fast_done,   // a queued load landed on the fast path
    input  wire            m_head,         // M's op is the ROB head (non-speculative) -- gates non-DRAM access
    input  wire            pt_nonspec,     // the port's request is non-speculative (a store, or the LQ's candidate at a live head)
    output wire [55:0]     ptw_addr,
    output wire            ptw_read,
    input  wire [63:0]     ptw_rdata,
    input  wire            ptw_rvalid,

    // ---- data memory port ----
    output reg  [AW-1:0]   mem_raddr,
    output reg             mem_ren,
    output reg             mem_runcached,
    input  wire [63:0]     mem_rdata,
    input  wire            mem_rvalid,     // the FSM's own response (the slow tag), level-held
    output wire            mem_rfast,      // this read is a fast one: tag it mem_rtag
    output wire [LDTW-1:0] mem_rtag,
    input  wire            mem_rvalid_c,   // a fast-tagged cache response, ONE cycle
    input  wire [LDTW-1:0] mem_rtag_resp,
    input  wire [63:0]     mem_rdata_c,
    input  wire            mem_rbusy,      // the last read is not yet accepted by the cache
    output wire            mem_wen,
    output wire [AW-1:0]   mem_waddr,
    output wire [AW-1:0]   mem_wabase,   // the access base PA (pa_q), NOT the per-beat address
    output wire [63:0]     mem_wdata,
    output wire [7:0]      mem_wmask,
    output wire            mem_wuncached,
    output wire            mem_cbo,
    output wire            mem_cbo_zero,
    output wire            mem_cbo_keep,
    input  wire            mem_wready,
    input  wire            mem_waccept,    // the write was TAKEN (the cache captured it); a plain store is done here, not at wready

    // ---- completion ----
    // `started` is the DISPATCH point, and the reason non-blocking loads stay precise with no
    // ROB walk: mis_flt and xl_flt are both qualified by xl_req (= req_valid & st == S_IDLE),
    // so misalignment, page-cross and translation -- PTW included, since S_IDLE is held until
    // t_ready -- are all decided BEFORE the access begins. Once the FSM leaves S_IDLE the
    // access cannot fault, so M can let go here.
    output wire            started,
    output wire            done,
    output wire            dtlb_walking,   // the data MMU is walking this cycle (HPM DT_WALK)
    output wire            dtlb_walk_beg,  // ...and this is the walk's first cycle (HPM DTLB_MISS)
    output wire            done_acc,       // the ACCESS part of `done` alone: an access this
                                           // stage started has completed. Never the translate
                                           // pass or a fault -- see ooo2_core's writeback valids
    output wire [63:0]     rd_val,
    output wire            fault,
    output wire [3:0]      fault_cause,
    output wire [63:0]     fault_tval,
    // COSIM memory-effect capture: the EXACT (unaligned) PA of the access and its kind.
    // pa_q is not usable for this -- for DRAM it is the 8-byte-ALIGNED base, while the
    // reference model reports the exact address.  Unused outside cosim (DCE'd).
    output reg  [55:0]     cos_pa,
    output reg  [1:0]      cos_kind,      // 0 = none, 1 = load, 2 = store
    output reg  [63:0]     cos_data,      // a plain store's value (raw rs2) and log2 size; size 4'hF = not a
    output reg  [3:0]      cos_size,      //   plain store's write (load, AMO's RMW, SC, cbo): not data-checked
    output wire            ld_busy,        // a LOAD access is in flight, hit or miss (MEM_LDINFL; counters only)
    // This LSU's invariants, made visible to hardware (rv_errlog); registered here, see
    // the INTEGRITY LOG block at the bottom of the file for the bit assignment.
    output wire [15:0]     err,
    output wire            idle);          // no memory op in flight (fence.i drain)

   localparam S_IDLE = 3'd0, S_LD = 3'd1, S_ST = 3'd2,
              S_LD2  = 3'd5, S_ST2 = 3'd6,   // second aligned word
              S_ARD  = 3'd3, S_AWR = 3'd4;

   reg [2:0]   st;
   reg [55:0]  pa_q;                       // translated physical address (WORD-ALIGNED)

   // ---- word-aligned D$ access (see header) --------------------------------------
   wire [2:0]  boff   = eff_vaddr[2:0];              // byte offset in the aligned word
   wire [4:0]  wend   = {2'd0, boff} + {1'd0, nb};   // one past the last byte in-word
   // MMIO must keep its EXACT address: devices decode by low address bits, so an
   // aligned-plus-mask access lands on the wrong register (this hung virtio-net at
   // boot), so only DRAM traffic is aligned here.
   //
   // ASSUMPTION, asserted below: no NON-DRAM access ever straddles an 8-byte word.
   // The header used to claim "only DRAM traffic goes through the cache", which is
   // false -- soc_top routes everything that is not a device window to the D$, and the
   // on-chip boot/monitor SRAM at LBASE (0x7000_0000) is not a device. So SRAM IS
   // cached, and being non-DRAM it is NOT aligned here; a straddling SRAM access would
   // reach the cache spanning and trip rv_cache.v's NO-SPAN $fatal.
   //
   // It holds for two different reasons, neither of them enforced by construction:
   //   MMIO  -- device registers are naturally-aligned and accessed at their own width.
   //   SRAM  -- the boot/monitor image is assumed to issue only aligned accesses
   //            (agreed 2026-08-19; the region exists for the Tenstorrent test suite
   //            and may become conditional, which would retire this case entirely).
   // An assumption that holds by luck on one address range is exactly what
   // docs/rtl-rules.md says must be an assertion, so it is one.
   // This SoC puts every device below 0x8000_0000 (CLINT 0x0200_0000,
   // PLIC 0x0C00_0000, UART 0x1000_0000, virtio 0x1000_2000/3000) and DRAM above it.
   localparam [55:0] LSU_DRAM_BASE = 56'h8000_0000;
   // The effective physical address: the MMU's for an M request, ooo2_sq's for a commit.
   wire [55:0] eff_pa;  wire eff_unc;   // the port's fields when it starts or continues, else M's (below)
   wire        pa_dram = (eff_pa >= LSU_DRAM_BASE);
   // The local SRAM (the ROM monitor's home, 0x7000_0000 on the platform) is memory behind
   // the D$ too, and below DRAM: unaligned, its byte loads reached the D$ as spans and the
   // monitor could not be simulated at all until 2026-09-05 (it worked on the board on the
   // D$'s span path, the one this LSU exists to keep the cache from ever seeing).
   wire        pa_lram = (eff_pa[55:LRAM_LG2] == LRAM_BASE[55:LRAM_LG2]);
   wire        pa_mem  = pa_dram | pa_lram;
   wire        xl_can = ~req_amo & ~eff_cbo & pa_mem;  // AMO pre-aligned; CBO is line-wide
   wire        xword  = xl_can & (wend > 5'd8);      // operand straddles two words
   reg         xword_q;
   reg  [2:0]  boff_q;
   reg  [3:0]  nb_q;      // load-format width/sign/NaN-box, latched with the request
   reg         sgn_q, fp_q;
   reg  [55:0] pa2_q;                                // the next aligned word
   reg  [63:0] ld_lo_q;                              // first word's data
   // boff_q is 1..7 whenever xword_q, so sh_up is 8..56 -- never a 64-bit shift.
   wire [5:0]  sh_dn  = {boff_q, 3'b000};
   wire [5:0]  sh_up  = 6'd0 - {boff_q, 3'b000};     // == 64 - 8*boff (mod 64)
   reg [63:0]  amo_old_q;                  // AMO's rd value, captured at the RMW read
   reg         nc_q;                       // Svpbmt: this access is NC/IO
   initial begin st = S_IDLE; end

   // ---------------------------------------------------------- classification
   wire        is_lr    = req_amo & (req_amo_func == 5'b00010);
   wire        is_sc    = req_amo & (req_amo_func == 5'b00011);
   // Which requester owns the access in flight. done/fault/rd_val are single outputs, so
   // without this a commit store finishing under M's pending op would be latched by M as
   // its own completion.
   // WHO reports the access and WHERE its fields came from are two questions, and an
   // early-started queued load answers them differently: its address and size are M's own
   // request -- this IS M's translate pass -- but ooo2_lq lands it, so it reports on pt_done.
   // One signal for both would let pt_*, the queue's NEXT candidate, drive nb/boff/st_mask
   // while an early load sat in S_LD: inert today, and precisely the shape of defect this
   // file's history is made of.
   reg         own_pt, own_pt_st, src_pt;
   initial     begin own_pt = 1'b0; own_pt_st = 1'b0; src_pt = 1'b0; end
   assign      pt_is_store = own_pt_st;
   // A PRE-TRANSLATED ACCESS MUST NOT STEAL THE FSM FROM A PAGE-TABLE WALK. The MMU is
   // driven by xl_req, which is qualified by `st == S_IDLE`; letting pt_start take the FSM
   // while a walk is in progress drops req_valid under the MMU, the walk restarts, and a
   // steady supply of queued loads keeps it restarting forever. Every -v- test hung.
   // `walking` is the MMU's own state register, not t_ready. t_ready is a function of
   // req_valid, so gating req_valid on it closes a combinational loop THROUGH the MMU --
   // which is exactly what the first attempt did, and Verilator reported it as "Active
   // region did not converge" rather than as a hang.
   wire        mmu_walking;
   // Named for the CPI stack (2026-09-05): a walk costs its cycles inside ST_MEM, where a
   // 16-entry direct-mapped dTLB could hide a whole run's loss (three runs in ~25 of the
   // sha256 loop read 0.71 with LSU 36% and the same miss rate) and nothing counted it.
   reg         mmu_walking_q;
   always @(posedge clk) mmu_walking_q <= reset ? 1'b0 : mmu_walking;
   assign dtlb_walking  = mmu_walking;
   assign dtlb_walk_beg = mmu_walking & ~mmu_walking_q;
   wire        xl_want  = req_valid & (st == S_IDLE);
   // A read may start only when the read request buffer (mem_raddr/mem_ren, one deep) is free:
   // not presenting a request this cycle and not still waiting for the cache's accept. And a
   // fast load may not reuse a tag whose response is still on its way (C4a).
   wire        port_free = ~mem_rbusy & ~mem_ren;
   reg [(1<<LDTW)-1:0] o_v;                      // per tag: a response is still coming
   wire        pt_start = pt_v & (st == S_IDLE) & ~mmu_walking
                        & (pt_store | (port_free & ~o_v[pt_tag]));
   // A TRANSLATE-ONLY REQUEST DOES NOT ARBITRATE, AND DOES NOT WAIT FOR THE FSM. It needs the
   // MMU and nothing else: no bank, no pa_q, no state. It used to be gated with everything
   // else on `~pt_start` and `st == S_IDLE`, which put the pre-translated port's grant -- the
   // store queue's live bits, the load queue's candidate, the alias matrix -- in series with
   // M's completion for every plain load and store, and M's completion is the wakeup
   // broadcast, the redirect and the hpm events: 1708 of the 3401 endpoints under +0.35 ns
   // in the 2026-09-03 routed checkpoint started at u_sq/v_reg for exactly this reason.
   // Now the MMU sees it whenever M presents it. A walk it starts continues while the FSM
   // runs a queued access (the walker's reads go through the D$ port arbiter, behind this
   // FSM's own access, and complete on their own), and the request is never withdrawn under
   // the walker by the FSM leaving S_IDLE -- which was the restart hazard the old gating
   // guarded against from the other side. The early START is still an FSM matter (xl_early).
   // An FSM-starting request likewise ASKS the MMU whenever it waits in S_IDLE; the port's
   // grant decides only whether it may START this cycle (xl_ok_f below). Gating the request
   // itself on ~pt_start put the grant inside `fault`, and `fault` is M's completion: the
   // 2026-09-03 build after the translate-only cut still had 1632 near-critical endpoints
   // starting at u_lq/sqt_reg, all through pt_start -> xl_f -> fault -> lsu_done -> m_done.
   wire        xl_x     = req_valid & req_xlate;
   wire        xl_f     = xl_want & ~req_xlate;

   // The request the FSM actually sees. In S_IDLE the selector is pt_start (src_pt still
   // holds the PREVIOUS access's source and would be stale); once running it is src_pt.
   // pt_pa's low bits ARE the VA's low bits -- a page offset survives translation -- so
   // boff/wend/alignment below are correct from it.
   wire        sel_pt      = (st == S_IDLE) ? pt_start : src_pt;
   wire [63:0] eff_vaddr   = sel_pt ? {8'd0, pt_pa}  : req_vaddr;
   wire [1:0]  eff_size    = sel_pt ? pt_size        : req_size;
   // The store data is REGISTERED at the handoff (st_data_q): the queue pops its entry then,
   // so the head has moved on by the time the write is presented.
   reg  [63:0] st_data_q;
   wire [63:0] eff_st_data = src_pt ? st_data_q      : req_st_data;
   wire        eff_cbo     = sel_pt ? 1'b0           : req_cbo;
   wire        eff_signed  = sel_pt ? pt_signed      : req_signed;
   wire        eff_fp      = sel_pt ? pt_fp          : req_fp;

   wire [3:0]  nb       = 4'd1 << eff_size;   // the access that STARTS: M's or the port's
   // M'S OWN FAULT TESTS USE M'S OWN WIDTH. They used `nb`, which follows sel_pt, i.e. in
   // S_IDLE the port's grant: in the cycle a queued access is granted, M's translate pass
   // judged its page crossing with the port's size -- wrong on its face, and it was also
   // the door the store queue's head walked through into M's completion (head -> l_block
   // -> the queue's candidate -> pt_start -> sel_pt -> nb -> xpage -> xo_ok -> done):
   // 5667 near-critical endpoints in the 2026-09-04 build C.
   wire [3:0]  req_nb   = 4'd1 << req_size;
   wire        wr_class = req_store | (req_amo & ~is_lr);   // store-class for translation/faults

   // ------------------------------------------------------------ translation
   wire        t_ready, t_fault, t_uncached, t_ok, t_fault_raw;
   wire [55:0] t_paddr;
   wire [3:0]  t_cause;
   // An FSM-starting access asks only while it is actually waiting in S_IDLE and the port is
   // not taking the FSM this cycle: once started, the access owns pa_q. The translate-only
   // pass asks unconditionally (xl_x). The MMU's t_ok/t_fault_raw carry no req_valid, so the
   // translate-only answers below are ANDed with xl_x alone and never see xl_f's gating.
   wire        xl_req = xl_x | xl_f;

   mmu #(.AW(56), .DRAM_TOP(DRAM_TOP)) u_mmu
     (.clk(clk), .reset(reset),
      .req_valid(xl_req), .req_vaddr(req_vaddr),
      .req_access(is_lr ? 2'd1 : req_amo ? 2'd3 : req_store ? 2'd2 : 2'd1),
      .priv(xl_priv), .sum(xl_sum), .mxr(xl_mxr), .satp(xl_satp), .flush(xl_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .walking(mmu_walking), .t_ready(t_ready), .t_paddr(t_paddr), .t_fault(t_fault), .t_cause(t_cause),
      .t_lvl(), .t_uncached(t_uncached), .t_ok(t_ok), .t_fault_raw(t_fault_raw));

   // A misaligned access whose byte span leaves the page needs a second translation.
   // Raise address-misaligned instead (cause 4 load / 6 store-AMO) and let software
   // emulate -- the OoO LSU makes the same call.
   wire xpage   = (({1'b0, req_vaddr[11:0]} + {9'd0, req_nb}) > 13'h1000);

   // AMOs (and LR/SC) must be naturally aligned. The RMW datapath is built around
   // the containing 8-byte word -- pa_q is aligned down and a_wmask is word-relative
   // -- so without this check an unaligned AMO corrupts neighbouring bytes silently
   // rather than trapping. nb-1 is the alignment mask (nb=8 truncates to 3'b000, so
   // the 4-bit subtract yields 4'd7 as intended).
   wire [3:0] al_mask = req_nb - 4'd1;
   wire amo_mis = req_amo & ((req_vaddr[3:0] & al_mask) != 4'd0);

   wire mis_flt = (xl_x & xpage) | (xl_f & (xpage | amo_mis));
   wire xl_flt  = (xl_x & t_ok & t_fault_raw) | (xl_f & t_ready & t_fault);

   assign fault       = req_valid & (mis_flt | xl_flt);
   assign fault_cause = mis_flt ? (wr_class ? 4'd6 : 4'd4) : t_cause;
   assign fault_tval  = req_vaddr;

   // an FSM-starting access can start: translated cleanly this cycle, FSM idle, port free
   wire xl_ok_f  = xl_f & t_ready & ~t_fault & ~xpage & ~pt_start & (req_store | port_free);
   // the translate-only pass completes: translated cleanly this cycle, whatever the FSM does
   wire xo_ok    = xl_x & t_ok & ~t_fault_raw & ~xpage;
   // NO disambiguation here any more. ooo2_lq owns the ordering test, against a REGISTERED
   // address, so it is off the translate path entirely -- that is the whole reason the queue
   // exists (see its header).
   // ...unless req_early says the same pass may issue the access. Only the START moves:
   // xo_ok still fires, so ooo2_lq is still filled and M is still released this cycle. The
   // start is an FSM matter, so it is the one translate-only thing that still needs S_IDLE
   // and yields to the port; when it yields, the load simply goes the queue's way, and
   // ooo2_lq is told which happened through xo_early rather than re-deriving it.
   // A speculative early access to NON-DRAM (device/MMIO) must not happen: the read has side
   // effects (e.g. popping a UART RX byte on the wrong path). Only DRAM/LRAM speculate early;
   // a non-DRAM load waits until M is the ROB head (non-speculative). See ooo2_lq for the
   // queued path's matching gate.
   wire xl_early = xo_ok & req_early & (t_mem | m_head) & (st == S_IDLE) & ~pt_start
                 & port_free & ~o_v[req_tag];   // the translate's region (see xo_mem)
   wire start_ok = pt_start | xl_ok_f | xl_early;
   assign xo_v     = xo_ok;
   assign xo_early = xl_early;
   assign xo_pa  = t_paddr;
   assign xo_unc = t_uncached;
   // THE TRANSLATE'S OWN REGION, NEVER eff_pa's. eff_pa is the PORT's address whenever the port
   // starts or chains an access in the same cycle a translate completes (pt_start | take_next),
   // and that is exactly when a load's fill would have taken the PORT's DRAM classification:
   // a device load recorded as idempotent, issued off the wrong path past ooo2_lq's head gate,
   // reading a read-to-clear register on the way (2026-09-17: the board's dead NIC under NFS
   // root; the cosim never took a PLIC interrupt until the OOO2_IRQ_STIM storm caught it).
   wire t_mem = (t_paddr >= LSU_DRAM_BASE) | (t_paddr[55:LRAM_LG2] == LRAM_BASE[55:LRAM_LG2]);
   assign xo_mem = t_mem;

   // ------------------------------------------------------- AMO RMW datapath
   wire        a_isw   = (req_size == 2'd2);
   wire        a_half  = req_vaddr[2];
   wire [31:0] a_old32 = a_half ? mem_rdata[63:32] : mem_rdata[31:0];
   wire [31:0] a_d32   = req_st_data[31:0];
   wire [63:0] a_data  = req_st_data;

   // LR/SC reservation: one word-granular reservation register.
   reg         rsv_v;
   reg [35:0]  rsv_w;
   wire [35:0] a_word = req_vaddr[38:3];
   wire        a_scok = rsv_v && (rsv_w == a_word);

   reg [63:0] a_resv;
   always @* begin
      case (req_amo_func)
        5'b00001: a_resv = a_data;                                                     // swap
        5'b00000: a_resv = a_isw ? {32'b0, a_old32 + a_d32}  : mem_rdata + a_data;      // add
        5'b00100: a_resv = a_isw ? {32'b0, a_old32 ^ a_d32}  : mem_rdata ^ a_data;      // xor
        5'b01100: a_resv = a_isw ? {32'b0, a_old32 & a_d32}  : mem_rdata & a_data;      // and
        5'b01000: a_resv = a_isw ? {32'b0, a_old32 | a_d32}  : mem_rdata | a_data;      // or
        5'b10000: a_resv = a_isw ? {32'b0, ($signed(a_old32)<$signed(a_d32))?a_old32:a_d32}
                                 : ($signed(mem_rdata)<$signed(a_data))?mem_rdata:a_data;
        5'b10100: a_resv = a_isw ? {32'b0, ($signed(a_old32)>$signed(a_d32))?a_old32:a_d32}
                                 : ($signed(mem_rdata)>$signed(a_data))?mem_rdata:a_data;
        5'b11000: a_resv = a_isw ? {32'b0, (a_old32<a_d32)?a_old32:a_d32}
                                 : (mem_rdata<a_data)?mem_rdata:a_data;
        5'b11100: a_resv = a_isw ? {32'b0, (a_old32>a_d32)?a_old32:a_d32}
                                 : (mem_rdata>a_data)?mem_rdata:a_data;
        5'b00011: a_resv = a_data;                                                     // SC stores rs2
        default:  a_resv = mem_rdata;                                                  // LR: no write
      endcase
   end
   wire [63:0] a_oldv  = a_isw ? {{32{a_old32[31]}}, a_old32} : mem_rdata;
   wire [63:0] a_rdval = is_sc ? (a_scok ? 64'd0 : 64'd1) : a_oldv;   // SC: 0=ok 1=fail
   wire        a_dowr  = is_lr ? 1'b0 : is_sc ? a_scok : 1'b1;
   wire [63:0] a_wdata = a_isw ? (a_half ? {a_resv[31:0], 32'b0} : {32'b0, a_resv[31:0]}) : a_resv;
   wire [7:0]  a_wmask = a_isw ? (a_half ? 8'hF0 : 8'h0F) : 8'hFF;

   // -------------------------------------------------------- load formatting
   // The port returns the aligned word, so the LSU shifts the operand down itself and
   // splices the second word in when the operand straddled.
   wire [63:0] mem_rdata_eff = (st == S_LD2)
                             ? ((ld_lo_q >> sh_dn) | (mem_rdata << sh_up))
                             : (mem_rdata >> sh_dn);
   // FORMATTED FROM THE LATCHED REQUEST, not the live one. req_size/req_signed/req_fp come
   // straight from the M-stage registers, which was safe only while M was guaranteed to still
   // be holding this very load. With M released at dispatch they belong to whatever
   // instruction is in M when the data comes back, and an `ld` returning 0x80044000 gets
   // formatted as a byte load -- i.e. 0. boff_q was already latched here for the same reason;
   // these three were not.
   wire [63:0] ld_val = (nb_q == 4'd1) ? (sgn_q ? {{56{mem_rdata_eff[7]}},  mem_rdata_eff[7:0]}
                                                : {56'd0, mem_rdata_eff[7:0]})
                      : (nb_q == 4'd2) ? (sgn_q ? {{48{mem_rdata_eff[15]}}, mem_rdata_eff[15:0]}
                                                : {48'd0, mem_rdata_eff[15:0]})
                      : (nb_q == 4'd4) ? (fp_q  ? {32'hffffffff, mem_rdata_eff[31:0]}   // FLW: NaN-box
                                       : sgn_q  ? {{32{mem_rdata_eff[31]}}, mem_rdata_eff[31:0]}
                                                : {32'd0, mem_rdata_eff[31:0]})
                      :                 mem_rdata_eff;

   // ------------------------------------------------------------ write port
   wire st_go = (st == S_ST) || (st == S_ST2), amo_go = (st == S_AWR);
   wire st2_go = (st == S_ST2);
   // nb_q, NOT nb: the queue pops at the handoff, so the port's size names the NEXT entry
   // while this store is presented (the first version wrote an `sd` with a one-byte mask).
   wire [7:0] st_mask = (8'd1 << nb_q) - 8'd1;
   assign mem_wen       = st_go | amo_go;
   assign mem_waddr     = {{(AW-56){1'b0}}, (st2_go ? pa2_q : pa_q)};
   // The access BASE, distinct from mem_waddr's per-beat address. A straddling store's second
   // beat addresses pa2_q, but straddling requires xl_can, which requires pa_dram -- so a
   // DEVICE access is NEVER in S_ST2. soc_top decodes devices from this base, which keeps
   // st2_go and pa2_q (and the mux they drive) out of the device-decode cone: that mux select
   // was the startpoint of the worst path in the design at 6 ns,
   //   FSM_onehot_st[3] -> st2_go -> is_uart_w -> dmem_wready -> lsu_done -> redirect
   //                    -> iMMU tag compare -> u_bp/ycorr_qv
   assign mem_wabase    = {{(AW-56){1'b0}}, pa_q};
   // Store data is placed at its byte offset inside the aligned word; the straddling
   // remainder starts at byte 0 of the next word.
   assign mem_wdata     = amo_go ? a_wdata
                        : st2_go ? (eff_st_data >> sh_up)
                                 : (eff_st_data << sh_dn);
   assign mem_wmask     = amo_go ? a_wmask
                        : eff_cbo ? 8'd0                              // CBO carries no data
                        : st2_go  ? (st_mask >> (4'd8 - {1'b0, boff_q}))
                                  : (st_mask << boff_q);
   assign mem_wuncached = nc_q & ~eff_cbo;
   assign mem_cbo       = st_go & eff_cbo;
   assign mem_cbo_zero  = mem_cbo & req_cbo_zero;
   assign mem_cbo_keep  = mem_cbo & req_cbo_keep;

   // ------------------------------------------------------------ completion
   // `started` is M's early-release signal for a non-blocking load. A commit store starting
   // is not M's access and must not pulse it.
   assign started = start_ok & ~pt_start;
   // WHAT ENDS A WRITE. A plain cached store (and an AMO's write) is done when the D$ TAKES
   // it (mem_waccept: address, data and mask captured, the write completes on its own). A
   // CBO or an uncached write waits for its completion (mem_wready): a cbo.flush must have
   // reached L2 before the doorbell store behind it, and an NC store must be in DDR before
   // a later device write can start the DMA that reads it. Decided by the request's class,
   // registered at start (nc_q) or held by M (eff_cbo) -- never by which ack shows up.
   wire st_fin   = (eff_cbo | nc_q) ? mem_wready : mem_waccept;
   // BACK-TO-BACK STORES WITHOUT THE TRIP THROUGH S_IDLE (2026-09-05). The D$'s accept is a
   // REGISTER now -- the combinational accept -> LSU -> store queue path of the first
   // version cost 0.65 ns and failed timing (build N) -- so it lands the cycle after the
   // door took the write, while this FSM still presents it (the door is shut that cycle:
   // S_CHECK). In that cycle the next queued store, if any, is loaded straight into S_ST,
   // which keeps the store stream at one write per two cycles: the D$'s S_FIN door takes
   // it. The queue pops at the handoff (pt_ack), never at the accept; nothing can pass a
   // store that sits here, because every access goes through this FSM.
   wire take_next = (st == S_ST) & st_fin & ~xword_q & src_pt & pt_v & pt_store & ~mmu_walking;   // src_pt: M's op (a CBO) is not a store to chain from
   assign eff_pa  = (pt_start | take_next) ? pt_pa  : t_paddr;
   assign eff_unc = (pt_start | take_next) ? pt_unc : t_uncached;

   // ---------------------------------------------- FAST PATH: a queued load, non-blocking (C4a)
   // A queued load that is cached, inside one word and to memory (DRAM or the local SRAM) needs
   // nothing from the FSM after its read is issued: the address is translated, the fault decided,
   // and the only per-request state is how to format the word when it comes back. So it starts
   // from S_IDLE and the FSM STAYS THERE; the next access may start behind it, and its response
   // is claimed by the TAG the load queue allocated (its entry index, rule B1), never by "there
   // is only one in flight". The formatting state lives per tag (o_*): M and the queue have long
   // moved on when the data returns. Straddles, device and uncached loads, AMOs and stores keep
   // the FSM path, whose responses carry one fixed tag of their own (the SoC's TAG_SLOW), so a
   // slow access and any number of fast ones are in flight together. Re-done from da28895b.
   localparam NLD = 1 << LDTW;
   wire [LDTW-1:0] ld_tag     = pt_start ? pt_tag : req_tag;
   wire            ld_fast_ok = start_ok & ((pt_start & ~pt_store) | xl_early)
                              & ~xword & ~eff_unc & pa_mem;
   reg  [3:0]      o_nb   [0:NLD-1];             // per tag: how to format the word
   reg  [NLD-1:0]  o_sgn, o_fp, o_kill;          // o_kill: the load was squashed while in flight
   reg  [2:0]      o_boff [0:NLD-1];
   integer         oi;
   initial begin o_v = {NLD{1'b0}}; o_sgn = {NLD{1'b0}}; o_fp = {NLD{1'b0}}; o_kill = {NLD{1'b0}};
                 for (oi = 0; oi < NLD; oi = oi + 1) begin o_nb[oi] = 4'd0; o_boff[oi] = 3'd0; end end
   // THE TAG IS FIXED FOR THE REQUEST'S WHOLE LIFETIME. mem_ren is registered, so the cache sees
   // the request one cycle after the start; pt_tag by then names the queue's NEXT candidate. The
   // tag rides with mem_raddr as a register of its own, written at exactly the sites that write
   // mem_raddr (the FSM below) -- not on every start. A store starts while the port is busy (it
   // needs no read); clearing the fast bit on ITS start once re-tagged a read still waiting for
   // the cache's accept as a slow one (tiny128 froze at 8.3 M retires, 2026-09-04).
   reg  [LDTW-1:0] rtag_q;
   reg             rfast_q;
   initial begin rtag_q = {LDTW{1'b0}}; rfast_q = 1'b0; end
   assign mem_rtag  = rtag_q;
   assign mem_rfast = rfast_q;
   wire ld_fast_ret = mem_rvalid_c & o_v[mem_rtag_resp];
   assign pt_rtag       = mem_rtag_resp;
   assign pt_fast_done  = ld_fast_ret & ~o_kill[mem_rtag_resp];   // a squashed load's data is dropped
   wire [5:0]  f_sh  = {o_boff[mem_rtag_resp], 3'b000};
   wire [63:0] f_eff = mem_rdata_c >> f_sh;
   wire [3:0]  f_nb  = o_nb[mem_rtag_resp];
   wire        f_sgn = o_sgn[mem_rtag_resp], f_fp = o_fp[mem_rtag_resp];
   wire [63:0] ld_fast_val =
        (f_nb == 4'd1) ? (f_sgn ? {{56{f_eff[7]}},  f_eff[7:0]}  : {56'd0, f_eff[7:0]})
      : (f_nb == 4'd2) ? (f_sgn ? {{48{f_eff[15]}}, f_eff[15:0]} : {48'd0, f_eff[15:0]})
      : (f_nb == 4'd4) ? (f_fp  ? {32'hffffffff, f_eff[31:0]}
                        : f_sgn ? {{32{f_eff[31]}}, f_eff[31:0]} : {32'd0, f_eff[31:0]})
      :                  f_eff;
   always @(posedge clk) begin
      if (reset) begin o_v <= {NLD{1'b0}}; o_kill <= {NLD{1'b0}}; end
      else begin
         if (ld_fast_ret) o_v[mem_rtag_resp] <= 1'b0;
         if (ld_fast_ok) begin
            o_v[ld_tag] <= 1'b1;  o_kill[ld_tag] <= 1'b0;  o_nb[ld_tag] <= nb;  o_boff[ld_tag] <= boff;
            o_sgn[ld_tag] <= eff_signed;  o_fp[ld_tag] <= eff_fp;
         end
         // a redirect fires at the ROB head: every fast load in flight is younger, wrong-path --
         // including one STARTING in this very cycle (the queue's candidate is younger than the
         // head too; the slow path's ld_sq kills that case the same way). The response still
         // comes (the tag stays allocated until then) and is dropped.
         if (flush) o_kill <= o_kill | o_v | (ld_fast_ok ? ({{(NLD-1){1'b0}}, 1'b1} << ld_tag) : {NLD{1'b0}});
      end
   end
   // B-rule protocol: a response is matched by a tag the requester allocated. These three
   // are the checks that caught C4a step 2's first attempt on its first simulation run,
   // and they are the ones that matter most once the D$ grows MSHRs (C5) -- so they are
   // wires, and the integrity log carries them onto the board.
   wire e_tag_reissue = ld_fast_ok & o_v[ld_tag] & ~(ld_fast_ret & (mem_rtag_resp == ld_tag));
   wire e_tag_orphan  = mem_rvalid_c & ~o_v[mem_rtag_resp];
   wire e_tag_reuse   = ld_fast_ok & ld_fast_ret & (mem_rtag_resp == ld_tag);
   always @(posedge clk) if (!reset) begin
      if (e_tag_reissue)
         $fatal(1, "ooo2_lsu: tag %0d reissued while its response is outstanding", ld_tag);
      if (e_tag_orphan)
         $fatal(1, "ooo2_lsu: fast response with tag %0d that nothing is waiting on", mem_rtag_resp);
      if (e_tag_reuse)
         $fatal(1, "ooo2_lsu: tag %0d lands and restarts in one cycle (the queue reused the slot)", ld_tag);
   end

`ifdef LSUDBG
   // +dbg_line=<PA>: every start, landing, kill and write touching that 64-byte line (rule G6: aimed by plusarg)
   reg [63:0] dbg_line; initial if (!$value$plusargs("dbg_line=%h", dbg_line)) dbg_line = 64'hFFFF_FFFF_FFFF_FFFF;
   always @(posedge clk) if (!reset) begin
      if (start_ok && (eff_pa[55:6] == dbg_line[55:6]))
         $display("[lsu t=%0t START pa=%h fast=%b tag=%0d pt=%b st=%b early=%b nb=%0d boff=%0d unc=%b xword=%b fsm=%0d o_v=%b port_free=%b]",
                  $time, eff_pa, ld_fast_ok, ld_tag, pt_start, pt_start & pt_store, xl_early, nb, boff, eff_unc, xword, st, o_v, port_free);
      if (mem_rvalid_c)
         $display("[lsu t=%0t FASTRESP tag=%0d o_v=%b kill=%b data=%h fmt=%h nb=%0d boff=%0d]", $time, mem_rtag_resp, o_v[mem_rtag_resp], o_kill[mem_rtag_resp], mem_rdata_c, ld_fast_val, f_nb, o_boff[mem_rtag_resp]);
      if (mem_wen && (mem_wabase[55:6] == dbg_line[55:6]))
         $display("[lsu t=%0t WRITE addr=%h data=%h mask=%b accept=%b]", $time, mem_waddr, mem_wdata, mem_wmask, mem_waccept);
      if (flush && |o_v) $display("[lsu t=%0t FLUSH o_v=%b]", $time, o_v);
   end
`endif
   // THE SLOW PATH YIELDS ITS COMPLETION CYCLE TO A FAST LANDING. One PRF write port, one ROB
   // completion port: a device response (the FSM's, level-held by the SoC until consumed) and a
   // fast cache response can land in the same cycle; the fast one is a single-cycle strobe, the
   // slow one waits a cycle.
   wire slow_rv  = mem_rvalid & ~ld_fast_ret;
   wire acc_done = ((st == S_ST)  & st_fin & ~xword_q)
                 | ((st == S_ST2) & st_fin)
                 | ((st == S_LD)  & slow_rv & ~xword_q)
                 | ((st == S_LD2) & slow_rv)
                 | ((st == S_ARD) & slow_rv & ~a_dowr)
                 | (amo_go & st_fin);
   // done/fault/rd_val are single outputs, so the completion is routed to whoever owns the
   // access. Without this a commit store finishing under M's pending op would be latched by
   // M as its own -- the same class of defect as rule D5's re-presented request.
   // `fault` needs no such split: it is qualified by xl_req, which is false unless M owns an
   // idle LSU, so a commit can never raise one (it was translated before it was buffered).
   assign done_acc = acc_done & ~own_pt;
   assign done     = fault | xo_ok | done_acc;
   assign pt_done  = acc_done & own_pt;
   // A LOAD'S LANDING FROM THE LOAD TERMS ALONE (plan item T1 (L) step 3, 2026-09-07). pt_done
   // & ~pt_is_store is the same value, but its cone is all of acc_done, and the store terms'
   // st_fin selects on eff_cbo, which selects on pt_start -- this cycle's NEW start -- so
   // every landing wake began at pt_v: on 142 of gate W6's 200 worst paths. The load terms
   // are the FSM state, the D$'s registered rvalid and xword_q. Asserted equal below.
   wire ld_fin = ((st == S_LD) & slow_rv & ~xword_q) | ((st == S_LD2) & slow_rv);
   assign pt_ld_done = ld_fin & own_pt & ~own_pt_st;
   // `!==` in the assertion (an X on either side is a defect too); `!=` in the err bit,
   // because hardware has no X to find.
   wire e_ld_done = (pt_ld_done != (pt_done & ~pt_is_store));
   always @(posedge clk)
      if (!reset && (pt_ld_done !== (pt_done & ~pt_is_store)))
         $fatal(1, "ooo2_lsu: pt_ld_done %b != pt_done & ~store %b (st=%0d)", pt_ld_done, pt_done & ~pt_is_store, st);

   // SPECULATIVE-LOAD SQUASH. With control flow off the M pipe, M now executes loads past an
   // unresolved branch, so a wrong-path load can be walking/accessing when a redirect fires.
   // Its landing must NOT write back (its physreg has been rolled back -- ooo2_pending's zombie
   // check). Stores/AMOs set own_pt_st and are commit-gated, so they are never speculative here.
   // The FSM still completes the access (own_pt clears on ld_fin); only the LANDING is killed.
   // THE LIVE FLUSH DOES NOT REACH THE KILL -- only the latch does. A load that lands in the
   // redirect cycle itself lands: every consumer orders its flush arm last (scheduler, ROB,
   // LQ, pending), so the landing completes a ROB slot the same edge drops and writes a
   // physreg the same edge returns to the free list, both harmless. Gating the landing on the
   // LIVE flush closed a combinational loop instead: redirect = m_red_fire needs m_done_red,
   // m_done_red yields to ld_land, ld_land was ~flush. Vivado cut that loop at an arbitrary
   // arc (nine TIMING-23 loops, every one through ld_land), turned synthesis RETIMING OFF for
   // the whole design because of it, and the redirect cone (mideleg, the ROB head, the SQ's
   // commit) sat in front of every scheduler's wakeup: mideleg -> redirect -> ld_land ->
   // we_ld -> e_r, 25 levels, the IW=3 wall. Verilator saw the same loop as UNOPTFLAT.
   wire ld_inflight = own_pt & ~own_pt_st;
   reg  ld_sq;  initial ld_sq = 1'b0;
   always @(posedge clk)
      if (reset)                    ld_sq <= 1'b0;
      else if (flush)               ld_sq <= 1'b1;   // a redirect: any load already in flight is wrong-path
      else if (pt_start | xl_early) ld_sq <= 1'b0;   // a fresh access start (incl. the early path) is correct-path
   assign pt_ld_kill = ld_inflight & ld_sq;
   assign pt_ack  = pt_start | take_next;
   assign rd_val = pt_fast_done ? ld_fast_val
                 : (st == S_ARD) ? a_rdval : amo_go ? amo_old_q : ld_val;
   assign idle   = (st == S_IDLE);
   assign ld_busy = ld_inflight;

   // The FSM below asserts these two inline; they are qualified by the state that
   // evaluates them, so the integrity-log bit means the same thing the $fatal does and
   // not merely "the address is odd". Declared here, where the FSM reads them.
   wire e_dev_spec = (st == S_IDLE) & start_ok & ~pa_mem & ~(pt_start ? pt_nonspec : m_head);
   wire e_dev_span = (st == S_IDLE) & start_ok & ~pa_mem & (wend > 5'd8);

   // ------------------------------------------------------------------- FSM
   always @(posedge clk) begin
      if (reset) begin
         st <= S_IDLE; mem_ren <= 1'b0; rsv_v <= 1'b0; mem_runcached <= 1'b0;
         cos_pa <= 56'd0; cos_kind <= 2'd0; cos_data <= 64'd0; cos_size <= 4'hF;
      end else begin
         mem_ren <= 1'b0;                                  // one-cycle request pulse
         case (st)
           S_IDLE:
             if (start_ok) begin
                // See the ASSUMPTION above: non-DRAM traffic is never aligned by this
                // LSU, so if it straddles, the cache sees a span it is asserted never to
                // see -- and for MMIO the second beat would address the wrong register.
                // A NON-DRAM ACCESS IS NEVER SPECULATIVE: the port's request carries the LQ's
                // head compare (or is a committed store), the early path carries M's.
                if (e_dev_spec)
                   $fatal(1, "ooo2_lsu: speculative non-DRAM access: pa=%h pt_start=%b xl_early=%b", eff_pa, pt_start, xl_early);
                if (e_dev_span)
                   $fatal(1, "ooo2_lsu: non-DRAM access straddles a word: pa=%h nb=%0d boff=%0d va=%h unc=%b pt_start=%b xl_early=%b req_early=%b m_head=%b pt_store=%b st=%0d",
                          eff_pa, nb, boff, req_vaddr, eff_unc, pt_start, xl_early, req_early, m_head, pt_store, st);
                own_pt        <= (pt_start | xl_early) & ~ld_fast_ok;   // ooo2_lq lands it; a fast load's landing is by tag
                own_pt_st     <= pt_start & pt_store;
                src_pt        <= pt_start;              // ...but the fields are M's
                nc_q          <= eff_unc;
                mem_runcached <= eff_unc;
                cos_pa        <= eff_pa;                      // exact, pre-alignment
                cos_kind      <= ((pt_start & pt_store) | req_store) ? 2'd2
                               : req_amo ? 2'd2 : 2'd1;
                cos_data      <= pt_start ? pt_data : req_st_data;
                // a cbo is a store class with no data of its own (cbo.zero: the reference sees eight
                // 8-byte stores, the DUT one line operation of size field 0): not data-checked
                cos_size      <= (((pt_start & pt_store) | req_store) & ~eff_cbo) ? {2'b0, eff_size} : 4'hF;
                xword_q <= xword;
                nb_q    <= nb;  sgn_q <= eff_signed;  fp_q <= eff_fp;
                boff_q  <= xl_can ? boff : 3'd0;   // AMO/CBO keep their own addressing
                pa2_q   <= (eff_pa & ~56'd7) + 56'd8;
                if (pt_start) st_data_q <= pt_data;
                if (pt_start ? pt_store : req_store) begin
                   pa_q <= xl_can ? (eff_pa & ~56'd7) : eff_pa;
                   st   <= S_ST;
                end else if (req_amo) begin
                   // An atomic reads, modifies and writes the CONTAINING 8-BYTE WORD:
                   // a_wdata/a_wmask are built relative to that word (a_half = addr[2]
                   // picks the .W half). The memory port is byte-address-relative, so
                   // the write must use the ALIGNED address too -- using the raw PA
                   // shifts a .W half-word write 4 bytes past its target.
                   pa_q      <= eff_pa & ~56'd7;
                   mem_raddr <= {{(AW-56){1'b0}}, eff_pa} & ~{{(AW-3){1'b0}}, 3'b111};
                   mem_ren   <= 1'b1;  rfast_q <= 1'b0;
                   st        <= S_ARD;
                end else begin
                   pa_q <= xl_can ? (eff_pa & ~56'd7) : eff_pa;
                   mem_raddr <= xl_can ? ({{(AW-56){1'b0}}, eff_pa} & ~{{(AW-3){1'b0}}, 3'b111})
                                       :  {{(AW-56){1'b0}}, eff_pa};
                   mem_ren   <= 1'b1;  rfast_q <= ld_fast_ok;  rtag_q <= ld_tag;
                   // a fast load leaves the FSM idle: everything it still needs is in o_*
                   st        <= ld_fast_ok ? S_IDLE : S_LD;
                end
             end
           S_LD:  if (slow_rv) begin
                     if (xword_q) begin
                        ld_lo_q   <= mem_rdata;
                        mem_raddr <= {{(AW-56){1'b0}}, pa2_q};
                        mem_ren   <= 1'b1;  rfast_q <= 1'b0;
                        st        <= S_LD2;
                     end else st <= S_IDLE;
                  end
           S_LD2: if (slow_rv) st <= S_IDLE;
           // A plain store leaves on ACCEPT, not on the ack (st_fin above): the cache captured
           // address, data and mask and finishes the write on its own, so the FSM is free for
           // the next request while that happens. Plan item 4, 2026-09-04: 5.00 -> 4.00 per store.
           S_ST:  if (st_fin) begin
                     if (take_next) begin                 // the next queued store, from here
                        own_pt <= 1'b1; own_pt_st <= 1'b1; src_pt <= 1'b1;
                        nc_q <= pt_unc; mem_runcached <= pt_unc;
                        cos_pa <= pt_pa; cos_kind <= 2'd2; cos_data <= pt_data; cos_size <= {2'b0, pt_size};
                        xword_q <= xword; nb_q <= nb; boff_q <= xl_can ? boff : 3'd0;
                        pa2_q <= (pt_pa & ~56'd7) + 56'd8;
                        pa_q  <= xl_can ? (pt_pa & ~56'd7) : pt_pa;
                        st_data_q <= pt_data;
                     end else st <= (xword_q ? S_ST2 : S_IDLE);
                  end
           S_ST2: if (st_fin) st <= S_IDLE;
           S_ARD: if (slow_rv) begin
                     amo_old_q <= a_rdval;
                     if (is_lr) begin rsv_v <= 1'b1; rsv_w <= a_word; end
                     if (is_sc) rsv_v <= 1'b0;
                     st <= a_dowr ? S_AWR : S_IDLE;
                  end
           S_AWR: if (st_fin) st <= S_IDLE;
           default: st <= S_IDLE;
         endcase
         // a plain store to the reserved word breaks the reservation
         // A committing store carries a PA and the reservation is held as a VA, so it cannot
         // compare. It clears unconditionally instead, which is architecturally free: SC is
         // permitted to fail spuriously, and this cannot livelock. The constrained LR/SC
         // sequence may not contain a store, so the only way to clear a live reservation is
         // an OLDER buffered store draining after the LR executed -- and that store is one
         // dynamic instruction, gone once it commits, so the retry succeeds.
         if (st_go && st_fin && rsv_v && (own_pt_st || (req_vaddr[38:3] == rsv_w)))
            rsv_v <= 1'b0;
      end
   end

   // req_early is a claim about the requester's own bookkeeping -- that ooo2_lq will land
   // this access -- so it is only meaningful on a translate-only LOAD. On anything else the
   // access would start and then be reported to nobody.
   wire e_req_early = req_early & ~(req_xlate & ~req_store & ~req_amo & ~req_cbo);
   // M's own accesses (AMO, LR/SC, CBO) write memory or reserve it: they start only at the
   // ROB head, never off a path a branch may still squash (rule D17).
   wire e_m_spec = (st == S_IDLE) & xl_ok_f & ~m_head;
   always @(posedge clk) if (!reset) begin
      if (e_req_early)
         $fatal(1, "ooo2_lsu: req_early on an access that is not a translate-only load");
      if (e_m_spec)
         $fatal(1, "ooo2_lsu: an AMO or CBO started off the ROB head: va=%h cbo=%b amo=%b", req_vaddr, req_cbo, req_amo);
   end

   // ---- INTEGRITY LOG (rv_errlog) ---------------------------------------------------
   // Every condition above, latched into a sticky bit that software can read off a board
   // that has already crashed. A1 makes these always-on in simulation; on hardware $fatal
   // is a no-op and the cone is deleted, so without this the shipping design enforces
   // nothing -- and simulation reaches ~1.5e9 cycles where Geekbench reaches ~3e11.
   //   0 dev_spec    a speculative access to a non-DRAM address
   //   1 dev_span    a non-DRAM access straddling an 8-byte word
   //   2 tag_reissue a load tag reissued while its response is outstanding
   //   3 tag_orphan  a fast response carrying a tag nothing is waiting on
   //   4 tag_reuse   a tag lands and restarts in one cycle (the queue reused the slot)
   //   5 ld_done     pt_ld_done disagrees with pt_done & ~store
   //   6 req_early   req_early on an access that is not a translate-only load
   //   7 m_spec      an AMO or CBO started off the ROB head
   reg [15:0] err_q;
   initial err_q = 16'd0;
   always @(posedge clk)
      err_q <= reset ? 16'd0
             : {8'd0, e_m_spec, e_req_early, e_ld_done, e_tag_reuse, e_tag_orphan, e_tag_reissue,
                e_dev_span, e_dev_spec};
   assign err = err_q;
endmodule

`default_nettype wire
