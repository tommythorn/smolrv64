`default_nettype none

// Blocking load/store unit for the in-order core -- stage M's memory engine.
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
// Semantics kept bit-identical to probe/lsu.v so the Simmerv cosim agrees:
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
    parameter [63:0] DRAM_BASE = 64'd0,
    parameter [63:0] DRAM_TOP  = 64'hFFFF_FFFF_FFFF_FFFF)
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
    output wire            xo_v,           // translation landed THIS cycle -> fill the entry
    output wire            xo_early,       // ...and the access started here too (req_early
                                           // honoured: the FSM was idle and the port free)


    // ---- translation context (from csr_file) ----
    input  wire [63:0]     xl_satp,
    input  wire [1:0]      xl_priv,
    input  wire            xl_sum,
    input  wire            xl_mxr,
    input  wire            xl_flush,
    output wire [55:0]     ptw_addr,
    output wire            ptw_read,
    input  wire [63:0]     ptw_rdata,
    input  wire            ptw_rvalid,

    // ---- data memory port ----
    output reg  [AW-1:0]   mem_raddr,
    output reg             mem_ren,
    output reg             mem_runcached,
    input  wire [63:0]     mem_rdata,
    input  wire            mem_rvalid,
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
   // NB: DRAM_BASE/DRAM_TOP cannot be used here -- on FPGA builds they are 0 and
   // all-ones (they exist for the MMU's unbacked-PA check), so they would make this
   // gate always true. This SoC puts every device below 0x8000_0000 (CLINT 0x0200_0000,
   // PLIC 0x0C00_0000, UART 0x1000_0000, virtio 0x1000_2000/3000) and DRAM above it.
   localparam [55:0] LSU_DRAM_BASE = 56'h8000_0000;
   // The effective physical address: the MMU's for an M request, ooo2_sq's for a commit.
   wire [55:0] eff_pa  = pt_start ? pt_pa : t_paddr;
   wire        eff_unc = pt_start ? pt_unc : t_uncached;
   wire        pa_dram = (eff_pa >= LSU_DRAM_BASE);
   wire        xl_can = ~req_amo & ~eff_cbo & pa_dram; // AMO pre-aligned; CBO is line-wide
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
   wire        xl_want  = req_valid & (st == S_IDLE);
   wire        pt_start = pt_v & (st == S_IDLE) & ~mmu_walking;
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
   wire [63:0] eff_st_data = sel_pt ? pt_data        : req_st_data;
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

   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_mmu
     (.clk(clk), .reset(reset),
      .req_valid(xl_req), .req_vaddr(req_vaddr),
      .req_access(is_lr ? 2'd1 : req_amo ? 2'd3 : req_store ? 2'd2 : 2'd1),
      .priv(xl_priv), .sum(xl_sum), .mxr(xl_mxr), .satp(xl_satp), .flush(xl_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .walking(mmu_walking), .t_ready(t_ready), .t_paddr(t_paddr), .t_fault(t_fault), .t_cause(t_cause),
      .t_uncached(t_uncached), .t_ok(t_ok), .t_fault_raw(t_fault_raw));

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
   wire xl_ok_f  = xl_f & t_ready & ~t_fault & ~xpage & ~pt_start;
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
   wire xl_early = xo_ok & req_early & (st == S_IDLE) & ~pt_start;
   wire start_ok = pt_start | xl_ok_f | xl_early;
   assign xo_v     = xo_ok;
   assign xo_early = xl_early;
   assign xo_pa  = t_paddr;
   assign xo_unc = t_uncached;

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
   wire [7:0] st_mask = (8'd1 << nb) - 8'd1;
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
   wire acc_done = ((st == S_ST)  & st_fin & ~xword_q)
                 | ((st == S_ST2) & st_fin)
                 | ((st == S_LD)  & mem_rvalid & ~xword_q)
                 | ((st == S_LD2) & mem_rvalid)
                 | ((st == S_ARD) & mem_rvalid & ~a_dowr)
                 | (amo_go & st_fin);
   // done/fault/rd_val are single outputs, so the completion is routed to whoever owns the
   // access. Without this a commit store finishing under M's pending op would be latched by
   // M as its own -- the same class of defect as rule D5's re-presented request.
   // `fault` needs no such split: it is qualified by xl_req, which is false unless M owns an
   // idle LSU, so a commit can never raise one (it was translated before it was buffered).
   assign done_acc = acc_done & ~own_pt;
   assign done     = fault | xo_ok | done_acc;
   assign pt_done  = acc_done & own_pt;
   assign pt_ack  = pt_start;
   assign rd_val = (st == S_ARD) ? a_rdval : amo_go ? amo_old_q : ld_val;
   assign idle   = (st == S_IDLE);

   // ------------------------------------------------------------------- FSM
   always @(posedge clk) begin
      if (reset) begin
         st <= S_IDLE; mem_ren <= 1'b0; rsv_v <= 1'b0; mem_runcached <= 1'b0;
         cos_pa <= 56'd0; cos_kind <= 2'd0;
      end else begin
         mem_ren <= 1'b0;                                  // one-cycle request pulse
         case (st)
           S_IDLE:
             if (start_ok) begin
                // See the ASSUMPTION above: non-DRAM traffic is never aligned by this
                // LSU, so if it straddles, the cache sees a span it is asserted never to
                // see -- and for MMIO the second beat would address the wrong register.
                if (~pa_dram & (wend > 5'd8))
                   $fatal(1, "ooo2_lsu: non-DRAM access straddles a word: pa=%h nb=%0d boff=%0d",
                          eff_pa, nb, boff);
                own_pt        <= pt_start | xl_early;   // ooo2_lq lands it either way
                own_pt_st     <= pt_start & pt_store;
                src_pt        <= pt_start;              // ...but the fields are M's
                nc_q          <= eff_unc;
                mem_runcached <= eff_unc;
                cos_pa        <= eff_pa;                      // exact, pre-alignment
                cos_kind      <= ((pt_start & pt_store) | req_store) ? 2'd2
                               : req_amo ? 2'd2 : 2'd1;
                xword_q <= xword;
                nb_q    <= nb;  sgn_q <= eff_signed;  fp_q <= eff_fp;
                boff_q  <= xl_can ? boff : 3'd0;   // AMO/CBO keep their own addressing
                pa2_q   <= (eff_pa & ~56'd7) + 56'd8;
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
                   mem_ren   <= 1'b1;
                   st        <= S_ARD;
                end else begin
                   pa_q <= xl_can ? (eff_pa & ~56'd7) : eff_pa;
                   mem_raddr <= xl_can ? ({{(AW-56){1'b0}}, eff_pa} & ~{{(AW-3){1'b0}}, 3'b111})
                                       :  {{(AW-56){1'b0}}, eff_pa};
                   mem_ren   <= 1'b1;
                   st        <= S_LD;
                end
             end
           S_LD:  if (mem_rvalid) begin
                     if (xword_q) begin
                        ld_lo_q   <= mem_rdata;
                        mem_raddr <= {{(AW-56){1'b0}}, pa2_q};
                        mem_ren   <= 1'b1;
                        st        <= S_LD2;
                     end else st <= S_IDLE;
                  end
           S_LD2: if (mem_rvalid) st <= S_IDLE;
           // A plain store leaves on ACCEPT, not on the ack (st_fin above): the cache captured
           // address, data and mask and finishes the write on its own, so the FSM is free for
           // the next request while that happens. Plan item 4, 2026-09-04: 5.00 -> 4.00 per store.
           S_ST:  if (st_fin) st <= (xword_q ? S_ST2 : S_IDLE);
           S_ST2: if (st_fin) st <= S_IDLE;
           S_ARD: if (mem_rvalid) begin
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
   always @(posedge clk) if (!reset) begin
      if (req_early & ~(req_xlate & ~req_store & ~req_amo & ~req_cbo))
         $fatal(1, "ooo2_lsu: req_early on an access that is not a translate-only load");
   end
endmodule

`default_nettype wire
