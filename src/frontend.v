`include "exec_pay.vh"
`default_nettype none

// Sharded-OoO frontend: PC -> fetch/align -> decode -> [registered boundary] ->
// rename. This is the whole front of the new core, ready to drop onto the
// scheduler/execution backend (it replaces the smolrv64 inner core + frontend;
// caches/TLB/devices are reused around it).
//
// Two pipeline stages: (1) Fetch+Decode is one combinational cloud from the PC
// register through imem read, the aligner, and decode_stage, terminating at the
// decode/rename boundary register inside decode_rename; (2) Rename. So a bundle
// fetched in cycle T is renamed in cycle T+1.
//
// Back-pressure is NOT wired yet: fetch.ready is tied high and the boundary
// register advances every cycle. Wiring `stall` back to freeze both is the next
// step. Per-slot PC is produced by fetch but not yet carried into rename (the
// branch/scheduler path will need it) -- a deliberate TODO.
module frontend
  #(parameter IW   = 4,
    parameter HW   = 8,
    parameter PCW  = 64,
    parameter SEQW = 8,
    parameter [PCW-1:0] RESET_PC = 0,
    parameter ABITS = 6,
    parameter AREGS = 64,
    parameter PBITS = 8,
    parameter NPHYS = 256,
    parameter POOL  = 64,
    parameter HPTR  = 6,
    parameter SBITS = 2,
    parameter NCHK  = 4,
    parameter CBITS = 2)
   (input  wire                    clk,
    input  wire                    reset,
    // redirect (branch mispredict / exception / CPR rollback)
    input  wire                    redirect,    // fetch -> target (also flushes the boundary)
    input  wire [PCW-1:0]          redirect_pc,
    input  wire [SEQW-1:0]         redirect_seq,
    input  wire                    solo_all,    // align one instruction per bundle (fault replay)
    input  wire                    irq_inject,  // inject the interrupt pseudo-op at the current PC
    // instruction memory (combinational read)
    output wire [PCW-1:0]          imem_addr,
    output wire [PCW-1:0]          imem_ipc,    // PC of the instruction being fetched (fault EPC)
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // back-pressure: accept a new bundle this cycle (else freeze fetch + boundary)
    input  wire                    accept,
    // checkpoint / commit control (rename time domain)
    input  wire                    create,       // per-bundle dispatch (alloc/MAP/pold)
    input  wire                    ckpt_create,  // per-checkpoint close (coarse CPR: span + chk_map)
    input  wire                    commit,
    input  wire [CBITS-1:0]        commit_idx,
    input  wire                    rollback,
    input  wire [CBITS-1:0]        rollback_idx,
    // branch resolve/training port (EX domain; oldest resolved CTI this cycle)
    input  wire                    res_v,
    input  wire                    res_cbr,
    input  wire                    res_call,
    input  wire                    res_ret,
    input  wire                    res_taken,
    input  wire [CBITS-1:0]        res_ckpt,
    input  wire [PCW-1:0]          res_tgt,
    input  wire                    res_rep,     // resolve caused this rollback -> GHR LSB repair
    // renamed bundle out (one cycle after fetch), aligned with r_valid/r_seq
    output wire [IW-1:0]           r_valid,
    output wire [IW*SEQW-1:0]      r_seq,
    output wire [IW*ABITS-1:0]     r_rd,
    output wire [IW-1:0]           r_rd_v,
    output wire [IW*PBITS-1:0]     ps1,
    output wire [IW*PBITS-1:0]     ps2,
    output wire [IW*PBITS-1:0]     ps3,
    output wire [IW*PBITS-1:0]     pdst,
    output wire [IW-1:0]           r_is_branch,
    output wire [IW*`PAYW-1:0]     r_pay,
    output wire [PCW-1:0]          r_pred_npc,  // dispatching bundle's chosen next PC
    output wire [CBITS-1:0]        r_ckpt,
    output wire [CBITS-1:0]        cur,
    output wire [SEQW-1:0]         cur_seq,     // fetch PC's seqno (for trap resume)
    output wire [IW-1:0]           stall);

   wire [IW-1:0]      f_slot_valid;
   wire [IW*32-1:0]   f_inst;
   wire [IW*PCW-1:0]  f_pc;
   wire [IW*SEQW-1:0] f_seq;
   wire               f_valid;
   wire               bp_v, f_brt;
   wire [PCW-1:0]     bp_tgt, f_npc, f_pnpc, f_ftn;

   fetch #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC)) u_fetch
     (.clk(clk), .reset(reset), .redirect(redirect), .redirect_pc(redirect_pc),
      .redirect_seq(redirect_seq), .solo_all(solo_all), .irq_inject(irq_inject),
      // apc is unconnected here, so its whole cone drops out; apred_v only feeds it.
      .pred_v(bp_v), .apred_v(bp_v), .pred_tgt(bp_tgt), .npc(f_npc), .apc(/* ooo2 only */),
      .pred_npc(f_pnpc), .pnpc_kind(), .ft_npc(f_ftn),
      .br_term(f_brt),
      .imem_addr(imem_addr), .imem_ipc(imem_ipc), .imem_data(imem_data),
      .imem_avail(imem_avail), .ready(accept), .valid(f_valid),
      .slot_valid(f_slot_valid), .inst(f_inst), .pc(f_pc), .seq(f_seq), .cur_seq(cur_seq));

   // branch predictor: predicts from registered BTB/RAS state only (no byte
   // inspection -- CTI class is trained at resolve), on the checkpoint wires the
   // renamer already consumes (create/cur/rollback/rollback_idx). imem_ipc =
   // pc_q = bundle base; ft_npc doubles as the call return address.
   predictor #(.PCW(PCW), .CBITS(CBITS), .NCHK(NCHK)) u_bp
     (.clk(clk), .reset(reset),
      .npc(f_npc), .fire(accept & f_valid), .base_pc(imem_ipc), .ft_npc(f_ftn),
      .cti_ok(f_brt),
      .pred_v(bp_v), .pred_tgt(bp_tgt),
      .create(ckpt_create), .cur(cur), .rollback(rollback), .rollback_idx(rollback_idx),
      .res_v(res_v), .res_cbr(res_cbr), .res_call(res_call), .res_ret(res_ret),
      .res_taken(res_taken), .res_ckpt(res_ckpt), .res_tgt(res_tgt), .res_rep(res_rep));

   decode_rename #(.IW(IW), .SEQW(SEQW), .ABITS(ABITS), .AREGS(AREGS),
                   .PBITS(PBITS), .NPHYS(NPHYS), .POOL(POOL), .HPTR(HPTR),
                   .SBITS(SBITS), .NCHK(NCHK), .CBITS(CBITS)) u_dr
     (.clk(clk), .reset(reset), .flush(redirect), .accept(accept),
      .inst(f_inst), .in_valid(f_slot_valid),
      .seq_in(f_seq), .pc_in(f_pc), .pred_npc_in(f_pnpc), .r_pred_npc(r_pred_npc),
      .create(create), .ckpt_create(ckpt_create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .ps3(ps3), .pdst(pdst),
      .r_is_branch(r_is_branch),
      .r_pay(r_pay), .r_ckpt(r_ckpt), .cur(cur), .stall(stall));
endmodule

`default_nettype wire
