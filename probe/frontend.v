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
    input  wire                    create,
    input  wire                    commit,
    input  wire [CBITS-1:0]        commit_idx,
    input  wire                    rollback,
    input  wire [CBITS-1:0]        rollback_idx,
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
    output wire [CBITS-1:0]        r_ckpt,
    output wire [CBITS-1:0]        cur,
    output wire [SEQW-1:0]         cur_seq,     // fetch PC's seqno (for trap resume)
    output wire [IW-1:0]           stall);

   wire [IW-1:0]      f_slot_valid;
   wire [IW*32-1:0]   f_inst;
   wire [IW*PCW-1:0]  f_pc;
   wire [IW*SEQW-1:0] f_seq;
   wire               f_valid;

   fetch #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC)) u_fetch
     (.clk(clk), .reset(reset), .redirect(redirect), .redirect_pc(redirect_pc),
      .redirect_seq(redirect_seq), .solo_all(solo_all), .irq_inject(irq_inject),
      .imem_addr(imem_addr), .imem_ipc(imem_ipc), .imem_data(imem_data),
      .imem_avail(imem_avail), .ready(accept), .valid(f_valid),
      .slot_valid(f_slot_valid), .inst(f_inst), .pc(f_pc), .seq(f_seq), .cur_seq(cur_seq));

   decode_rename #(.IW(IW), .SEQW(SEQW), .ABITS(ABITS), .AREGS(AREGS),
                   .PBITS(PBITS), .NPHYS(NPHYS), .POOL(POOL), .HPTR(HPTR),
                   .SBITS(SBITS), .NCHK(NCHK), .CBITS(CBITS)) u_dr
     (.clk(clk), .reset(reset), .flush(redirect), .accept(accept),
      .inst(f_inst), .in_valid(f_slot_valid),
      .seq_in(f_seq), .pc_in(f_pc),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .ps3(ps3), .pdst(pdst),
      .r_is_branch(r_is_branch),
      .r_pay(r_pay), .r_ckpt(r_ckpt), .cur(cur), .stall(stall));
endmodule

`default_nettype wire
