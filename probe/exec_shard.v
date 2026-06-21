`default_nettype none

// One shard's execute slice: read this shard's RF copy, run exec_alu, drive a
// writeback. The writeback broadcast (all shards' results) comes in and is
// written into this shard's RF copy at the next edge; this shard's own result is
// also broadcast out for the siblings (and for the scheduler wake).
//
// Forwarding for 1-cycle ALU ops is write-before-read, no bypass net: a producer
// issued in cycle T reads operands and computes combinationally in T; its result
// rides wb_* and is registered into EVERY shard's RF copy at edge T->T+1; the
// dependent (woken at that same edge) issues in T+1 and simply reads the RF.
//
// Loads/branches/stores: agu_addr (= rs1+imm) and cmp_* are produced for the
// (later) LSU and branch-resolution units; they do not write back here. A store
// has no rd so wb_valid is naturally 0. Loads' real value comes from the LSU, so
// here wb is suppressed for memory ops (is_mem) -- they complete later.
module exec_shard
  #(parameter SHARDS = 4,
    parameter SBITS  = 2,
    parameter NPHYS  = 128,
    parameter PBITS  = 7,
    parameter POOL   = 32,
    parameter IDXB   = 5)
   (input  wire                    clk,
    // issue from this shard's scheduler
    input  wire                    iss_valid,
    input  wire [PBITS-1:0]        iss_pdst,
    input  wire                    iss_pdst_v,    // writes a register
    input  wire [PBITS-1:0]        iss_ps1,
    input  wire [PBITS-1:0]        iss_ps2,
    // execute payload (decode_exec ctl + imm/pc)
    input  wire [5:0]              alu_op,
    input  wire                    alu_w,
    input  wire                    alu_uw,
    input  wire [1:0]              op1_sel,
    input  wire                    op2_imm,
    input  wire                    res_link,
    input  wire                    is_rvc,
    input  wire                    is_mem,
    input  wire [63:0]             imm,
    input  wire [63:0]             pc,
    // writeback broadcast (all shards) -> RF writes
    input  wire [SHARDS-1:0]       wb_valid_in,
    input  wire [SHARDS*PBITS-1:0] wb_pr_in,
    input  wire [SHARDS*64-1:0]    wb_val_in,
    // this shard's writeback out (one broadcast lane + the scheduler wake source)
    output wire                    wb_valid,
    output wire [PBITS-1:0]        wb_pr,
    output wire [63:0]             wb_val,
    // for the later LSU / branch unit
    output wire [63:0]             agu_addr,
    output wire                    cmp_eq,
    output wire                    cmp_lt,
    output wire                    cmp_ltu);

   wire [63:0] rs1_val, rs2_val;
   rf_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
              .POOL(POOL), .IDXB(IDXB)) rf
     (.clk(clk), .wr_valid(wb_valid_in), .wr_pr(wb_pr_in), .wr_val(wb_val_in),
      .ra1(iss_ps1), .ra2(iss_ps2), .rd1(rs1_val), .rd2(rs2_val));

   wire [63:0] result;
   exec_alu ea
     (.alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw), .op1_sel(op1_sel),
      .op2_imm(op2_imm), .res_link(res_link), .is_rvc(is_rvc),
      .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm), .pc(pc),
      .result(result), .addr(agu_addr),
      .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   // ALU/link ops write back now; memory ops complete via the LSU (later)
   assign wb_valid = iss_valid & iss_pdst_v & ~is_mem;
   assign wb_pr    = iss_pdst;
   assign wb_val   = result;
endmodule

`default_nettype wire
