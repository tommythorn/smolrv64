`default_nettype none

// Bypassed execute variant (timing experiment vs the write-before-read
// exec_shard). Two stages so the RF read leaves the ALU's cycle:
//   RR (read):  rf_shard read ps1/ps2 -> register operands + ctl
//   EX (exec):  full bypass mux -> exec_alu -> register writeback
//
// "Full bypass" = NB = 2*SHARDS sources: every shard's EX-stage result (this
// cycle) and WB-stage result (last cycle), because between a consumer's RR read
// and the producer's RF write there are two in-flight results per shard. The EX
// path is therefore bypass-compare + NB:1 mux + ALU -- no LUTRAM read in series,
// which is the point of the experiment. RF is still written (wb_*_in) and read
// (RR) so both stage paths are measured.
module exec_shard_bp
  #(parameter SHARDS = 4,
    parameter SBITS  = 2,
    parameter NPHYS  = 128,
    parameter PBITS  = 7,
    parameter POOL   = 32,
    parameter IDXB   = 5,
    parameter NB     = 8)        // bypass sources = 2*SHARDS
   (input  wire                    clk,
    // RR-stage issue + payload
    input  wire                    iss_valid,
    input  wire [PBITS-1:0]        iss_pdst,
    input  wire                    iss_pdst_v,
    input  wire [PBITS-1:0]        iss_ps1,
    input  wire [PBITS-1:0]        iss_ps2,
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
    // RF write broadcast (from WB results)
    input  wire [SHARDS-1:0]       wb_valid_in,
    input  wire [SHARDS*PBITS-1:0] wb_pr_in,
    input  wire [SHARDS*64-1:0]    wb_val_in,
    // bypass sources (EX results this cycle + WB results last cycle)
    input  wire [NB-1:0]           byp_valid,
    input  wire [NB*PBITS-1:0]     byp_pr,
    input  wire [NB*64-1:0]        byp_val,
    // outputs
    output wire                    wb_valid,
    output wire [PBITS-1:0]        wb_pr,
    output wire [63:0]             wb_val,
    output wire [63:0]             agu_addr,
    output wire                    cmp_eq, cmp_lt, cmp_ltu);

   // -------- RR stage: read RF --------
   wire [63:0] rr_rs1, rr_rs2;
   rf_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
              .POOL(POOL), .IDXB(IDXB)) rf
     (.clk(clk), .wr_valid(wb_valid_in), .wr_pr(wb_pr_in), .wr_val(wb_val_in),
      .ra1(iss_ps1), .ra2(iss_ps2), .rd1(rr_rs1), .rd2(rr_rs2));

   // -------- RR/EX pipeline register --------
   reg               ex_valid, ex_pdstv, ex_w, ex_uw, ex_o2i, ex_link, ex_rvc, ex_mem;
   reg [PBITS-1:0]   ex_pdst, ex_ps1, ex_ps2;
   reg [5:0]         ex_op;
   reg [1:0]         ex_o1s;
   reg [63:0]        ex_rs1, ex_rs2, ex_imm, ex_pc;
   initial begin ex_valid=0; ex_pdstv=0; end
   always @(posedge clk) begin
      ex_valid<=iss_valid; ex_pdst<=iss_pdst; ex_pdstv<=iss_pdst_v;
      ex_ps1<=iss_ps1; ex_ps2<=iss_ps2; ex_rs1<=rr_rs1; ex_rs2<=rr_rs2;
      ex_op<=alu_op; ex_w<=alu_w; ex_uw<=alu_uw; ex_o1s<=op1_sel; ex_o2i<=op2_imm;
      ex_link<=res_link; ex_rvc<=is_rvc; ex_mem<=is_mem; ex_imm<=imm; ex_pc<=pc;
   end

   // -------- EX stage: full bypass mux --------
   reg  [63:0] bv1, bv2;
   integer k;
   always @* begin
      bv1 = ex_rs1;
      bv2 = ex_rs2;
      for (k = 0; k < NB; k = k + 1) begin
         if (byp_valid[k] && (ex_ps1 != {PBITS{1'b0}}) &&
             (byp_pr[k*PBITS +: PBITS] == ex_ps1)) bv1 = byp_val[k*64 +: 64];
         if (byp_valid[k] && (ex_ps2 != {PBITS{1'b0}}) &&
             (byp_pr[k*PBITS +: PBITS] == ex_ps2)) bv2 = byp_val[k*64 +: 64];
      end
   end

   wire [63:0] result;
   exec_alu ea
     (.alu_op(ex_op), .alu_w(ex_w), .alu_uw(ex_uw), .op1_sel(ex_o1s),
      .op2_imm(ex_o2i), .res_link(ex_link), .is_rvc(ex_rvc),
      .rs1_val(bv1), .rs2_val(bv2), .imm(ex_imm), .pc(ex_pc),
      .result(result), .addr(agu_addr),
      .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   // -------- EX/WB register --------
   reg            wbv;
   reg [PBITS-1:0] wbp;
   reg [63:0]     wbd;
   initial wbv = 0;
   always @(posedge clk) begin
      wbv <= ex_valid & ex_pdstv & ~ex_mem;
      wbp <= ex_pdst;
      wbd <= result;
   end
   assign wb_valid = wbv;
   assign wb_pr    = wbp;
   assign wb_val   = wbd;
endmodule

`default_nettype wire
