`include "exec_pay.vh"
`default_nettype none

// Decode -> rename, across a registered stage boundary.
//
//   inst/in_valid/seq_in --[decode_stage (comb)]--> contract --[REG]--> renamer
//
// decode_stage produces the renamer's exact input contract (operands with
// explicit valids, {ARCH|SLOT} source redirects, last-writer-wins map_writer).
// We register that whole contract once at the decode/rename boundary, then feed
// the registered copy to renamer_bundle. The O(W^2) cross-slot matrix is thus
// computed once, in decode, and is no longer in the rename critical path -- the
// register cuts the two stages apart. The decoded valid/seq are registered too
// so they stay aligned with the renamed operands for downstream stages.
//
// Back-pressure (freeze the boundary register on stall / no checkpoint / <2 free
// regs) is deliberately NOT wired yet -- this composition establishes the clean
// datapath; the boundary advances every cycle. See the plan's back-pressure note.
module decode_rename
  #(parameter IW     = 4,    // bundle width = SHARDS
    parameter SEQW   = 8,
    parameter ABITS  = 6,
    parameter AREGS  = 64,
    parameter PBITS  = 8,
    parameter NPHYS  = 256,
    parameter POOL   = 64,
    parameter HPTR   = 6,
    parameter SBITS  = 2,
    parameter NCHK   = 4,
    parameter CBITS  = 2)
   (input  wire                 clk,
    input  wire                 reset,    // squashes the boundary (no alloc/MAP write)
    input  wire                 flush,    // redirect: squash the in-flight (wrong-path) bundle
    input  wire                 accept,   // back-pressure: latch a new bundle (else hold)
    // raw aligner words
    input  wire [IW*32-1:0]     inst,
    input  wire [IW-1:0]        in_valid,
    input  wire [IW*SEQW-1:0]   seq_in,
    input  wire [IW*64-1:0]     pc_in,    // per-slot PC (for the execute payload)
    input  wire [63:0]          pred_npc_in, // fetch's chosen next PC for this bundle
    // checkpoint / commit control (already in the rename time domain)
    input  wire                 create,       // per-bundle dispatch
    input  wire                 ckpt_create,  // per-checkpoint close (coarse CPR)
    input  wire                 commit,
    input  wire [CBITS-1:0]     commit_idx,
    input  wire                 rollback,
    input  wire [CBITS-1:0]     rollback_idx,
    // renamed bundle (one cycle after inst), aligned with r_valid/r_seq
    output wire [IW-1:0]        r_valid,
    output wire [IW*SEQW-1:0]   r_seq,
    output wire [IW*ABITS-1:0]  r_rd,
    output wire [IW-1:0]        r_rd_v,
    output wire [IW*PBITS-1:0]  ps1,
    output wire [IW*PBITS-1:0]  ps2,
    output wire [IW*PBITS-1:0]  ps3,
    output wire [IW*PBITS-1:0]  pdst,
    output wire [IW-1:0]        r_is_branch,  // for speculative checkpoint creation
    output wire [IW*`PAYW-1:0]  r_pay,    // packed execute payload (ctl+imm+pc+branch)
    output wire [63:0]          r_pred_npc, // the bundle's chosen next PC (one CTI/bundle ->
                                            // bundle-scalar; recorded per ckpt at dispatch)
    output wire [CBITS-1:0]     r_ckpt,   // the renamed bundle's checkpoint (= cur)
    output wire [CBITS-1:0]     cur,      // freelist's open span (for commit_ctl)
    output wire [IW-1:0]        stall);

   // ---------------------------------------------------------- decode (comb)
   wire [IW-1:0]        d_valid, d_rd_v, d_rs1_v, d_rs2_v, d_rs3_v;
   wire [IW*SEQW-1:0]   d_seq;
   wire [IW*ABITS-1:0]  d_rd, d_rs1, d_rs2, d_rs3;
   wire [IW*32-1:0]     d_expanded;
   wire [IW-1:0]        d_s1_is_slot, d_s2_is_slot, d_s3_is_slot, d_map_writer, d_d_is_slot;
   wire [IW*SBITS-1:0]  d_s1_slot, d_s2_slot, d_s3_slot, d_d_slot;
   wire [IW-1:0]        d_is_rvc, d_alu_w, d_alu_uw, d_op2_imm, d_res_link, d_is_mem;
   wire [IW-1:0]        d_is_store, d_mem_signed;
   wire [IW*2-1:0]      d_mem_size;
   wire [IW-1:0]        d_is_branch, d_is_jump, d_is_mul;
   wire [IW*3-1:0]      d_br_func;
   wire [IW*64-1:0]     d_imm;
   wire [IW*6-1:0]      d_alu_op;
   wire [IW*2-1:0]      d_op1_sel;
   wire [IW-1:0]        d_is_csr, d_is_serialize, d_is_amo, d_illegal, d_is_fencei;
   wire [IW-1:0]        d_is_cbo, d_cbo_zero, d_cbo_keep;
   wire [IW*3-1:0]      d_csr_func;
   wire [IW*5-1:0]      d_amo_func;

   decode_stage #(.IW(IW), .SEQW(SEQW), .ABITS(ABITS), .SBITS(SBITS)) dec
     (.inst(inst), .in_valid(in_valid), .seq_in(seq_in),
      .valid(d_valid), .seq(d_seq), .is_rvc(d_is_rvc), .expanded(d_expanded),
      .rd(d_rd), .rd_v(d_rd_v), .rs1(d_rs1), .rs1_v(d_rs1_v),
      .rs2(d_rs2), .rs2_v(d_rs2_v), .rs3(d_rs3), .rs3_v(d_rs3_v), .imm(d_imm), .has_imm(), .legal(),
      .s1_is_slot(d_s1_is_slot), .s1_slot(d_s1_slot),
      .s2_is_slot(d_s2_is_slot), .s2_slot(d_s2_slot),
      .s3_is_slot(d_s3_is_slot), .s3_slot(d_s3_slot), .map_writer(d_map_writer),
      .d_is_slot(d_d_is_slot), .d_slot(d_d_slot),
      .alu_op(d_alu_op), .alu_w(d_alu_w), .alu_uw(d_alu_uw), .op1_sel(d_op1_sel),
      .op2_imm(d_op2_imm), .res_link(d_res_link), .is_mem(d_is_mem),
      .is_store(d_is_store), .mem_size(d_mem_size), .mem_signed(d_mem_signed),
      .is_branch(d_is_branch), .br_func(d_br_func), .is_jump(d_is_jump), .is_mul(d_is_mul),
      .is_csr(d_is_csr), .csr_func(d_csr_func), .is_serialize(d_is_serialize),
      .is_amo(d_is_amo), .amo_func(d_amo_func), .is_fencei(d_is_fencei),
      .is_cbo(d_is_cbo), .cbo_zero(d_cbo_zero), .cbo_keep(d_cbo_keep), .illegal(d_illegal));

   // -------------------------------------------- decode/rename boundary reg
   reg [IW-1:0]        q_valid, q_rd_v, q_s1_is_slot, q_s2_is_slot, q_s3_is_slot, q_map_writer, q_d_is_slot;
   reg [IW*SEQW-1:0]   q_seq;
   reg [IW*ABITS-1:0]  q_rd, q_rs1, q_rs2, q_rs3;
   reg [IW*32-1:0]     q_insn;
   reg [IW*SBITS-1:0]  q_s1_slot, q_s2_slot, q_s3_slot, q_d_slot;
   // payload registered alongside the rename contract
   reg [IW-1:0]        q_is_rvc, q_alu_w, q_alu_uw, q_op2_imm, q_res_link, q_is_mem;
   reg [IW-1:0]        q_is_store, q_mem_signed;
   reg [IW*2-1:0]      q_mem_size;
   reg [IW-1:0]        q_is_branch, q_is_jump, q_is_mul;
   reg [IW*3-1:0]      q_br_func;
   reg [IW*64-1:0]     q_imm, q_pc;
   reg [63:0]          q_pnpc;
   reg [IW*6-1:0]      q_alu_op;
   reg [IW*2-1:0]      q_op1_sel;
   reg [IW-1:0]        q_is_csr, q_is_serialize, q_is_amo, q_illegal, q_is_fencei;
   reg [IW-1:0]        q_is_cbo, q_cbo_zero, q_cbo_keep;
   reg [IW*3-1:0]      q_csr_func;
   reg [IW*5-1:0]      q_amo_func;
   initial begin
      q_valid = 0; q_rd_v = 0; q_s1_is_slot = 0; q_s2_is_slot = 0;
      q_map_writer = 0; q_seq = 0; q_rd = 0; q_rs1 = 0; q_rs2 = 0;
      q_s1_slot = 0; q_s2_slot = 0; q_is_serialize = 0; q_is_csr = 0; q_illegal = 0;
      q_is_fencei = 0;
   end
   // Boundary update policy: a redirect/reset squashes the in-flight bundle; else
   // when `accept` is high we latch the next decoded bundle; else (back-pressure
   // stall) we HOLD the current bundle so it can be re-presented to rename until
   // it dispatches. On squash we clear only the bits that cause downstream action
   // (q_valid gates consumers, q_rd_v gates allocation, q_map_writer gates the
   // MAP write); the rest are don't-care while their valids are 0.
   wire squash = reset | flush;
   always @(posedge clk) begin
      if (squash) begin
         q_valid <= {IW{1'b0}}; q_rd_v <= {IW{1'b0}};
         q_map_writer <= {IW{1'b0}}; q_is_branch <= {IW{1'b0}}; q_d_is_slot <= {IW{1'b0}};
         q_is_serialize <= {IW{1'b0}}; q_illegal <= {IW{1'b0}}; q_is_fencei <= {IW{1'b0}};
      end else if (accept) begin
         q_valid      <= d_valid;
         q_rd_v       <= d_rd_v;
         q_map_writer <= d_map_writer;
         q_is_branch  <= d_is_branch;
         q_is_jump    <= d_is_jump; q_br_func <= d_br_func;
         q_seq        <= d_seq;
         q_rd         <= d_rd;
         q_rs1        <= d_rs1;      q_rs2        <= d_rs2;     q_rs3 <= d_rs3;
         q_insn       <= d_expanded;
         q_s1_is_slot <= d_s1_is_slot; q_s1_slot  <= d_s1_slot;
         q_s2_is_slot <= d_s2_is_slot; q_s2_slot  <= d_s2_slot;
         q_s3_is_slot <= d_s3_is_slot; q_s3_slot  <= d_s3_slot;
         q_d_is_slot  <= d_d_is_slot;   q_d_slot   <= d_d_slot;
         q_imm <= d_imm; q_pc <= pc_in; q_pnpc <= pred_npc_in;
         q_alu_op <= d_alu_op; q_alu_w <= d_alu_w; q_alu_uw <= d_alu_uw;
         q_op1_sel <= d_op1_sel; q_op2_imm <= d_op2_imm; q_res_link <= d_res_link;
         q_is_rvc <= d_is_rvc; q_is_mem <= d_is_mem;
         q_is_store <= d_is_store; q_mem_size <= d_mem_size; q_mem_signed <= d_mem_signed;
         q_is_mul <= d_is_mul;
         q_is_csr <= d_is_csr; q_csr_func <= d_csr_func; q_is_serialize <= d_is_serialize;
         q_is_amo <= d_is_amo; q_amo_func <= d_amo_func;
         q_is_cbo <= d_is_cbo; q_cbo_zero <= d_cbo_zero; q_cbo_keep <= d_cbo_keep;
         q_illegal <= d_illegal; q_is_fencei <= d_is_fencei;
      end
   end

   assign r_valid = q_valid;
   assign r_seq   = q_seq;
   assign r_pred_npc = q_pnpc;
   assign r_rd    = q_rd;
   assign r_rd_v  = q_rd_v;
   assign r_is_branch = q_is_branch;

   // An illegal-instruction trap only fires for a CORRECTLY-FETCHED op. A bundle is
   // fetched with one (slot-0) translation, so an op that straddles a page boundary or
   // lies in a different page than slot 0 may have been assembled from the wrong physical
   // bytes (a not-yet-mapped second page) and can spuriously decode as illegal. Those are
   // fetch-window artifacts -- suppress their illegal flag; the next fetch (re-based in the
   // new page) raises the correct fetch page fault instead. Genuine illegal ops (e.g. an
   // all-zero word) sit within their page and are unaffected.
   wire [IW-1:0] ill_ok;
   genvar q;
   generate for (q = 0; q < IW; q = q + 1) begin : illchk
      assign ill_ok[q] = q_illegal[q]
                       & (q_pc[q*64+12 +: 52] == q_pc[12 +: 52])                       // same page as base
                       & (((q_pc[q*64 +: 64] + 64'd3) >> 12) == (q_pc[q*64 +: 64] >> 12)); // op not straddling
   end endgenerate

   // pack the execute payload per slot (see exec_pay.vh)
   genvar p;
   generate for (p = 0; p < IW; p = p + 1) begin : pay
      assign r_pay[p*`PAYW +: `PAYW] =
        { q_cbo_keep[p], q_cbo_zero[p], q_is_cbo[p],
          q_insn[p*32 +: 32],
          q_is_fencei[p], ill_ok[p],
          q_amo_func[p*5 +: 5], q_is_amo[p],
          q_is_serialize[p], q_csr_func[p*3 +: 3], q_is_csr[p],
          q_is_mul[p],
          q_mem_signed[p], q_mem_size[p*2 +: 2], q_is_store[p],
          q_br_func[p*3 +: 3], q_is_jump[p], q_is_branch[p],
          q_pc[p*64 +: 64], q_imm[p*64 +: 64], q_is_mem[p], q_is_rvc[p],
          q_res_link[p], q_op2_imm[p], q_op1_sel[p*2 +: 2], q_alu_uw[p],
          q_alu_w[p], q_alu_op[p*6 +: 6] };
   end endgenerate

   // ---------------------------------------------------------- rename core
   renamer_bundle #(.SHARDS(IW), .ABITS(ABITS), .AREGS(AREGS), .PBITS(PBITS),
                    .NPHYS(NPHYS), .POOL(POOL), .HPTR(HPTR), .SBITS(SBITS),
                    .NCHK(NCHK), .CBITS(CBITS)) rn
     (.clk(clk), .reset(reset),
      .rs1(q_rs1), .rs2(q_rs2), .rs3(q_rs3), .rd(q_rd), .rd_v(q_rd_v),
      .s1_is_slot(q_s1_is_slot), .s1_slot(q_s1_slot),
      .s2_is_slot(q_s2_is_slot), .s2_slot(q_s2_slot),
      .s3_is_slot(q_s3_is_slot), .s3_slot(q_s3_slot), .map_writer(q_map_writer),
      .d_is_slot(q_d_is_slot), .d_slot(q_d_slot),
      .create(create), .ckpt_create(ckpt_create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .ps1(ps1), .ps2(ps2), .ps3(ps3), .pdst(pdst), .cur(cur), .stall(stall));

   // the renamed bundle is allocated into the freelist's current span
   assign r_ckpt = cur;
endmodule

`default_nettype wire
