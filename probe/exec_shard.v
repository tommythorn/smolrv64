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
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter POOL   = 64,
    parameter IDXB   = 6,
    parameter SEQW   = 8,
    parameter CBITS  = 2)
   (input  wire                    clk,
    // issue from this shard's scheduler
    input  wire                    iss_valid,
    input  wire [SEQW-1:0]         iss_seq,
    input  wire [PBITS-1:0]        iss_pdst,
    input  wire                    iss_pdst_v,    // writes a register
    input  wire [PBITS-1:0]        iss_ps1,
    input  wire [PBITS-1:0]        iss_ps2,
    input  wire [CBITS-1:0]        iss_ckpt,      // checkpoint (captured for a div's deferred completion)
    // squash: abort an in-flight divide whose seqno is rolled back
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // execute payload (decode_exec ctl + imm/pc)
    input  wire [5:0]              alu_op,
    input  wire                    alu_w,
    input  wire                    alu_uw,
    input  wire [1:0]              op1_sel,
    input  wire                    op2_imm,
    input  wire                    res_link,
    input  wire                    is_rvc,
    input  wire                    is_mem,
    input  wire                    is_branch,
    input  wire                    is_jump,
    input  wire                    is_mul,        // M ext (op = {alu_w,br_func}): mul = comb, div = iterative
    input  wire [2:0]              br_func,
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
    // branch/jump resolution (this shard)
    output wire                    br_redirect,   // valid only when iss_valid & (is_branch|is_jump)
    output wire [63:0]             br_target,
    output wire [SEQW-1:0]         br_seq,
    // for the LSU
    output wire [63:0]             agu_addr,
    output wire [63:0]             st_data,       // store data (= rs2) for the store buffer
    output wire                    cmp_eq,
    output wire                    cmp_lt,
    output wire                    cmp_ltu,
    // iterative divide: structural-hazard backpressure + deferred completion
    output wire                    exec_busy,     // divider running -> scheduler holds this shard
    output wire                    div_done,      // a divide completed this cycle (-> commit_ctl)
    output wire [CBITS-1:0]        div_done_ckpt);

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

   // M extension (op = {is_w,funct3} = {alu_w,br_func}): both classes are deferred,
   // multi-cycle units that complete off the single-cycle path (mul = 3-cycle pipelined,
   // div/rem = iterative). funct3[2] selects which. One M-op per shard at a time, so
   // they share the start/capture/completion logic; exec_busy stalls the shard's issue
   // while one is in flight (keeping the WB lane free), and a squash aborts a wrong-path
   // one. Latency-1 ops (ALU/link) write back here; loads complete via the LSU.
   wire mul_op = is_mul & ~br_func[2];
   wire div_op = is_mul &  br_func[2];

   function automatic older;          // a strictly older than b (wrap-safe)
      input [SEQW-1:0] a, bb; older = ($signed(a - bb) < 0);
   endfunction

   wire        mbusy, mdone;  wire [63:0] mres;     // 3-cycle multiply
   wire        dbusy, ddone;  wire [63:0] dres;     // iterative divide
   wire        munit_busy = mbusy | dbusy;
   reg  [PBITS-1:0] m_pdst;
   reg  [SEQW-1:0]  m_seq;
   reg  [CBITS-1:0] m_ck;
   // Don't START an M-op squashed this same cycle (a branch can redirect the cycle a
   // younger M-op issues, before *_busy is set -- the mid-run abort couldn't catch it).
   wire m_squash_now = squash & older(squash_seq, iss_seq);
   wire m_start = iss_valid & is_mul & ~munit_busy & ~m_squash_now;
   wire m_abort = munit_busy & squash & older(squash_seq, m_seq);   // in-flight M-op rolled back

   mul3 mu (.clk(clk), .reset(1'b0), .start(m_start & mul_op), .abort(m_abort),
            .rs1(rs1_val), .rs2(rs2_val), .f3(br_func), .is_w(alu_w),
            .busy(mbusy), .done(mdone), .result(mres));
   divider dv (.clk(clk), .reset(1'b0), .start(m_start & div_op), .abort(m_abort),
               .rs1(rs1_val), .rs2(rs2_val), .f3(br_func), .is_w(alu_w),
               .busy(dbusy), .done(ddone), .result(dres));
   always @(posedge clk) if (m_start) begin
      m_pdst <= iss_pdst; m_seq <= iss_seq; m_ck <= iss_ckpt;
   end
   wire m_complete = (mdone | ddone) & ~m_abort;   // squash in the result cycle suppresses
   wire [63:0] m_res = mdone ? mres : dres;        // only one in flight -> one done at a time

   // ALU/link write back now; M-ops defer to m_complete; mem completes via LSU.
   wire normal_wb = iss_valid & iss_pdst_v & ~is_mem & ~is_mul;
   assign wb_valid = normal_wb | m_complete;
   assign wb_pr    = m_complete ? m_pdst : iss_pdst;
   assign wb_val   = m_complete ? m_res  : result;

   assign exec_busy     = munit_busy;
   assign div_done      = m_complete;       // "M-op completed" (mul or divide) -> commit_ctl
   assign div_done_ckpt = m_ck;

   // branch/jump resolution (predict not-taken): redirect on taken branch / any jump
   wire bu_redirect;
   branch_unit bu
     (.is_branch(is_branch), .is_jump(is_jump), .is_jalr(is_jump & op2_imm),
      .br_func(br_func), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu),
      .pc(pc), .imm(imm), .agu_addr(agu_addr),
      .redirect(bu_redirect), .target(br_target));
   assign br_redirect = iss_valid & bu_redirect;
   assign br_seq      = iss_seq;
   assign st_data     = rs2_val;          // store data path (no op2_imm mux)
endmodule

`default_nettype wire
