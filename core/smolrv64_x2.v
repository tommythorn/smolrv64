`default_nettype none

// THE TWO CLOCKS' CROSSINGS. clk2x is twice clk, phase-aligned: every clk edge is a clk2x edge
// (smolrv64_shell: both through BUFGCE_DIVs of ui_clk). A clk cycle has two clk2x halves.
//
// The rules every crossing in the design follows:
//   - clk logic reads a clk2x flop that changes ONLY at the clk2x edges that are clk edges (the end
//     of a second half), so what it sees is stable for its whole cycle and its paths keep clk's
//     period;
//   - clk2x logic counts a clk pulse in one half (the first: `& ~h2`), as the pulse is there for both;
//   - a valid/ready channel either way transfers only at a clk edge: the clk2x side sees the other
//     side's valid or ready in the second half alone (`& h2`), and holds its own until the transfer.

// THE CONSTRAINTS (smolrv64_shell.xdc) follow the rules: a clk2x register that changes only at clk
// edges carries (* x2_cedge = "yes" *), and its paths into clk logic have clk's period; one whose
// captures from clk logic take effect only at clk edges carries (* x2_ccap = "yes" *), and the paths
// from clk logic into it have clk's period. A mark on a register that breaks its rule hides a real
// path: the rule is the register's comment's to state.
// h2: the clk2x cycle is the second half of clk's. tg flips at every clk edge and tg2 is tg as clk2x
// last saw it, so they differ in the first half and agree in the second. In a `posedge clk2x` block,
// `if (h2)` is "this edge is a clk edge".
// h2 itself is a clk2x flop (h2r: next cycle's is this cycle's complement, from tg and tg2), so the
// cones that read it start in the clk2x domain and not at tg's clk flop.
module smolrv64_x2ph
  (input  wire clk,
   input  wire clk2x,
   output wire h2);
   reg tg = 1'b0, tg2 = 1'b0;
   (* max_fanout = 64 *) reg h2r = 1'b1;
   always @(posedge clk)   tg  <= ~tg;
   always @(posedge clk2x) begin
      tg2 <= tg;
      h2r <= (tg2 != tg);
   end
   assign h2 = h2r;
   always @(posedge clk2x) if (h2r != (tg2 == tg))
      $fatal(1, "smolrv64_x2ph: the registered phase %0d is not the clocks' (tg %0d, tg2 %0d)", h2r, tg, tg2);
endmodule

// A clk2x pulse, with its payload, for clk logic: sv is high for exactly one clk cycle after a clk2x
// cycle with v, and sd is the LATEST payload of the clk cycle (two pulses in one clk cycle are one
// for the clk side: a later redirect supersedes an earlier, a predictor update is a hint). Both
// change only at clk edges. A pulse in the first half waits in pv/pd for the edge.
module smolrv64_x2f2s #(parameter integer W = 1)
  (input  wire         clk2x,
   input  wire         reset,
   input  wire         h2,
   input  wire         v,
   input  wire [W-1:0] d,
   output wire         sv,
   output wire [W-1:0] sd,
   output wire         busy);     // a pulse is pending or held: the clk side has not finished with it
   reg         pv = 1'b0;
   reg [W-1:0] pd;
   (* x2_cedge = "yes" *) reg         svr = 1'b0;   // they change only at clk edges
   (* x2_cedge = "yes" *) reg [W-1:0] sdr;
   always @(posedge clk2x) begin
      if (reset) begin
         pv <= 1'b0;  svr <= 1'b0;
      end else if (h2) begin
         svr <= v | pv;  pv <= 1'b0;
         if (v) sdr <= d;  else if (pv) sdr <= pd;
      end else if (v) begin
         pv <= 1'b1;  pd <= d;
      end
   end
   assign sv = svr;  assign sd = sdr;
   assign busy = v | pv | svr;
endmodule

// THE FPU ON clk, BEHIND A CROSSING (fp_unit's own interface on the clk2x side). fpnew's stages close
// clk's period with five pipeline registers (main's), and the 2x clock buys a single unit nothing:
// it takes an op a clk cycle, as at 166.67 MHz. The ISSUE: the core writes an op into ib (its ready
// is ib's emptiness, a flop); at a clk edge ib moves into ob, which the FPU reads (it changes only at
// clk edges) and which holds until the FPU takes it there (its ready read in the second half alone:
// x2_ccap). The RESULT, one clk cycle long, is registered in the first half and is the core's for one
// clk2x cycle. A FLUSH reaches the FPU through smolrv64_x2f2s; until it has, every result is of an op
// the flush killed (the core flushes them all: they are younger than its redirect) and is dropped,
// as are ib and ob.
module smolrv64_x2fpu #(parameter integer TAGW = 24, parameter integer RDV_BIT = TAGW - 2,
                        parameter integer FSHARD_BIT = 0, parameter integer NFLIGHT = 4, parameter integer PIPE_REGS = 5)
  (input  wire             clk2x,
   input  wire             clk,          // the FPU's
   input  wire             reset,        // clk2x's
   input  wire             h2,
   input  wire             iss_valid,
   output wire             iss_ready,
   input  wire [3:0]       iss_op,
   input  wire             iss_op_mod,
   input  wire [2:0]       iss_src_fmt,
   input  wire [2:0]       iss_dst_fmt,
   input  wire [1:0]       iss_int_fmt,
   input  wire [2:0]       iss_rnd,
   input  wire [191:0]     iss_operands,
   input  wire [TAGW-1:0]  iss_tag,
   output wire             res_valid,
   output wire [63:0]      res_data,
   output wire [4:0]       res_fflags,
   output wire [TAGW-1:0]  res_tag,
   output wire             res_fp,
   output wire             res_int,
   input  wire             flush,
   output wire             busy,         // ib or ob holds an op (what the FPU holds, the core counts)
   output reg              err,
   output reg  [15:0]      dbg);
   localparam integer PW = 4 + 1 + 3 + 3 + 2 + 3 + 192 + TAGW;
   wire [PW-1:0] iss_p = {iss_op, iss_op_mod, iss_src_fmt, iss_dst_fmt, iss_int_fmt, iss_rnd, iss_operands, iss_tag};
   reg           ib_v = 1'b0;
   reg  [PW-1:0] ib;
   (* x2_cedge = "yes", x2_ccap = "yes" *) reg ob_v = 1'b0;
   (* x2_cedge = "yes" *) reg [PW-1:0] ob;
   wire          s_ready;                       // the FPU's (clk's)
   wire          fl_s, fl_busy;
   smolrv64_x2f2s #(.W(1)) u_xfl (.clk2x(clk2x), .reset(reset), .h2(h2), .v(flush), .d(1'b0),
                                  .sv(fl_s), .sd(), .busy(fl_busy));
   reg           fl_seen = 1'b0;                // a flush since the last clk edge
   wire          fl_now  = fl_seen | flush;
   wire          ob_take = ob_v & s_ready;      // the FPU takes ob at this clk edge (h2's)
   wire          ib_mv   = ib_v & (~ob_v | ob_take);
   assign iss_ready = ~ib_v;
   always @(posedge clk2x) begin
      fl_seen <= ~reset & ~h2 & fl_now;
      if (reset | flush)                        ib_v <= 1'b0;
      else if (iss_valid & ~ib_v)               ib_v <= 1'b1;
      else if (h2 & ib_mv)                      ib_v <= 1'b0;
      if (iss_valid & ~ib_v) ib <= iss_p;
      if (h2) begin
         ob_v <= ~reset & ~fl_now & (ib_mv | (ob_v & ~ob_take));
         if (ib_mv) ob <= ib;
      end
   end
   reg rst_s = 1'b1;
   always @(posedge clk) rst_s <= reset;
   wire [3:0] o_op; wire o_mod; wire [2:0] o_sf, o_df; wire [1:0] o_if; wire [2:0] o_rnd; wire [191:0] o_ops; wire [TAGW-1:0] o_tag;
   assign {o_op, o_mod, o_sf, o_df, o_if, o_rnd, o_ops, o_tag} = ob;
   wire s_rv, s_fp, s_int, s_busy, s_err;  wire [63:0] s_data;  wire [4:0] s_ff;  wire [TAGW-1:0] s_tag;  wire [15:0] s_dbg;
   fp_unit #(.TAGW(TAGW), .NFLIGHT(NFLIGHT), .PIPE_REGS(PIPE_REGS), .RDV_BIT(RDV_BIT), .FSHARD_BIT(FSHARD_BIT)) u_fpu
     (.clk(clk), .reset(rst_s),
      .iss_valid(ob_v), .iss_ready(s_ready), .iss_op(o_op), .iss_op_mod(o_mod), .iss_src_fmt(o_sf), .iss_dst_fmt(o_df),
      .iss_int_fmt(o_if), .iss_rnd(o_rnd), .iss_operands(o_ops), .iss_tag(o_tag),
      .res_valid(s_rv), .res_ready(1'b1), .res_data(s_data), .res_fflags(s_ff), .res_tag(s_tag), .res_fp(s_fp), .res_int(s_int),
      .flush(fl_s), .busy(s_busy), .err(s_err), .dbg(s_dbg));
   // the result, taken once (the first half), unless a flush is on its way to the FPU or in it
   reg            rv = 1'b0, rfp = 1'b0, rint = 1'b0;
   reg [63:0]     rdata;  reg [4:0] rff;  reg [TAGW-1:0] rtag;
   always @(posedge clk2x) begin
      // res_fp and res_int are the result's valid by its destination kind (fp_unit's): taken as rv is
      rv   <= ~reset & ~h2 & s_rv & ~fl_busy;
      rfp  <= ~reset & ~h2 & s_fp & ~fl_busy;
      rint <= ~reset & ~h2 & s_int & ~fl_busy;
      rdata <= s_data;  rff <= s_ff;  rtag <= s_tag;
      err <= ~reset & s_err;  dbg <= s_dbg;
   end
   assign res_valid = rv;  assign res_data = rdata;  assign res_fflags = rff;  assign res_tag = rtag;
   assign res_fp = rfp;  assign res_int = rint;
   assign busy = ib_v | ob_v;
endmodule

// THE DIVIDER ON clk, BEHIND A CROSSING (divider's interface on the clk2x side; its iterations close
// clk's period, as at 166.67 MHz). The clk side is never aborted: an abort and the start after it
// could reach it in one clk cycle, where its abort wins. Instead the clk2x side drops a start not
// yet handed over and DISCARDS the result of the divide the divider is running (disc), and a new
// start waits (ps) until the divider is idle. START: a register that changes only at clk edges
// (dst, one clk cycle) with its operands. DONE: the divider's result cycle, registered in the first
// half, is the core's for one clk2x cycle unless discarded. busy: a start waiting, or a divide
// running that is still wanted -- the core's own rule (no start into a busy unit) holds on that.
module smolrv64_x2div
  (input  wire        clk2x,
   input  wire        clk,
   input  wire        reset,       // clk2x's
   input  wire        h2,
   input  wire        start,
   input  wire        abort,
   input  wire [63:0] rs1,
   input  wire [63:0] rs2,
   input  wire [2:0]  f3,
   input  wire        is_w,
   output wire        busy,
   output wire        done,
   output wire [63:0] result);
   reg         ps_v = 1'b0;                     // a start waiting for the divider
   reg  [63:0] ps_a, ps_b;  reg [2:0] ps_f3;  reg ps_w;
   (* x2_cedge = "yes", x2_ccap = "yes" *) reg dst = 1'b0;      // the divider's start, one clk cycle
   (* x2_cedge = "yes" *) reg [63:0] da, db;  (* x2_cedge = "yes" *) reg [2:0] df3;  (* x2_cedge = "yes" *) reg dw;
   reg         run = 1'b0, disc = 1'b0;         // the divider holds a divide; its result is not wanted
   wire        d_busy, d_done;                  // the divider's (clk's)
   wire [63:0] d_res;
   reg         dn = 1'b0;  reg [63:0] dres;
   wire        got  = ~h2 & d_done;             // its result cycle, taken in the first half
   wire        take = h2 & dst;                 // the divider takes the start at this clk edge
   always @(posedge clk2x) begin
      if (reset) begin
         ps_v <= 1'b0;  dst <= 1'b0;  run <= 1'b0;  disc <= 1'b0;  dn <= 1'b0;
      end else begin
         dn <= got & ~disc & ~abort;
         if (got) begin run <= 1'b0;  disc <= 1'b0; end
         if (take) run <= 1'b1;
         if (h2) dst <= ps_v & ~dst & ~d_busy & ~run & ~abort;
         if (h2 & ps_v & ~dst & ~d_busy & ~run & ~abort) begin
            da <= ps_a;  db <= ps_b;  df3 <= ps_f3;  dw <= ps_w;  ps_v <= 1'b0;
         end
         // an abort drops the waiting start and discards what the divider holds or is taking now;
         // a start in the abort's own cycle is the killed op's (the divider's own abort won over it)
         if (abort) begin
            ps_v <= 1'b0;
            if (dst | (run & ~got)) disc <= 1'b1;
         end
         if (start & ~abort) begin ps_v <= 1'b1;  ps_a <= rs1;  ps_b <= rs2;  ps_f3 <= f3;  ps_w <= is_w; end
      end
      dres <= d_res;
   end
   always @(posedge clk2x) if (!reset) begin
      if (start & ps_v & ~abort) $fatal(1, "smolrv64_x2div: a start while one waits");
      if (got & ~run)            $fatal(1, "smolrv64_x2div: the divider finished with nothing running");
   end
   reg rst_s = 1'b1;
   always @(posedge clk) rst_s <= reset;
   divider u_div
     (.clk(clk), .reset(rst_s), .start(dst), .abort(1'b0), .rs1(da), .rs2(db), .f3(df3), .is_w(dw),
      .busy(d_busy), .done(d_done), .result(d_res));
   assign busy = ps_v | (run & ~disc) | (dst & ~disc);
   assign done = dn;
   assign result = dres;
endmodule

`default_nettype wire
