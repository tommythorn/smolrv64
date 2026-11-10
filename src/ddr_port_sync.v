`default_nettype none
// ddr_port_sync -- rv_soc_top's DDR memory port (clk_p, the core's probe_clk) across into ui_clk
// (clk_m) where clk_p is clk_m / DIV (DIV >= 2; 2 on the shipping build) from a BUFGCE_DIV: related
// clocks, so the crossing is registers, not asynchronous FIFOs.
//
// THE PHASE. clk_p's edges fall on every DIV-th clk_m edge, late by the BUFGCE_DIV's insertion delay
// (about 2.7 ns of the 3 ns on the board). The clk_p side toggles p_t every cycle; clk_m samples it
// on its falling edge, half a cycle from where it changes, so the change shows at the clk_m edge two
// after clk_p's ideal edge. `ph` is a one-hot ring of the DIV clk_m edges of a clk_p cycle, set from
// that change and rotating between: ph[0] is CAP, the edge on clk_p's ideal edge (3.3 ns after its
// real one on the board), and ph[1 % DIV] is LAUNCH, the edge after it. At DIV 2 the edges alternate,
// CAP and LAUNCH. Both are registers: only the compare's flop sees the falling edge's half cycle.
//
// clk_p -> clk_m: a request presented by the port in a clk_p cycle is captured at that cycle's CAP
// edge (timed as a multicycle path of DIV: the XDC's ddr_port_sync constraints).
// clk_m -> clk_p: everything the port sees (the request's ready, the read beats, the write dones)
// changes only at LAUNCH, one ui cycle after a clk_p edge, and holds a clk_p cycle, so exactly one
// clk_p edge samples it, with Vivado's default analysis (setup 3 ns plus the skew, hold the skew's
// margin). The read beats and write dones land in a clk_p register first, by the core: the route
// from their launch registers, by the controller, is register to register.
//
// THE HANDSHAKE. The request's ready is a promise made at LAUNCH: the capture register will be free
// at the coming CAP (it only fills at a CAP), so the request the port presents that clk_p cycle is
// captured then and the port sees it taken at its next edge. Only that one bit, the request's valid, depends on clk_p's
// logic: the capture's enable is clk_m's own (the CAP and the promise), so no clk_p signal fans out
// to the 600 capture flops. A read beat and a write done go back at one an other edge, so the port
// sees each for exactly one of its cycles; the caches always take them (rv_mem_arbiter's
// mr_ready/mw_ready), asserted.
//
// PLACEMENT. A request has a second clk_m register between its capture registers, by the core, and
// the controller; a beat or a done crosses from its launch register, by the controller, to its clk_p
// register by the core. Both keep a request and a beat a clk_p cycle.
//
// THE SNOOP CHANNEL runs the other way: a DMA master's line, to be cleaned (keep) or flushed in the
// D$ before the master moves it, one at a time. The request is a toggle with its line, launched at
// an other edge as a read beat is (m_s_valid held until m_s_ack); the core side presents it while
// its toggle differs from the answer's (p_s_valid, until p_s_ack), and the answer's toggle comes back
// captured at CAP as a request is (c_atog). The release after a flush (m_s_rel, a pulse: the DMA
// write is in the controller) goes the same way as the request, a toggle, a pulse (p_s_rel) again.
// Both land in clk_p registers first, as the beats do.
module ddr_port_sync #(
   parameter integer IDW = 5,
   parameter integer DIV = 2              // clk_m edges a clk_p cycle
) (
   // ---- core side (clk_p) ----
   input  wire           clk_p,
   input  wire           reset_p,
   input  wire           p_q_valid,
   output wire           p_q_ready,
   input  wire [IDW-1:0] p_q_id,
   input  wire           p_q_we,
   input  wire [57:0]    p_q_addr,
   input  wire [63:0]    p_q_wmask,
   input  wire [511:0]   p_q_wdata,
   output wire           p_r_valid,
   input  wire           p_r_ready,
   output wire [IDW-1:0] p_r_id,
   output wire [1:0]     p_r_beat,
   output wire           p_r_last,
   output wire [127:0]   p_r_data,
   output wire           p_w_valid,
   input  wire           p_w_ready,
   output wire [IDW-1:0] p_w_id,
   // ---- memory side (clk_m = ui_clk) ----
   input  wire           clk_m,
   input  wire           reset_m,
   output reg            m_q_valid,
   input  wire           m_q_ready,
   output reg  [IDW-1:0] m_q_id,
   output reg            m_q_we,
   output reg  [57:0]    m_q_addr,
   output reg  [63:0]    m_q_wmask,
   output reg  [511:0]   m_q_wdata,
   input  wire           m_r_valid,
   output wire           m_r_ready,
   input  wire [IDW-1:0] m_r_id,
   input  wire [1:0]     m_r_beat,
   input  wire           m_r_last,
   input  wire [127:0]   m_r_data,
   input  wire           m_w_valid,
   output wire           m_w_ready,
   input  wire [IDW-1:0] m_w_id,
   // ---- the snoop channel: clk_m -> clk_p, the answer back ----
   input  wire           m_s_valid,             // held until m_s_ack
   input  wire [25:0]    m_s_line,              // the line, as the DMA masters' AXI address names it
   input  wire           m_s_keep,              // clean; else flush
   output reg            m_s_ack,
   input  wire           m_s_rel,
   output wire           p_s_valid,             // held until p_s_ack
   output reg  [25:0]    p_s_line,
   output reg            p_s_keep,
   input  wire           p_s_ack,
   output wire           p_s_rel
);
   // ---- the phase: clk_p's toggle, sampled by clk_m's falling edge ----
   reg p_t;
   always @(posedge clk_p) p_t <= reset_p ? 1'b0 : ~p_t;
   reg m_tf, m_tr;
   always @(negedge clk_m) m_tf <= p_t;
   always @(posedge clk_m) m_tr <= m_tf;
   wire chg = m_tf != m_tr;             // this is the edge two after clk_p's ideal edge
   reg [DIV-1:0] ph;                    // one-hot: this edge's place in the clk_p cycle
   always @(posedge clk_m)
      ph <= chg ? ({{(DIV-1){1'b0}}, 1'b1} << (3 % DIV)) : {ph[DIV-2:0], ph[DIV-1]};
   wire cap = ph[0];                    // CAP
   wire lch = ph[1 % DIV];              // LAUNCH

   // ---- requests: the promise at LAUNCH, the capture at CAP, then the output register ----
   reg            c_v;                     // the capture register: valid, and the request's fields
   reg  [IDW-1:0] c_id;  reg c_we;  reg [57:0] c_addr;  reg [63:0] c_wmask;  reg [511:0] c_wdata;
   // the port's ready: launched at LAUNCH, held through CAP. It is the enable of the L2's
   // 512-bit write-back data among others (rv_l2 d_q_wdata), with an ui cycle to get there, so it
   // is replicated beside its loads.
   (* max_fanout = 32 *) reg promise;
   assign p_q_ready = promise;
   wire           c_go = c_v && (!m_q_valid || m_q_ready);   // the capture moves on to the output
   always @(posedge clk_m) begin
      if (cap && promise) begin            // clk_m's own enable; the fields load whatever they hold
         c_id <= p_q_id;  c_we <= p_q_we;  c_addr <= p_q_addr;  c_wmask <= p_q_wmask;  c_wdata <= p_q_wdata;
      end
      if (c_go) begin
         m_q_id <= c_id;  m_q_we <= c_we;  m_q_addr <= c_addr;  m_q_wmask <= c_wmask;  m_q_wdata <= c_wdata;
      end
      if (reset_m) begin
         c_v <= 1'b0;  m_q_valid <= 1'b0;  promise <= 1'b0;
      end else begin
         if (lch) promise <= !c_v || c_go;                     // free at the coming CAP
         if (cap && promise) c_v <= p_q_valid; else if (c_go) c_v <= 1'b0;
         if (c_go) m_q_valid <= 1'b1; else if (m_q_ready) m_q_valid <= 1'b0;
      end
   end

   // ---- read beats and write dones: launched at LAUNCH, one a clk_p cycle, then a clk_p register ----
   reg           pr_v, pr_last, pw_v;       // clk_m: the launch registers, by the controller
   reg [IDW-1:0] pr_id, pw_id;
   reg [1:0]     pr_beat;
   reg [127:0]   pr_data;
   reg           q_rv, q_rlast, q_wv;       // clk_p: the port's, by the core
   reg [IDW-1:0] q_rid, q_wid;
   reg [1:0]     q_rbeat;
   reg [127:0]   q_rdata;
   assign m_r_ready = lch;                  // a beat or a done moves at LAUNCH
   assign m_w_ready = lch;
   assign p_r_valid = q_rv;  assign p_r_id = q_rid;  assign p_r_beat = q_rbeat;
   assign p_r_last  = q_rlast;  assign p_r_data = q_rdata;
   assign p_w_valid = q_wv;  assign p_w_id = q_wid;
   always @(posedge clk_m) begin
      if (lch) begin
         pr_id <= m_r_id;  pr_beat <= m_r_beat;  pr_last <= m_r_last;  pr_data <= m_r_data;  pw_id <= m_w_id;
      end
      if (reset_m) begin pr_v <= 1'b0;  pw_v <= 1'b0; end
      else if (lch) begin pr_v <= m_r_valid;  pw_v <= m_w_valid; end
   end
   always @(posedge clk_p) begin
      q_rv <= !reset_p && pr_v;  q_wv <= !reset_p && pw_v;
      q_rid <= pr_id;  q_rbeat <= pr_beat;  q_rlast <= pr_last;  q_rdata <= pr_data;  q_wid <= pw_id;
   end

   // ---- the snoop channel ----
   reg            s_tog, s_out, s_keep;  reg [25:0] s_line;   // clk_m: launched at LAUNCH
   reg            s_tq, a_tog;                                 // clk_p: the request's toggle, the answer's
   reg            c_atog;                                      // clk_m: the answer, captured at CAP
   assign p_s_valid = s_tq ^ a_tog;
   always @(posedge clk_p) begin
      s_tq <= !reset_p && s_tog;  p_s_line <= s_line;  p_s_keep <= s_keep;
      a_tog <= reset_p ? 1'b0 : a_tog ^ p_s_ack;
   end
   always @(posedge clk_m) begin
      if (cap) c_atog <= a_tog;
      m_s_ack <= 1'b0;
      if (reset_m) begin
         s_tog <= 1'b0;  s_out <= 1'b0;  c_atog <= 1'b0;
      end else if (s_out) begin
         if (c_atog == s_tog) begin s_out <= 1'b0;  m_s_ack <= 1'b1; end
      end else if (m_s_valid && !m_s_ack && lch) begin
         s_tog <= ~s_tog;  s_out <= 1'b1;  s_line <= m_s_line;  s_keep <= m_s_keep;
      end
   end
   reg            r_tog, r_pend;                               // clk_m: the release
   reg            r_tq, r_seen;                                // clk_p
   assign p_s_rel = r_tq ^ r_seen;
   always @(posedge clk_p) begin r_tq <= !reset_p && r_tog;  r_seen <= !reset_p && r_tq; end
   always @(posedge clk_m)
      if (reset_m) begin r_tog <= 1'b0;  r_pend <= 1'b0; end
      else if (lch && (r_pend || m_s_rel)) begin r_tog <= ~r_tog;  r_pend <= 1'b0; end
      else if (m_s_rel) r_pend <= 1'b1;

`ifndef SYNTHESIS
   always @(posedge clk_p) if (!reset_p) begin
      if (p_s_ack && !p_s_valid) $fatal(1, "ddr_port_sync: a snoop answered with none asked");
      if (p_r_valid && !p_r_ready) $fatal(1, "ddr_port_sync: a read beat the port did not take (id %0d)", p_r_id);
      if (p_w_valid && !p_w_ready) $fatal(1, "ddr_port_sync: a write done the port did not take (id %0d)", p_w_id);
   end
   always @(posedge clk_m) if (!reset_m && c_v && cap && promise && !c_go)
      $fatal(1, "ddr_port_sync: a request captured over one not yet moved on");
   // the ring against the compare it is set from, once clk_p runs: one change a clk_p cycle, at the
   // edge two after CAP; the ring one-hot
   reg ph_on = 1'b0;
   initial if (DIV < 2) $fatal(1, "ddr_port_sync: DIV %0d (clk_p is clk_m / DIV, DIV >= 2)", DIV);
   always @(posedge clk_m) if (!reset_m) begin
      if (chg) ph_on <= 1'b1;
      if (ph_on && (chg != ph[2 % DIV] || $countones(ph) != 1))
         $fatal(1, "ddr_port_sync: the phase ring %b against the compare %0d", ph, chg);
   end
`endif
endmodule

`default_nettype wire
