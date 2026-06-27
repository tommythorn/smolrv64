`default_nettype none

// ddr_line_cdc -- clock-domain crossing for soc_top's 512-bit line req/ack memory port.
// The probe core runs on a DIVIDED ui_clk (BUFGCE_DIV, synchronous to ui_clk but slower);
// the DDR4 MIG + AXI arbiter + ddr_line_axi bridge all run on ui_clk. This bridges the
// single-outstanding ddr_* handshake between the two with a classic 4-phase full handshake:
// data is captured only while the request is asserted (stable), so a 2-FF synchronizer on
// the control bits is sufficient (no data MUX glitch risk).
//
//   probe side (clk_p, slow):  p_req/p_we/p_addr/p_wdata -> ; <- p_rdata/p_ack(1-cycle pulse)
//   mem   side (clk_m, ui_clk): m_req/m_we/m_addr/m_wdata -> ddr_line_axi ; <- m_rdata/m_ack
//
// Protocol: p_req asserted (held) -> sync to clk_m -> m_req pulse to the bridge -> bridge
// returns m_ack(+m_rdata) -> latch "done" level -> sync back to clk_p -> p_ack pulse +
// p_rdata -> consumer drops p_req -> sync drop clears "done". Single outstanding throughout.
module ddr_line_cdc (
   // ---- probe (slow) side: soc_top.ddr_* connects here ----
   input  wire         clk_p,
   input  wire         reset_p,
   input  wire         p_req,
   input  wire         p_we,
   input  wire [57:0]  p_addr,
   input  wire [511:0] p_wdata,
   output reg  [511:0] p_rdata,
   output reg          p_ack,        // 1-cycle pulse in clk_p

   // ---- mem (ui_clk) side: drives ddr_line_axi ----
   input  wire         clk_m,
   input  wire         reset_m,
   output reg          m_req,
   output reg          m_we,
   output reg  [57:0]  m_addr,
   output reg  [511:0] m_wdata,
   input  wire [511:0] m_rdata,
   input  wire         m_ack
);
   // ---------- clk_p -> clk_m: request level ----------
   reg  p_busy;                       // a transaction is in flight (req held)
   wire p_start = p_req & ~p_busy;
   (* async_reg="true" *) reg req_m0, req_m1;     // p_busy synced into clk_m
   wire req_m = req_m1;

   // ---------- clk_m side FSM ----------
   localparam M_IDLE=2'd0, M_REQ=2'd1, M_DONE=2'd2;
   reg [1:0]   ms;
   reg         m_done;                // level: result captured, held until req drops
   reg [511:0] m_rdata_q;
   // request payload synced into clk_m (stable while p_busy held -> plain sample is safe)
   (* async_reg="true" *) reg        p_we_m;
   (* async_reg="true" *) reg [57:0] p_addr_m;
   (* async_reg="true" *) reg [511:0] p_wdata_m;
   always @(posedge clk_m) begin
      if (reset_m) begin ms<=M_IDLE; m_req<=1'b0; m_done<=1'b0; end
      else begin
         m_req <= 1'b0;
         case (ms)
           M_IDLE: if (req_m) begin           // capture the (stable) request payload
                      m_we<=p_we_m; m_addr<=p_addr_m; m_wdata<=p_wdata_m;
                      m_req<=1'b1; ms<=M_REQ;
                   end
           M_REQ:  if (m_ack) begin m_rdata_q<=m_rdata; m_done<=1'b1; ms<=M_DONE; end
           M_DONE: if (!req_m) begin m_done<=1'b0; ms<=M_IDLE; end   // consumer dropped req
         endcase
      end
   end
   always @(posedge clk_m) begin
      req_m0 <= p_busy; req_m1 <= req_m0;
      p_we_m <= p_we; p_addr_m <= p_addr; p_wdata_m <= p_wdata;
   end

   // ---------- clk_m -> clk_p: done level ----------
   (* async_reg="true" *) reg done_p0, done_p1;
   always @(posedge clk_p) begin done_p0 <= m_done; done_p1 <= done_p0; end
   wire done_p = done_p1;

   always @(posedge clk_p) begin
      if (reset_p) begin p_busy<=1'b0; p_ack<=1'b0; end
      else begin
         p_ack <= 1'b0;
         if (p_start) p_busy <= 1'b1;           // launch: hold req payload stable
         else if (p_busy && done_p) begin       // result arrived
            p_rdata <= m_rdata_q;                // stable: held while m_done asserted
            p_ack   <= 1'b1;
            p_busy  <= 1'b0;
         end
      end
   end
endmodule

`default_nettype wire
