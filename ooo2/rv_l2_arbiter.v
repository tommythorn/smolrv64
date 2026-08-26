`default_nettype none

// Fixed-priority L2/memory arbiter: merges NREQ line requesters (I$ fill, D$ fill,
// D$ write-through/back, PTW-as-line) onto ONE memory line port.
//
// Each requester drives the same handshake as cache.v's L2 port: a 1-cycle req
// pulse with we/addr/wdata held stable until its ack. Because a pulse can arrive
// while the arbiter is busy serving someone else, requests are LATCHED (pend) and
// the held addr/wdata are sampled at grant time. One transaction in flight; the
// granted requester gets a 1-cycle ack + the shared rdata. Lower index = higher
// priority (assign D$ write/fill below I$/PTW in soc_top so stores/loads win).
//
// The memory port mirrors the same contract (1-cycle mem_req pulse -> mem_ack with
// mem_rdata), so a cache.v-style behavioral RAM (or the real DRAM bridge) drops in.
module rv_l2_arbiter #(
   parameter NREQ = 4,
   parameter AW   = 58,     // line address width (PA[63:6])
   parameter DW   = 512     // line data width
) (
   input  wire             clk,
   input  wire             reset,
   // ---- requesters (packed) ----
   input  wire [NREQ-1:0]    req,        // 1-cycle pulse
   input  wire [NREQ-1:0]    we,         // 1 = write
   input  wire [NREQ*AW-1:0] addr,       // held until ack
   input  wire [NREQ*DW-1:0] wdata,      // held until ack
   output reg  [NREQ-1:0]    ack,        // 1-cycle to the granted requester
   output reg  [DW-1:0]      rdata,      // valid with ack (read data)
   // ---- single memory line port ----
   output reg                mem_req,
   output reg                mem_we,
   output reg  [AW-1:0]      mem_addr,
   output reg  [DW-1:0]      mem_wdata,
   input  wire [DW-1:0]      mem_rdata,
   input  wire               mem_ack
);
   localparam IW = (NREQ <= 1) ? 1 : $clog2(NREQ);
   localparam S_IDLE = 1'b0, S_BUSY = 1'b1;

   reg [NREQ-1:0] pend;          // latched outstanding requests
   reg            state;
   reg [IW-1:0]   gnt;
   integer k;

   // lowest-index pending requester
   reg [IW-1:0] sel; reg sel_v;
   always @* begin
      sel = {IW{1'b0}}; sel_v = 1'b0;
      for (k = NREQ-1; k >= 0; k = k-1) if (pend[k]) begin sel = k[IW-1:0]; sel_v = 1'b1; end
   end

   always @(posedge clk) begin
      ack <= {NREQ{1'b0}};
      mem_req <= 1'b0;
      if (reset) begin state <= S_IDLE; pend <= {NREQ{1'b0}}; end
      else begin
         // latch incoming request pulses (addr/wdata stay held by the requester)
         for (k = 0; k < NREQ; k = k+1) if (req[k]) pend[k] <= 1'b1;
         case (state)
           S_IDLE: if (sel_v) begin
              gnt       <= sel;
              mem_req   <= 1'b1;
              mem_we    <= we[sel];
              mem_addr  <= addr [sel*AW +: AW];
              mem_wdata <= wdata[sel*DW +: DW];
              state     <= S_BUSY;
           end
           S_BUSY: if (mem_ack) begin
              rdata     <= mem_rdata;
              ack[gnt]  <= 1'b1;
              pend[gnt] <= 1'b0;       // (a req[gnt] pulse this same cycle re-sets it above)
              state     <= S_IDLE;
           end
         endcase
      end
   end
endmodule

`default_nettype wire
