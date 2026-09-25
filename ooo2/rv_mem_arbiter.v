`default_nettype none
// rv_mem_arbiter -- the caches' single path to memory: NC clients onto one tagged memory port.
//
// The port (docs/PLAN-2026-09-25-dcache-vhpr.md, "The memory port") has three valid/ready
// channels: a request {id, we, line address, byte mask, line data}, read data in four 128-bit
// beats {id, beat, last, data}, and write done {id}. Any number of transactions may be
// outstanding. A response is matched by the id its request carried (rule B1), never by order:
// the DDR4 controller answers in order, but the on-chip SRAM answers sooner, so across targets
// responses reorder.
//
// Ids are {client, slot}: a client names its own transactions with SW-bit slots and never sees
// another client's. Among waiting requests a READ wins over a write (a demand fill must not
// queue behind a write-back); within a class the lower client index wins. The chosen request is
// registered before it leaves, so nothing on the port depends on a client's live valid.
// Responses go to the client the id names in the cycle they arrive (the port's side registers
// them); clients always take them (no ready on that side).
module rv_mem_arbiter #(
   parameter integer NC  = 2,                    // clients
   parameter integer CB  = 1,                    // client-index bits, clog2(NC)
   parameter integer SW  = 4,                    // slot bits per client
   parameter integer IDW = CB + SW,
   parameter integer AW  = 58                    // line address width (PA[63:6])
) (
   input  wire              clk,
   input  wire              reset,
   // ---- clients (flattened, client i at [i*W +: W]) ----
   input  wire [NC-1:0]       cq_valid,
   output wire [NC-1:0]       cq_ready,
   input  wire [NC*SW-1:0]    cq_slot,
   input  wire [NC-1:0]       cq_we,
   input  wire [NC*AW-1:0]    cq_addr,
   input  wire [NC*64-1:0]    cq_wmask,
   input  wire [NC*512-1:0]   cq_wdata,
   output wire [NC-1:0]       cr_valid,          // read data, to the client the id names
   output wire [SW-1:0]       cr_slot,
   output wire [1:0]          cr_beat,
   output wire                cr_last,
   output wire [127:0]        cr_data,
   output wire [NC-1:0]       cw_valid,          // write done
   output wire [SW-1:0]       cw_slot,
   // ---- the memory port ----
   output reg                 mq_valid,
   input  wire                mq_ready,
   output reg  [IDW-1:0]      mq_id,
   output reg                 mq_we,
   output reg  [AW-1:0]       mq_addr,
   output reg  [63:0]         mq_wmask,
   output reg  [511:0]        mq_wdata,
   input  wire                mr_valid,
   output wire                mr_ready,
   input  wire [IDW-1:0]      mr_id,
   input  wire [1:0]          mr_beat,
   input  wire                mr_last,
   input  wire [127:0]        mr_data,
   input  wire                mw_valid,
   output wire                mw_ready,
   input  wire [IDW-1:0]      mw_id
);
   // ---- grant: a read first, then a write; lowest index within the class ----
   wire [NC-1:0] rd_want = cq_valid & ~cq_we;
   wire [NC-1:0] wr_want = cq_valid &  cq_we;
   wire [NC-1:0] want    = (|rd_want) ? rd_want : wr_want;
   wire [NC-1:0] gnt     = want & (~want + 1'b1);             // lowest set bit
   wire          take    = (|gnt) & (~mq_valid | mq_ready);   // the output register is free
   assign cq_ready = take ? gnt : {NC{1'b0}};

   // the granted client's index and fields (one-hot select, no priority chain)
   reg [CB-1:0]  g_ix;
   reg [SW-1:0]  g_slot;
   reg           g_we;
   reg [AW-1:0]  g_addr;
   reg [63:0]    g_wmask;
   reg [511:0]   g_wdata;
   integer i;
   always @* begin
      g_ix = {CB{1'b0}}; g_slot = {SW{1'b0}}; g_we = 1'b0; g_addr = {AW{1'b0}};
      g_wmask = 64'd0; g_wdata = 512'd0;
      for (i = 0; i < NC; i = i + 1) if (gnt[i]) begin
         g_ix    = i[CB-1:0];
         g_slot  = cq_slot [i*SW  +: SW];
         g_we    = cq_we[i];
         g_addr  = cq_addr [i*AW  +: AW];
         g_wmask = cq_wmask[i*64  +: 64];
         g_wdata = cq_wdata[i*512 +: 512];
      end
   end

   always @(posedge clk) begin
      if (reset) mq_valid <= 1'b0;
      else if (take)     mq_valid <= 1'b1;
      else if (mq_ready) mq_valid <= 1'b0;
      if (take) begin
         mq_id <= {g_ix, g_slot}; mq_we <= g_we; mq_addr <= g_addr;
         mq_wmask <= g_wmask; mq_wdata <= g_wdata;
      end
   end

   // ---- responses: routed by the id's client field ----
   assign mr_ready = 1'b1;
   assign mw_ready = 1'b1;
   wire [CB-1:0] r_ix = mr_id[IDW-1:SW];
   wire [CB-1:0] w_ix = mw_id[IDW-1:SW];
   genvar gc;
   generate for (gc = 0; gc < NC; gc = gc + 1) begin : g_rsp
      assign cr_valid[gc] = mr_valid & (r_ix == gc);
      assign cw_valid[gc] = mw_valid & (w_ix == gc);
   end endgenerate
   assign cr_slot = mr_id[SW-1:0];  assign cr_beat = mr_beat;  assign cr_last = mr_last;
   assign cr_data = mr_data;        assign cw_slot = mw_id[SW-1:0];

   always @(posedge clk) if (!reset) begin
      if (|(gnt & (gnt - 1'b1)))
         $fatal(1, "rv_mem_arbiter: two clients granted in one cycle (gnt=%b)", gnt);
      if (mr_valid && r_ix >= NC)
         $fatal(1, "rv_mem_arbiter: read data for client %0d, which does not exist (id=%h)", r_ix, mr_id);
      if (mw_valid && w_ix >= NC)
         $fatal(1, "rv_mem_arbiter: write done for client %0d, which does not exist (id=%h)", w_ix, mw_id);
   end
endmodule

// rv_mem_line_client -- a single-outstanding line requester (rv_cache's and rv_icache's `l2_*`
// port) as a memory-port client. The requester pulses l2_req with we/addr/wdata held until its
// ack, as the old line arbiter required; this presents that request until the arbiter takes it,
// gathers the read beats into the line, and acks a read in the cycle of its last beat (that beat
// merged into l2_rdata as it arrives) or a write in the cycle of its write done. One transaction
// at a time, asserted: a request while one is owed is a requester that dropped its hold.
module rv_mem_line_client #(
   parameter integer SW = 4,
   parameter integer AW = 58
) (
   input  wire            clk,
   input  wire            reset,
   input  wire            l2_req,
   input  wire            l2_we,
   input  wire [AW-1:0]   l2_addr,
   input  wire [511:0]    l2_wdata,
   output wire            l2_ack,
   output wire [511:0]    l2_rdata,
   output wire            cq_valid,
   input  wire            cq_ready,
   output wire [SW-1:0]   cq_slot,
   output wire            cq_we,
   output wire [AW-1:0]   cq_addr,
   output wire [63:0]     cq_wmask,
   output wire [511:0]    cq_wdata,
   input  wire            cr_valid,
   input  wire [SW-1:0]   cr_slot,
   input  wire [1:0]      cr_beat,
   input  wire            cr_last,
   input  wire [127:0]    cr_data,
   input  wire            cw_valid,
   input  wire [SW-1:0]   cw_slot
);
   reg pend;      // the request waits for the arbiter
   reg owed;      // taken; its response is outstanding
   reg we_q;
   reg [511:0] line;   // the beats so far
   assign cq_valid = pend;
   assign cq_slot  = {SW{1'b0}};
   assign cq_we    = l2_we;         // held by the requester until its ack
   assign cq_addr  = l2_addr;
   assign cq_wmask = {64{1'b1}};
   assign cq_wdata = l2_wdata;
   assign l2_ack = (cr_valid & cr_last) | cw_valid;
   genvar gq;
   generate for (gq = 0; gq < 4; gq = gq + 1) begin : g_q
      assign l2_rdata[gq*128 +: 128] = (cr_valid && cr_beat == gq) ? cr_data : line[gq*128 +: 128];
   end endgenerate
   always @(posedge clk) begin
      if (reset) begin pend <= 1'b0; owed <= 1'b0; end
      else begin
         if (l2_req) begin pend <= 1'b1; we_q <= l2_we; end
         if (pend && cq_ready) begin pend <= 1'b0; owed <= 1'b1; end
         if (l2_ack) owed <= 1'b0;
      end
      if (cr_valid) line[cr_beat*128 +: 128] <= cr_data;
   end
   always @(posedge clk) if (!reset) begin
      if (l2_req && (pend || owed))
         $fatal(1, "rv_mem_line_client: a request while one is owed (pend=%b owed=%b)", pend, owed);
      if ((cr_valid || cw_valid) && !owed)
         $fatal(1, "rv_mem_line_client: a response with nothing owed (read=%b write=%b)", cr_valid, cw_valid);
      if (cr_valid && we_q)
         $fatal(1, "rv_mem_line_client: read data for a write");
      if (cw_valid && !we_q)
         $fatal(1, "rv_mem_line_client: write done for a read");
      if ((cr_valid && cr_slot != {SW{1'b0}}) || (cw_valid && cw_slot != {SW{1'b0}}))
         $fatal(1, "rv_mem_line_client: a response for slot %0d; this client only uses slot 0",
                cr_valid ? cr_slot : cw_slot);
   end
endmodule

`default_nettype wire
