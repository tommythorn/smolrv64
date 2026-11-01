`default_nettype none

// DDR latency hardware performance monitor.
//
// Taps rv_soc_top's EXTERNAL DDR memory port and measures each transaction's latency in
// CORE-CLOCK cycles: from the cycle its request is accepted to the cycle its read's last beat
// or its write done arrives -- exactly the latency the caches see, and exactly what the sim
// DDR model draws. So the HW distribution captured here feeds straight back into the model.
// Transactions overlap: each id's acceptance cycle is kept until that id completes (rule B1:
// the completion names its id).
//
// Per direction (read = ~we, write = we) it keeps:
//   * an 8-bin log2 histogram: bin = floor(log2(lat))+1 capped at 7. Latency is >= 2 (the
//     start is written the cycle after the acceptance; a DDR round trip is far longer, asserted),
//     so bins 0 and 1 stay empty; bins are [1] [2-3] [4-7] [8-15] [16-31] [32-63] [64+] (matches the
//     perftool bucket() convention so the same reader/plots apply).
//   * SUM of latencies (mean = sum / count) and transaction COUNT.
//
// Read-only via a 64-bit MMIO window (rv_soc_top decodes the base; this module just sees the
// byte offset). ANY write to the window clears all counters -- snapshot-then-clear lets a
// tool isolate a phase. Pure observation: the taps are fan-out only, off the core path.
//
// MMIO map (8 bytes/slot, slot = offset[7:3]):
//   0x00..0x38  rd_bin[0..7]      0x40..0x78  wr_bin[0..7]
//   0x80 rd_sum   0x88 wr_sum   0x90 rd_cnt   0x98 wr_cnt
module ddr_hpm #(
   parameter LW  = 12,                     // latency width (a wait longer than 2^LW-1 wraps)
   parameter IDW = 5                       // the port's id width
)(
   input  wire           clk,
   input  wire           reset,
   // observation taps off the external DDR memory port
   input  wire           q_fire,           // a request accepted
   input  wire [IDW-1:0] q_id,
   input  wire           r_done,           // a read's last beat taken
   input  wire [IDW-1:0] r_id,
   input  wire           w_done,           // a write done taken
   input  wire [IDW-1:0] w_id,
   // MMIO: byte offset within the window, 64-bit read data, and a clear strobe (any write)
   input  wire [7:0]     raddr,
   output reg  [63:0]    rdata,
   input  wire           clr
);
   localparam [LW-1:0] ONE = {{(LW-1){1'b0}}, 1'b1};
   // ---- per-id acceptance cycle; a completion's latency is now - start (>= 2) ----
   // The acceptance (q_fire: the port's valid and the DDR crossing's ready, a promise launched in the
   // other clock with half a cycle to arrive) is registered with its id and cycle, and written a cycle
   // later: the same start, and the crossing's ready no longer the enable of a RAM's write. A
   // completion comes many cycles after its acceptance; one meeting its id's write still pending is
   // asserted.
   reg  [LW-1:0] now, qf_now;
   reg  [IDW-1:0] qf_id;
   reg           qf_v;
   reg  [LW-1:0] start [0:(1<<IDW)-1];
   reg           rstb, wstb;               // 1-cycle: a read / a write completed
   reg  [LW-1:0] rlat, wlat;
   initial qf_v = 1'b0;
   always @(posedge clk) begin
      now <= reset ? {LW{1'b0}} : now + ONE;
      qf_v <= ~reset & q_fire;  qf_id <= q_id;  qf_now <= now;
      if (qf_v) start[qf_id] <= qf_now;
      rstb <= ~reset & r_done;  rlat <= now - start[r_id];
      wstb <= ~reset & w_done;  wlat <= now - start[w_id];
   end
   always @(posedge clk)
      if (!reset && qf_v && ((r_done && r_id == qf_id) || (w_done && w_id == qf_id)))
         $fatal(1, "ddr_hpm: id %0d completes the cycle after its acceptance, before its start is written", qf_id);
   // ---- log2 bin of the completed latency (highest set bit + 1, capped at 7) ----
   function [2:0] binof; input [LW-1:0] v; integer i; begin
      binof = 3'd0;
      for (i = 0; i < LW; i = i + 1) if (v[i]) binof = (i >= 6) ? 3'd7 : (i[2:0] + 3'd1);
   end endfunction
   wire [2:0] rbin = binof(rlat);
   wire [2:0] wbin = binof(wlat);

   // ---- counters ----
   reg [63:0] rdb [0:7];
   reg [63:0] wrb [0:7];
   reg [63:0] rd_sum, wr_sum, rd_cnt, wr_cnt;
   integer k;
   always @(posedge clk) begin
      if (reset || clr) begin
         for (k = 0; k < 8; k = k + 1) begin rdb[k] <= 64'd0; wrb[k] <= 64'd0; end
         rd_sum <= 64'd0; wr_sum <= 64'd0; rd_cnt <= 64'd0; wr_cnt <= 64'd0;
      end else begin
         if (rstb) begin
            rdb[rbin] <= rdb[rbin] + 64'd1;  rd_sum <= rd_sum + {{(64-LW){1'b0}}, rlat};  rd_cnt <= rd_cnt + 64'd1;
         end
         if (wstb) begin
            wrb[wbin] <= wrb[wbin] + 64'd1;  wr_sum <= wr_sum + {{(64-LW){1'b0}}, wlat};  wr_cnt <= wr_cnt + 64'd1;
         end
      end
   end

   // ---- MMIO read mux ----
   wire [4:0] slot = raddr[7:3];
   always @* begin
      case (slot)
        5'd0,5'd1,5'd2,5'd3,5'd4,5'd5,5'd6,5'd7:        rdata = rdb[slot[2:0]];
        5'd8,5'd9,5'd10,5'd11,5'd12,5'd13,5'd14,5'd15:  rdata = wrb[slot[2:0]];
        5'd16: rdata = rd_sum;
        5'd17: rdata = wr_sum;
        5'd18: rdata = rd_cnt;
        5'd19: rdata = wr_cnt;
        default: rdata = 64'd0;
      endcase
   end
endmodule

`default_nettype wire
