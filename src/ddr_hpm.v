`default_nettype none

// DDR latency hardware performance monitor.
//
// Taps rv_soc_top's EXTERNAL DDR line port (ddr_req/ddr_we/ddr_ack) and measures the
// per-transaction request->ack latency in CORE-CLOCK cycles -- exactly the latency the
// core sees, and exactly what the sim DDR model approximates with `memlat`. So the HW
// distribution captured here feeds straight back into a more accurate sim model.
//
// Per direction (read = ~we, write = we) it keeps:
//   * an 8-bin log2 histogram: bin = floor(log2(lat))+1 capped at 7. Latency is >=1, so
//     bin0 is unused; bins are [1] [2-3] [4-7] [8-15] [16-31] [32-63] [64+] (matches the
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
   parameter LW = 12                       // latency counter width (caps at 2^LW-1 cycles)
)(
   input  wire        clk,
   input  wire        reset,
   // observation taps off the external DDR line port
   input  wire        ddr_req,
   input  wire        ddr_we,
   input  wire        ddr_ack,
   // MMIO: byte offset within the window, 64-bit read data, and a clear strobe (any write)
   input  wire [7:0]  raddr,
   output reg  [63:0] rdata,
   input  wire        clr
);
   localparam [LW-1:0] ONE = {{(LW-1){1'b0}}, 1'b1};

   // ---- per-transaction latency: count cycles from the first req cycle to the ack ----
   reg           meas;       // a transaction is being timed
   reg  [LW-1:0] lat;        // cycles elapsed (1 on the first req cycle)
   reg           we_l;       // latched direction
   reg           stb;        // 1-cycle: a transaction completed this cycle
   reg  [LW-1:0] stb_lat;
   reg           stb_we;
   always @(posedge clk) begin
      stb <= 1'b0;
      if (reset) meas <= 1'b0;
      else if (!meas) begin
         if (ddr_req) begin
            if (ddr_ack) begin                  // 1-cycle transaction (req & ack same cycle)
               stb <= 1'b1; stb_lat <= ONE; stb_we <= ddr_we;
            end else begin                      // start timing
               meas <= 1'b1; lat <= ONE; we_l <= ddr_we;
            end
         end
      end else begin
         if (ddr_ack) begin                     // completed: record the latency
            stb <= 1'b1; stb_lat <= lat; stb_we <= we_l; meas <= 1'b0;
         end else if (lat != {LW{1'b1}})         // saturate rather than wrap a pathological wait
            lat <= lat + 1'b1;
      end
   end

   // ---- log2 bin of the completed latency (highest set bit + 1, capped at 7) ----
   function [2:0] binof; input [LW-1:0] v; integer i; begin
      binof = 3'd0;
      for (i = 0; i < LW; i = i + 1) if (v[i]) binof = (i >= 6) ? 3'd7 : (i[2:0] + 3'd1);
   end endfunction
   wire [2:0] bin = binof(stb_lat);

   // ---- counters ----
   reg [63:0] rdb [0:7];
   reg [63:0] wrb [0:7];
   reg [63:0] rd_sum, wr_sum, rd_cnt, wr_cnt;
   wire [63:0] lat_ext = {{(64-LW){1'b0}}, stb_lat};
   integer k;
   always @(posedge clk) begin
      if (reset || clr) begin
         for (k = 0; k < 8; k = k + 1) begin rdb[k] <= 64'd0; wrb[k] <= 64'd0; end
         rd_sum <= 64'd0; wr_sum <= 64'd0; rd_cnt <= 64'd0; wr_cnt <= 64'd0;
      end else if (stb) begin
         if (stb_we) begin
            wrb[bin] <= wrb[bin] + 64'd1;  wr_sum <= wr_sum + lat_ext;  wr_cnt <= wr_cnt + 64'd1;
         end else begin
            rdb[bin] <= rdb[bin] + 64'd1;  rd_sum <= rd_sum + lat_ext;  rd_cnt <= rd_cnt + 64'd1;
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
