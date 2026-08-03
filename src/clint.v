`default_nettype none

// CLINT (core-local interruptor) -- a faithful port of SmolRV64's CLINT so the
// probe stays 100% software-compatible (same base, offsets, and tick rate).
//
// Memory map (base 0x0200_0000, 64 KiB region):
//   0x0000          msip       bit0 = machine software interrupt pending (MSIP)
//   0x4000/0x4004   mtimecmp   64-bit timer compare (lo/hi word)
//   0xBFF8/0xBFFC   mtime      64-bit monotonic time   (lo/hi word)
//
// mtime advances one tick every SCALE_DIV core clocks (SmolRV64: 3333 @ 333.33 MHz
// => ~100.01 kHz, matching the DTS timebase-frequency). The interrupt lines are
// registered (mtip = mtime >= mtimecmp, msip = msip reg) to keep them off the
// fetch/decode critical path, exactly as SmolRV64 does. mtime is exported so the
// CSR file can serve the TIME counter and the Sstc stimecmp (STIP) comparison.
//
// This is the device only: the platform routes MMIO loads/stores in the CLINT
// region to {we,addr,wdata,wmask}/rdata, and wires {mtip,msip} into the core's
// hw_ip port. No bus arbitration here (the SoC integration owns that).
module clint
  #(parameter SCALE_DIV = 3333)         // core clocks per mtime tick (>=2)
   (input  wire        clk,
    input  wire        reset,
    // MMIO port: 16-bit region offset, byte-masked 64-bit write, combinational read
    input  wire        we,
    input  wire [15:0] addr,            // offset within the CLINT region
    input  wire [63:0] wdata,
    input  wire [7:0]  wmask,
    output reg  [63:0] rdata,
    // interrupt lines + time, to the core's hw_ip / CSR file
    output reg         mtip,            // MTIP: mtime >= mtimecmp (registered)
    output wire        msip,            // MSIP: software interrupt
    output wire [63:0] o_mtime);        // free-running time (for TIME csr / stimecmp)

   localparam SCW = $clog2(SCALE_DIV);  // scaler width

   reg [63:0]     mtime;
   reg [63:0]     mtimecmp;
   reg            msip_r;
   reg [SCW-1:0]  scaler;

   assign msip    = msip_r;
   assign o_mtime = mtime;

   always @(posedge clk) begin
      if (reset) begin
         mtime    <= 64'd0;
         mtimecmp <= ~64'd0;            // max => no spurious timer interrupt at reset
         msip_r   <= 1'b0;
         scaler   <= SCALE_DIV[SCW-1:0] - 1'b1;
         mtip     <= 1'b0;
      end else begin
         // prescaled monotonic time
         if (scaler == 0) begin
            scaler <= SCALE_DIV[SCW-1:0] - 1'b1;
            mtime  <= mtime + 64'd1;
         end else
            scaler <= scaler - 1'b1;

         // registered compare (a software mtimecmp write also clears MTIP next cycle)
         mtip <= (mtime >= mtimecmp);

         // MMIO writes (word-granular like SmolRV64: wmask[4] picks a full 64-bit store)
         if (we) case (addr)
            16'h0000: msip_r <= wdata[0];
            16'h4000: if (wmask[4]) mtimecmp        <= wdata;
                      else          mtimecmp[31:0]  <= wdata[31:0];
            16'h4004:               mtimecmp[63:32] <= wdata[31:0];
            16'hBFF8: if (wmask[4]) mtime           <= wdata;
                      else          mtime[31:0]     <= wdata[31:0];
            16'hBFFC:               mtime[63:32]    <= wdata[31:0];
            default: ;
         endcase
      end
   end

   // combinational read
   always @* begin
      case (addr)
         16'h0000: rdata = {63'd0, msip_r};
         16'h4000: rdata = mtimecmp;
         16'h4004: rdata = {32'd0, mtimecmp[63:32]};
         16'hBFF8: rdata = mtime;
         16'hBFFC: rdata = {32'd0, mtime[63:32]};
         default:  rdata = 64'd0;
      endcase
   end
endmodule

`default_nettype wire
