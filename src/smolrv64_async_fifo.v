module smolrv64_async_fifo #(
   parameter WIDTH = 64,
   parameter ADDR_BITS = 2,
   // "auto" (Vivado picks), "block" (force BRAM — better timing for
   // CDC paths under congestion), "distributed" (SLICEM LUTRAM)
   parameter MEMORY_TYPE = "auto"
) (
   input  wire             wr_clock,
   input  wire             rd_clock,
   input  wire             reset,
   input  wire             wr_valid,
   output wire             wr_ready,
   input  wire [WIDTH-1:0] wr_data,
   output wire             rd_valid,
   input  wire             rd_ready,
   output wire [WIDTH-1:0] rd_data
);
`ifdef SYNTHESIS
   wire full;
   wire empty;

   assign wr_ready = !full;
   assign rd_valid = !empty;

   xpm_fifo_async #(
      .CDC_SYNC_STAGES      ( 2 ),
      .DOUT_RESET_VALUE     ( "0" ),
      .ECC_MODE             ( "no_ecc" ),
      .FIFO_MEMORY_TYPE     ( MEMORY_TYPE ),
      .FIFO_READ_LATENCY    ( 0 ),
      .FIFO_WRITE_DEPTH     ( 1 << ADDR_BITS ),
      .FULL_RESET_VALUE     ( 0 ),
      .PROG_EMPTY_THRESH    ( 3 ),
      .PROG_FULL_THRESH     ( (1 << ADDR_BITS) - 2 ),
      .RD_DATA_COUNT_WIDTH  ( ADDR_BITS + 1 ),
      .READ_DATA_WIDTH      ( WIDTH ),
      .READ_MODE            ( "fwft" ),
      .RELATED_CLOCKS       ( 1 ),
      .SIM_ASSERT_CHK       ( 0 ),
      .USE_ADV_FEATURES     ( "0000" ),
      .WAKEUP_TIME          ( 0 ),
      .WRITE_DATA_WIDTH     ( WIDTH ),
      .WR_DATA_COUNT_WIDTH  ( ADDR_BITS + 1 )
   ) xpm_fifo_async_inst (
      .almost_empty  ( ),
      .almost_full   ( ),
      .data_valid    ( ),
      .dbiterr       ( ),
      .dout          ( rd_data ),
      .empty         ( empty ),
      .full          ( full ),
      .overflow      ( ),
      .prog_empty    ( ),
      .prog_full     ( ),
      .rd_data_count ( ),
      .rd_rst_busy   ( ),
      .sbiterr       ( ),
      .underflow     ( ),
      .wr_ack        ( ),
      .wr_data_count ( ),
      .wr_rst_busy   ( ),
      .din           ( wr_data ),
      .injectdbiterr ( 1'b0 ),
      .injectsbiterr ( 1'b0 ),
      .rd_clk        ( rd_clock ),
      .rd_en         ( rd_valid && rd_ready ),
      .rst           ( reset ),
      .sleep         ( 1'b0 ),
      .wr_clk        ( wr_clock ),
      .wr_en         ( wr_valid && wr_ready )
   );
`else
   localparam DEPTH = 1 << ADDR_BITS;
   reg [WIDTH-1:0] fifo_mem [0:DEPTH-1];
   reg [ADDR_BITS-1:0] wr_ptr = 0;
   reg [ADDR_BITS-1:0] rd_ptr = 0;
   reg [ADDR_BITS:0] count = 0;
   wire wr_fire = wr_valid && wr_ready;
   wire rd_fire = rd_valid && rd_ready;

   assign wr_ready = count != {1'b1, {ADDR_BITS{1'b0}}};
   assign rd_valid = count != 0;
   assign rd_data = fifo_mem[rd_ptr];

   always @(posedge wr_clock) begin
      if (reset) begin
         wr_ptr <= 0;
         rd_ptr <= 0;
         count <= 0;
      end else begin
         if (wr_fire) begin
            fifo_mem[wr_ptr] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
         end
         if (rd_fire)
            rd_ptr <= rd_ptr + 1'b1;
         case ({wr_fire, rd_fire})
           2'b10: count <= count + 1'b1;
           2'b01: count <= count - 1'b1;
           default: count <= count;
         endcase
      end
   end
`endif
endmodule
