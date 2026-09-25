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
   // Simulation: a real two-clock FIFO. Each side keeps its own pointer; the other side sees it
   // through a two-flop synchronizer (CDC_SYNC_STAGES=2, as the XPM instance above), so full and
   // empty are conservative by the synchronizer's latency exactly as in hardware. First-word
   // fall-through: rd_data is the head whenever rd_valid.
   localparam DEPTH = 1 << ADDR_BITS;
   reg [WIDTH-1:0]   fifo_mem [0:DEPTH-1];
   reg [ADDR_BITS:0] wr_ptr = 0, rd_ptr = 0;             // one wrap bit above the index
   reg [ADDR_BITS:0] rd_ptr_w0 = 0, rd_ptr_w1 = 0;       // rd_ptr as the write side sees it
   reg [ADDR_BITS:0] wr_ptr_r0 = 0, wr_ptr_r1 = 0;       // wr_ptr as the read side sees it
   wire wr_fire = wr_valid && wr_ready;
   wire rd_fire = rd_valid && rd_ready;
   assign wr_ready = (wr_ptr - rd_ptr_w1) != DEPTH[ADDR_BITS:0];
   assign rd_valid = (wr_ptr_r1 != rd_ptr);
   assign rd_data  = fifo_mem[rd_ptr[ADDR_BITS-1:0]];
   always @(posedge wr_clock) begin
      if (reset) begin wr_ptr <= 0; rd_ptr_w0 <= 0; rd_ptr_w1 <= 0; end
      else begin
         if (wr_fire) begin fifo_mem[wr_ptr[ADDR_BITS-1:0]] <= wr_data; wr_ptr <= wr_ptr + 1'b1; end
         rd_ptr_w0 <= rd_ptr; rd_ptr_w1 <= rd_ptr_w0;
      end
   end
   always @(posedge rd_clock) begin
      if (reset) begin rd_ptr <= 0; wr_ptr_r0 <= 0; wr_ptr_r1 <= 0; end
      else begin
         if (rd_fire) rd_ptr <= rd_ptr + 1'b1;
         wr_ptr_r0 <= wr_ptr; wr_ptr_r1 <= wr_ptr_r0;
      end
   end
`endif
endmodule
