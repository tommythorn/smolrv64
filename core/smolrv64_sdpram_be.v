// smolrv64_sdpram_be: smolrv64_sdpram with a write enable per byte. One RAMB36 in simple-dual-port
// mode is 512 x 72 with eight byte enables, so a 64-bit word written a byte at a time costs nothing
// over a whole-word write: a store writes its bytes without reading the word first.
module smolrv64_sdpram_be #(
   parameter ADDR_WIDTH = 9,
   parameter NBYTES     = 8,
   parameter LAT        = 1                 // read latency: 2 adds the block RAM's output register
) (
   input  wire                    clock,
   input  wire [ADDR_WIDTH-1:0]   rd_addr,
   output wire [8*NBYTES-1:0]     rd_data,
   input  wire [NBYTES-1:0]       wr_be,
   input  wire [ADDR_WIDTH-1:0]   wr_addr,
   input  wire [8*NBYTES-1:0]     wr_data
);
   localparam W = 8 * NBYTES;
   initial if (LAT < 1 || LAT > 2) $fatal(1, "smolrv64_sdpram_be %m: read latency %0d is not 1 or 2", LAT);
   initial if (W > 72) $fatal(1, "smolrv64_sdpram_be %m: %0d bits is wider than one RAMB36 word", W);
`ifdef SYNTHESIS
   xpm_memory_sdpram #(
      .ADDR_WIDTH_A        ( ADDR_WIDTH ),
      .ADDR_WIDTH_B        ( ADDR_WIDTH ),
      .AUTO_SLEEP_TIME     ( 0 ),
      .BYTE_WRITE_WIDTH_A  ( 8 ),
      .CASCADE_HEIGHT      ( 0 ),
      .CLOCKING_MODE       ( "common_clock" ),
      .ECC_MODE            ( "no_ecc" ),
      .MEMORY_INIT_FILE    ( "none" ),
      .MEMORY_INIT_PARAM   ( "0" ),
      .MEMORY_OPTIMIZATION ( "true" ),
      .MEMORY_PRIMITIVE    ( "block" ),
      .MEMORY_SIZE         ( W * (1 << ADDR_WIDTH) ),
      .MESSAGE_CONTROL     ( 0 ),
      .READ_DATA_WIDTH_B   ( W ),
      .READ_LATENCY_B      ( LAT ),
      .READ_RESET_VALUE_B  ( "0" ),
      .RST_MODE_A          ( "SYNC" ),
      .RST_MODE_B          ( "SYNC" ),
      .SIM_ASSERT_CHK      ( 0 ),
      .USE_EMBEDDED_CONSTRAINT( 0 ),
      .USE_MEM_INIT        ( 1 ),
      .WAKEUP_TIME         ( "disable_sleep" ),
      .WRITE_DATA_WIDTH_A  ( W ),
      .WRITE_MODE_B        ( "read_first" )
   ) xpm_memory_sdpram_inst (
      .dbiterrb       ( ),
      .doutb          ( rd_data ),
      .sbiterrb       ( ),
      .addra          ( wr_addr ),
      .addrb          ( rd_addr ),
      .clka           ( clock ),
      .clkb           ( clock ),
      .dina           ( wr_data ),
      .ena            ( 1'b1 ),
      .enb            ( 1'b1 ),
      .injectdbiterra ( 1'b0 ),
      .injectsbiterra ( 1'b0 ),
      .regceb         ( 1'b1 ),
      .rstb           ( 1'b0 ),
      .sleep          ( 1'b0 ),
      .wea            ( wr_be )
   );
`else
   (* ram_style = "block" *) reg [W-1:0] ram [0:(1 << ADDR_WIDTH)-1];
   reg [W-1:0] rd_q = 0, rd_q2 = 0;
   integer i, b;
   initial for (i = 0; i < (1 << ADDR_WIDTH); i = i + 1) ram[i] = 0;
   assign rd_data = (LAT == 2) ? rd_q2 : rd_q;
   always @(posedge clock) begin
      rd_q2 <= rd_q;
      // A collision reads X, as the block RAM's does (smolrv64_sdpram explains why the model must
      // not be kinder than the hardware): the write lands, the read of that word does not.
      if ((|wr_be) && (rd_addr == wr_addr)) rd_q <= {W{1'bx}};
      else                                   rd_q <= ram[rd_addr];
      for (b = 0; b < NBYTES; b = b + 1)
         if (wr_be[b]) ram[wr_addr][b*8 +: 8] <= wr_data[b*8 +: 8];
   end
`endif
endmodule
