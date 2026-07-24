module smolrv64_sdpram #(
   parameter ADDR_WIDTH = 14,
   parameter DATA_WIDTH = 64,
   parameter READ_LATENCY = 1
) (
   input  wire                  clock,
   input  wire [ADDR_WIDTH-1:0] rd_addr,
   output wire [DATA_WIDTH-1:0] rd_data,
   input  wire                  wr_en,
   input  wire [ADDR_WIDTH-1:0] wr_addr,
   input  wire [DATA_WIDTH-1:0] wr_data
);
   // ---- hardware-proven-geometry guard (added after the wide-I$ RDW=128 regression) ----
   // A 128-bit sdpram I$ bank passed EVERY verilator test (behavioral reg-array model, below)
   // yet fetched garbage on real BRAM. DATA_WIDTH beyond one RAMB36 SDP word (72b) forces BRAM
   // WIDTH-cascade, whose read-latency/mux behavior neither the behavioral nor the XPM sim model
   // reflects -- so no verilator run can validate it. Hard-fail SYNTHESIS on an unproven-wide
   // geometry so it can never silently reach a bit; a deliberate experiment sets SDPRAM_ALLOW_WIDE
   // and MUST be post-synth / HW smoke-tested before trust. See project_sdpram_behavioral_vs_xpm.
   localparam PROVEN_MAX_W = 72;   // single RAMB36 SDP word; wider needs width-cascade
   initial if (DATA_WIDTH > PROVEN_MAX_W) begin
`ifdef SDPRAM_ALLOW_WIDE
      $display("[sdpram %m] NOTE: DATA_WIDTH=%0d > %0d (BRAM width-cascade) allowed via SDPRAM_ALLOW_WIDE", DATA_WIDTH, PROVEN_MAX_W);
`elsif SYNTHESIS
      $fatal(1, "smolrv64_sdpram %m: DATA_WIDTH=%0d > %0d needs BRAM width-cascade and is NOT hardware-proven (this class killed the wide-I$ bit). Post-synth/HW smoke-test it, then synthesize with -DSDPRAM_ALLOW_WIDE.", DATA_WIDTH, PROVEN_MAX_W);
`else
      $warning("[sdpram %m] DATA_WIDTH=%0d > %0d is a BRAM width-cascade geometry sim CANNOT validate; it will HARD-FAIL synthesis unless -DSDPRAM_ALLOW_WIDE. (wide-I$ regression)", DATA_WIDTH, PROVEN_MAX_W);
`endif
   end
`ifdef SMOLRV64_USE_XPM
   wire [0:0] wr_en_vec = wr_en;

   xpm_memory_sdpram #(
      .ADDR_WIDTH_A        ( ADDR_WIDTH ),
      .ADDR_WIDTH_B        ( ADDR_WIDTH ),
      .AUTO_SLEEP_TIME     ( 0 ),
      .BYTE_WRITE_WIDTH_A  ( DATA_WIDTH ),
      .CASCADE_HEIGHT      ( 0 ),
      .CLOCKING_MODE       ( "common_clock" ),
      .ECC_MODE            ( "no_ecc" ),
      .MEMORY_INIT_FILE    ( "none" ),
      .MEMORY_INIT_PARAM   ( "0" ),
      .MEMORY_OPTIMIZATION ( "true" ),
      .MEMORY_PRIMITIVE    ( "block" ),
      .MEMORY_SIZE         ( DATA_WIDTH * (1 << ADDR_WIDTH) ),
      .MESSAGE_CONTROL     ( 0 ),
      .READ_DATA_WIDTH_B   ( DATA_WIDTH ),
      .READ_LATENCY_B      ( READ_LATENCY ),
      .READ_RESET_VALUE_B  ( "0" ),
      .RST_MODE_A          ( "SYNC" ),
      .RST_MODE_B          ( "SYNC" ),
      .SIM_ASSERT_CHK      ( 0 ),
      .USE_EMBEDDED_CONSTRAINT( 0 ),
      .USE_MEM_INIT        ( 1 ),
      .WAKEUP_TIME         ( "disable_sleep" ),
      .WRITE_DATA_WIDTH_A  ( DATA_WIDTH ),
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
      .wea            ( wr_en_vec )
   );
`elsif SYNTHESIS
   wire [0:0] wr_en_vec = wr_en;

   xpm_memory_sdpram #(
      .ADDR_WIDTH_A        ( ADDR_WIDTH ),
      .ADDR_WIDTH_B        ( ADDR_WIDTH ),
      .AUTO_SLEEP_TIME     ( 0 ),
      .BYTE_WRITE_WIDTH_A  ( DATA_WIDTH ),
      .CASCADE_HEIGHT      ( 0 ),
      .CLOCKING_MODE       ( "common_clock" ),
      .ECC_MODE            ( "no_ecc" ),
      .MEMORY_INIT_FILE    ( "none" ),
      .MEMORY_INIT_PARAM   ( "0" ),
      .MEMORY_OPTIMIZATION ( "true" ),
      .MEMORY_PRIMITIVE    ( "block" ),
      .MEMORY_SIZE         ( DATA_WIDTH * (1 << ADDR_WIDTH) ),
      .MESSAGE_CONTROL     ( 0 ),
      .READ_DATA_WIDTH_B   ( DATA_WIDTH ),
      .READ_LATENCY_B      ( READ_LATENCY ),
      .READ_RESET_VALUE_B  ( "0" ),
      .RST_MODE_A          ( "SYNC" ),
      .RST_MODE_B          ( "SYNC" ),
      .SIM_ASSERT_CHK      ( 0 ),
      .USE_EMBEDDED_CONSTRAINT( 0 ),
      .USE_MEM_INIT        ( 1 ),
      .WAKEUP_TIME         ( "disable_sleep" ),
      .WRITE_DATA_WIDTH_A  ( DATA_WIDTH ),
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
      .wea            ( wr_en_vec )
   );
`else
   (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram[0:(1 << ADDR_WIDTH)-1];
   reg [DATA_WIDTH-1:0] rd_data_r = 0;
   reg [DATA_WIDTH-1:0] rd_data_rr = 0;
   integer ram_init_i;

   initial begin
      for (ram_init_i = 0; ram_init_i < (1 << ADDR_WIDTH); ram_init_i = ram_init_i + 1)
         ram[ram_init_i] = 0;
   end

   assign rd_data = READ_LATENCY == 1 ? rd_data_r : rd_data_rr;

   always @(posedge clock) begin
      rd_data_r <= ram[rd_addr];
      rd_data_rr <= rd_data_r;
      if (wr_en)
         ram[wr_addr] <= wr_data;
   end
`endif
endmodule
