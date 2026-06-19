module regfile(input wire         clock,
               input wire         write_valid,
               input wire [ 4:0]  write_addr,
               input wire [63:0]  write_data,
               input wire [ 4:0]  read_addr_0,
               input wire [ 4:0]  read_addr_1,

`ifdef ASYNC_RF
               output wire [63:0] read_data_0,
               output wire [63:0] read_data_1
`else
               output reg  [63:0] read_data_0,
               output reg  [63:0] read_data_1
`endif
);

   (* ram_style = "block" *)
   // Since the memory currently is baked into SmolRV64 and we
   // don't have a device tree, we take the shortcut of
   // - embedding the frequency into register 7 of the UART
   // - initializing sp to the end of physical memory.
   // This is only true for now and will definitely change.
   reg  [63:0] regfile[31:0];
   reg [8*200:0] rf_path;
   initial begin
`ifndef SYNTHESIS
      if ($value$plusargs("rf=%s", rf_path))
         $readmemh(rf_path, regfile, 0, 31);
      else
`endif
         $readmemh("rf.hex", regfile, 0, 31);
   end

   always @(posedge clock) begin
`ifndef ASYNC_RF
      // Non-blocking: samples read_addr at the clock edge (before any blocking
      // assignments from other always blocks), ensuring deterministic simulation.
      read_data_0 <= regfile[read_addr_0];
      read_data_1 <= regfile[read_addr_1];
`endif

      if (write_valid) regfile[write_addr] <= write_data;
   end

`ifdef ASYNC_RF
   assign read_data_0 = regfile[read_addr_0];
   assign read_data_1 = regfile[read_addr_1];
`endif
endmodule
