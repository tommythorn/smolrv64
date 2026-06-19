module fregfile(input wire         clock,
                input wire         write_valid,
                input wire [ 4:0]  write_addr,
                input wire [63:0]  write_data,
                input wire [ 4:0]  read_addr_0,
                input wire [ 4:0]  read_addr_1,
                input wire [ 4:0]  read_addr_2,

`ifdef ASYNC_RF
                output wire [63:0] read_data_0,
                output wire [63:0] read_data_1,
                output wire [63:0] read_data_2
`else
                output reg  [63:0] read_data_0,
                output reg  [63:0] read_data_1,
                output reg  [63:0] read_data_2
`endif
);
   (* ram_style = "block" *)
   reg  [63:0] fregfile[31:0];
   integer i;
   initial for (i = 0; i < 32; i = i + 1) fregfile[i] = 0;

   always @(posedge clock) begin
`ifndef ASYNC_RF
      read_data_0 <= fregfile[read_addr_0];
      read_data_1 <= fregfile[read_addr_1];
      read_data_2 <= fregfile[read_addr_2];
`endif

      if (write_valid) fregfile[write_addr] <= write_data;
   end

`ifdef ASYNC_RF
   assign read_data_0 = fregfile[read_addr_0];
   assign read_data_1 = fregfile[read_addr_1];
   assign read_data_2 = fregfile[read_addr_2];
`endif
endmodule
