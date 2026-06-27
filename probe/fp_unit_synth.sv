`default_nettype none
// (committed copy of the OOC synth tie-off: passthrough fp_unit for FPGA bring-up before
// real/shared CVFPU -- task 23. FSD/FLD are LSU ops so kernel boot needs no FP compute.)

// Synthesizable tie-off fp_unit for the OOC integer-core Fmax check ONLY. Mirrors
// fp_unit's ports and the 4-stage deferred-completion pipe (so the scheduler's FP
// wake structure synthesizes realistically), but drops the sim-only real-number
// arithmetic ($bitstoreal) -- the result is a passthrough. The real FPGA build uses
// the actual CVFPU on its own divided clock; FP is never on the 333 MHz path.
module fp_unit #(parameter TAGW = 24)
   (input  wire             clk,
    input  wire             reset,
    input  wire             iss_valid,
    output wire             iss_ready,
    input  wire [3:0]       iss_op,
    input  wire             iss_op_mod,
    input  wire [2:0]       iss_src_fmt,
    input  wire [2:0]       iss_dst_fmt,
    input  wire [1:0]       iss_int_fmt,
    input  wire [2:0]       iss_rnd,
    input  wire [191:0]     iss_operands,
    input  wire [TAGW-1:0]  iss_tag,
    output wire             res_valid,
    input  wire             res_ready,
    output wire [63:0]      res_data,
    output wire [4:0]       res_fflags,
    output wire [TAGW-1:0]  res_tag,
    input  wire             flush,
    output wire             busy);

   localparam LAT = 4;
   wire [63:0] result_c = iss_operands[63:0];   // passthrough (synth-clean)

   reg [LAT-1:0]   v_pipe;
   reg [63:0]      d_pipe [0:LAT-1];
   reg [TAGW-1:0]  t_pipe [0:LAT-1];
   integer k;
   wire fire = iss_valid & iss_ready;
   always @(posedge clk) begin
      if (reset || flush) v_pipe <= {LAT{1'b0}};
      else begin
         v_pipe <= {v_pipe[LAT-2:0], fire};
         d_pipe[0] <= result_c; t_pipe[0] <= iss_tag;
         for (k = 1; k < LAT; k = k + 1) begin
            d_pipe[k] <= d_pipe[k-1]; t_pipe[k] <= t_pipe[k-1];
         end
      end
   end

   assign iss_ready  = ~busy;
   assign busy       = |v_pipe;
   assign res_valid  = v_pipe[LAT-1];
   assign res_data   = d_pipe[LAT-1];
   assign res_tag    = t_pipe[LAT-1];
   assign res_fflags = 5'd0;
endmodule

`default_nettype wire
