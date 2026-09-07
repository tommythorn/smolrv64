`default_nettype none

// Behavioral stand-in for fp_unit, used ONLY by the iverilog unit-TB flow (run-tb.sh):
// CVFPU (fpnew) uses SystemVerilog concurrent assertions iverilog can't parse, so the
// backend TBs that instantiate the core compile against this instead. It mirrors fp_unit's
// port list and a deferred (multi-cycle) completion so the scheduler's FP deferred-wake path
// behaves the same; the arithmetic covers the common double ops (enough for any directed FP
// poke). The real cvfpu is used by the verilator builds (ooo2/run-ooo2-vl.sh). PIPE_REGS and
// NFLIGHT are accepted so the core's instantiation elaborates; the stub holds one op at a time.
module fp_unit #(parameter TAGW = 24,
                 parameter int unsigned PIPE_REGS = 4,
                 parameter int unsigned NFLIGHT = 1)
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
    input  wire [191:0]     iss_operands,   // {op2, op1, op0}
    input  wire [TAGW-1:0]  iss_tag,
    output wire             res_valid,
    input  wire             res_ready,
    output wire [63:0]      res_data,
    output wire [4:0]       res_fflags,
    output wire [TAGW-1:0]  res_tag,
    input  wire             flush,
    output wire             busy);

   localparam LAT = 4;                       // mimic the 4-stage cvfpu pipe latency

   wire [63:0] o0 = iss_operands[63:0];
   wire [63:0] o1 = iss_operands[127:64];
   wire [63:0] o2 = iss_operands[191:128];

   // double-precision behavioral arithmetic (fpnew op: ADD=2 MUL=3 DIV=4 SQRT=5 MINMAX=7)
   real ra, rb, rr;
   reg [63:0] result_c;
   always @* begin
      ra = $bitstoreal(o1); rb = $bitstoreal(o2);
      case (iss_op)
        4'd2: rr = iss_op_mod ? (ra - rb) : (ra + rb);   // ADD / SUB
        4'd3: rr = ra * rb;                              // MUL
        4'd4: rr = (rb == 0.0) ? ra : (ra / rb);         // DIV
        default: rr = ra;
      endcase
      result_c = $realtobits(rr);
   end
   // REFUSE what the stub cannot compute -- silent garbage here cost days: FCVT fell
   // through to rr=ra (F2I returned RAW FLOAT BITS as the int result) and every single-
   // precision op was computed as a double, wedging the ubuntu boot in the systemd-
   // generator window in a way that mimicked the FPGA hang. Double-only ADD/SUB/MUL/DIV
   // is all it honestly implements; anything else must fail LOUDLY at first use.
   always @(posedge clk) if (iss_valid & iss_ready) begin
      if (!(iss_op == 4'd2 || iss_op == 4'd3 || iss_op == 4'd4))
         $fatal(1, "fp_unit_stub: unimplemented fpnew op %0d (conversions/sqrt/minmax return garbage) -- use the real CVFPU", iss_op);
      if (iss_src_fmt != 3'd1 || iss_dst_fmt != 3'd1)
         $fatal(1, "fp_unit_stub: non-double fmt src=%0d dst=%0d (stub computes double only) -- use the real CVFPU", iss_src_fmt, iss_dst_fmt);
   end

   // shift-register pipe carrying valid/result/tag for LAT cycles
   reg [LAT-1:0]   v_pipe;
   reg [63:0]      d_pipe [0:LAT-1];
   reg [TAGW-1:0]  t_pipe [0:LAT-1];
   integer k;
   wire fire = iss_valid & iss_ready;
   always @(posedge clk) begin
      if (reset || flush) begin
         v_pipe <= {LAT{1'b0}};
      end else begin
         v_pipe <= {v_pipe[LAT-2:0], fire};
         d_pipe[0] <= result_c; t_pipe[0] <= iss_tag;
         for (k = 1; k < LAT; k = k + 1) begin
            d_pipe[k] <= d_pipe[k-1]; t_pipe[k] <= t_pipe[k-1];
         end
      end
   end

   assign iss_ready  = ~busy;
   assign busy       = |v_pipe;                 // single-op-at-a-time, like the real unit gate
   assign res_valid  = v_pipe[LAT-1];
   assign res_data   = d_pipe[LAT-1];
   assign res_tag    = t_pipe[LAT-1];
   assign res_fflags = 5'd0;
endmodule

`default_nettype wire
