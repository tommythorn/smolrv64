module smolrv64_cvfpu #(
   parameter int unsigned TAG_WIDTH = 8
) (
   input  logic                 clock,
   input  logic                 reset,

   input  logic                 in_valid,
   output logic                 in_ready,
   input  logic [2:0][63:0]     operands,
   input  logic [2:0]           rnd_mode,
   input  logic [3:0]           op,
   input  logic                 op_mod,
   input  logic [2:0]           src_fmt,
   input  logic [2:0]           dst_fmt,
   input  logic [1:0]           int_fmt,
   input  logic [TAG_WIDTH-1:0] tag_in,

   output logic [63:0]          result,
   output logic [4:0]           fflags,
   output logic [TAG_WIDTH-1:0] tag_out,
   output logic                 out_valid,
   input  logic                 out_ready,

   input  logic                 flush,
   output logic                 busy
);
   localparam fpnew_pkg::fpu_features_t Features = fpnew_pkg::RV64D;
   localparam fpnew_pkg::fpu_implementation_t Implementation = '{
      PipeRegs:   '{default: 1},
      UnitTypes:  '{'{default: fpnew_pkg::MERGED},   // ADDMUL
                    '{default: fpnew_pkg::MERGED},   // DIVSQRT
                    '{default: fpnew_pkg::PARALLEL}, // NONCOMP
                    '{default: fpnew_pkg::MERGED}},  // CONV
      PipeConfig: fpnew_pkg::BEFORE
   };

   fpnew_pkg::status_t status;
   logic early_valid;

   fpnew_top #(
      .Features       ( Features       ),
      .Implementation ( Implementation ),
      .DivSqrtSel     ( fpnew_pkg::THMULTI ),
      .TagType        ( logic [TAG_WIDTH-1:0] )
   ) cvfpu_inst (
      .clk_i         ( clock ),
      .rst_ni        ( !reset ),
      .operands_i    ( operands ),
      .rnd_mode_i    ( fpnew_pkg::roundmode_e'(rnd_mode) ),
      .op_i          ( fpnew_pkg::operation_e'(op) ),
      .op_mod_i      ( op_mod ),
      .src_fmt_i     ( fpnew_pkg::fp_format_e'(src_fmt) ),
      .dst_fmt_i     ( fpnew_pkg::fp_format_e'(dst_fmt) ),
      .int_fmt_i     ( fpnew_pkg::int_format_e'(int_fmt) ),
      .vectorial_op_i( 1'b0 ),
      .tag_i         ( tag_in ),
      .simd_mask_i   ( '1 ),
      .in_valid_i    ( in_valid ),
      .in_ready_o    ( in_ready ),
      .flush_i       ( flush ),
      .result_o      ( result ),
      .status_o      ( status ),
      .tag_o         ( tag_out ),
      .out_valid_o   ( out_valid ),
      .out_ready_i   ( out_ready ),
      .busy_o        ( busy ),
      .early_valid_o ( early_valid )
   );

   assign fflags = {status.NV, status.DZ, status.OF, status.UF, status.NX};

endmodule
