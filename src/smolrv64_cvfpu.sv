module smolrv64_cvfpu #(
   parameter int unsigned TAG_WIDTH = 8,
   parameter int unsigned PIPE_REGS = 4
) (
   input  wire logic                 clock,
   input  wire logic                 fpu_clock,
   input  wire logic                 reset,

   input  wire logic                 in_valid,
   output wire logic                 in_ready,
   input  wire logic [2:0][63:0]     operands,
   input  wire logic [2:0]           rnd_mode,
   input  wire logic [3:0]           op,
   input  wire logic                 op_mod,
   input  wire logic [2:0]           src_fmt,
   input  wire logic [2:0]           dst_fmt,
   input  wire logic [1:0]           int_fmt,
   input  wire logic [TAG_WIDTH-1:0] tag_in,

   output wire logic [63:0]          result,
   output wire logic [4:0]           fflags,
   output wire logic [TAG_WIDTH-1:0] tag_out,
   output wire logic                 out_valid,
   input  wire logic                 out_ready,

   input  wire logic                 flush,
   output wire logic                 busy
);
   localparam fpnew_pkg::fpu_features_t Features = fpnew_pkg::RV64D;
   localparam fpnew_pkg::fpu_implementation_t Implementation = '{
      PipeRegs:   '{default: PIPE_REGS},
      UnitTypes:  '{'{default: fpnew_pkg::MERGED},   // ADDMUL
                    '{default: fpnew_pkg::MERGED},   // DIVSQRT
                    '{default: fpnew_pkg::PARALLEL}, // NONCOMP
                    '{default: fpnew_pkg::MERGED}},  // CONV
      PipeConfig: fpnew_pkg::DISTRIBUTED
   };

   typedef struct packed {
      logic [2:0][63:0]     operands;
      logic [2:0]           rnd_mode;
      logic [3:0]           op;
      logic                 op_mod;
      logic [2:0]           src_fmt;
      logic [2:0]           dst_fmt;
      logic [1:0]           int_fmt;
      logic [TAG_WIDTH-1:0] tag;
   } request_t;

   request_t core_req_q = '0;
   logic     core_req_pending_q = 1'b0;
   logic     core_req_toggle_q = 1'b0;

   logic [63:0]          core_result_q = '0;
   logic [4:0]           core_fflags_q = '0;
   logic [TAG_WIDTH-1:0] core_tag_q = '0;
   logic                 core_out_valid_q = 1'b0;

   (* ASYNC_REG = "TRUE" *) logic [1:0] core_resp_sync_q = 2'b00;
   logic                                core_resp_seen_q = 1'b0;

   request_t fpu_req_q = '0;
   fpnew_pkg::status_t fpu_status;
   logic [63:0]          fpu_result;
   logic [TAG_WIDTH-1:0] fpu_tag;
   logic                 fpu_in_valid_q = 1'b0;
   logic                 fpu_in_ready;
   logic                 fpu_out_valid;
   logic                 fpu_busy;
   logic                 fpu_early_valid;
   logic                 fpu_resp_toggle_q = 1'b0;
   logic [63:0]          fpu_resp_result_q = '0;
   logic [4:0]           fpu_resp_fflags_q = '0;
   logic [TAG_WIDTH-1:0] fpu_resp_tag_q = '0;

   (* ASYNC_REG = "TRUE" *) logic [1:0] fpu_req_sync_q = 2'b00;
   (* ASYNC_REG = "TRUE" *) logic [1:0] fpu_reset_sync_q = 2'b11;
   logic                                fpu_req_seen_q = 1'b0;
   logic                                fpu_reset;

   typedef enum logic [1:0] {
      FPU_IDLE,
      FPU_ISSUE,
      FPU_WAIT
   } fpu_state_t;
   fpu_state_t fpu_state_q = FPU_IDLE;

   assign in_ready = !core_req_pending_q && !core_out_valid_q;
   assign result = core_result_q;
   assign fflags = core_fflags_q;
   assign tag_out = core_tag_q;
   assign out_valid = core_out_valid_q;
   assign busy = core_req_pending_q || core_out_valid_q;

   always_ff @(posedge clock) begin
      if (reset || flush) begin
         core_req_q <= '0;
         core_req_pending_q <= 1'b0;
         core_req_toggle_q <= 1'b0;
         core_result_q <= '0;
         core_fflags_q <= '0;
         core_tag_q <= '0;
         core_out_valid_q <= 1'b0;
         core_resp_sync_q <= 2'b00;
         core_resp_seen_q <= 1'b0;
      end else begin
         core_resp_sync_q <= {core_resp_sync_q[0], fpu_resp_toggle_q};

         if (core_out_valid_q && out_ready)
            core_out_valid_q <= 1'b0;

         if (core_resp_sync_q[1] != core_resp_seen_q) begin
            core_resp_seen_q <= core_resp_sync_q[1];
            core_req_pending_q <= 1'b0;
            core_result_q <= fpu_resp_result_q;
            core_fflags_q <= fpu_resp_fflags_q;
            core_tag_q <= fpu_resp_tag_q;
            core_out_valid_q <= 1'b1;
         end

         if (in_valid && in_ready) begin
            core_req_q.operands <= operands;
            core_req_q.rnd_mode <= rnd_mode;
            core_req_q.op <= op;
            core_req_q.op_mod <= op_mod;
            core_req_q.src_fmt <= src_fmt;
            core_req_q.dst_fmt <= dst_fmt;
            core_req_q.int_fmt <= int_fmt;
            core_req_q.tag <= tag_in;
            core_req_pending_q <= 1'b1;
            core_req_toggle_q <= !core_req_toggle_q;
         end
      end
   end

   always_ff @(posedge fpu_clock or posedge reset) begin
      if (reset)
         fpu_reset_sync_q <= 2'b11;
      else
         fpu_reset_sync_q <= {fpu_reset_sync_q[0], 1'b0};
   end

   assign fpu_reset = fpu_reset_sync_q[1];

   always_ff @(posedge fpu_clock) begin
      if (fpu_reset) begin
         fpu_req_q <= '0;
         fpu_in_valid_q <= 1'b0;
         fpu_resp_toggle_q <= 1'b0;
         fpu_resp_result_q <= '0;
         fpu_resp_fflags_q <= '0;
         fpu_resp_tag_q <= '0;
         fpu_req_sync_q <= 2'b00;
         fpu_req_seen_q <= 1'b0;
         fpu_state_q <= FPU_IDLE;
      end else begin
         fpu_req_sync_q <= {fpu_req_sync_q[0], core_req_toggle_q};

         case (fpu_state_q)
            FPU_IDLE: begin
               fpu_in_valid_q <= 1'b0;
               if (fpu_req_sync_q[1] != fpu_req_seen_q) begin
                  fpu_req_seen_q <= fpu_req_sync_q[1];
                  fpu_req_q <= core_req_q;
                  fpu_in_valid_q <= 1'b1;
                  fpu_state_q <= FPU_ISSUE;
               end
            end

            FPU_ISSUE: begin
               if (fpu_in_valid_q && fpu_in_ready) begin
                  fpu_in_valid_q <= 1'b0;
                  if (fpu_out_valid) begin
                     fpu_resp_result_q <= fpu_result;
                     fpu_resp_fflags_q <= {fpu_status.NV, fpu_status.DZ, fpu_status.OF, fpu_status.UF, fpu_status.NX};
                     fpu_resp_tag_q <= fpu_tag;
                     fpu_resp_toggle_q <= !fpu_resp_toggle_q;
                     fpu_state_q <= FPU_IDLE;
                  end else begin
                     fpu_state_q <= FPU_WAIT;
                  end
               end
            end

            FPU_WAIT: begin
               if (fpu_out_valid) begin
                  fpu_resp_result_q <= fpu_result;
                  fpu_resp_fflags_q <= {fpu_status.NV, fpu_status.DZ, fpu_status.OF, fpu_status.UF, fpu_status.NX};
                  fpu_resp_tag_q <= fpu_tag;
                  fpu_resp_toggle_q <= !fpu_resp_toggle_q;
                  fpu_state_q <= FPU_IDLE;
               end
            end

            default: begin
               fpu_in_valid_q <= 1'b0;
               fpu_state_q <= FPU_IDLE;
            end
         endcase
      end
   end

   fpnew_top #(
      .Features       ( Features       ),
      .Implementation ( Implementation ),
      .DivSqrtSel     ( fpnew_pkg::THMULTI ),
      .TagType        ( logic [TAG_WIDTH-1:0] )
   ) cvfpu_inst (
      .clk_i         ( fpu_clock ),
      .rst_ni        ( !fpu_reset ),
      .operands_i    ( fpu_req_q.operands ),
      .rnd_mode_i    ( fpnew_pkg::roundmode_e'(fpu_req_q.rnd_mode) ),
      .op_i          ( fpnew_pkg::operation_e'(fpu_req_q.op) ),
      .op_mod_i      ( fpu_req_q.op_mod ),
      .src_fmt_i     ( fpnew_pkg::fp_format_e'(fpu_req_q.src_fmt) ),
      .dst_fmt_i     ( fpnew_pkg::fp_format_e'(fpu_req_q.dst_fmt) ),
      .int_fmt_i     ( fpnew_pkg::int_format_e'(fpu_req_q.int_fmt) ),
      .vectorial_op_i( 1'b0 ),
      .tag_i         ( fpu_req_q.tag ),
      .simd_mask_i   ( '1 ),
      .in_valid_i    ( fpu_in_valid_q ),
      .in_ready_o    ( fpu_in_ready ),
      .flush_i       ( 1'b0 ),
      .result_o      ( fpu_result ),
      .status_o      ( fpu_status ),
      .tag_o         ( fpu_tag ),
      .out_valid_o   ( fpu_out_valid ),
      .out_ready_i   ( 1'b1 ),
      .busy_o        ( fpu_busy ),
      .early_valid_o ( fpu_early_valid )
   );

endmodule
