// Standalone unit test for axi_r_reg_slice: random producer + random-backpressure
// consumer, scoreboard checks in-order, lossless transfer of N beats.
`timescale 1ns/1ps
module tb_r_slice;
   localparam DW = 64, IDW = 3, N = 5000;
   reg clock = 0, reset = 1;
   always #5 clock = ~clock;

   reg  [IDW-1:0] s_rid;   reg [DW-1:0] s_rdata;  reg [1:0] s_rresp; reg s_rlast;
   reg            s_rvalid;  wire s_rready;
   wire [IDW-1:0] m_rid;   wire [DW-1:0] m_rdata; wire [1:0] m_rresp; wire m_rlast;
   wire           m_rvalid;  reg  m_rready;

   axi_r_reg_slice #(.IDW(IDW), .DW(DW)) dut (
      .clock(clock), .reset(reset),
      .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp), .s_rlast(s_rlast),
      .s_rvalid(s_rvalid), .s_rready(s_rready),
      .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast),
      .m_rvalid(m_rvalid), .m_rready(m_rready));

   integer recv = 0, errors = 0;
   reg [DW-1:0] genv = 0;     // next payload value to present (increments per load)
   reg [DW-1:0] exp_data;     // next expected data = sequence counter
   wire accepted  = s_rvalid && s_rready;
   wire slot_free = !s_rvalid || accepted;   // may present a new beat this cycle

   // Producer: load a new unique payload only when the slot is free (never
   // overwrite an un-accepted beat); genv is the monotonic generation counter.
   always @(posedge clock) begin
      if (reset) begin
         s_rvalid <= 0; genv <= 0;
      end else if (slot_free) begin
         if (genv < N && ($random % 3 != 0)) begin
            s_rvalid <= 1;
            s_rdata  <= genv;
            s_rid    <= genv[IDW-1:0];
            s_rresp  <= genv[1:0];
            s_rlast  <= genv[0];
            genv     <= genv + 1;
         end else begin
            s_rvalid <= 0;
         end
      end
   end

   // Consumer: random ready; check ordering + integrity
   always @(posedge clock) begin
      if (reset) begin m_rready <= 0; exp_data <= 0; end
      else begin
         m_rready <= ($random % 2 == 0);
         if (m_rvalid && m_rready) begin
            if (m_rdata !== exp_data) begin
               $display("MISMATCH recv=%0d got=%0d exp=%0d", recv, m_rdata, exp_data);
               errors <= errors + 1;
            end
            if (m_rid !== exp_data[IDW-1:0] || m_rresp !== exp_data[1:0] || m_rlast !== exp_data[0]) begin
               $display("SIDEBAND mismatch at recv=%0d", recv); errors <= errors + 1;
            end
            exp_data <= exp_data + 1;
            recv     <= recv + 1;
         end
      end
   end

   initial begin
      repeat (4) @(posedge clock);
      reset <= 0;
      wait (recv == N);
      @(posedge clock);
      if (errors == 0) $display("R-SLICE PASS: %0d beats, in-order, lossless", recv);
      else             $display("R-SLICE FAIL: %0d errors", errors);
      $finish;
   end

   initial begin #500000 $display("R-SLICE TIMEOUT gen=%0d recv=%0d", genv, recv); $finish; end
endmodule
