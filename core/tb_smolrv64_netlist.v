`timescale 1ns/1ps
// The ROM monitor booted on rv_soc_top -- the post-synthesis functional netlist under xsim
// (tools/netlist-boot.sh, rule F5) or the RTL (-DRTL_RUN) -- with a full port trace: every retire, store, DDR request and UART byte, one
// line per event, so a netlist run and an RTL run of the same tree diff to their first divergence.
//   +maxcyc=N (default 400000)  +maxchars=N (default 60)
module tb;
   reg clk = 0; always #3 clk = ~clk;
   reg reset = 1;
   wire retire, retire2, dmem_wen; wire [63:0] dmem_waddr, dmem_wdata; wire [7:0] dmem_wmask;
   wire ddr_q_valid, ddr_q_we, ddr_r_ready, ddr_w_ready; wire [4:0] ddr_q_id; wire [57:0] ddr_q_addr;
   wire [63:0] ddr_q_wmask; wire [511:0] ddr_q_wdata;
   reg d_busy = 0, d_we = 0, ddr_r_valid = 0, ddr_w_valid = 0; reg [4:0] d_id = 0; reg [1:0] d_beat = 0;
   wire uart_rx_ready, uart_tx_valid; wire [7:0] uart_tx_data;
   wire fbdiag_reset_req; wire [14:0] virtio_addr; wire virtio_read, virtio_write; wire [31:0] virtio_wdata; wire [3:0] virtio_be;
   wire [17:0] irq_dbg; wire [1:0] cache_par_err; wire [63:0] cache_par_dbg;
   rv_soc_top `ifdef RTL_RUN #(.RESET_PC(64'h70000000)) `endif dut
     (.clk(clk), .reset(reset), .retire(retire), .retire2(retire2),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_q_valid(ddr_q_valid), .ddr_q_ready(~d_busy), .ddr_q_id(ddr_q_id), .ddr_q_we(ddr_q_we), .ddr_q_addr(ddr_q_addr),
      .ddr_q_wmask(ddr_q_wmask), .ddr_q_wdata(ddr_q_wdata),
      .ddr_r_valid(ddr_r_valid), .ddr_r_ready(ddr_r_ready), .ddr_r_id(d_id), .ddr_r_beat(d_beat), .ddr_r_last(d_beat == 2'd3),
      .ddr_r_data(128'd0), .ddr_w_valid(ddr_w_valid), .ddr_w_ready(ddr_w_ready), .ddr_w_id(d_id),
      .uart_rx_we(1'b0), .uart_rx_data(8'd0), .uart_rx_ready(uart_rx_ready),
      .uart_tx_valid(uart_tx_valid), .uart_tx_data(uart_tx_data), .uart_tx_ready(1'b1),
      .fbdiag_reset_req(fbdiag_reset_req),
      .virtio_addr(virtio_addr), .virtio_read(virtio_read), .virtio_write(virtio_write), .virtio_wdata(virtio_wdata), .virtio_be(virtio_be),
      .virtio_rdata(32'd0), .virtio_rvalid(1'b0), .virtio_irq(1'b0), .virtio_net_irq(1'b0), .virtio_kbd_irq(1'b0),
      .irq_dbg(irq_dbg), .cache_par_err(cache_par_err), .cache_par_dbg(cache_par_dbg));
   // DDR answers zeros, one transaction at a time: a read's four beats, or a write's done
   always @(posedge clk) begin
      ddr_w_valid <= 1'b0;
      if (ddr_r_valid) begin d_beat <= d_beat + 2'd1; if (d_beat == 2'd3) begin ddr_r_valid <= 1'b0; d_busy <= 1'b0; end end
      else if (ddr_w_valid) d_busy <= 1'b0;
      else if (d_busy) begin if (d_we) ddr_w_valid <= 1'b1; else begin ddr_r_valid <= 1'b1; d_beat <= 2'd0; end end
      if (ddr_q_valid & ~d_busy) begin d_busy <= 1'b1; d_we <= ddr_q_we; d_id <= ddr_q_id; end
   end
   integer cyc = 0, nchar = 0, nret = 0, ndmem = 0, nddr = 0, maxcyc = 400000, maxchars = 60;
   initial begin
      if (!$value$plusargs("maxcyc=%d", maxcyc)) maxcyc = 400000;
      if (!$value$plusargs("maxchars=%d", maxchars)) maxchars = 60;
   end
   always @(posedge clk) begin
      cyc <= cyc + 1;
      if (cyc == 20) reset <= 0;
      if (retire | retire2) begin nret <= nret + retire + retire2; $display("[%0d] R%0d", cyc, retire + retire2); end
      if (dmem_wen) begin ndmem <= ndmem + 1; $display("[%0d] W %h %h %h", cyc, dmem_waddr, dmem_wdata, dmem_wmask); end
      if (ddr_q_valid & ~d_busy) begin nddr <= nddr + 1; $display("[%0d] DDR %s %h", cyc, ddr_q_we ? "wr" : "rd", ddr_q_addr); end
      if (uart_tx_valid) begin nchar <= nchar + 1; $display("[%0d] UART %02h '%c'", cyc, uart_tx_data, uart_tx_data); end
      if (fbdiag_reset_req) $display("[%0d] FBDIAG reset request", cyc);
      if (nchar >= maxchars || cyc >= maxcyc) begin
         $display("SUMMARY: cycles=%0d retires=%0d chars=%0d dmem_writes=%0d ddr=%0d", cyc, nret, nchar, ndmem, nddr); $finish;
      end
   end

`ifdef NETLIST_PROBE
   // Per-cycle probe of the rename, PRF, ALU and ROB ports (the same names the netlist keeps
   // when synthesized with -flatten_hierarchy none), cycles 30..130.
   always @(posedge clk) if (cyc >= 30 && cyc <= 130)
      $display("[%0d] P rn=%0d,%0d,%0d,%0d,%0h,%0h,%0h rnb=%0d,%0d,%0h prf=%0h,%0h,%0h,%0h,%0d,%0h,%0h,%0d,%0h,%0h alu=%0h,%0h,%0h,%0h,%0h rob=%0d,%0d,%0d,%0d", cyc,
         dut.core.u_rename.r_valid, dut.core.u_rename.r_rd, dut.core.u_rename.r_rs1, dut.core.u_rename.r_rs2,
         dut.core.u_rename.r_prd, dut.core.u_rename.r_sprs1, dut.core.u_rename.r_sprs2,
         dut.core.u_rename.r_valid_b, dut.core.u_rename.r_rd_b, dut.core.u_rename.r_prd_b,
         dut.core.u_prf.ra1, dut.core.u_prf.ra2, dut.core.u_prf.rd1, dut.core.u_prf.rd2,
         dut.core.u_prf.we_ie, dut.core.u_prf.wa_ie, dut.core.u_prf.wd_ie, dut.core.u_prf.we_ld, dut.core.u_prf.wa_ld, dut.core.u_prf.wd_ld,
         dut.core.u_x.u_alu.rs1_val, dut.core.u_x.u_alu.rs2_val, dut.core.u_x.u_alu.imm, dut.core.u_x.u_alu.pc, dut.core.u_x.u_alu.result,
         dut.core.u_rob.d_valid, dut.core.u_rob.d_idx, dut.core.u_rob.c_valid, dut.core.u_rob.d_valid2);
`endif
endmodule
