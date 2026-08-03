// axi_aww_reg_slice unit test: random producers + random-backpressure consumer on both
// channels; scoreboards check lossless in-order delivery of N beats per channel.
`timescale 1ns/1ps
module tb_aww_slice;
   localparam N = 5000;
   reg clock=0, reset=1;  always #5 clock=~clock;
   reg  [2:0] s_awid; reg [30:0] s_awaddr; reg [7:0] s_awlen; reg s_awvalid; wire s_awready;
   reg  [63:0] s_wdata; reg [7:0] s_wstrb; reg s_wlast, s_wvalid; wire s_wready;
   wire [2:0] m_awid; wire [30:0] m_awaddr; wire [7:0] m_awlen; wire m_awvalid; reg m_awready;
   wire [63:0] m_wdata; wire [7:0] m_wstrb; wire m_wlast, m_wvalid; reg m_wready;
   axi_aww_reg_slice dut(.clock(clock), .reset(reset),
      .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen), .s_awsize(3'd3),
      .s_awburst(2'd1), .s_awlock(1'b0), .s_awcache(4'd3), .s_awprot(3'd0), .s_awqos(4'd0),
      .s_awvalid(s_awvalid), .s_awready(s_awready),
      .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast), .s_wvalid(s_wvalid), .s_wready(s_wready),
      .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awsize(), .m_awburst(),
      .m_awlock(), .m_awcache(), .m_awprot(), .m_awqos(),
      .m_awvalid(m_awvalid), .m_awready(m_awready),
      .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid), .m_wready(m_wready));
   integer arecv=0, wrecv=0, errors=0;
   reg [31:0] agen=0, wgen=0, aexp=0, wexp=0;
   always @(posedge clock) begin
      if (reset) begin s_awvalid<=0; s_wvalid<=0; agen<=0; wgen<=0; end
      else begin
         if (!s_awvalid || (s_awvalid&&s_awready)) begin
            if (agen<N && ($random%3!=0)) begin
               s_awvalid<=1; s_awaddr<=agen[30:0]; s_awid<=agen[2:0]; s_awlen<=agen[7:0]; agen<=agen+1;
            end else s_awvalid<=0;
         end
         if (!s_wvalid || (s_wvalid&&s_wready)) begin
            if (wgen<N && ($random%3!=0)) begin
               s_wvalid<=1; s_wdata<={32'h5a5a0000|wgen,wgen}; s_wstrb<=wgen[7:0]; s_wlast<=wgen[0]; wgen<=wgen+1;
            end else s_wvalid<=0;
         end
      end
   end
   always @(posedge clock) begin
      if (reset) begin m_awready<=0; m_wready<=0; aexp<=0; wexp<=0; end
      else begin
         m_awready<=($random%2==0); m_wready<=($random%2==0);
         if (m_awvalid && m_awready) begin
            if (m_awaddr!==aexp[30:0] || m_awid!==aexp[2:0] || m_awlen!==aexp[7:0]) begin
               $display("AW MISMATCH recv=%0d got=%h exp=%h", arecv, m_awaddr, aexp); errors<=errors+1; end
            aexp<=aexp+1; arecv<=arecv+1;
         end
         if (m_wvalid && m_wready) begin
            if (m_wdata[31:0]!==wexp || m_wstrb!==wexp[7:0] || m_wlast!==wexp[0]) begin
               $display("W MISMATCH recv=%0d got=%h exp=%h", wrecv, m_wdata, wexp); errors<=errors+1; end
            wexp<=wexp+1; wrecv<=wrecv+1;
         end
      end
   end
   initial begin
      repeat(4) @(posedge clock); reset<=0;
      wait (arecv==N && wrecv==N); @(posedge clock);
      if (errors==0) $display("AWW-SLICE PASS: %0d+%0d beats, in-order, lossless", arecv, wrecv);
      else $display("AWW-SLICE FAIL: %0d errors", errors);
      $finish;
   end
   initial begin #900000 $display("AWW-SLICE TIMEOUT a=%0d w=%0d", arecv, wrecv); $finish; end
endmodule
