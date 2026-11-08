`timescale 1ns / 1ps
`default_nettype none
// tb_virtio_snd -- virtio_snd as Linux's virtio_snd driver drives it.
//
// The bench is the driver: it probes the device, reads the config space (no jacks, one stream, no
// channel maps), sets up the four queues, and asks PCM_INFO (formats U8|S16, rates 11025|22050|44100,
// output, one or two channels), JACK_INFO (not supported) and an unsupported rate. Then, for each of
// the twelve configurations: SET_PARAMS, PREPARE, three txq buffers (the {stream_id} header, the PCM
// bytes split across two descriptors at odd addresses, the writable status), START -- and every
// 44.1 kHz frame that leaves is compared, in order, with the bench's own conversion of the bytes
// (U8 to signed 16, mono to both channels, each frame repeated 1, 2 or 4 times). The FIFO is small
// (64 frames), so the device waits on it mid-buffer. STOP; then a buffer posted while stopped comes
// back unplayed at RELEASE, its status OK.
module tb_virtio_snd;
   reg clk = 1'b0, reset = 1'b1;
   always #1.5 clk = ~clk;
   localparam integer TPF = 256;                // cycles a 44.1 kHz frame (CLK_NUM / CLK_DEN = 44100 * 256): time for the next buffer's walk

   reg  [11:0] address = 12'd0;  reg rd = 1'b0, wr = 1'b0;  reg [31:0] wdata = 32'd0;  reg [3:0] be = 4'hf;
   wire [31:0] rdata;  wire irq;  wire fv;  wire [31:0] fr;
   wire [2:0]  awid, awsize, awprot, arid, arsize, arprot;  wire [31:0] awaddr, araddr;
   wire [7:0]  awlen, arlen, wstrb;  wire [1:0] awburst, arburst;  wire [3:0] awcache, awqos, arcache, arqos;
   wire        awlock, awvalid, wlast, wvalid, bready, arlock, arvalid, rready;  wire [63:0] wdata_ax;
   reg         awready = 1'b0, wready = 1'b0, bvalid = 1'b0, arready = 1'b0, rvalid = 1'b0;
   reg  [63:0] rdata_ax = 64'd0;  reg rlast_ax = 1'b0;

   virtio_snd #(.CLK_NUM(44100 * TPF), .CLK_DEN(1), .FIFO_LG(6)) dut (
      .clock(clk), .reset(reset),
      .address(address), .read(rd), .read_data(rdata), .write(wr), .write_data(wdata), .byteenable(be),
      .irq(irq), .frame_valid(fv), .frame(fr),
      .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
      .m_axi_awburst(awburst), .m_axi_awlock(awlock), .m_axi_awcache(awcache), .m_axi_awprot(awprot),
      .m_axi_awqos(awqos), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
      .m_axi_wdata(wdata_ax), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
      .m_axi_bid(3'd0), .m_axi_bresp(2'b00), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
      .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
      .m_axi_arburst(arburst), .m_axi_arlock(arlock), .m_axi_arcache(arcache), .m_axi_arprot(arprot),
      .m_axi_arqos(arqos), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
      .m_axi_rid(3'd0), .m_axi_rdata(rdata_ax), .m_axi_rresp(2'b00), .m_axi_rlast(rlast_ax),
      .m_axi_rvalid(rvalid), .m_axi_rready(rready));

   // ---- DRAM: 64 KiB at physical 0x1_0010_0000 (AXI 0x8010_0000), random stalls, bursts ----
   localparam [31:0] PA_HI = 32'd1;
   localparam [31:0] PBASE = 32'h0010_0000;
   reg [63:0] mem [0:8191];
   reg [31:0] seed = 32'd11;
   function rnd(input integer pct);
      begin seed = seed * 1103515245 + 12345;  rnd = (seed[30:16] % 100) < pct; end
   endfunction
   function [12:0] widx(input [31:0] a);
      begin
         if (a[31:16] != 16'h8010) $fatal(1, "DMA outside the bench's DRAM: AXI %h", a);
         widx = a[15:3];
      end
   endfunction
   reg [31:0] ar_a;  reg [7:0] ar_n;  reg ar_busy = 1'b0;
   reg [31:0] aw_q;  reg [7:0] aw_n;  reg aw_have = 1'b0;
   always @(posedge clk) begin
      // reads: one burst at a time, a beat a cycle when the dice allow
      arready <= !arready && arvalid && !ar_busy && rnd(50);
      if (arvalid && arready) begin ar_a <= araddr;  ar_n <= arlen;  ar_busy <= 1'b1; end
      if (rvalid && rready) begin
         rvalid <= 1'b0;
         if (rlast_ax) ar_busy <= 1'b0;
      end else if (ar_busy && !rvalid && rnd(70)) begin
         rdata_ax <= mem[widx(ar_a)];  rvalid <= 1'b1;  rlast_ax <= ar_n == 8'd0;
         ar_a <= ar_a + 32'd8;  ar_n <= ar_n - 8'd1;
      end
      // writes: the address, then its beats
      awready <= !awready && awvalid && !aw_have && rnd(50);
      if (awvalid && awready) begin aw_q <= awaddr;  aw_n <= awlen;  aw_have <= 1'b1; end
      wready <= aw_have && !bvalid && !wready && wvalid && rnd(60);
      if (wvalid && wready) begin : wr_mem
         integer i;
         for (i = 0; i < 8; i = i + 1) if (wstrb[i]) mem[widx(aw_q)][8*i +: 8] = wdata_ax[8*i +: 8];
         aw_q <= aw_q + 32'd8;
         if (wlast) begin aw_have <= 1'b0;  bvalid <= 1'b1; end
      end
      if (bvalid && bready) bvalid <= 1'b0;
   end
   task wr8(input [31:0] pa, input [7:0] v);  mem[pa[15:3]][{pa[2:0], 3'd0} +: 8] = v; endtask
   task wr16(input [31:0] pa, input [15:0] v);  mem[pa[15:3]][{pa[2:0], 3'd0} +: 16] = v; endtask
   task wr32(input [31:0] pa, input [31:0] v);  mem[pa[15:3]][{pa[2:0], 3'd0} +: 32] = v; endtask
   task wr64(input [31:0] pa, input [63:0] v);  mem[pa[15:3]] = v; endtask
   function [7:0]  rd8(input [31:0] pa);  rd8 = mem[pa[15:3]][{pa[2:0], 3'd0} +: 8]; endfunction
   function [15:0] rd16(input [31:0] pa);  rd16 = mem[pa[15:3]][{pa[2:0], 3'd0} +: 16]; endfunction
   function [31:0] rd32(input [31:0] pa);  rd32 = mem[pa[15:3]][{pa[2:0], 3'd0} +: 32]; endfunction

   // ---- MMIO ----
   integer errors = 0;
   task mw(input [11:0] a, input [31:0] d);
      begin @(negedge clk); address = a; wdata = d; be = 4'hf; wr = 1'b1; @(negedge clk); wr = 1'b0; end
   endtask
   task mr(input [11:0] a, output [31:0] d);
      begin @(negedge clk); address = a; rd = 1'b1; #0.1 d = rdata; @(negedge clk); rd = 1'b0; end
   endtask
   task check(input cond, input [8*48-1:0] what);
      begin if (!cond) begin $display("FAIL: %0s", what);  errors = errors + 1; end end
   endtask
   task expect32(input [11:0] a, input [31:0] want, input [8*32-1:0] what);
      reg [31:0] got;
      begin
         mr(a, got);
         if (got !== want) begin $display("FAIL: %0s: %h, expected %h", what, got, want); errors = errors + 1; end
      end
   endtask

   // ---- the queues: 16 entries each; queue q's table, avail and used rings in its own 1 KiB ----
   localparam integer QN = 16;
   function [31:0] q_desc(input integer q);  q_desc = PBASE + q * 32'h400;          endfunction
   function [31:0] q_avail(input integer q); q_avail = PBASE + q * 32'h400 + 32'h100; endfunction
   function [31:0] q_used(input integer q);  q_used = PBASE + q * 32'h400 + 32'h200; endfunction
   reg [15:0] aidx [0:3];          // the driver's avail index
   reg [15:0] uidx [0:3];          // the used entries the driver has taken
   integer    chains [0:3];        // chains posted: a chain uses descriptors (n * 4 .. n * 4 + 3) mod 16
   // the chain being built
   reg [31:0] c_addr [0:3];  reg [31:0] c_len [0:3];  reg c_wr [0:3];  integer c_n;
   task post(input integer q);
      integer i;  reg [15:0] d0, di;
      begin
         d0 = (chains[q] * 4) % QN;
         for (i = 0; i < c_n; i = i + 1) begin
            di = d0 + i;
            wr64(q_desc(q) + di * 16, {PA_HI, c_addr[i]});
            wr32(q_desc(q) + di * 16 + 8, c_len[i]);
            wr16(q_desc(q) + di * 16 + 12, (i + 1 < c_n ? 16'd1 : 16'd0) | (c_wr[i] ? 16'd2 : 16'd0));
            wr16(q_desc(q) + di * 16 + 14, di + 16'd1);
         end
         wr16(q_avail(q) + 4 + (aidx[q] % QN) * 2, d0);
         aidx[q] = aidx[q] + 16'd1;
         wr16(q_avail(q) + 2, aidx[q]);
         chains[q] = chains[q] + 1;
         mw(12'h050, q);
      end
   endtask
   // wait for the device to return the next chain on queue q; its used length
   task reap(input integer q, output [31:0] len);
      integer t;
      begin
         t = 0;
         while (rd16(q_used(q) + 2) == uidx[q]) begin
            @(negedge clk);  t = t + 1;
            if (t > 2_000_000) $fatal(1, "queue %0d: no buffer returned", q);
         end
         len = rd32(q_used(q) + 4 + (uidx[q] % QN) * 8 + 4);
         uidx[q] = uidx[q] + 16'd1;
      end
   endtask

   // a control request: its bytes at REQ, a 4-byte response at RSP (and the info array at INF)
   localparam [31:0] REQ = PBASE + 32'h1000, RSP = PBASE + 32'h1100, INF = PBASE + 32'h1201;
   task ctl(input [31:0] code, input [31:0] a1, input [31:0] a2, input [31:0] a3, input [31:0] a4, input [31:0] a5,
            input integer nreq, input integer ninfo, input [31:0] want, input [8*32-1:0] what);
      reg [31:0] len;
      begin
         wr32(REQ, code);  wr32(REQ + 4, a1);  wr32(REQ + 8, a2);  wr32(REQ + 12, a3);  wr32(REQ + 16, a4);  wr32(REQ + 20, a5);
         wr32(RSP, 32'hdeadbeef);
         c_addr[0] = REQ;  c_len[0] = nreq;  c_wr[0] = 1'b0;
         c_addr[1] = RSP;  c_len[1] = 4;     c_wr[1] = 1'b1;
         c_addr[2] = INF;  c_len[2] = ninfo; c_wr[2] = 1'b1;
         c_n = ninfo != 0 ? 3 : 2;
         post(0);
         reap(0, len);
         if (rd32(RSP) != want || len != 4 + (want == 32'h8000 ? ninfo : 0)) begin
            $display("FAIL: %0s: status %h len %0d, expected %h", what, rd32(RSP), len, want);  errors = errors + 1;
         end
      end
   endtask

   // ---- the frames that leave, against the bench's model ----
   reg [31:0] want_f [0:4095];  integer n_want = 0, n_got = 0;  reg capture = 1'b0, seen = 1'b0;
   always @(posedge clk) if (!reset && fv && capture) begin
      if (!seen && fr == 32'd0) ;                         // silence before the first frame plays
      else if (n_got < n_want) begin
         seen <= 1'b1;
         if (fr !== want_f[n_got]) begin
            $display("FAIL: frame %0d = %h, expected %h", n_got, fr, want_f[n_got]);  errors = errors + 1;
         end
         n_got <= n_got + 1;
      end
   end

   localparam [7:0] FMT_U8 = 8'd4, FMT_S16 = 8'd5;
   integer cfg_i, b, k, i, f, nfr;
   reg [7:0]  fmt, rate, chn;  integer bpf, rep;
   reg [31:0] len;
   reg [7:0]  pcm [0:1023];
   reg [15:0] l, r;
   localparam [31:0] TXH = PBASE + 32'h1400, PCM0 = PBASE + 32'h2003, STS = PBASE + 32'h1500;
   initial begin
      for (i = 0; i < 8192; i = i + 1) mem[i] = 64'd0;
      for (i = 0; i < 4; i = i + 1) begin aidx[i] = 16'd0;  uidx[i] = 16'd0;  chains[i] = 0; end
      repeat (5) @(negedge clk);  reset = 1'b0;

      // ---- probe ----
      expect32(12'h000, 32'h7472_6976, "magic");
      expect32(12'h004, 32'd2, "version");
      expect32(12'h008, 32'd25, "device id");
      expect32(12'h100, 32'd0, "config jacks");
      expect32(12'h104, 32'd1, "config streams");
      expect32(12'h108, 32'd0, "config chmaps");
      mw(12'h070, 32'd1);  mw(12'h070, 32'd3);
      mw(12'h024, 32'd1);  mw(12'h020, 32'd1);           // VERSION_1
      mw(12'h070, 32'd11);
      for (i = 0; i < 4; i = i + 1) begin
         mw(12'h030, i);
         expect32(12'h034, 32'd64, "queue num max");
         mw(12'h038, QN);
         mw(12'h080, q_desc(i));   mw(12'h084, PA_HI);
         mw(12'h090, q_avail(i));  mw(12'h094, PA_HI);
         mw(12'h0a0, q_used(i));   mw(12'h0a4, PA_HI);
         mw(12'h044, 32'd1);
      end
      mw(12'h030, 32'd4);
      expect32(12'h034, 32'd0, "queue 4 does not exist");
      mw(12'h070, 32'd15);                                // DRIVER_OK

      // ---- control: PCM_INFO, JACK_INFO, a rate the stream does not play ----
      for (i = 0; i < 32; i = i + 1) wr8(INF + i, 8'hee);
      ctl(32'h100, 0, 1, 32, 0, 0, 16, 32, 32'h8000, "PCM_INFO");
      check(rd32(INF) == 0 && rd32(INF + 4) == 0, "PCM_INFO hda_fn_nid, features");
      check(rd32(INF + 8) == 32'h30 && rd32(INF + 12) == 0, "PCM_INFO formats U8|S16");
      check(rd32(INF + 16) == 32'h54 && rd32(INF + 20) == 0, "PCM_INFO rates 11025|22050|44100");
      check(rd8(INF + 24) == 0 && rd8(INF + 25) == 1 && rd8(INF + 26) == 2, "PCM_INFO output, 1..2 channels");
      ctl(32'h1, 0, 1, 32, 0, 0, 16, 0, 32'h8002, "JACK_INFO");
      ctl(32'h101, 0, 4096, 1024, 0, {8'd0, 8'd7, FMT_S16, 8'd2}, 24, 0, 32'h8002, "SET_PARAMS 48000");
      expect32(12'h060, 32'd1, "used-buffer interrupt");
      mw(12'h064, 32'd1);
      expect32(12'h060, 32'd0, "interrupt acknowledged");

      // ---- the twelve configurations ----
      capture = 1'b1;
      for (cfg_i = 0; cfg_i < 12; cfg_i = cfg_i + 1) begin
         fmt  = cfg_i[0] ? FMT_S16 : FMT_U8;
         chn  = cfg_i[1] ? 8'd2 : 8'd1;
         rate = cfg_i / 4 == 0 ? 8'd6 : cfg_i / 4 == 1 ? 8'd4 : 8'd2;
         rep  = cfg_i / 4 == 0 ? 1 : cfg_i / 4 == 1 ? 2 : 4;
         bpf  = (fmt == FMT_S16 ? 2 : 1) * chn;
         ctl(32'h101, 0, 4096, 1024, 0, {8'd0, rate, fmt, chn}, 24, 0, 32'h8000, "SET_PARAMS");
         ctl(32'h102, 0, 0, 0, 0, 0, 8, 0, 32'h8000, "PREPARE");
         // three buffers of 23 frames, each frame's bytes odd (never a zero sample)
         nfr = 23;
         n_want = 0;  n_got = 0;  seen = 1'b0;
         for (b = 0; b < 3; b = b + 1) begin
            for (i = 0; i < nfr * bpf; i = i + 1) begin
               pcm[i] = ((b * 97 + i * 37 + cfg_i * 13) & 8'hfe) | 8'h01;
               wr8(PCM0 + b * 32'h100 + i, pcm[i]);
            end
            for (f = 0; f < nfr; f = f + 1) begin
               if (fmt == FMT_S16) begin
                  l = {pcm[f*bpf+1], pcm[f*bpf]};
                  r = chn == 2 ? {pcm[f*bpf+3], pcm[f*bpf+2]} : l;
               end else begin
                  l = {pcm[f*bpf] ^ 8'h80, 8'h00};
                  r = chn == 2 ? {pcm[f*bpf+1] ^ 8'h80, 8'h00} : l;
               end
               for (k = 0; k < rep; k = k + 1) begin want_f[n_want] = {r, l};  n_want = n_want + 1; end
            end
            wr32(TXH + b * 16, 32'd0);  wr64(STS + b * 16, 64'hffff_ffff_ffff_ffff);
            k = nfr * bpf / 3;                            // the bytes split across two descriptors
            c_addr[0] = TXH + b * 16;          c_len[0] = 4;              c_wr[0] = 1'b0;
            c_addr[1] = PCM0 + b * 32'h100;     c_len[1] = k;              c_wr[1] = 1'b0;
            c_addr[2] = PCM0 + b * 32'h100 + k; c_len[2] = nfr * bpf - k;  c_wr[2] = 1'b0;
            c_addr[3] = STS + b * 16;           c_len[3] = 8;              c_wr[3] = 1'b1;
            c_n = 4;
            post(2);
         end
         ctl(32'h104, 0, 0, 0, 0, 0, 8, 0, 32'h8000, "START");
         for (b = 0; b < 3; b = b + 1) begin
            reap(2, len);
            check(len == 8, "tx used length 8");
            check(rd32(STS + b * 16) == 32'h8000 && rd32(STS + b * 16 + 4) == 0, "tx status OK, latency 0");
         end
         i = 0;
         while (n_got < n_want && i < n_want * TPF * 2 + 4000) begin @(negedge clk);  i = i + 1; end
         if (n_got != n_want) begin
            $display("FAIL: configuration %0d (fmt %0d, %0d ch, rate %0d): %0d of %0d frames played",
                     cfg_i, fmt, chn, rate, n_got, n_want);
            errors = errors + 1;
         end
         ctl(32'h105, 0, 0, 0, 0, 0, 8, 0, 32'h8000, "STOP");
         ctl(32'h103, 0, 0, 0, 0, 0, 8, 0, 32'h8000, "RELEASE");
      end
      capture = 1'b0;

      // ---- a buffer posted while stopped waits, and comes back unplayed at RELEASE ----
      ctl(32'h101, 0, 4096, 1024, 0, {8'd0, 8'd6, FMT_S16, 8'd2}, 24, 0, 32'h8000, "SET_PARAMS");
      ctl(32'h102, 0, 0, 0, 0, 0, 8, 0, 32'h8000, "PREPARE");
      wr32(TXH, 32'd0);  wr64(STS, 64'hffff_ffff_ffff_ffff);
      c_addr[0] = TXH;  c_len[0] = 4;  c_wr[0] = 1'b0;
      c_addr[1] = PCM0; c_len[1] = 400;  c_wr[1] = 1'b0;
      c_addr[2] = STS;  c_len[2] = 8;  c_wr[2] = 1'b1;
      c_n = 3;
      post(2);
      repeat (2000) @(negedge clk);
      check(rd16(q_used(2) + 2) == uidx[2], "a stopped stream keeps its buffer");
      ctl(32'h103, 0, 0, 0, 0, 0, 8, 0, 32'h8000, "RELEASE");
      reap(2, len);
      check(len == 8 && rd32(STS) == 32'h8000, "RELEASE returns the waiting buffer");

      if (errors == 0) $display("tb_virtio_snd: ALL TESTS PASSED");
      else             $display("tb_virtio_snd: %0d FAILURES", errors);
      $finish;
   end
   initial begin #50_000_000;  $display("FAIL: tb_virtio_snd timed out");  $finish; end
endmodule

`default_nettype wire
