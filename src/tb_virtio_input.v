`timescale 1ns / 1ps
`default_nettype none
// tb_virtio_input -- virtio_input as Linux's virtio_input driver sees it, and its translation of
// terminal bytes against simmerv's own.
//
// The bench is the driver: it probes the device, reads the config space, sets up eventq with
// FEWER buffers than events (so the device runs dry and must wait for the driver's notify) and
// statusq, then feeds every byte and sequence of kbd_vectors.txt -- produced by simmerv's
// sim/src/term_keys.rs, regenerate with `cargo run --release --manifest-path
// tools/kbd-vectors/Cargo.toml > src/kbd_vectors.txt` -- at UART pace, idle between cases as
// simmerv's translate() is called per read. Every event the device writes is compared, in order,
// with what simmerv's term_keys::send would emit for the expected keys. Then: an LED update on
// statusq is returned, the interrupt is raised and acknowledged, and a paste into a stalled
// device loses exactly what the 512-byte FIFO cannot hold, counted.
module tb_virtio_input;
   reg clk = 1'b0, reset = 1'b1;
   always #1.5 clk = ~clk;
   localparam integer TMO = 400;               // ESC timeout, cycles (1 ms on the board)

   reg  [11:0] address = 12'd0;  reg rd = 1'b0, wr = 1'b0;  reg [31:0] wdata = 32'd0;  reg [3:0] be = 4'hf;
   wire [31:0] rdata;  wire irq;
   reg         key_valid = 1'b0;  reg [7:0] key_byte = 8'd0;
   wire [2:0]  awid, awsize, awprot, arid, arsize, arprot;  wire [30:0] awaddr, araddr;
   wire [7:0]  awlen, arlen, wstrb;  wire [1:0] awburst, arburst;  wire [3:0] awcache, awqos, arcache, arqos;
   wire        awlock, awvalid, wlast, wvalid, bready, arlock, arvalid, rready;  wire [63:0] wdata_ax;
   reg         awready = 1'b0, wready = 1'b0, bvalid = 1'b0, arready = 1'b0, rvalid = 1'b0;
   reg  [63:0] rdata_ax = 64'd0;

   virtio_input #(.ESC_TIMEOUT(TMO)) dut (
      .clock(clk), .reset(reset),
      .address(address), .read(rd), .read_data(rdata), .write(wr), .write_data(wdata), .byteenable(be),
      .irq(irq), .key_valid(key_valid), .key_byte(key_byte),
      .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
      .m_axi_awburst(awburst), .m_axi_awlock(awlock), .m_axi_awcache(awcache), .m_axi_awprot(awprot),
      .m_axi_awqos(awqos), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
      .m_axi_wdata(wdata_ax), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
      .m_axi_bid(3'd0), .m_axi_bresp(2'b00), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
      .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
      .m_axi_arburst(arburst), .m_axi_arlock(arlock), .m_axi_arcache(arcache), .m_axi_arprot(arprot),
      .m_axi_arqos(arqos), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
      .m_axi_rid(3'd0), .m_axi_rdata(rdata_ax), .m_axi_rresp(2'b00), .m_axi_rlast(1'b1),
      .m_axi_rvalid(rvalid), .m_axi_rready(rready));

   // ---- DRAM: 64 KiB at physical 0x8010_0000 (AXI 0x0010_0000), random stalls ----
   localparam [31:0] PBASE = 32'h8010_0000;
   reg [63:0] mem [0:8191];
   reg [31:0] seed = 32'd7;
   function rnd(input integer pct);
      begin seed = seed * 1103515245 + 12345;  rnd = (seed[30:16] % 100) < pct; end
   endfunction
   function [12:0] widx(input [30:0] a);
      begin
         if (a[30:16] != 15'h0010) $fatal(1, "DMA outside the bench's DRAM: AXI %h", a);
         widx = a[15:3];
      end
   endfunction
   reg [30:0] aw_q;  reg aw_have = 1'b0, w_have = 1'b0;  reg [63:0] w_q;  reg [7:0] s_q;
   always @(posedge clk) begin
      arready <= !arready && arvalid && !rvalid && rnd(50);
      if (arvalid && arready) begin rdata_ax <= mem[widx(araddr)];  rvalid <= 1'b1; end
      else if (rvalid && rready) rvalid <= 1'b0;
      awready <= !awready && awvalid && !aw_have && rnd(50);
      wready  <= !wready && wvalid && !w_have && rnd(50);
      if (awvalid && awready) begin aw_q <= awaddr;  aw_have <= 1'b1; end
      if (wvalid && wready)   begin w_q <= wdata_ax;  s_q <= wstrb;  w_have <= 1'b1; end
      if (aw_have && w_have && !bvalid) begin : wr_mem
         integer i;
         for (i = 0; i < 8; i = i + 1) if (s_q[i]) mem[widx(aw_q)][8*i +: 8] = w_q[8*i +: 8];
         aw_have <= 1'b0;  w_have <= 1'b0;  bvalid <= 1'b1;
      end else if (bvalid && bready) bvalid <= 1'b0;
   end
   function [63:0] rd64(input [31:0] pa);  rd64 = mem[pa[15:3]]; endfunction
   task wr16(input [31:0] pa, input [15:0] v);  mem[pa[15:3]][{pa[2:0], 3'd0} +: 16] = v; endtask
   task wr32(input [31:0] pa, input [31:0] v);  mem[pa[15:3]][{pa[2:0], 3'd0} +: 32] = v; endtask
   task wr64(input [31:0] pa, input [63:0] v);  mem[pa[15:3]] = v; endtask
   function [15:0] rd16(input [31:0] pa);  rd16 = mem[pa[15:3]][{pa[2:0], 3'd0} +: 16]; endfunction
   function [31:0] rd32(input [31:0] pa);  rd32 = mem[pa[15:3]][{pa[2:0], 3'd0} +: 32]; endfunction

   // ---- MMIO, as the bridge presents it: a read pulse with combinational data ----
   integer errors = 0;
   task mw(input [11:0] a, input [31:0] d, input [3:0] e);
      begin @(negedge clk); address = a; wdata = d; be = e; wr = 1'b1; @(negedge clk); wr = 1'b0; be = 4'hf; end
   endtask
   task mr(input [11:0] a, output [31:0] d);
      begin @(negedge clk); address = a; rd = 1'b1; #0.1 d = rdata; @(negedge clk); rd = 1'b0; end
   endtask
   task expect32(input [11:0] a, input [31:0] want, input [8*24-1:0] what);
      reg [31:0] got;
      begin
         mr(a, got);
         if (got !== want) begin $display("FAIL: %0s: %h, expected %h", what, got, want); errors = errors + 1; end
      end
   endtask
   // virtio_input_config page: select, subsel, then size and the bytes back, one at a time
   task cfg(input [7:0] sel, input [7:0] sub, input [7:0] want_size, input [127:0] want, input [8*16-1:0] what);
      reg [31:0] d;  integer i;
      begin
         mw(12'h100, {24'd0, sel}, 4'b0001);
         mw(12'h101, {24'd0, sub}, 4'b0001);
         mr(12'h102, d);
         if (d[7:0] != want_size) begin $display("FAIL: config %0s size %0d, expected %0d", what, d[7:0], want_size); errors = errors + 1; end
         for (i = 0; i < want_size; i = i + 1) begin
            mr(12'h108 + i[11:0], d);
            if (d[7:0] != want[8*i +: 8]) begin
               $display("FAIL: config %0s byte %0d = %h, expected %h", what, i, d[7:0], want[8*i +: 8]);  errors = errors + 1;
            end
         end
      end
   endtask

   // ---- the queues: eventq 8 entries, only 4 buffers ever posted; statusq 4 entries ----
   localparam [31:0] Q0D = PBASE + 32'h1000, Q0A = PBASE + 32'h1100, Q0U = PBASE + 32'h1200;
   localparam [31:0] Q1D = PBASE + 32'h2000, Q1A = PBASE + 32'h2100, Q1U = PBASE + 32'h2200;
   localparam [31:0] EBUF = PBASE + 32'h3000, LBUF = PBASE + 32'h3800;
   localparam integer QN = 8, NBUF = 4;
   reg [15:0] avail0 = 16'd0, used_seen = 16'd0;
   task post(input [15:0] d);         // make descriptor d available on eventq
      begin
         wr16(Q0A + 32'd4 + 32'd2 * (avail0 % QN), d);
         avail0 = avail0 + 16'd1;
         wr16(Q0A + 32'd2, avail0);
         mw(12'h050, 32'd0, 4'hf);       // QueueNotify(0)
      end
   endtask
   task setup_queue(input [31:0] sel, input [31:0] num, input [31:0] d, input [31:0] a, input [31:0] u);
      begin
         mw(12'h030, sel, 4'hf);
         expect32(12'h034, 32'd64, "QueueNumMax");
         mw(12'h038, num, 4'hf);
         mw(12'h080, d, 4'hf);  mw(12'h084, 32'd0, 4'hf);
         mw(12'h090, a, 4'hf);  mw(12'h094, 32'd0, 4'hf);
         mw(12'h0a0, u, 4'hf);  mw(12'h0a4, 32'd0, 4'hf);
         mw(12'h044, 32'd1, 4'hf);
      end
   endtask

   // ---- expected events, from the vectors; delivered events, from the used ring ----
   reg [63:0] want_ev [0:65535];  integer n_want = 0;
   reg [63:0] got_ev  [0:65535];  integer n_got = 0;
   task key(input [6:0] code, input down);
      begin
         want_ev[n_want] = {31'd0, down, 9'd0, code, 16'd1};  want_ev[n_want + 1] = 64'd0;
         n_want = n_want + 2;
      end
   endtask
   task press(input [6:0] code, input [2:0] mods);   // simmerv term_keys::send
      begin
         if (mods[2]) key(7'd29, 1);  if (mods[0]) key(7'd42, 1);  if (mods[1]) key(7'd56, 1);
         key(code, 1);  key(code, 0);
         if (mods[1]) key(7'd56, 0);  if (mods[0]) key(7'd42, 0);  if (mods[2]) key(7'd29, 0);
      end
   endtask
   // The driver side: take each used element, record its event, re-post its buffer. One process,
   // so its MMIO notifies never interleave with each other.
   reg draining = 1'b1;
   always begin : drain_loop
      @(posedge clk);
      if (!reset && draining && rd16(Q0U + 32'd2) != used_seen) begin : drain
      reg [31:0] id, len;
      id  = rd32(Q0U + 32'd4 + 32'd8 * (used_seen % QN));
      len = rd32(Q0U + 32'd8 + 32'd8 * (used_seen % QN));
      if (len != 32'd8 || id >= NBUF) begin $display("FAIL: used elem id %0d len %0d", id, len); errors = errors + 1; end
      got_ev[n_got] = rd64(EBUF + 32'd8 * id);
      n_got = n_got + 1;
      used_seen = used_seen + 16'd1;
      post(id[15:0]);
      end
   end

   task send_byte(input [7:0] c);
      begin
         @(negedge clk); key_valid = 1'b1; key_byte = c; @(negedge clk); key_valid = 1'b0;
         repeat (20) @(negedge clk);     // UART pace, far shorter than the ESC timeout
      end
   endtask

   integer fd, rc, i, k, n_cases;
   string  line, w2, w3, w4;  byte tag;
   reg [7:0] in_bytes [0:255];  integer n_in;
   reg [31:0] d;
   initial begin
      for (i = 0; i < 8192; i = i + 1) mem[i] = 64'd0;
      repeat (5) @(negedge clk);  reset = 1'b0;

      // ---- probe, as virtio_mmio's probe and virtinput_probe do ----
      expect32(12'h000, 32'h7472_6976, "MagicValue");
      expect32(12'h004, 32'd2, "Version");
      expect32(12'h008, 32'd18, "DeviceID");
      mw(12'h070, 32'd1, 4'hf);  mw(12'h070, 32'd3, 4'hf);           // ACKNOWLEDGE, DRIVER
      mw(12'h014, 32'd1, 4'hf);  expect32(12'h010, 32'h3, "DeviceFeatures[1]");
      mw(12'h024, 32'd1, 4'hf);  mw(12'h020, 32'h3, 4'hf);           // VERSION_1 | ACCESS_PLATFORM
      mw(12'h070, 32'd11, 4'hf);                                     // FEATURES_OK
      cfg(8'h01, 8'h00, 8'd16, "draobyek vremmis", "ID_NAME");
      cfg(8'h03, 8'h00, 8'd8, {64'd0, 64'h0001_0001_0627_0006}, "ID_DEVIDS");
      cfg(8'h11, 8'h01, 8'd16, 128'he080ffdf01cffffffffffffffffffffe, "EV_KEY bits");
      cfg(8'h11, 8'h14, 8'd1, 128'h01, "EV_REP bits");
      cfg(8'h11, 8'h02, 8'd0, 128'h0, "EV_REL bits");
      cfg(8'h12, 8'h00, 8'd0, 128'h0, "ABS_INFO");
      for (i = 0; i < NBUF; i = i + 1) begin                         // event buffers: 8 bytes, WRITE
         wr64(Q0D + 16 * i, {32'd0, EBUF + 32'd8 * i});
         wr64(Q0D + 16 * i + 8, {16'd0, 16'd2, 32'd8});
      end
      setup_queue(0, QN, Q0D, Q0A, Q0U);
      setup_queue(1, 4, Q1D, Q1A, Q1U);
      mw(12'h070, 32'd15, 4'hf);                                     // DRIVER_OK
      for (i = 0; i < NBUF; i = i + 1) post(i[15:0]);

      // ---- every vector ----
      fd = $fopen("kbd_vectors.txt", "r");
      if (fd == 0) $fatal(1, "kbd_vectors.txt not found (run from src/)");
      n_cases = 0;
      while (!$feof(fd)) begin
         rc = $fgets(line, fd);
         if (rc > 0 && line.getc(0) != "#") begin
            n_in = 0;
            rc = $sscanf(line, "%c %s %s %s", tag, w2, w3, w4);
            if (tag == "B") begin
               in_bytes[0] = w2.atohex();  n_in = 1;
               if (w3 != "none") press(w3.atoi(), w4.atoi());
            end else if (tag == "S") begin : sexp
               integer j, c, m, pos;  byte ch;
               for (j = 0; j < w2.len() / 2; j = j + 1) in_bytes[j] = w2.substr(2 * j, 2 * j + 1).atohex();
               n_in = w2.len() / 2;
               if (w3 != "-") begin                // "code:mods,code:mods"
                  c = 0;  m = 0;  pos = 0;
                  for (j = 0; j <= w3.len(); j = j + 1) begin
                     ch = j < w3.len() ? w3.getc(j) : ",";
                     if (ch == ":") pos = 1;
                     else if (ch == ",") begin press(c[6:0], m[2:0]);  c = 0;  m = 0;  pos = 0; end
                     else if (pos == 0) c = c * 10 + (ch - "0");
                     else m = m * 10 + (ch - "0");
                  end
               end
            end
            if (n_in > 0) begin
               for (k = 0; k < n_in; k = k + 1) send_byte(in_bytes[k]);
               repeat (TMO + 200) @(negedge clk);   // one read() of simmerv's: let the ESC timeout settle it
               n_cases = n_cases + 1;
            end
         end
      end
      $fclose(fd);
      wait (n_got >= n_want);
      repeat (2000) @(negedge clk);
      if (n_got != n_want) begin $display("FAIL: %0d events delivered, %0d expected", n_got, n_want); errors = errors + 1; end
      k = 0;
      for (i = 0; i < n_want && i < n_got; i = i + 1)
         if (got_ev[i] !== want_ev[i]) begin
            if (k < 10) $display("FAIL: event %0d = %h, expected %h", i, got_ev[i], want_ev[i]);
            k = k + 1;  errors = errors + 1;
         end
      $display("%0d vector cases, %0d events compared", n_cases, n_want);

      // ---- statusq: an LED update is returned with len 0, and interrupts ----
      mw(12'h064, 32'h3, 4'hf);                                      // clear any pending
      wr64(Q1D, {32'd0, LBUF});  wr64(Q1D + 8, {16'd0, 16'd0, 32'd8});
      wr16(Q1A + 32'd4, 16'd0);  wr16(Q1A + 32'd2, 16'd1);
      mw(12'h050, 32'd1, 4'hf);
      repeat (500) @(negedge clk);
      if (rd16(Q1U + 32'd2) != 16'd1 || rd32(Q1U + 32'd4) != 32'd0 || rd32(Q1U + 32'd8) != 32'd0) begin
         $display("FAIL: statusq used idx %0d id %0d len %0d", rd16(Q1U + 32'd2), rd32(Q1U + 32'd4), rd32(Q1U + 32'd8));
         errors = errors + 1;
      end
      if (!irq) begin $display("FAIL: no interrupt after the statusq return"); errors = errors + 1; end
      expect32(12'h060, 32'h1, "InterruptStatus");
      mw(12'h064, 32'h1, 4'hf);
      repeat (3) @(negedge clk);          // virtio_mmio applies a write one cycle after it lands
      if (irq) begin $display("FAIL: interrupt still up after the ack"); errors = errors + 1; end

      // ---- a paste into a stalled device: the FIFO keeps 512 bytes, the expander one key ----
      draining = 1'b0;
      repeat (500) @(negedge clk);
      expect32(12'hf00, 32'd0, "bytes lost before");
      for (i = 0; i < 600 + NBUF; i = i + 1) begin
         @(negedge clk); key_valid = 1'b1; key_byte = "a";
      end
      @(negedge clk); key_valid = 1'b0;
      repeat (500) @(negedge clk);
      // NBUF events' worth of buffers are posted: the first key's 4 events fill them; then one
      // key waits in the expander and 512 bytes in the FIFO.
      expect32(12'hf00, 32'd600 + NBUF - 1 - 1 - 512, "bytes lost to a full FIFO");

      if (errors == 0) $display("PASS");
      else             $display("FAIL: %0d errors", errors);
      $finish;
   end
   initial begin #200_000_000; $display("FAIL: timeout (%0d of %0d events)", n_got, n_want); $finish; end
endmodule

`default_nettype wire
