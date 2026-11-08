`timescale 1ns / 1ps
`default_nettype none
// virtio_snd -- the sound device: one output stream, played at 44.1 kHz 16-bit stereo.
//
// A virtio-mmio v2 sound device (ID 25, virtio 1.2 section 5.14), docs/PLAN-2026-11-08-virtio-snd-
// hdmi-audio.md. Config space: no jacks, one stream, no channel maps. The stream plays U8 or S16,
// one or two channels, at 11025, 22050 or 44100 Hz -- twelve configurations -- and every one leaves
// as the same 44.1 kHz, 16-bit, stereo frames (frame_valid, frame), one per tick of the playback
// clock: U8 becomes (x - 128) << 8, mono goes to both channels, a 22.05 or 11.025 kHz frame is
// repeated two or four times. With nothing to play the frame is silence; the clock never stops.
//
// Four queues, as the driver sets them up: controlq (0) and txq (2) are served; eventq (1) keeps
// the buffers it is given (the device sends no events) and rxq (3) is never used (no capture).
// The engine serves one buffer at a time, walking its descriptor chain: the readable bytes (a
// control request, or a txq buffer's {stream_id} header then its PCM bytes), then the writable
// ones (the response, or the txq buffer's {status, latency_bytes}). PCM bytes enter the frame
// FIFO as it has room, so a txq buffer is returned once its last byte is in (11.6 ms ahead of the
// speaker at most, the FIFO's depth). A started stream is consumed; a stopped one waits;
// PCM_RELEASE returns every pending txq buffer unplayed (Linux waits for them in sync_stop).
module virtio_snd #(
    // the playback clock: 44100 Hz from a clock of CLK_NUM / CLK_DEN Hz (ui_clk is 1e9 / 3)
    parameter integer CLK_NUM = 1_000_000_000,
    parameter integer CLK_DEN = 3,
    parameter integer FIFO_LG = 9                    // frames the FIFO holds: 512, 11.6 ms
)(
    input  wire        clock,
    input  wire        reset,

    input  wire [11:0] address,
    input  wire        read,
    output wire [31:0] read_data,
    input  wire        write,
    input  wire [31:0] write_data,
    input  wire [ 3:0] byteenable,
    output wire        irq,

    output reg         frame_valid,        // one 44.1 kHz stereo frame: {right, left}, 16-bit signed
    output reg  [31:0] frame,

    output wire [ 2:0] m_axi_awid,
    output wire [31:0] m_axi_awaddr,
    output wire [ 7:0] m_axi_awlen,
    output wire [ 2:0] m_axi_awsize,
    output wire [ 1:0] m_axi_awburst,
    output wire        m_axi_awlock,
    output wire [ 3:0] m_axi_awcache,
    output wire [ 2:0] m_axi_awprot,
    output wire [ 3:0] m_axi_awqos,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [63:0] m_axi_wdata,
    output wire [ 7:0] m_axi_wstrb,
    output wire        m_axi_wlast,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [ 2:0] m_axi_bid,
    input  wire [ 1:0] m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    output wire [ 2:0] m_axi_arid,
    output wire [31:0] m_axi_araddr,
    output wire [ 7:0] m_axi_arlen,
    output wire [ 2:0] m_axi_arsize,
    output wire [ 1:0] m_axi_arburst,
    output wire        m_axi_arlock,
    output wire [ 3:0] m_axi_arcache,
    output wire [ 2:0] m_axi_arprot,
    output wire [ 3:0] m_axi_arqos,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [ 2:0] m_axi_rid,
    input  wire [63:0] m_axi_rdata,
    input  wire [ 1:0] m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready
);
   // virtio_snd's codes (virtio 1.2 5.14.6)
   localparam [31:0] R_JACK_INFO = 32'h1, R_JACK_REMAP = 32'h2, R_PCM_INFO = 32'h100, R_PCM_SET_PARAMS = 32'h101,
                     R_PCM_PREPARE = 32'h102, R_PCM_RELEASE = 32'h103, R_PCM_START = 32'h104, R_PCM_STOP = 32'h105,
                     R_CHMAP_INFO = 32'h200;
   localparam [31:0] S_OK = 32'h8000, S_BAD_MSG = 32'h8001, S_NOT_SUPP = 32'h8002;
   localparam [7:0]  FMT_U8 = 8'd4, FMT_S16 = 8'd5, RATE_11025 = 8'd2, RATE_22050 = 8'd4, RATE_44100 = 8'd6;

   // ============================ registers and config space ============================
   wire        notify;  wire [31:0] notify_q;
   reg         used_irq;
   wire [127:0] qs_num;  wire [3:0] qs_ready;  wire [255:0] qs_desc, qs_driver, qs_device;
   wire [ 7:0] dev_status;
   wire [31:0] mmio_rdata;

   // config: jacks, streams, chmaps (le32 each), read right-aligned at the byte addressed
   wire [127:0] cfg_bytes = {32'd0, 32'd0, 32'd1, 32'd0};
   wire [3:0]   cfg_off   = address[3:0];
   wire [31:0]  cfg_rdata = (address[7:4] == 4'd0 && cfg_off < 4'd12) ? cfg_bytes[{cfg_off, 3'd0} +: 32] : 32'd0;

   virtio_mmio #(.DEVICE_ID(32'd25), .QUEUE_NUM_MAX(32'd64), .QUEUE_COUNT(32'd4)) u_mmio (
      .clock(clock), .reset(reset),
      .address(address), .read(read), .read_data(mmio_rdata),
      .write(write && address[11:8] != 4'hf), .write_data(write_data), .byteenable(byteenable),
      .config_read_data(cfg_rdata),
      .config_write(), .config_offset(), .config_write_data(), .config_byteenable(),
      .irq(irq), .queue_notify_pulse(notify), .queue_notify_value(notify_q),
      .used_buffer_interrupt(used_irq), .config_change_interrupt(1'b0),
      .driver_features_0(), .driver_features_1(),
      .queues_num(qs_num), .queues_ready(qs_ready), .queues_desc(qs_desc),
      .queues_driver(qs_driver), .queues_device(qs_device),
      .device_status(dev_status));
   wire driver_ok = dev_status[2];

   // counters (0xF00, read-only): frames played, frames of silence while running (underruns),
   // txq buffers returned, control requests answered; and the stream's parameters
   reg [31:0] n_frames, n_under, n_tx, n_ctl;
   reg [ 7:0] p_fmt, p_rate, p_ch;
   reg        running;
   wire [31:0] dbg_rdata = address[3:2] == 2'd0 ? n_frames : address[3:2] == 2'd1 ? n_under
                         : address[3:2] == 2'd2 ? n_tx : {n_ctl[7:0], running, p_ch[6:0], p_rate, p_fmt};
   assign read_data = address[11:8] == 4'hf ? dbg_rdata : mmio_rdata;

   // ================================ the frame FIFO ================================
   localparam integer FD = 1 << FIFO_LG;
   reg  [31:0]      fifo [0:FD-1];
   reg  [FIFO_LG:0] f_wp, f_rp;
   wire [FIFO_LG:0] f_n = f_wp - f_rp;
   // room for a frame repeated four times, with the push a cycle behind its scheduling
   wire             f_room4 = f_n <= FD - 8;
   reg              f_push;  reg [31:0] f_wd;
   reg              f_clear;                            // a release or prepare drops what waits
   always @(posedge clock) if (f_push) fifo[f_wp[FIFO_LG-1:0]] <= f_wd;
   always @(posedge clock) if (!reset && f_push && f_n == FD)
      $fatal(1, "virtio_snd: the frame FIFO overflows");

   // ============================ the playback clock and output ============================
   localparam integer STEP = 44100 * CLK_DEN;
   reg  [31:0] pc_acc;
   wire [32:0] pc_n = {1'b0, pc_acc} + STEP;
   wire        tick = pc_n >= CLK_NUM;
   always @(posedge clock) begin
      frame_valid <= 1'b0;
      if (reset) begin
         pc_acc <= 32'd0;  f_rp <= {(FIFO_LG+1){1'b0}};  frame <= 32'd0;  n_frames <= 32'd0;  n_under <= 32'd0;
      end else begin
         pc_acc <= tick ? pc_n - CLK_NUM : pc_n[31:0];
         if (f_clear) f_rp <= f_wp;
         else if (tick) begin
            frame_valid <= 1'b1;
            if (running && f_n != 0) begin
               frame <= fifo[f_rp[FIFO_LG-1:0]];  f_rp <= f_rp + 1'b1;  n_frames <= n_frames + 32'd1;
            end else begin
               frame <= 32'd0;
               if (running) n_under <= n_under + 32'd1;
            end
         end
      end
   end

   // ================================ the PCM converter ================================
   // One byte at a time (pcm_v): a frame's bytes assembled, then the frame pushed `rep` times. A
   // byte is taken only while no frame waits to be pushed and the FIFO has room for one more.
   wire        fmt16 = p_fmt == FMT_S16;
   wire        st2   = p_ch == 8'd2;
   wire [2:0]  nb    = (fmt16 ? 3'd2 : 3'd1) << st2;   // bytes a frame
   wire [2:0]  rep   = p_rate == RATE_44100 ? 3'd1 : p_rate == RATE_22050 ? 3'd2 : 3'd4;
   reg  [23:0] fa;                                       // the frame's bytes so far
   reg  [2:0]  fk;                                       // ...how many
   reg  [2:0]  rp_left;                                  // copies of fr still to push
   reg  [31:0] fr;
   wire        pcm_ready = rp_left == 3'd0 && f_room4;
   wire        pcm_v;                                    // a byte (pcm_b) this cycle; the engine's
   wire [7:0]  pcm_b;
   wire        pcm_drop;                                 // ...discarded (a release)
   function [15:0] smp(input [7:0] lo, input [7:0] hi);  // one channel's sample, 16-bit signed
      smp = fmt16 ? {hi, lo} : {lo ^ 8'h80, 8'h00};
   endfunction
   always @(posedge clock) begin
      f_push <= 1'b0;
      if (reset) f_wp <= {(FIFO_LG+1){1'b0}};
      else if (f_push) f_wp <= f_wp + 1'b1;
      if (reset || f_clear) begin
         fk <= 3'd0;  rp_left <= 3'd0;
      end else begin
         if (rp_left != 3'd0) begin
            f_push <= 1'b1;  f_wd <= fr;  rp_left <= rp_left - 3'd1;
         end
         if (pcm_v && !pcm_drop) begin : take
            reg [31:0] b;  reg [15:0] l, r;
            b = {pcm_b, fa};                              // the frame's bytes with this one, last on top
            if (fk + 3'd1 == nb) begin
               // the bytes in order: b[31-8*(nb-1) ...]: frame byte i is b[8*(4-nb+i) +: 8]
               b = b >> (8 * (3'd4 - nb));
               l = smp(b[7:0], b[15:8]);
               r = st2 ? (fmt16 ? smp(b[23:16], b[31:24]) : smp(b[15:8], 8'd0)) : l;
               fr <= {r, l};  rp_left <= rep;  fk <= 3'd0;
            end else fk <= fk + 3'd1;
            fa <= b[31:8];
         end
      end
   end
   always @(posedge clock) if (!reset && pcm_v && !pcm_drop && !pcm_ready)
      $fatal(1, "virtio_snd: a PCM byte offered while the converter has no room");

   // ================================ the DMA engine ================================
   wire        dma_cmd_ready, dma_rsp_valid, dma_rsp_error;
   wire [63:0] dma_rsp_rdata;
   reg         dma_cmd_valid, dma_cmd_write, dma_cmd_stream;
   reg  [31:0] dma_cmd_addr;
   reg  [63:0] dma_cmd_wdata;
   reg  [ 7:0] dma_cmd_wstrb;
   dma_master u_dma (
      .clock(clock), .reset(reset),
      .cmd_valid(dma_cmd_valid), .cmd_ready(dma_cmd_ready), .cmd_write(dma_cmd_write), .cmd_stream(dma_cmd_stream),
      .cmd_addr(dma_cmd_addr), .cmd_wdata(dma_cmd_wdata), .cmd_wstrb(dma_cmd_wstrb),
      .rsp_valid(dma_rsp_valid), .rsp_rdata(dma_rsp_rdata), .rsp_error(dma_rsp_error),
      .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
      .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst), .m_axi_awlock(m_axi_awlock),
      .m_axi_awcache(m_axi_awcache), .m_axi_awprot(m_axi_awprot), .m_axi_awqos(m_axi_awqos),
      .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
      .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
      .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
      .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid),
      .m_axi_bready(m_axi_bready),
      .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
      .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst), .m_axi_arlock(m_axi_arlock),
      .m_axi_arcache(m_axi_arcache), .m_axi_arprot(m_axi_arprot), .m_axi_arqos(m_axi_arqos),
      .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
      .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
      .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready));

`include "dram_window.vh"
   function q_ok(input ready, input [31:0] num, input [63:0] d, input [63:0] a, input [63:0] u);
      q_ok = ready && num != 32'd0 && dram_ok(d) && dram_ok(a) && dram_ok(u);
   endfunction
   wire q0_ok = driver_ok && q_ok(qs_ready[0], qs_num[31:0], qs_desc[63:0], qs_driver[63:0], qs_device[63:0]);
   wire q2_ok = driver_ok && q_ok(qs_ready[2], qs_num[95:64], qs_desc[191:128], qs_driver[191:128], qs_device[191:128]);
   always @(posedge clock)
      if (!reset && ((q0_ok && (qs_num[31:0] & (qs_num[31:0] - 32'd1)) != 32'd0) || (q2_ok && (qs_num[95:64] & (qs_num[95:64] - 32'd1)) != 32'd0)))
         $fatal(1, "virtio_snd: queue size %0d/%0d is not a power of two", qs_num[31:0], qs_num[95:64]);

   localparam [3:0] E_IDLE = 4'd0, E_AVAIL = 4'd1, E_RING = 4'd2, E_DADDR = 4'd3, E_DINFO = 4'd4, E_READ = 4'd5,
                    E_BYTES = 4'd6, E_WRITE = 4'd7, E_USED_ID = 4'd8, E_USED_LEN = 4'd9, E_USED_IDX = 4'd10;
   reg [3:0]  es;
   reg        eq;                 // the queue being served: 0 controlq, 1 txq
   reg        issued;
   reg [15:0] last0, last2, used0, used2;
   reg        pend0, pend2;       // a notify the engine has not looked at yet
   reg        flushing;           // a release: txq buffers go back unplayed
   reg [15:0] head, dix;          // the chain's head, and the descriptor in hand
   reg [31:0] q_desc, q_driver, q_device;
   reg [15:0] q_mask;
   reg [31:0] d_addr;             // the descriptor's buffer, as an AXI address (advanced as it is read or written)
   reg [31:0] d_left;             // ...its bytes still to read or write
   reg [15:0] d_flags;
   reg [7:0]  nd;                 // descriptors walked in this chain (a loop is a driver bug)
   reg [31:0] rd_n;               // readable bytes taken so far
   reg [63:0] bw;  reg [2:0] bw_i, bw_e;   // a read word's bytes still to take: [bw_i, bw_e]
   reg [7:0]  rq [0:23];          // the request's first bytes (a control request is at most 24)
   reg [7:0]  rs [0:35];          // the response's bytes
   reg [5:0]  rs_len, wr_n;       // ...its length, and the bytes written so far
   reg        rs_made;            // the response has been formed (at the first writable byte)
   wire [15:0] last = eq ? last2 : last0;
   wire [15:0] used = eq ? used2 : used0;
   wire [31:0] ring_a = q_driver + 32'd4 + {14'd0, last & q_mask, 1'b0};
   wire [31:0] used_a = q_device + 32'd4 + {12'd0, used & q_mask, 3'd0};
   wire [31:0] uidx_a = q_device + 32'd2;
   wire [31:0] desc_a = q_desc + {12'd0, dix & q_mask, 4'd0};
   function [15:0] get16(input [63:0] d, input [2:0] off);
      get16 = d[{off, 3'd0} +: 16];
   endfunction
   // the request's words: code, then stream / start_id, count, size (a wire per read: rule F4)
   wire [31:0] rq_w0 = {rq[3], rq[2], rq[1], rq[0]},     rq_w1 = {rq[7], rq[6], rq[5], rq[4]};
   wire [31:0] rq_w2 = {rq[11], rq[10], rq[9], rq[8]},   rq_w3 = {rq[15], rq[14], rq[13], rq[12]};
   task pick(input which);
      begin
         eq       <= which;
         q_desc   <= dram_axi(which ? qs_desc[191:128]   : qs_desc[63:0]);
         q_driver <= dram_axi(which ? qs_driver[191:128] : qs_driver[63:0]);
         q_device <= dram_axi(which ? qs_device[191:128] : qs_device[63:0]);
         q_mask   <= (which ? qs_num[79:64] : qs_num[15:0]) - 16'd1;
         rd_n <= 32'd0;  wr_n <= 6'd0;  rs_made <= 1'b0;  rs_len <= 6'd0;  nd <= 8'd0;
      end
   endtask

   // the bytes of one read word, one a cycle: into the request, or (txq, past the 4-byte header)
   // into the converter as it takes them
   wire bytes_on = es == E_BYTES;
   wire is_pcm   = eq && rd_n >= 32'd4;
   assign pcm_v    = bytes_on && is_pcm && (pcm_ready || flushing);
   assign pcm_drop = flushing;
   assign pcm_b    = bw[{bw_i, 3'd0} +: 8];
   wire byte_go  = bytes_on && (!is_pcm || pcm_ready || flushing);

   // the response, formed at the first writable byte from the request (controlq) or fixed (txq)
   reg  [31:0] rsp_code;  reg rsp_info;
   wire [31:0] rq_code = rq_w0;
   wire [7:0]  sp_ch = rq[20], sp_fmt = rq[21], sp_rate = rq[22];
   always @* begin
      rsp_info = 1'b0;
      case (rq_code)
         R_PCM_INFO:       begin rsp_code = (rq_w1 == 32'd0 && rq_w2 >= 32'd1 && rq_w3 >= 32'd32) ? S_OK : S_BAD_MSG;
                                 rsp_info = rsp_code == S_OK; end
         R_PCM_SET_PARAMS: rsp_code = (rq_w1 == 32'd0 && (sp_ch == 8'd1 || sp_ch == 8'd2)
                                       && (sp_fmt == FMT_U8 || sp_fmt == FMT_S16)
                                       && (sp_rate == RATE_11025 || sp_rate == RATE_22050 || sp_rate == RATE_44100)) ? S_OK : S_NOT_SUPP;
         R_PCM_PREPARE, R_PCM_RELEASE, R_PCM_START, R_PCM_STOP:
                           rsp_code = rq_w1 == 32'd0 ? S_OK : S_BAD_MSG;
         default:          rsp_code = S_NOT_SUPP;      // JACK_*, CHMAP_INFO: there are none
      endcase
   end
   // virtio_snd_pcm_info for stream 0: hda_fn_nid 0, features 0, formats U8|S16, rates 11025|22050|44100,
   // direction output, 1..2 channels
   localparam [255:0] PCM_INFO = {40'd0, 8'd2, 8'd1, 8'd0, 64'h54, 64'h30, 32'd0, 32'd0};
   task make_rsp;
      integer i;
      begin
         rs_made <= 1'b1;
         if (eq) begin                                 // {status OK, latency_bytes 0}
            for (i = 0; i < 4; i = i + 1) begin rs[i] <= S_OK[8*i +: 8];  rs[4+i] <= 8'd0; end
            rs_len <= 6'd8;
         end else begin
            for (i = 0; i < 4; i = i + 1) rs[i] <= rsp_code[8*i +: 8];
            for (i = 0; i < 32; i = i + 1) rs[4+i] <= PCM_INFO[8*i +: 8];
            rs_len <= rsp_info ? 6'd36 : 6'd4;
            // the request's effect
            n_ctl <= n_ctl + 32'd1;
            if (rsp_code == S_OK)
               case (rq_code)
                  R_PCM_SET_PARAMS: begin p_ch <= sp_ch;  p_fmt <= sp_fmt;  p_rate <= sp_rate; end
                  R_PCM_PREPARE:    begin running <= 1'b0;  flushing <= 1'b0; end
                  R_PCM_RELEASE:    begin running <= 1'b0;  flushing <= 1'b1; end
                  R_PCM_START:      begin running <= 1'b1;  flushing <= 1'b0; end
                  R_PCM_STOP:       running <= 1'b0;
                  default: ;
               endcase
         end
      end
   endtask
   // a write of the response's next bytes into the writable descriptor: up to the word's end
   wire [2:0]  wo     = d_addr[2:0];
   wire [3:0]  w_room = 4'd8 - {1'b0, wo};
   wire [5:0]  w_rem  = rs_len - wr_n;
   wire [3:0]  w_c1   = (d_left < {28'd0, w_room}) ? d_left[3:0] : w_room;
   wire [3:0]  w_cnt  = ({2'd0, w_rem} < {2'd0, w_c1}) ? w_rem[3:0] : w_c1;    // the least of the three
   reg  [63:0] w_data;  reg [7:0] w_strb;
   always @* begin : wbytes
      integer l;
      w_data = 64'd0;  w_strb = 8'd0;
      for (l = 0; l < 8; l = l + 1)
         if (l >= wo && l < wo + w_cnt) begin
            w_data[8*l +: 8] = rs[wr_n + l - wo];
            w_strb[l] = 1'b1;
         end
   end

   wire want_tx = pend2 && q2_ok && (running || flushing);
   always @(posedge clock) begin
      dma_cmd_valid <= 1'b0;
      used_irq      <= 1'b0;
      f_clear       <= 1'b0;
      if (reset || dev_status == 8'd0) begin
         es <= E_IDLE;  issued <= 1'b0;  pend0 <= 1'b0;  pend2 <= 1'b0;
         last0 <= 16'd0;  last2 <= 16'd0;  used0 <= 16'd0;  used2 <= 16'd0;
         running <= 1'b0;  flushing <= 1'b0;  f_clear <= 1'b1;
         p_fmt <= FMT_S16;  p_ch <= 8'd2;  p_rate <= RATE_44100;
         if (reset) begin n_tx <= 32'd0;  n_ctl <= 32'd0; end
      end else begin
         if (notify && notify_q == 32'd0) pend0 <= 1'b1;
         if (notify && notify_q == 32'd2) pend2 <= 1'b1;
         if (dma_rsp_valid && dma_rsp_error)
            $fatal(1, "virtio_snd: DMA error in state %0d", es);
         if (dma_rsp_valid && !issued)
            $fatal(1, "virtio_snd: a DMA response with no command out (state %0d)", es);
         case (es)
            E_IDLE: begin
               issued <= 1'b0;
               if (pend0 && q0_ok)  begin pick(1'b0);  pend0 <= 1'b0;  es <= E_AVAIL; end
               else if (want_tx)    begin pick(1'b1);  pend2 <= 1'b0;  es <= E_AVAIL; end
            end
            E_AVAIL: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_stream <= 1'b0;
                           dma_cmd_addr <= q_driver;  issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;
                        // a buffer waits: serve it, and look again after it (the pending flag)
                        if (get16(dma_rsp_rdata, q_driver[2:0] + 3'd2) != last) begin
                           es <= E_RING;
                           if (eq) pend2 <= 1'b1; else pend0 <= 1'b1;
                        end else es <= E_IDLE;
                     end
            E_RING:  if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_stream <= 1'b0;
                           dma_cmd_addr <= ring_a;  issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;
                        head <= get16(dma_rsp_rdata, ring_a[2:0]);  dix <= get16(dma_rsp_rdata, ring_a[2:0]);
                        es <= E_DADDR;
                     end
            E_DADDR: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_stream <= 1'b0;
                           dma_cmd_addr <= desc_a;  issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;
                        if (!dram_ok(dma_rsp_rdata))
                           $fatal(1, "virtio_snd: descriptor %0d's buffer %h is outside DRAM", dix, dma_rsp_rdata);
                        d_addr <= dram_axi(dma_rsp_rdata);  es <= E_DINFO;
                     end
            E_DINFO: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_stream <= 1'b0;
                           dma_cmd_addr <= desc_a + 32'd8;  issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        // {next, flags, len}; F_NEXT 1, F_WRITE 2 (no indirect: not offered)
                        issued <= 1'b0;  nd <= nd + 8'd1;
                        d_left <= dma_rsp_rdata[31:0];  d_flags <= dma_rsp_rdata[47:32];
                        dix <= dma_rsp_rdata[63:48];          // the next descriptor, if F_NEXT
                        if (dma_rsp_rdata[34] || nd == 8'd255)
                           $fatal(1, "virtio_snd: queue %0d descriptor %0d: indirect, or a chain past 255", eq ? 2 : 0, dix);
                        if (dma_rsp_rdata[33]) begin
                           if (!rs_made) make_rsp;
                           es <= E_WRITE;
                        end else begin
                           if (rs_made)
                              $fatal(1, "virtio_snd: queue %0d: a readable descriptor after a writable one", eq ? 2 : 0);
                           es <= E_READ;
                        end
                     end
            E_READ:  if (d_left == 32'd0) es <= d_flags[0] ? E_DADDR : E_USED_ID;
                     else if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_stream <= is_pcm;
                           dma_cmd_addr <= d_addr;  issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin : got_word
                        reg [3:0] room, take;
                        issued <= 1'b0;
                        room = 4'd8 - {1'b0, d_addr[2:0]};
                        take = (d_left < {28'd0, room}) ? d_left[3:0] : room;
                        bw <= dma_rsp_rdata;  bw_i <= d_addr[2:0];  bw_e <= d_addr[2:0] + take[2:0] - 3'd1;
                        d_addr <= d_addr + {28'd0, take};  d_left <= d_left - {28'd0, take};
                        es <= E_BYTES;
                     end
            E_BYTES: if (byte_go) begin
                        if (!is_pcm && rd_n < 32'd24) rq[rd_n[4:0]] <= pcm_b;
                        rd_n <= rd_n + 32'd1;
                        if (bw_i == bw_e) es <= d_left == 32'd0 ? (d_flags[0] ? E_DADDR : E_USED_ID) : E_READ;
                        else bw_i <= bw_i + 3'd1;
                     end
            // (the write in flight is answered before the descriptor is left: the next state's
            // command would otherwise take its response)
            E_WRITE: if (issued) begin
                        if (dma_rsp_valid) issued <= 1'b0;
                     end else if (d_left == 32'd0 || wr_n == rs_len) begin
                        es <= d_flags[0] ? E_DADDR : E_USED_ID;
                     end else if (dma_cmd_ready) begin
                        dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_stream <= 1'b0;
                        dma_cmd_addr <= d_addr;  dma_cmd_wdata <= w_data;  dma_cmd_wstrb <= w_strb;
                        issued <= 1'b1;
                        d_addr <= d_addr + {28'd0, w_cnt};  d_left <= d_left - {28'd0, w_cnt};
                        wr_n <= wr_n + {2'd0, w_cnt};
                     end
            E_USED_ID: if (!issued) begin
                        if (!rs_made)
                           $fatal(1, "virtio_snd: queue %0d: a chain with no writable descriptor", eq ? 2 : 0);
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_stream <= 1'b0;  dma_cmd_addr <= used_a;
                           dma_cmd_wdata <= {2{16'd0, head}};  dma_cmd_wstrb <= 8'h0f << used_a[2:0];
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin issued <= 1'b0;  es <= E_USED_LEN; end
            E_USED_LEN: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_stream <= 1'b0;  dma_cmd_addr <= used_a + 32'd4;
                           dma_cmd_wdata <= {2{26'd0, wr_n}};  dma_cmd_wstrb <= 8'h0f << used_a[2:0] ^ 8'hff;
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin issued <= 1'b0;  es <= E_USED_IDX; end
            E_USED_IDX: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_stream <= 1'b0;  dma_cmd_addr <= uidx_a;
                           dma_cmd_wdata <= {4{used + 16'd1}};  dma_cmd_wstrb <= 8'h03 << uidx_a[2:0];
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;  used_irq <= 1'b1;
                        if (eq) begin used2 <= used2 + 16'd1;  last2 <= last2 + 16'd1;  n_tx <= n_tx + 32'd1; end
                        else    begin used0 <= used0 + 16'd1;  last0 <= last0 + 16'd1; end
                        // a release or prepare drops the frames still waiting, once its answer is out
                        if (!eq && rq_code == R_PCM_RELEASE || !eq && rq_code == R_PCM_PREPARE) f_clear <= 1'b1;
                        es <= E_IDLE;
                     end
            default: $fatal(1, "virtio_snd: DMA engine in undefined state %0d", es);
         endcase
      end
   end
endmodule

`default_nettype wire
