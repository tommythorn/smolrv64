`timescale 1ns / 1ps
`default_nettype none
// virtio_input -- the keyboard of simmerv's `--graphics`, fed from the serial line.
//
// A virtio-mmio v2 input device (ID 18) at 0x1000_4000, PLIC source 4, exactly as simmerv's
// src/device/virtio_input.rs presents it: the same config space (name "simmerv keyboard",
// devids, the EV_KEY bitmap, EV_REP so the guest autorepeats), two queues -- eventq (0), which
// the device fills with 8-byte virtio_input_events, and statusq (1), whose LED updates it only
// returns. Its DMA is non-coherent, like virtio-blk's (dma-noncoherent in the DTS).
//
// The board has no keyboard: its keys arrive as terminal bytes on the UART, and the platform
// steers them here when the operator selects the graphics console (key[3]). They are turned
// back into key presses as simmerv's sim/src/term_keys.rs does when its display is a terminal:
// printable ASCII (US layout), control characters, xterm escape sequences with modifiers, and
// ESC + byte = Alt. The one difference is the lone ESC: simmerv calls an ESC that ends a host
// read() the Esc key; here it is an ESC followed by nothing for ESC_TIMEOUT cycles (1 ms; a
// terminal sends a sequence in one write, 3.3 us a byte at 3 Mbps). tb_virtio_input checks the
// translation against vectors produced by simmerv's own translate() (kbd_vectors.txt).
//
// Each key press becomes events as simmerv's term_keys::send emits them: the modifiers down
// (Ctrl, Shift, Alt), the key down, the key up, the modifiers up in reverse, each followed by
// SYN_REPORT. Events wait while the driver has no buffer posted; bytes wait in a 512-byte FIFO
// (a paste); a byte that finds it full is counted (0x1000_4F00), never silently lost.
module virtio_input #(
    parameter integer ESC_TIMEOUT = 333_333        // ui_clk cycles: 1 ms at 333.33 MHz
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

    input  wire        key_valid,      // one terminal byte
    input  wire [ 7:0] key_byte,

    output wire [ 2:0] m_axi_awid,
    output wire [30:0] m_axi_awaddr,
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
    output wire [30:0] m_axi_araddr,
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
   localparam [15:0] EV_SYN = 16'h00, EV_KEY = 16'h01;
   localparam [6:0]  K_ESC = 7'd1, K_TAB = 7'd15, K_LCTRL = 7'd29, K_LSHIFT = 7'd42, K_LALT = 7'd56;
   localparam [2:0]  M_SHIFT = 3'd1, M_ALT = 3'd2, M_CTRL = 3'd4;

   // ============================ registers and config space ============================
   wire        notify;  wire [31:0] notify_q;
   reg         used_irq;
   wire [31:0] q0_num, q1_num;  wire q0_ready, q1_ready;
   wire [63:0] q0_desc, q0_driver, q0_device, q1_desc, q1_driver, q1_device;
   wire [ 7:0] dev_status;
   wire        cfg_write;  wire [7:0] cfg_off;  wire [31:0] cfg_wdata;  wire [3:0] cfg_be;
   wire [31:0] mmio_rdata, cfg_rdata;

   virtio_mmio #(.DEVICE_ID(32'd18), .QUEUE_NUM_MAX(32'd64), .QUEUE_COUNT(32'd2)) u_mmio (
      .clock(clock), .reset(reset),
      .address(address), .read(read), .read_data(mmio_rdata),
      .write(write && address[11:8] != 4'hf), .write_data(write_data), .byteenable(byteenable),
      .config_read_data(cfg_rdata),
      .config_write(cfg_write), .config_offset(cfg_off), .config_write_data(cfg_wdata),
      .config_byteenable(cfg_be),
      .irq(irq), .queue_notify_pulse(notify), .queue_notify_value(notify_q),
      .used_buffer_interrupt(used_irq), .config_change_interrupt(1'b0),
      .driver_features_0(), .driver_features_1(),
      .queue_num(), .queue_ready(), .queue_desc(), .queue_driver(), .queue_device(),
      .queue0_num(q0_num), .queue0_ready(q0_ready), .queue0_desc(q0_desc),
      .queue0_driver(q0_driver), .queue0_device(q0_device),
      .queue1_num(q1_num), .queue1_ready(q1_ready), .queue1_desc(q1_desc),
      .queue1_driver(q1_driver), .queue1_device(q1_device),
      .device_status(dev_status));

   // virtio_input_config: select, subsel, size, 5 reserved, then the union. Written a byte at a
   // time (select, then subsel); read back a byte at a time, right-aligned at the address.
   reg [7:0] cfg_select, cfg_subsel;
   always @(posedge clock)
      if (reset) begin cfg_select <= 8'd0;  cfg_subsel <= 8'd0; end
      else if (cfg_write) begin : cfg_wr
         integer i;
         for (i = 0; i < 4; i = i + 1)
            if (cfg_be[i]) begin
               if (cfg_off + i == 0) cfg_select <= cfg_wdata[8*i +: 8];
               if (cfg_off + i == 1) cfg_subsel <= cfg_wdata[8*i +: 8];
            end
      end

   localparam [127:0] NAME  = "draobyek vremmis";   // "simmerv keyboard", byte 0 first
   localparam [127:0] EVKEY = 128'he080ffdf01cffffffffffffffffffffe;
   localparam [63:0]  DEVIDS = 64'h0001_0001_0627_0006; // bustype BUS_VIRTUAL, vendor, product, version
   reg [7:0] cfg_size;
   always @* begin
      case (cfg_select)
         8'h01:   cfg_size = 8'd16;                                   // ID_NAME
         8'h03:   cfg_size = 8'd8;                                    // ID_DEVIDS
         8'h11:   cfg_size = cfg_subsel == 8'h01 ? 8'd16 :            // EV_BITS / EV_KEY
                             cfg_subsel == 8'h14 ? 8'd1  : 8'd0;      // EV_BITS / EV_REP
         default: cfg_size = 8'd0;
      endcase
   end
   function [7:0] cfg_byte(input [7:0] k);
      reg [7:0] j;
      begin
         j = k - 8'd8;
         if (k == 8'd0)      cfg_byte = cfg_select;
         else if (k == 8'd1) cfg_byte = cfg_subsel;
         else if (k == 8'd2) cfg_byte = cfg_size;
         else if (k < 8'd8 || j >= cfg_size) cfg_byte = 8'd0;
         else case (cfg_select)
            8'h01:   cfg_byte = NAME[{j[3:0], 3'd0} +: 8];
            8'h03:   cfg_byte = DEVIDS[{j[2:0], 3'd0} +: 8];
            8'h11:   cfg_byte = cfg_subsel == 8'h01 ? EVKEY[{j[3:0], 3'd0} +: 8] : 8'h01;
            default: cfg_byte = 8'd0;
         endcase
      end
   endfunction
   wire [7:0] cfg_k = address[7:0];   // offset within the config space (it starts at 0x100)
   assign cfg_rdata = {cfg_byte(cfg_k + 8'd3), cfg_byte(cfg_k + 8'd2), cfg_byte(cfg_k + 8'd1), cfg_byte(cfg_k)};

   // Diagnostics at 0x1000_4F00 (not part of the virtio device): bytes lost to a full FIFO,
   // events delivered.
   reg [15:0] bytes_lost, events_done;
   assign read_data = address[11:8] == 4'hf ? (address[2] ? {16'd0, events_done} : {16'd0, bytes_lost})
                                            : mmio_rdata;

   // ================================ byte FIFO, 512 deep ================================
   reg  [7:0] bq [0:511];
   reg  [9:0] bq_w, bq_r;
   wire       bq_empty = bq_w == bq_r;
   wire       bq_full  = bq_w == {~bq_r[9], bq_r[8:0]};
   wire [7:0] b = bq[bq_r[8:0]];
   wire       b_take;
   always @(posedge clock) begin
      if (reset) begin bq_w <= 10'd0;  bq_r <= 10'd0;  bytes_lost <= 16'd0; end
      else begin
         if (key_valid && !bq_full) begin bq[bq_w[8:0]] <= key_byte;  bq_w <= bq_w + 10'd1; end
         if (key_valid && bq_full) bytes_lost <= bytes_lost + 16'd1;
         if (b_take) bq_r <= bq_r + 10'd1;
      end
   end

   // ========================= translator: bytes -> key presses =========================
   // byte_key: simmerv's term_keys::byte_key, every byte but ESC -- {valid, keycode, mods}.
   function [10:0] byte_key(input [7:0] c);
      begin
         byte_key = 11'd0;
         case (c)
         8'h00: byte_key = {1'b1, 7'd57, 3'd4};  8'h01: byte_key = {1'b1, 7'd30, 3'd4};  8'h02: byte_key = {1'b1, 7'd48, 3'd4};
         8'h03: byte_key = {1'b1, 7'd46, 3'd4};  8'h04: byte_key = {1'b1, 7'd32, 3'd4};  8'h05: byte_key = {1'b1, 7'd18, 3'd4};
         8'h06: byte_key = {1'b1, 7'd33, 3'd4};  8'h07: byte_key = {1'b1, 7'd34, 3'd4};  8'h08: byte_key = {1'b1, 7'd14, 3'd0};
         8'h09: byte_key = {1'b1, 7'd15, 3'd0};  8'h0a: byte_key = {1'b1, 7'd28, 3'd0};  8'h0b: byte_key = {1'b1, 7'd37, 3'd4};
         8'h0c: byte_key = {1'b1, 7'd38, 3'd4};  8'h0d: byte_key = {1'b1, 7'd28, 3'd0};  8'h0e: byte_key = {1'b1, 7'd49, 3'd4};
         8'h0f: byte_key = {1'b1, 7'd24, 3'd4};  8'h10: byte_key = {1'b1, 7'd25, 3'd4};  8'h11: byte_key = {1'b1, 7'd16, 3'd4};
         8'h12: byte_key = {1'b1, 7'd19, 3'd4};  8'h13: byte_key = {1'b1, 7'd31, 3'd4};  8'h14: byte_key = {1'b1, 7'd20, 3'd4};
         8'h15: byte_key = {1'b1, 7'd22, 3'd4};  8'h16: byte_key = {1'b1, 7'd47, 3'd4};  8'h17: byte_key = {1'b1, 7'd17, 3'd4};
         8'h18: byte_key = {1'b1, 7'd45, 3'd4};  8'h19: byte_key = {1'b1, 7'd21, 3'd4};  8'h1a: byte_key = {1'b1, 7'd44, 3'd4};
         8'h1c: byte_key = {1'b1, 7'd43, 3'd4};  8'h1d: byte_key = {1'b1, 7'd27, 3'd4};  8'h1e: byte_key = {1'b1, 7'd7, 3'd5};
         8'h1f: byte_key = {1'b1, 7'd12, 3'd5};  8'h20: byte_key = {1'b1, 7'd57, 3'd0};  8'h21: byte_key = {1'b1, 7'd2, 3'd1};
         8'h22: byte_key = {1'b1, 7'd40, 3'd1};  8'h23: byte_key = {1'b1, 7'd4, 3'd1};  8'h24: byte_key = {1'b1, 7'd5, 3'd1};
         8'h25: byte_key = {1'b1, 7'd6, 3'd1};  8'h26: byte_key = {1'b1, 7'd8, 3'd1};  8'h27: byte_key = {1'b1, 7'd40, 3'd0};
         8'h28: byte_key = {1'b1, 7'd10, 3'd1};  8'h29: byte_key = {1'b1, 7'd11, 3'd1};  8'h2a: byte_key = {1'b1, 7'd9, 3'd1};
         8'h2b: byte_key = {1'b1, 7'd13, 3'd1};  8'h2c: byte_key = {1'b1, 7'd51, 3'd0};  8'h2d: byte_key = {1'b1, 7'd12, 3'd0};
         8'h2e: byte_key = {1'b1, 7'd52, 3'd0};  8'h2f: byte_key = {1'b1, 7'd53, 3'd0};  8'h30: byte_key = {1'b1, 7'd11, 3'd0};
         8'h31: byte_key = {1'b1, 7'd2, 3'd0};  8'h32: byte_key = {1'b1, 7'd3, 3'd0};  8'h33: byte_key = {1'b1, 7'd4, 3'd0};
         8'h34: byte_key = {1'b1, 7'd5, 3'd0};  8'h35: byte_key = {1'b1, 7'd6, 3'd0};  8'h36: byte_key = {1'b1, 7'd7, 3'd0};
         8'h37: byte_key = {1'b1, 7'd8, 3'd0};  8'h38: byte_key = {1'b1, 7'd9, 3'd0};  8'h39: byte_key = {1'b1, 7'd10, 3'd0};
         8'h3a: byte_key = {1'b1, 7'd39, 3'd1};  8'h3b: byte_key = {1'b1, 7'd39, 3'd0};  8'h3c: byte_key = {1'b1, 7'd51, 3'd1};
         8'h3d: byte_key = {1'b1, 7'd13, 3'd0};  8'h3e: byte_key = {1'b1, 7'd52, 3'd1};  8'h3f: byte_key = {1'b1, 7'd53, 3'd1};
         8'h40: byte_key = {1'b1, 7'd3, 3'd1};  8'h41: byte_key = {1'b1, 7'd30, 3'd1};  8'h42: byte_key = {1'b1, 7'd48, 3'd1};
         8'h43: byte_key = {1'b1, 7'd46, 3'd1};  8'h44: byte_key = {1'b1, 7'd32, 3'd1};  8'h45: byte_key = {1'b1, 7'd18, 3'd1};
         8'h46: byte_key = {1'b1, 7'd33, 3'd1};  8'h47: byte_key = {1'b1, 7'd34, 3'd1};  8'h48: byte_key = {1'b1, 7'd35, 3'd1};
         8'h49: byte_key = {1'b1, 7'd23, 3'd1};  8'h4a: byte_key = {1'b1, 7'd36, 3'd1};  8'h4b: byte_key = {1'b1, 7'd37, 3'd1};
         8'h4c: byte_key = {1'b1, 7'd38, 3'd1};  8'h4d: byte_key = {1'b1, 7'd50, 3'd1};  8'h4e: byte_key = {1'b1, 7'd49, 3'd1};
         8'h4f: byte_key = {1'b1, 7'd24, 3'd1};  8'h50: byte_key = {1'b1, 7'd25, 3'd1};  8'h51: byte_key = {1'b1, 7'd16, 3'd1};
         8'h52: byte_key = {1'b1, 7'd19, 3'd1};  8'h53: byte_key = {1'b1, 7'd31, 3'd1};  8'h54: byte_key = {1'b1, 7'd20, 3'd1};
         8'h55: byte_key = {1'b1, 7'd22, 3'd1};  8'h56: byte_key = {1'b1, 7'd47, 3'd1};  8'h57: byte_key = {1'b1, 7'd17, 3'd1};
         8'h58: byte_key = {1'b1, 7'd45, 3'd1};  8'h59: byte_key = {1'b1, 7'd21, 3'd1};  8'h5a: byte_key = {1'b1, 7'd44, 3'd1};
         8'h5b: byte_key = {1'b1, 7'd26, 3'd0};  8'h5c: byte_key = {1'b1, 7'd43, 3'd0};  8'h5d: byte_key = {1'b1, 7'd27, 3'd0};
         8'h5e: byte_key = {1'b1, 7'd7, 3'd1};  8'h5f: byte_key = {1'b1, 7'd12, 3'd1};  8'h60: byte_key = {1'b1, 7'd41, 3'd0};
         8'h61: byte_key = {1'b1, 7'd30, 3'd0};  8'h62: byte_key = {1'b1, 7'd48, 3'd0};  8'h63: byte_key = {1'b1, 7'd46, 3'd0};
         8'h64: byte_key = {1'b1, 7'd32, 3'd0};  8'h65: byte_key = {1'b1, 7'd18, 3'd0};  8'h66: byte_key = {1'b1, 7'd33, 3'd0};
         8'h67: byte_key = {1'b1, 7'd34, 3'd0};  8'h68: byte_key = {1'b1, 7'd35, 3'd0};  8'h69: byte_key = {1'b1, 7'd23, 3'd0};
         8'h6a: byte_key = {1'b1, 7'd36, 3'd0};  8'h6b: byte_key = {1'b1, 7'd37, 3'd0};  8'h6c: byte_key = {1'b1, 7'd38, 3'd0};
         8'h6d: byte_key = {1'b1, 7'd50, 3'd0};  8'h6e: byte_key = {1'b1, 7'd49, 3'd0};  8'h6f: byte_key = {1'b1, 7'd24, 3'd0};
         8'h70: byte_key = {1'b1, 7'd25, 3'd0};  8'h71: byte_key = {1'b1, 7'd16, 3'd0};  8'h72: byte_key = {1'b1, 7'd19, 3'd0};
         8'h73: byte_key = {1'b1, 7'd31, 3'd0};  8'h74: byte_key = {1'b1, 7'd20, 3'd0};  8'h75: byte_key = {1'b1, 7'd22, 3'd0};
         8'h76: byte_key = {1'b1, 7'd47, 3'd0};  8'h77: byte_key = {1'b1, 7'd17, 3'd0};  8'h78: byte_key = {1'b1, 7'd45, 3'd0};
         8'h79: byte_key = {1'b1, 7'd21, 3'd0};  8'h7a: byte_key = {1'b1, 7'd44, 3'd0};  8'h7b: byte_key = {1'b1, 7'd26, 3'd1};
         8'h7c: byte_key = {1'b1, 7'd43, 3'd1};  8'h7d: byte_key = {1'b1, 7'd27, 3'd1};  8'h7e: byte_key = {1'b1, 7'd41, 3'd1};
         8'h7f: byte_key = {1'b1, 7'd14, 3'd0};
            default: byte_key = 11'd0;
         endcase
      end
   endfunction
   // sequence_key: simmerv's term_keys::sequence_key -- {valid, keycode, mods}. p1v says a
   // second parameter was given; its value is xterm's 1 + modifiers.
   function [10:0] sequence_key(input [15:0] p0, input p1v, input [15:0] p1, input [7:0] fin);
      reg [2:0] m;  reg [6:0] k;  reg ok;  reg [15:0] p1m, f, n;   // no SV casts: Vivado reads .v as Verilog-2001
      begin
         p1m = p1 - 16'd1;  f = {8'd0, fin} - "P";
         m  = !p1v || p1 == 16'd0 ? 3'd0 : p1m[2:0];
         ok = 1'b1;  k = 7'd0;
         case (fin)
            "A": k = 7'd103;  "B": k = 7'd108;  "C": k = 7'd106;  "D": k = 7'd105;
            "H": k = 7'd102;  "F": k = 7'd107;
            "P", "Q", "R", "S": k = 7'd59 + f[6:0];
            "Z": begin k = K_TAB;  m = M_SHIFT; end          // Shift-Tab, whatever the modifiers
            "~": case (p0)
                    16'd1, 16'd7: k = 7'd102;  16'd2: k = 7'd110;  16'd3: k = 7'd111;
                    16'd4, 16'd8: k = 7'd107;  16'd5: k = 7'd104;  16'd6: k = 7'd109;
                    16'd11, 16'd12, 16'd13, 16'd14, 16'd15: begin n = p0 - 16'd11;  k = 7'd59 + n[6:0]; end
                    16'd17, 16'd18, 16'd19, 16'd20, 16'd21: begin n = p0 - 16'd17;  k = 7'd64 + n[6:0]; end
                    16'd23: k = 7'd87;  16'd24: k = 7'd88;
                    default: ok = 1'b0;
                 endcase
            default: ok = 1'b0;
         endcase
         sequence_key = {ok, k, m};
      end
   endfunction
   function [15:0] sat_digit(input [15:0] p, input [7:0] c);   // p*10 + digit, saturating
      reg [19:0] t;
      begin
         t = {4'd0, p} * 20'd10 + {16'd0, c[3:0]};
         sat_digit = t > 20'd65535 ? 16'hffff : t[15:0];
      end
   endfunction

   localparam [1:0] T_IDLE = 2'd0, T_ESC = 2'd1, T_CSI = 2'd2;
   reg  [1:0]  ts;
   reg  [15:0] p0, p1;
   reg  [1:0]  pidx;               // parameter being parsed; 2 = past the ones that matter
   reg  [18:0] tmo;
   // The expander holds one key press; the translator acts only when it is free.
   reg         st_v;  reg [6:0] st_key;  reg [2:0] st_mods;
   reg  [3:0]  st_step;            // event index: transition st_step>>1, {KEY, SYN} by st_step[0]
   wire        tr_go = !st_v;
   assign b_take = tr_go && !bq_empty;
   wire [10:0] bk  = byte_key(b);
   wire [10:0] sk  = sequence_key(p0, pidx != 2'd0, p1, b);
   wire        tmo_hit = tmo == ESC_TIMEOUT[18:0];
   task emit(input [10:0] kv, input [2:0] extra);
      begin
         if (kv[10]) begin st_v <= 1'b1;  st_key <= kv[9:3];  st_mods <= kv[2:0] | extra;  st_step <= 4'd0; end
      end
   endtask
   wire        ev_pop;
   wire [2:0]  n_mods = {2'b00, st_mods[2]} + {2'b00, st_mods[0]} + {2'b00, st_mods[1]};
   wire [3:0]  n_ev   = {n_mods[1:0], 2'b00} + 4'd3;   // 2*(2*mods + 2) events, minus one: the last index
   always @(posedge clock) begin
      if (reset) begin
         ts <= T_IDLE;  st_v <= 1'b0;  tmo <= 19'd0;
      end else begin
         if (ev_pop) begin
            if (st_step == n_ev) st_v <= 1'b0;
            st_step <= st_step + 4'd1;
         end
         if (tr_go) case (ts)
            T_IDLE: if (!bq_empty) begin
                       if (b == 8'h1b) begin ts <= T_ESC;  tmo <= 19'd0; end
                       else emit(bk, 3'd0);
                    end
            T_ESC:  if (!bq_empty) begin
                       if (b == "[" || b == "O") begin ts <= T_CSI;  p0 <= 16'd0;  p1 <= 16'd0;  pidx <= 2'd0;  tmo <= 19'd0; end
                       else begin emit(b == 8'h1b ? {1'b1, K_ESC, 3'd0} : bk, M_ALT);  ts <= T_IDLE; end
                    end else if (tmo_hit) begin
                       emit({1'b1, K_ESC, 3'd0}, 3'd0);  ts <= T_IDLE;    // a lone ESC: the Esc key
                    end else tmo <= tmo + 19'd1;
            T_CSI:  if (!bq_empty) begin
                       tmo <= 19'd0;
                       if (b >= "0" && b <= "9") begin
                          if (pidx == 2'd0) p0 <= sat_digit(p0, b);
                          if (pidx == 2'd1) p1 <= sat_digit(p1, b);
                       end else if (b == ";") begin
                          if (pidx != 2'd2) pidx <= pidx + 2'd1;
                       end else begin
                          if (b >= 8'h40 && b <= 8'h7e) emit(sk, 3'd0);   // anything else: dropped
                          ts <= T_IDLE;
                       end
                    end else if (tmo_hit) ts <= T_IDLE;                   // an unfinished sequence
                    else tmo <= tmo + 19'd1;
            default: $fatal(1, "virtio_input: translator in undefined state %0d", ts);
         endcase
      end
   end

   // ===================== expander: one key press -> its events =====================
   // Transitions: modifiers down in Ctrl, Shift, Alt order; the key down; the key up; the
   // modifiers up in reverse. t indexes them; each is EV_KEY then SYN_REPORT.
   wire [2:0] t = st_step[3:1];
   reg  [6:0] t_code;  reg t_down;
   always @* begin : expand
      reg [6:0] mods_l [0:2];  reg [1:0] n;  reg [2:0] ri;
      n = 2'd0;  mods_l[0] = 7'd0;  mods_l[1] = 7'd0;  mods_l[2] = 7'd0;
      if (st_mods[2]) begin mods_l[n] = K_LCTRL;  n = n + 2'd1; end
      if (st_mods[0]) begin mods_l[n] = K_LSHIFT; n = n + 2'd1; end
      if (st_mods[1]) begin mods_l[n] = K_LALT;   n = n + 2'd1; end
      if (t < {1'b0, n})                    begin t_code = mods_l[t[1:0]];                  t_down = 1'b1; end
      else if (t == {1'b0, n})              begin t_code = st_key;                          t_down = 1'b1; end
      else if (t == {1'b0, n} + 3'd1)       begin t_code = st_key;                          t_down = 1'b0; end
      else begin ri = 3'd2 * {1'b0, n} + 3'd1 - t;  t_code = mods_l[ri[1:0]];  t_down = 1'b0; end
   end
   wire        ev_v = st_v;
   wire [63:0] ev   = st_step[0] ? 64'd0                                           // SYN_REPORT
                                 : {31'd0, t_down, 9'd0, t_code, EV_KEY};          // value, code, type

   // ================================ the DMA engine ================================
   wire        dma_cmd_ready, dma_rsp_valid, dma_rsp_error;
   wire [63:0] dma_rsp_rdata;
   reg         dma_cmd_valid, dma_cmd_write;
   reg  [30:0] dma_cmd_addr;
   reg  [63:0] dma_cmd_wdata;
   reg  [ 7:0] dma_cmd_wstrb;
   axi_single_beat_master dma_master (
      .clock(clock), .reset(reset),
      .cmd_valid(dma_cmd_valid), .cmd_ready(dma_cmd_ready), .cmd_write(dma_cmd_write),
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

   wire driver_ok = dev_status[2];
   function q_ok(input ready, input [31:0] num, input [63:0] d, input [63:0] a, input [63:0] u);
      q_ok = ready && num != 32'd0 && d[63:32] == 32'd0 && a[63:32] == 32'd0 && u[63:32] == 32'd0;
   endfunction
   wire q0_ok = driver_ok && q_ok(q0_ready, q0_num, q0_desc, q0_driver, q0_device);
   wire q1_ok = driver_ok && q_ok(q1_ready, q1_num, q1_desc, q1_driver, q1_device);
   always @(posedge clock)
      if (!reset && ((q0_ok && (q0_num & (q0_num - 32'd1)) != 32'd0) || (q1_ok && (q1_num & (q1_num - 32'd1)) != 32'd0)))
         $fatal(1, "virtio_input: queue size %0d/%0d is not a power of two", q0_num, q1_num);

   localparam [3:0] E_IDLE = 4'd0, E_AVAIL = 4'd1, E_RING = 4'd2, E_DADDR = 4'd3, E_DINFO = 4'd4,
                    E_EVENT = 4'd5, E_USED_ID = 4'd6, E_USED_LEN = 4'd7, E_USED_IDX = 4'd8;
   reg [3:0]  es;
   reg        eq;                 // the queue being served: 0 eventq, 1 statusq
   reg        issued;             // this state's DMA command is out; waiting for its response
   reg [15:0] last0, last1, used0, used1, avail0, avail1;
   reg        need_notify0, status_pending;
   // A notify for eventq that lands while its avail.idx read is in flight already announced the
   // buffer that read may have missed: waiting for the NEXT notify would be a lost wakeup.
   reg        notified0;
   reg [15:0] head;
   reg [63:0] buf_addr;
   reg [30:0] buf_axi;
   // The queue being served, latched as the engine picks it, in AXI address bits: q_ok has
   // shown the upper halves zero, and every ring lives in DRAM, so the arithmetic is mod 2^31
   // (what axi_a would have truncated to anyway). Muxing the two queues and adding in 64 bits on
   // the way to dma_cmd_addr was 16 logic levels, a failing ui_clk path (2026-10-04).
   reg  [30:0] q_desc, q_driver, q_device;
   reg  [15:0] q_mask;
   wire [15:0] last     = eq ? last1 : last0;
   wire [15:0] used     = eq ? used1 : used0;
   wire [30:0] ring_a   = q_driver + 31'd4 + {14'd0, last & q_mask, 1'b0};
   wire [30:0] used_a   = q_device + 31'd4 + {12'd0, used & q_mask, 3'd0};
   wire [30:0] uidx_a   = q_device + 31'd2;
   wire [30:0] desc_a   = q_desc + {11'd0, head & q_mask, 4'd0};
   function [15:0] get16(input [63:0] d, input [2:0] off);
      get16 = d[{off, 3'd0} +: 16];
   endfunction
   function [30:0] axi_a(input [63:0] a);       // DRAM physical address -> AXI, as virtio_blk
      axi_a = a[63:31] <= 33'd1 ? a[30:0] : 31'd0;
   endfunction
   task pick(input which);                       // serve queue `which` from the next state on
      begin
         eq       <= which;
         q_desc   <= axi_a(which ? q1_desc   : q0_desc);
         q_driver <= axi_a(which ? q1_driver : q0_driver);
         q_device <= axi_a(which ? q1_device : q0_device);
         q_mask   <= (which ? q1_num[15:0] : q0_num[15:0]) - 16'd1;
      end
   endtask
   assign ev_pop = es == E_EVENT && issued && dma_rsp_valid;

   always @(posedge clock) begin
      dma_cmd_valid <= 1'b0;
      used_irq      <= 1'b0;
      if (reset || dev_status == 8'd0) begin
         es <= E_IDLE;  issued <= 1'b0;  status_pending <= 1'b0;  need_notify0 <= 1'b0;  notified0 <= 1'b0;
         last0 <= 16'd0;  last1 <= 16'd0;  used0 <= 16'd0;  used1 <= 16'd0;  avail0 <= 16'd0;  avail1 <= 16'd0;
         if (reset) events_done <= 16'd0;
      end else begin
         if (notify && notify_q == 32'd1) status_pending <= 1'b1;
         if (notify && notify_q == 32'd0) begin need_notify0 <= 1'b0;  notified0 <= 1'b1; end
         if (dma_rsp_valid && dma_rsp_error)
            $fatal(1, "virtio_input: DMA error in state %0d", es);
         case (es)
            E_IDLE: begin
               issued <= 1'b0;
               if (status_pending && q1_ok) begin
                  pick(1'b1);  status_pending <= 1'b0;  es <= E_AVAIL;
               end else if (ev_v && q0_ok && !need_notify0) begin
                  pick(1'b0);  es <= last0 != avail0 ? E_RING : E_AVAIL;
               end
            end
            E_AVAIL: if (!issued) begin                 // re-read avail.idx
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_addr <= q_driver;
                           issued <= 1'b1;  notified0 <= 1'b0;
                        end
                     end else if (dma_rsp_valid) begin : got_avail
                        reg [15:0] a;
                        issued <= 1'b0;
                        a = get16(dma_rsp_rdata, q_driver[2:0] + 3'd2);
                        if (eq) avail1 <= a; else avail0 <= a;
                        if (a != last) es <= E_RING;
                        else begin
                           es <= E_IDLE;
                           // No buffer: wait for the driver to post one -- unless it already has.
                           if (!eq && !(notified0 || (notify && notify_q == 32'd0))) need_notify0 <= 1'b1;
                        end
                     end
            E_RING:  if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;  dma_cmd_addr <= ring_a;
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;
                        head <= get16(dma_rsp_rdata, ring_a[2:0]);
                        es <= eq ? E_USED_ID : E_DADDR;     // a status buffer is only returned
                     end
            E_DADDR: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;
                           dma_cmd_addr <= desc_a;
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;  buf_addr <= dma_rsp_rdata;  buf_axi <= axi_a(dma_rsp_rdata);  es <= E_DINFO;
                     end
            E_DINFO: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b0;
                           dma_cmd_addr <= desc_a + 31'd8;
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        // {next, flags, len}: a device-writable buffer of at least one event.
                        if (dma_rsp_rdata[31:0] < 32'd8 || !dma_rsp_rdata[33] || buf_addr[2:0] != 3'd0)
                           $fatal(1, "virtio_input: event buffer %h len %0d flags %h is not one aligned, writable event",
                                  buf_addr, dma_rsp_rdata[31:0], dma_rsp_rdata[47:32]);
                        issued <= 1'b0;  es <= E_EVENT;
                     end
            E_EVENT: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_addr <= buf_axi;
                           dma_cmd_wdata <= ev;  dma_cmd_wstrb <= 8'hff;
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;  events_done <= events_done + 16'd1;  es <= E_USED_ID;
                     end
            E_USED_ID: if (!issued) begin              // used elem: id (head), then len
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_addr <= used_a;
                           dma_cmd_wdata <= {2{16'd0, head}};  dma_cmd_wstrb <= 8'h0f << used_a[2:0];
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin issued <= 1'b0;  es <= E_USED_LEN; end
            E_USED_LEN: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_addr <= used_a + 31'd4;
                           dma_cmd_wdata <= {2{eq ? 32'd0 : 32'd8}};  dma_cmd_wstrb <= 8'h0f << used_a[2:0] ^ 8'hff;
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin issued <= 1'b0;  es <= E_USED_IDX; end
            E_USED_IDX: if (!issued) begin
                        if (dma_cmd_ready) begin
                           dma_cmd_valid <= 1'b1;  dma_cmd_write <= 1'b1;  dma_cmd_addr <= uidx_a;
                           dma_cmd_wdata <= {4{used + 16'd1}};  dma_cmd_wstrb <= 8'h03 << uidx_a[2:0];
                           issued <= 1'b1;
                        end
                     end else if (dma_rsp_valid) begin
                        issued <= 1'b0;  used_irq <= 1'b1;
                        if (eq) begin used1 <= used1 + 16'd1;  last1 <= last1 + 16'd1; end
                        else    begin used0 <= used0 + 16'd1;  last0 <= last0 + 16'd1; end
                        // statusq: return everything posted, then idle; eventq: one event per pass
                        es <= eq && last1 + 16'd1 != avail1 ? E_RING : E_IDLE;
                     end
            default: $fatal(1, "virtio_input: DMA engine in undefined state %0d", es);
         endcase
      end
   end
endmodule

`default_nettype wire
