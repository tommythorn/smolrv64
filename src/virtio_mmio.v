`timescale 1ns / 1ps
`default_nettype none

module virtio_mmio #(
    parameter [31:0] DEVICE_ID = 32'd0,
    parameter [31:0] VENDOR_ID = 32'h736d_6f6c,
    parameter [31:0] DEVICE_FEATURES_0 = 32'd0,
    parameter [31:0] DEVICE_FEATURES_1 = 32'h0000_0003,
    parameter [31:0] QUEUE_NUM_MAX = 32'd8,
    parameter [31:0] QUEUE_COUNT = 32'd1,
    parameter integer NQ = 4                    // queues this transport holds (QUEUE_COUNT <= NQ)
) (
    input  wire        clock,
    input  wire        reset,

    input  wire [11:0] address,
    input  wire        read,
    output reg  [31:0] read_data,
    input  wire        write,
    input  wire [31:0] write_data,
    input  wire [ 3:0] byteenable,

    // Device config space (offset 0x100 up). Reads return config_read_data: the device's config
    // bytes at `address`, right-aligned (the LSU bus's convention). Writes, of any width, come
    // out one cycle later as config_write with the offset from 0x100 -- virtio-input selects
    // its config page with byte writes, which the common registers do not take.
    input  wire [31:0] config_read_data,
    output reg         config_write,
    output reg  [ 7:0] config_offset,
    output reg  [31:0] config_write_data,
    output reg  [ 3:0] config_byteenable,

    output wire        irq,
    output reg         queue_notify_pulse,
    output reg  [31:0] queue_notify_value,

    input  wire        used_buffer_interrupt,
    input  wire        config_change_interrupt,

    output reg  [31:0] driver_features_0,
    output reg  [31:0] driver_features_1,
    // queue q's registers at [q*32 +: 32], [q], [q*64 +: 64]: up to NQ queues (QUEUE_COUNT live)
    output wire [NQ*32-1:0] queues_num,
    output wire [NQ-1:0]    queues_ready,
    output wire [NQ*64-1:0] queues_desc,
    output wire [NQ*64-1:0] queues_driver,
    output wire [NQ*64-1:0] queues_device,
    output reg  [ 7:0] device_status
);
   localparam [31:0] VIRTIO_MAGIC = 32'h7472_6976; /* "virt" little-endian */
   localparam [31:0] VIRTIO_VERSION = 32'd2;

   localparam [11:0] REG_MAGIC_VALUE      = 12'h000;
   localparam [11:0] REG_VERSION          = 12'h004;
   localparam [11:0] REG_DEVICE_ID        = 12'h008;
   localparam [11:0] REG_VENDOR_ID        = 12'h00c;
   localparam [11:0] REG_DEVICE_FEATURES  = 12'h010;
   localparam [11:0] REG_DEVICE_FEAT_SEL  = 12'h014;
   localparam [11:0] REG_DRIVER_FEATURES  = 12'h020;
   localparam [11:0] REG_DRIVER_FEAT_SEL  = 12'h024;
   localparam [11:0] REG_QUEUE_SEL        = 12'h030;
   localparam [11:0] REG_QUEUE_NUM_MAX    = 12'h034;
   localparam [11:0] REG_QUEUE_NUM        = 12'h038;
   localparam [11:0] REG_QUEUE_READY      = 12'h044;
   localparam [11:0] REG_QUEUE_NOTIFY     = 12'h050;
   localparam [11:0] REG_INTERRUPT_STATUS = 12'h060;
   localparam [11:0] REG_INTERRUPT_ACK    = 12'h064;
   localparam [11:0] REG_STATUS           = 12'h070;
   localparam [11:0] REG_QUEUE_DESC_LOW   = 12'h080;
   localparam [11:0] REG_QUEUE_DESC_HIGH  = 12'h084;
   localparam [11:0] REG_QUEUE_AVAIL_LOW  = 12'h090;
   localparam [11:0] REG_QUEUE_AVAIL_HIGH = 12'h094;
   localparam [11:0] REG_QUEUE_USED_LOW   = 12'h0a0;
   localparam [11:0] REG_QUEUE_USED_HIGH  = 12'h0a4;
   localparam [11:0] REG_CONFIG_GEN       = 12'h0fc;

   reg [31:0] device_features_sel;
   reg [31:0] driver_features_sel;
   reg [31:0] queue_sel;
   reg [1:0]  interrupt_status;
   reg [31:0] q_num    [0:NQ-1];
   reg        q_ready  [0:NQ-1];
   reg [63:0] q_desc   [0:NQ-1];
   reg [63:0] q_driver [0:NQ-1];
   reg [63:0] q_device [0:NQ-1];

   wire [11:0] reg_addr = address & 12'hffc;
   // WRITE path pipelined one cycle: the CDC bridge's registered address travels a
   // long route to this decode, and the address->CE cone (queue*_q clock enables)
   // was a chronic -0.003ns setup endpoint. Writes are posted through the bridge
   // with multi-cycle turnaround, so applying them one clock later is invisible
   // (a subsequent read of the written register arrives several cycles after).
   // READS stay on the live address -- the read mux was not the failing cone.
   reg  [11:0] wr_addr_q;
   reg  [31:0] wr_data_q;
   reg         wr_word_q;
   always @(posedge clock) begin
      wr_addr_q <= address & 12'hffc;
      wr_data_q <= write_data;
      wr_word_q <= write && (&byteenable) && address[11:8] == 4'h0 && !reset;
      config_write      <= write && address[11:8] != 4'h0 && !reset;
      config_offset     <= address[7:0];
      config_write_data <= write_data;
      config_byteenable <= byteenable;
   end
   // The common registers are 32-bit (virtio-mmio 4.2.2): a narrower write there is a driver bug
   // this block would otherwise drop without a trace.
   always @(posedge clock)
      if (!reset && write && address[11:8] == 4'h0 && !(&byteenable))
         $fatal(1, "virtio_mmio: %0d-byte-enable write to common register %h", byteenable, address);
   generate if (QUEUE_COUNT > NQ) begin: qcount_chk
      initial $fatal(1, "virtio_mmio: QUEUE_COUNT %0d exceeds the %0d queues this transport holds", QUEUE_COUNT, NQ);
   end endgenerate
   // Which queue QueueSel names, decoded when it is written (one-hot; none when out of range): a
   // 32-bit compare of queue_sel sat at the head of every register read's mux, the board's worst
   // ui_clk path (virtio_net, then the keyboard, 2026-10-04).
   reg  [NQ-1:0] qsel;
   reg  [1:0]    qsel_i;            // ...as an index (valid with |qsel)
   wire          q_active = |qsel;
   integer       k;

   assign irq = interrupt_status != 2'd0;
   genvar gq;
   generate for (gq = 0; gq < NQ; gq = gq + 1) begin: qout
      assign queues_num[gq*32 +: 32]    = q_num[gq];
      assign queues_ready[gq]           = q_ready[gq];
      assign queues_desc[gq*64 +: 64]   = q_desc[gq];
      assign queues_driver[gq*64 +: 64] = q_driver[gq];
      assign queues_device[gq*64 +: 64] = q_device[gq];
   end endgenerate

   always @* begin
      if (!read) begin
         read_data = 32'd0;
      end else begin
         case (reg_addr)
           REG_MAGIC_VALUE:      read_data = VIRTIO_MAGIC;
           REG_VERSION:          read_data = VIRTIO_VERSION;
           REG_DEVICE_ID:        read_data = DEVICE_ID;
           REG_VENDOR_ID:        read_data = VENDOR_ID;
           REG_DEVICE_FEATURES:  read_data = device_features_sel == 32'd0 ? DEVICE_FEATURES_0 :
                                             device_features_sel == 32'd1 ? DEVICE_FEATURES_1 : 32'd0;
           REG_DEVICE_FEAT_SEL:  read_data = device_features_sel;
           REG_DRIVER_FEATURES:  read_data = driver_features_sel == 32'd0 ? driver_features_0 :
                                             driver_features_sel == 32'd1 ? driver_features_1 : 32'd0;
           REG_DRIVER_FEAT_SEL:  read_data = driver_features_sel;
           REG_QUEUE_SEL:        read_data = queue_sel;
           REG_QUEUE_NUM_MAX:    read_data = q_active ? QUEUE_NUM_MAX : 32'd0;
           REG_QUEUE_NUM:        read_data = q_active ? q_num[qsel_i] : 32'd0;
           REG_QUEUE_READY:      read_data = {31'd0, q_active && q_ready[qsel_i]};
           REG_INTERRUPT_STATUS: read_data = {30'd0, interrupt_status};
           REG_STATUS:           read_data = {24'd0, device_status};
           REG_QUEUE_DESC_LOW:   read_data = q_active ? q_desc[qsel_i][31:0]    : 32'd0;
           REG_QUEUE_DESC_HIGH:  read_data = q_active ? q_desc[qsel_i][63:32]   : 32'd0;
           REG_QUEUE_AVAIL_LOW:  read_data = q_active ? q_driver[qsel_i][31:0]  : 32'd0;
           REG_QUEUE_AVAIL_HIGH: read_data = q_active ? q_driver[qsel_i][63:32] : 32'd0;
           REG_QUEUE_USED_LOW:   read_data = q_active ? q_device[qsel_i][31:0]  : 32'd0;
           REG_QUEUE_USED_HIGH:  read_data = q_active ? q_device[qsel_i][63:32] : 32'd0;
           REG_CONFIG_GEN:       read_data = 32'd0;
           default:              read_data = address[11:8] != 4'h0 ? config_read_data : 32'd0;
         endcase
      end
   end

   // every queue's registers back to their reset values (reset, and a driver writing status 0)
   task clear_queues;
      begin
         queue_sel <= 32'd0;
         for (k = 0; k < NQ; k = k + 1) begin
            qsel[k] <= (k == 0) && (QUEUE_COUNT >= 32'd1);
            q_num[k] <= 32'd0;  q_ready[k] <= 1'b0;
            q_desc[k] <= 64'd0;  q_driver[k] <= 64'd0;  q_device[k] <= 64'd0;
         end
         qsel_i <= 2'd0;
      end
   endtask

   always @(posedge clock) begin
      if (reset) begin
         device_features_sel <= 32'd0;
         driver_features_sel <= 32'd0;
         driver_features_0 <= 32'd0;
         driver_features_1 <= 32'd0;
         clear_queues;
         device_status <= 8'd0;
         interrupt_status <= 2'd0;
         queue_notify_pulse <= 1'b0;
         queue_notify_value <= 32'd0;
      end else begin
         queue_notify_pulse <= 1'b0;
         if (used_buffer_interrupt)
            interrupt_status[0] <= 1'b1;
         if (config_change_interrupt)
            interrupt_status[1] <= 1'b1;

         if (wr_word_q) begin
            case (wr_addr_q)
              REG_DEVICE_FEAT_SEL: device_features_sel <= wr_data_q;
              REG_DRIVER_FEAT_SEL: driver_features_sel <= wr_data_q;
              REG_DRIVER_FEATURES: begin
                 if (driver_features_sel == 32'd0)
                    driver_features_0 <= wr_data_q;
                 else if (driver_features_sel == 32'd1)
                    driver_features_1 <= wr_data_q;
              end
              REG_QUEUE_SEL: begin
                 queue_sel <= wr_data_q;
                 for (k = 0; k < NQ; k = k + 1)
                    qsel[k] <= wr_data_q == k && k < QUEUE_COUNT;
                 qsel_i <= wr_data_q[1:0];
              end
              REG_QUEUE_NUM:        if (q_active) q_num[qsel_i] <= wr_data_q;
              REG_QUEUE_READY:      if (q_active) q_ready[qsel_i] <= wr_data_q[0];
              REG_QUEUE_NOTIFY: begin
                 queue_notify_pulse <= 1'b1;
                 queue_notify_value <= wr_data_q;
              end
              REG_INTERRUPT_ACK: interrupt_status <= interrupt_status & ~wr_data_q[1:0];
              REG_STATUS: begin
                 device_status <= wr_data_q[7:0];
                 if (wr_data_q[7:0] == 8'd0) begin
                    driver_features_0 <= 32'd0;
                    driver_features_1 <= 32'd0;
                    clear_queues;
                    interrupt_status <= 2'd0;
                 end
              end
              REG_QUEUE_DESC_LOW:   if (q_active) q_desc[qsel_i][31:0]    <= wr_data_q;
              REG_QUEUE_DESC_HIGH:  if (q_active) q_desc[qsel_i][63:32]   <= wr_data_q;
              REG_QUEUE_AVAIL_LOW:  if (q_active) q_driver[qsel_i][31:0]  <= wr_data_q;
              REG_QUEUE_AVAIL_HIGH: if (q_active) q_driver[qsel_i][63:32] <= wr_data_q;
              REG_QUEUE_USED_LOW:   if (q_active) q_device[qsel_i][31:0]  <= wr_data_q;
              REG_QUEUE_USED_HIGH:  if (q_active) q_device[qsel_i][63:32] <= wr_data_q;
              default: begin
              end
            endcase
         end
      end
   end
endmodule

`default_nettype wire
