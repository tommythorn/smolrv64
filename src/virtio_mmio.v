`timescale 1ns / 1ps
`default_nettype none

module virtio_mmio #(
    parameter [31:0] DEVICE_ID = 32'd0,
    parameter [31:0] VENDOR_ID = 32'h736d_6f6c,
    parameter [31:0] DEVICE_FEATURES_0 = 32'd0,
    parameter [31:0] DEVICE_FEATURES_1 = 32'h0000_0003,
    parameter [31:0] QUEUE_NUM_MAX = 32'd8,
    parameter [31:0] QUEUE_COUNT = 32'd1,
    parameter [31:0] CONFIG_CAPACITY_SECTORS = 32'd0 /* virtio-blk config: 512B sectors */
) (
    input  wire        clock,
    input  wire        reset,

    input  wire [11:0] address,
    input  wire        read,
    output reg  [31:0] read_data,
    input  wire        write,
    input  wire [31:0] write_data,
    input  wire [ 3:0] byteenable,

    output wire        irq,
    output reg         queue_notify_pulse,
    output reg  [31:0] queue_notify_value,

    input  wire        used_buffer_interrupt,
    input  wire        config_change_interrupt,

    output reg  [31:0] driver_features_0,
    output reg  [31:0] driver_features_1,
    output wire [31:0] queue_num,
    output wire        queue_ready,
    output wire [63:0] queue_desc,
    output wire [63:0] queue_driver,
    output wire [63:0] queue_device,
    output wire [31:0] queue0_num,
    output wire        queue0_ready,
    output wire [63:0] queue0_desc,
    output wire [63:0] queue0_driver,
    output wire [63:0] queue0_device,
    output wire [31:0] queue1_num,
    output wire        queue1_ready,
    output wire [63:0] queue1_desc,
    output wire [63:0] queue1_driver,
    output wire [63:0] queue1_device,
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
   /* Device config space (offset 0x100). virtio-blk: capacity is a 64-bit LE
    * field of 512-byte sectors at config offset 0. Other devices leave
    * CONFIG_CAPACITY_SECTORS = 0 and never read config (no F_MAC/F_STATUS/etc). */
   localparam [11:0] REG_CONFIG_CAP_LOW   = 12'h100;
   localparam [11:0] REG_CONFIG_CAP_HIGH  = 12'h104;

   reg [31:0] device_features_sel;
   reg [31:0] driver_features_sel;
   reg [31:0] queue_sel;
   reg [1:0]  interrupt_status;
   reg [31:0] queue0_num_q;
   reg        queue0_ready_q;
   reg [63:0] queue0_desc_q;
   reg [63:0] queue0_driver_q;
   reg [63:0] queue0_device_q;
   reg [31:0] queue1_num_q;
   reg        queue1_ready_q;
   reg [63:0] queue1_desc_q;
   reg [63:0] queue1_driver_q;
   reg [63:0] queue1_device_q;

   wire [11:0] reg_addr = address & 12'hffc;
   wire       write_word = write && &byteenable;
   wire       queue0_selected = queue_sel == 32'd0 && QUEUE_COUNT >= 32'd1;
   wire       queue1_selected = queue_sel == 32'd1 && QUEUE_COUNT >= 32'd2;
   wire       active_queue_selected = queue0_selected || queue1_selected;

   assign irq = interrupt_status != 2'd0;
   assign queue0_num = queue0_num_q;
   assign queue0_ready = queue0_ready_q;
   assign queue0_desc = queue0_desc_q;
   assign queue0_driver = queue0_driver_q;
   assign queue0_device = queue0_device_q;
   assign queue1_num = queue1_num_q;
   assign queue1_ready = queue1_ready_q;
   assign queue1_desc = queue1_desc_q;
   assign queue1_driver = queue1_driver_q;
   assign queue1_device = queue1_device_q;
   assign queue_num = !active_queue_selected ? 32'd0 :
                      queue1_selected ? queue1_num_q : queue0_num_q;
   assign queue_ready = active_queue_selected &&
                        (queue1_selected ? queue1_ready_q : queue0_ready_q);
   assign queue_desc = !active_queue_selected ? 64'd0 :
                       queue1_selected ? queue1_desc_q : queue0_desc_q;
   assign queue_driver = !active_queue_selected ? 64'd0 :
                         queue1_selected ? queue1_driver_q : queue0_driver_q;
   assign queue_device = !active_queue_selected ? 64'd0 :
                         queue1_selected ? queue1_device_q : queue0_device_q;

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
           REG_QUEUE_NUM_MAX:    read_data = active_queue_selected ? QUEUE_NUM_MAX : 32'd0;
           REG_QUEUE_NUM:        read_data = queue_num;
           REG_QUEUE_READY:      read_data = {31'd0, queue_ready};
           REG_INTERRUPT_STATUS: read_data = {30'd0, interrupt_status};
           REG_STATUS:           read_data = {24'd0, device_status};
           REG_QUEUE_DESC_LOW:   read_data = queue_desc[31:0];
           REG_QUEUE_DESC_HIGH:  read_data = queue_desc[63:32];
           REG_QUEUE_AVAIL_LOW:  read_data = queue_driver[31:0];
           REG_QUEUE_AVAIL_HIGH: read_data = queue_driver[63:32];
           REG_QUEUE_USED_LOW:   read_data = queue_device[31:0];
           REG_QUEUE_USED_HIGH:  read_data = queue_device[63:32];
           REG_CONFIG_GEN:       read_data = 32'd0;
           REG_CONFIG_CAP_LOW:   read_data = CONFIG_CAPACITY_SECTORS;
           REG_CONFIG_CAP_HIGH:  read_data = 32'd0;
           default:              read_data = 32'd0;
         endcase
      end
   end

   always @(posedge clock) begin
      if (reset) begin
         device_features_sel <= 32'd0;
         driver_features_sel <= 32'd0;
         driver_features_0 <= 32'd0;
         driver_features_1 <= 32'd0;
         queue_sel <= 32'd0;
         queue0_num_q <= 32'd0;
         queue0_ready_q <= 1'b0;
         queue0_desc_q <= 64'd0;
         queue0_driver_q <= 64'd0;
         queue0_device_q <= 64'd0;
         queue1_num_q <= 32'd0;
         queue1_ready_q <= 1'b0;
         queue1_desc_q <= 64'd0;
         queue1_driver_q <= 64'd0;
         queue1_device_q <= 64'd0;
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

         if (write_word) begin
            case (reg_addr)
              REG_DEVICE_FEAT_SEL: device_features_sel <= write_data;
              REG_DRIVER_FEAT_SEL: driver_features_sel <= write_data;
              REG_DRIVER_FEATURES: begin
                 if (driver_features_sel == 32'd0)
                    driver_features_0 <= write_data;
                 else if (driver_features_sel == 32'd1)
                    driver_features_1 <= write_data;
              end
              REG_QUEUE_SEL: queue_sel <= write_data;
              REG_QUEUE_NUM: begin
                 if (queue0_selected)
                    queue0_num_q <= write_data;
                 else if (queue1_selected)
                    queue1_num_q <= write_data;
              end
              REG_QUEUE_READY: begin
                 if (queue0_selected)
                    queue0_ready_q <= write_data[0];
                 else if (queue1_selected)
                    queue1_ready_q <= write_data[0];
              end
              REG_QUEUE_NOTIFY: begin
                 queue_notify_pulse <= 1'b1;
                 queue_notify_value <= write_data;
              end
              REG_INTERRUPT_ACK: interrupt_status <= interrupt_status & ~write_data[1:0];
              REG_STATUS: begin
                 device_status <= write_data[7:0];
                 if (write_data[7:0] == 8'd0) begin
                    driver_features_0 <= 32'd0;
                    driver_features_1 <= 32'd0;
                    queue_sel <= 32'd0;
                    queue0_num_q <= 32'd0;
                    queue0_ready_q <= 1'b0;
                    queue0_desc_q <= 64'd0;
                    queue0_driver_q <= 64'd0;
                    queue0_device_q <= 64'd0;
                    queue1_num_q <= 32'd0;
                    queue1_ready_q <= 1'b0;
                    queue1_desc_q <= 64'd0;
                    queue1_driver_q <= 64'd0;
                    queue1_device_q <= 64'd0;
                    interrupt_status <= 2'd0;
                 end
              end
              REG_QUEUE_DESC_LOW: begin
                 if (queue0_selected)
                    queue0_desc_q[31:0] <= write_data;
                 else if (queue1_selected)
                    queue1_desc_q[31:0] <= write_data;
              end
              REG_QUEUE_DESC_HIGH: begin
                 if (queue0_selected)
                    queue0_desc_q[63:32] <= write_data;
                 else if (queue1_selected)
                    queue1_desc_q[63:32] <= write_data;
              end
              REG_QUEUE_AVAIL_LOW: begin
                 if (queue0_selected)
                    queue0_driver_q[31:0] <= write_data;
                 else if (queue1_selected)
                    queue1_driver_q[31:0] <= write_data;
              end
              REG_QUEUE_AVAIL_HIGH: begin
                 if (queue0_selected)
                    queue0_driver_q[63:32] <= write_data;
                 else if (queue1_selected)
                    queue1_driver_q[63:32] <= write_data;
              end
              REG_QUEUE_USED_LOW: begin
                 if (queue0_selected)
                    queue0_device_q[31:0] <= write_data;
                 else if (queue1_selected)
                    queue1_device_q[31:0] <= write_data;
              end
              REG_QUEUE_USED_HIGH: begin
                 if (queue0_selected)
                    queue0_device_q[63:32] <= write_data;
                 else if (queue1_selected)
                    queue1_device_q[63:32] <= write_data;
              end
              default: begin
              end
            endcase
         end
      end
   end
endmodule

`default_nettype wire
