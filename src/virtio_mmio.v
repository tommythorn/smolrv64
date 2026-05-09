`timescale 1ns / 1ps
`default_nettype none

module virtio_mmio #(
    parameter [31:0] DEVICE_ID = 32'd0,
    parameter [31:0] VENDOR_ID = 32'h736d_6f6c,
    parameter [31:0] DEVICE_FEATURES_0 = 32'd0,
    parameter [31:0] DEVICE_FEATURES_1 = 32'h0000_0001,
    parameter [31:0] QUEUE_NUM_MAX = 32'd8,
    parameter [31:0] CONFIG_WORD_0 = 32'd0,
    parameter [31:0] CONFIG_WORD_1 = 32'd0,
    parameter [31:0] CONFIG_WORD_2 = 32'd0,
    parameter [31:0] CONFIG_WORD_3 = 32'd0,
    parameter [31:0] CONFIG_WORD_4 = 32'd0,
    parameter [31:0] CONFIG_WORD_5 = 32'd0,
    parameter [31:0] CONFIG_WORD_6 = 32'd0,
    parameter [31:0] CONFIG_WORD_7 = 32'd0
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
    output reg  [31:0] queue_num,
    output reg         queue_ready,
    output reg  [63:0] queue_desc,
    output reg  [63:0] queue_driver,
    output reg  [63:0] queue_device,
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

   wire [11:0] reg_addr = address & 12'hffc;
   wire       write_word = write && &byteenable;
   wire       active_queue_selected = queue_sel == 32'd0;

   assign irq = interrupt_status != 2'd0;

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
           REG_QUEUE_NUM:        read_data = active_queue_selected ? queue_num : 32'd0;
           REG_QUEUE_READY:      read_data = active_queue_selected ? {31'd0, queue_ready} : 32'd0;
           REG_INTERRUPT_STATUS: read_data = {30'd0, interrupt_status};
           REG_STATUS:           read_data = {24'd0, device_status};
           REG_QUEUE_DESC_LOW:   read_data = active_queue_selected ? queue_desc[31:0] : 32'd0;
           REG_QUEUE_DESC_HIGH:  read_data = active_queue_selected ? queue_desc[63:32] : 32'd0;
           REG_QUEUE_AVAIL_LOW:  read_data = active_queue_selected ? queue_driver[31:0] : 32'd0;
           REG_QUEUE_AVAIL_HIGH: read_data = active_queue_selected ? queue_driver[63:32] : 32'd0;
           REG_QUEUE_USED_LOW:   read_data = active_queue_selected ? queue_device[31:0] : 32'd0;
           REG_QUEUE_USED_HIGH:  read_data = active_queue_selected ? queue_device[63:32] : 32'd0;
           REG_CONFIG_GEN:       read_data = 32'd0;
           default: begin
              case (reg_addr)
                12'h100: read_data = CONFIG_WORD_0;
                12'h104: read_data = CONFIG_WORD_1;
                12'h108: read_data = CONFIG_WORD_2;
                12'h10c: read_data = CONFIG_WORD_3;
                12'h110: read_data = CONFIG_WORD_4;
                12'h114: read_data = CONFIG_WORD_5;
                12'h118: read_data = CONFIG_WORD_6;
                12'h11c: read_data = CONFIG_WORD_7;
                default: read_data = 32'd0;
              endcase
           end
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
         queue_num <= 32'd0;
         queue_ready <= 1'b0;
         queue_desc <= 64'd0;
         queue_driver <= 64'd0;
         queue_device <= 64'd0;
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
                 if (active_queue_selected)
                    queue_num <= write_data;
              end
              REG_QUEUE_READY: begin
                 if (active_queue_selected)
                    queue_ready <= write_data[0];
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
                    queue_num <= 32'd0;
                    queue_ready <= 1'b0;
                    queue_desc <= 64'd0;
                    queue_driver <= 64'd0;
                    queue_device <= 64'd0;
                    interrupt_status <= 2'd0;
                 end
              end
              REG_QUEUE_DESC_LOW: begin
                 if (active_queue_selected)
                    queue_desc[31:0] <= write_data;
              end
              REG_QUEUE_DESC_HIGH: begin
                 if (active_queue_selected)
                    queue_desc[63:32] <= write_data;
              end
              REG_QUEUE_AVAIL_LOW: begin
                 if (active_queue_selected)
                    queue_driver[31:0] <= write_data;
              end
              REG_QUEUE_AVAIL_HIGH: begin
                 if (active_queue_selected)
                    queue_driver[63:32] <= write_data;
              end
              REG_QUEUE_USED_LOW: begin
                 if (active_queue_selected)
                    queue_device[31:0] <= write_data;
              end
              REG_QUEUE_USED_HIGH: begin
                 if (active_queue_selected)
                    queue_device[63:32] <= write_data;
              end
              default: begin
              end
            endcase
         end
      end
   end
endmodule

`default_nettype wire
