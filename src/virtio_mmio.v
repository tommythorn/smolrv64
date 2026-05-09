`default_nettype none

module virtio_mmio #(
    parameter [31:0] DEVICE_ID = 32'd0,
    parameter [31:0] VENDOR_ID = 32'h736d_6f6c,
    parameter [31:0] DEVICE_FEATURES_0 = 32'd0,
    parameter [31:0] DEVICE_FEATURES_1 = 32'h0000_0001,
    parameter [31:0] QUEUE_NUM_MAX = 32'd8
) (
    input  wire        clock,
    input  wire        reset,

    input  wire [ 7:0] address,
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

   localparam [7:0] REG_MAGIC_VALUE      = 8'h00;
   localparam [7:0] REG_VERSION          = 8'h04;
   localparam [7:0] REG_DEVICE_ID        = 8'h08;
   localparam [7:0] REG_VENDOR_ID        = 8'h0c;
   localparam [7:0] REG_DEVICE_FEATURES  = 8'h10;
   localparam [7:0] REG_DEVICE_FEAT_SEL  = 8'h14;
   localparam [7:0] REG_DRIVER_FEATURES  = 8'h20;
   localparam [7:0] REG_DRIVER_FEAT_SEL  = 8'h24;
   localparam [7:0] REG_QUEUE_SEL        = 8'h30;
   localparam [7:0] REG_QUEUE_NUM_MAX    = 8'h34;
   localparam [7:0] REG_QUEUE_NUM        = 8'h38;
   localparam [7:0] REG_QUEUE_READY      = 8'h44;
   localparam [7:0] REG_QUEUE_NOTIFY     = 8'h50;
   localparam [7:0] REG_INTERRUPT_STATUS = 8'h60;
   localparam [7:0] REG_INTERRUPT_ACK    = 8'h64;
   localparam [7:0] REG_STATUS           = 8'h70;
   localparam [7:0] REG_QUEUE_DESC_LOW   = 8'h80;
   localparam [7:0] REG_QUEUE_DESC_HIGH  = 8'h84;
   localparam [7:0] REG_QUEUE_AVAIL_LOW  = 8'h90;
   localparam [7:0] REG_QUEUE_AVAIL_HIGH = 8'h94;
   localparam [7:0] REG_QUEUE_USED_LOW   = 8'ha0;
   localparam [7:0] REG_QUEUE_USED_HIGH  = 8'ha4;
   localparam [7:0] REG_CONFIG_GEN       = 8'hfc;

   reg [31:0] device_features_sel;
   reg [31:0] driver_features_sel;
   reg [31:0] queue_sel;
   reg [1:0]  interrupt_status;

   wire [7:0] reg_addr = address & 8'hfc;
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
