`timescale 1ns / 1ps
`default_nettype none

// virtio_blk_dma_top -- virtio_blk with its own dma_master, for virtio_blk_tb (a bench's top: on the
// board the devices share one dma_master in the shell, a port each).
module virtio_blk_dma_top #(
    parameter [31:0] QUEUE_SIZE       = 32'd8,        /* virtqueue depth, power of two */
    parameter [15:0] SD_SLOW_HALF     = 16'd416,      /* sd_spi_host SCK dividers */
    parameter [15:0] SD_FAST_HALF     = 16'd12,       /*   (~400 kHz / ~12 MHz) */
    parameter [15:0] SD_INIT_TICKS    = 16'd10        /*   init idle bytes */
)(
    input  wire        clock,
    input  wire        reset,

    input  wire        queue_notify_pulse,
    input  wire [31:0] queue_notify_value,
    input  wire [31:0] queue_num,
    input  wire        queue_ready,
    input  wire [63:0] queue_desc,
    input  wire [63:0] queue_driver,
    input  wire [63:0] queue_device,
    input  wire [ 7:0] device_status,
    output wire         used_buffer_interrupt,
    output wire [31:0] capacity_sectors,    // from the card's CSD (for virtio config)

    // Runtime SD transfer-clock override (SCK half-period; 0 = compile default).
    input  wire [15:0] sd_fast_half,

    // Debug readout (parent muxes into an MMIO overlay).
    input  wire [ 1:0] debug_sel,
    output wire [31:0] debug_word,
    // Always-on hang-diagnosis bus for the device-side ILA (ui_clk):
    //   [21:16]=state  [15:9]=sd_spi_state  [8]=sd_busy [7]=sd_done [6]=sd_error [5]=sd_ready
    //   [4]=dma_rsp_error  [3:0]=sectors_left[3:0]
    output wire [21:0] dbg,

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
    output wire        m_axi_rready,

    // SD card pins (SPI mode).
    output wire        sd_sck,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output wire        sd_cs_n
);
   wire        cv, cr, cw, cs, rv, re;
   wire [31:0] ca;  wire [63:0] cd, rd;  wire [7:0] cm;
   virtio_blk #(.QUEUE_SIZE(QUEUE_SIZE), .SD_SLOW_HALF(SD_SLOW_HALF), .SD_FAST_HALF(SD_FAST_HALF), .SD_INIT_TICKS(SD_INIT_TICKS)) u_blk (
      .clock(clock), .reset(reset), .queue_notify_pulse(queue_notify_pulse), .queue_notify_value(queue_notify_value),
      .queue_num(queue_num), .queue_ready(queue_ready), .queue_desc(queue_desc), .queue_driver(queue_driver),
      .queue_device(queue_device), .device_status(device_status), .used_buffer_interrupt(used_buffer_interrupt), .capacity_sectors(capacity_sectors),
      .sd_fast_half(sd_fast_half), .debug_sel(debug_sel), .debug_word(debug_word), .dbg(dbg),
      .sd_sck(sd_sck), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_cs_n(sd_cs_n),
      .dma_cmd_valid(cv), .dma_cmd_ready(cr), .dma_cmd_write(cw), .dma_cmd_stream(cs),
      .dma_cmd_addr(ca), .dma_cmd_wdata(cd), .dma_cmd_wstrb(cm),
      .dma_rsp_valid(rv), .dma_rsp_rdata(rd), .dma_rsp_error(re));
   dma_master u_dma (.clock(clock), .reset(reset),
      .cmd_valid(cv), .cmd_ready(cr), .cmd_write(cw), .cmd_stream(cs),
      .cmd_addr(ca), .cmd_wdata(cd), .cmd_wstrb(cm),
      .rsp_valid(rv), .rsp_rdata(rd), .rsp_error(re),
      .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
      .m_axi_awburst(m_axi_awburst), .m_axi_awlock(m_axi_awlock), .m_axi_awcache(m_axi_awcache), .m_axi_awprot(m_axi_awprot),
      .m_axi_awqos(m_axi_awqos), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready), .m_axi_wdata(m_axi_wdata),
      .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
      .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
      .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
      .m_axi_arburst(m_axi_arburst), .m_axi_arlock(m_axi_arlock), .m_axi_arcache(m_axi_arcache), .m_axi_arprot(m_axi_arprot),
      .m_axi_arqos(m_axi_arqos), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready), .m_axi_rid(m_axi_rid),
      .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid),
      .m_axi_rready(m_axi_rready));
endmodule

`default_nettype wire
