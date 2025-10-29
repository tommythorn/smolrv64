`timescale 1ns/1ps
`default_nettype none

// Bridge between smolrv64's 256-bit DRAM bus and the DDR4 MIG native app interface.
// Runs in c0_ddr4_ui_clk domain (same domain as CPU).
//
// DDR4 MIG parameters: APP_DATA_WIDTH=256, APP_ADDR_WIDTH=29, APP_MASK_WIDTH=32
// Each burst address = one 32-byte block.
//
// Write mask convention (both CPU and MIG): 1 = mask out (do NOT write this byte).

module ddr4_adapter(
    input  wire         clk,
    input  wire         rst_n,

    // CPU DRAM bus (from smolrv64)
    input  wire [25:0]  dram_burst_addr,    // 32-byte burst address (phys[30:5])
    input  wire         dram_read,          // pulse: issue a read this cycle
    input  wire         dram_write,         // pulse: issue a write this cycle
    input  wire [255:0] dram_writedata,     // write data (full 256-bit burst)
    input  wire [31:0]  dram_byte_mask,     // 1=mask out (don't write), MIG convention
    output reg          dram_readdatavalid, // pulse: dram_readdata valid this cycle
    output reg  [255:0] dram_readdata,      // read data
    output wire         dram_write_ready,   // 1 = adapter is IDLE, can accept a write

    // DDR4 native app interface (to ddr4_0 IP)
    output reg  [28:0]  app_addr,
    output reg  [2:0]   app_cmd,
    output reg          app_en,
    input  wire         app_rdy,
    output reg  [255:0] app_wdf_data,
    output reg          app_wdf_end,
    output reg  [31:0]  app_wdf_mask,
    output reg          app_wdf_wren,
    input  wire         app_wdf_rdy,
    input  wire [255:0] app_rd_data,
    input  wire         app_rd_data_end,    // unused (single-burst reads only)
    input  wire         app_rd_data_valid
);

    localparam IDLE    = 2'd0;
    localparam WR_CMD  = 2'd1;
    localparam RD_CMD  = 2'd2;
    localparam RD_WAIT = 2'd3;

    reg [1:0] state = IDLE;

    assign dram_write_ready = (state == IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= IDLE;
            app_en             <= 0;
            app_wdf_wren       <= 0;
            app_wdf_end        <= 0;
            dram_readdatavalid <= 0;
        end else begin
            // Default: deassert one-cycle pulses
            app_en             <= 0;
            app_wdf_wren       <= 0;
            dram_readdatavalid <= 0;

            case (state)
                IDLE: begin
                    if (dram_write) begin
                        // Latch write command and data
                        app_addr     <= {3'b0, dram_burst_addr};
                        app_cmd      <= 3'b000;  // WRITE
                        app_wdf_data <= dram_writedata;
                        app_wdf_mask <= dram_byte_mask;  // 1=mask (same convention as MIG)
                        app_wdf_end  <= 1;
                        // Assert command and write data simultaneously
                        app_en       <= 1;
                        app_wdf_wren <= 1;
                        if (app_rdy && app_wdf_rdy)
                            state <= IDLE;   // accepted in one cycle
                        else
                            state <= WR_CMD; // wait for acceptance
                    end else if (dram_read) begin
                        // Latch read command
                        app_addr <= {3'b0, dram_burst_addr};
                        app_cmd  <= 3'b001;  // READ
                        app_en   <= 1;
                        if (app_rdy)
                            state <= RD_WAIT;
                        else
                            state <= RD_CMD;
                    end
                end

                WR_CMD: begin
                    // Re-present command and write data until MIG accepts both
                    app_en       <= 1;
                    app_wdf_wren <= 1;
                    // app_addr, app_cmd, app_wdf_data, app_wdf_mask, app_wdf_end
                    // retain their values from IDLE
                    if (app_rdy && app_wdf_rdy)
                        state <= IDLE;
                end

                RD_CMD: begin
                    // Re-present read command until MIG accepts it
                    app_en <= 1;
                    // app_addr, app_cmd retain values from IDLE
                    if (app_rdy)
                        state <= RD_WAIT;
                end

                RD_WAIT: begin
                    // Wait for read data to come back from DRAM
                    if (app_rd_data_valid) begin
                        dram_readdata      <= app_rd_data;
                        dram_readdatavalid <= 1;
                        state              <= IDLE;
                    end
                end
            endcase
        end
    end

endmodule
