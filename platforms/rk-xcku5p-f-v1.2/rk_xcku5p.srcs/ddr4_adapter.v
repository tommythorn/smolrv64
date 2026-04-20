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
    input  wire         dram_abandon_read,  // pulse: CPU abandoned the in-flight read

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

    localparam IDLE     = 3'd0;
    localparam WR_CMD   = 3'd1;
    localparam RD_CMD   = 3'd2;
    localparam RD_WAIT  = 3'd3;
    localparam RD_DRAIN = 3'd4;  // abandoned read: swallow MIG response, then IDLE

    reg [2:0] state = IDLE;
    // Tracks whether the write data FIFO entry has been accepted by the MIG.
    // Prevents re-presenting the same data when app_rdy and app_wdf_rdy go high
    // in different cycles (e.g. app_wdf_rdy=1 first during DDR4 refresh when
    // app_rdy=0).  Without this flag the data would be written to the FIFO
    // twice, causing every subsequent write to use the previous write's data.
    reg        wdf_done = 0;

    assign dram_write_ready = (state == IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= IDLE;
            app_en             <= 0;
            app_wdf_wren       <= 0;
            app_wdf_end        <= 0;
            dram_readdatavalid <= 0;
            wdf_done           <= 0;
        end else begin
            // Default: deassert one-cycle pulses
            app_en             <= 0;
            app_wdf_wren       <= 0;
            dram_readdatavalid <= 0;

            case (state)
                IDLE: begin
                    if (dram_write) begin
                        // Latch write command and data
                        // app_addr is a byte address; dram_burst_addr is the 32-byte burst
                        // index (phys_addr[30:5]), so multiply by 32 (shift left 5).
                        // Use bits [23:0] of dram_burst_addr for the 29-bit MIG address:
                        //   {dram_burst_addr[23:0], 5'b0} = phys_addr[28:0]
                        // (bits [25:24] = phys_addr[30:29] = 0 for valid DDR4 range 0x80000000-0x9FFFFFFF)
                        app_addr     <= {dram_burst_addr[23:0], 5'b0};
                        app_cmd      <= 3'b000;  // WRITE
                        app_wdf_data <= dram_writedata;
                        app_wdf_mask <= dram_byte_mask;  // 1=mask (same convention as MIG)
                        app_wdf_end  <= 1;
                        // Assert command and write data; wait for MIG acceptance in WR_CMD.
                        // Do NOT shortcut to IDLE even if app_rdy && app_wdf_rdy look ready here:
                        // app_en/app_wdf_wren are registered (<=) so they only become 1 NEXT cycle.
                        // The acceptance check must happen in WR_CMD where app_en is already high.
                        app_en       <= 1;
                        app_wdf_wren <= 1;
                        wdf_done     <= 0;
                        state        <= WR_CMD;
                    end else if (dram_read) begin
                        // Latch read command; always go through RD_CMD for proper handshake.
                        // app_en is registered (<=) so it becomes 1 NEXT cycle (in RD_CMD).
                        // RD_CMD checks app_rdy while app_en is already high — that is the
                        // correct simultaneous-assertion required by the MIG native interface.
                        app_addr <= {dram_burst_addr[23:0], 5'b0};  // byte address = burst_idx * 32
                        app_cmd  <= 3'b001;  // READ
                        app_en   <= 1;
                        state <= RD_CMD;
                    end
                end

                WR_CMD: begin
                    // Write data FIFO: present data until MIG accepts it (app_wdf_rdy=1
                    // while app_wdf_wren=1).  Once accepted, stop presenting to prevent
                    // a duplicate FIFO entry if app_rdy is still 0 (e.g. DDR4 refresh).
                    if (!wdf_done) begin
                        if (app_wdf_rdy) begin
                            wdf_done <= 1;  // data accepted this cycle; don't re-present
                        end else begin
                            app_wdf_wren <= 1;  // FIFO not ready; re-present next cycle
                        end
                    end
                    // Write command: keep presenting until MIG accepts it.
                    if (app_rdy) begin
                        state <= IDLE;
                    end else begin
                        app_en <= 1;
                    end
                end

                RD_CMD: begin
                    // Re-present read command until MIG accepts it.
                    // app_en is already 1 (from IDLE or previous RD_CMD).
                    // On the acceptance cycle, stop asserting app_en so no duplicate
                    // command is issued on the first cycle of RD_WAIT.
                    if (app_rdy) begin
                        // app_addr, app_cmd retain values; default deasserts app_en
                        state <= RD_WAIT;
                    end else begin
                        app_en <= 1;
                    end
                end

                RD_WAIT: begin
                    // Wait for read data to come back from DRAM
                    if (app_rd_data_valid) begin
                        dram_readdata      <= app_rd_data;
                        dram_readdatavalid <= 1;
                        state              <= IDLE;
                    end else if (dram_abandon_read) begin
                        // CPU gave up (bus timeout).  Stay quiet until the
                        // MIG's eventual response arrives, then silently
                        // drop it and return to IDLE.  Crucially, do NOT
                        // pulse dram_readdatavalid — the CPU has moved on
                        // and would latch stale data for a different fetch.
                        state <= RD_DRAIN;
                    end
                end

                RD_DRAIN: begin
                    // Swallow the abandoned read's late MIG response.
                    if (app_rd_data_valid)
                        state <= IDLE;
                end
            endcase
        end
    end

endmodule
