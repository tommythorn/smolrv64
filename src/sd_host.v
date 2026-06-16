`timescale 1ns / 1ps
`default_nettype none

// Native-mode SD host controller (conservative first cut: SDHC/SDXC, 1-bit
// DAT, default speed, single block).
//
// Owns the CMD/DAT PHY and the protocol, so the rest of the system only sees a
// block interface:
//
//   - issue a request:  pulse `req_valid` with `req_write`/`req_sector`
//   - completion:        one-cycle `done` (or `error`); `busy` high meanwhile
//   - data exchange:     a 512-byte block buffer (64 x 64-bit LE words),
//                        addressed by the caller while !busy (read it after a
//                        READ, fill it before a WRITE)
//   - `ready` rises once the init handshake (CMD0/8/55+41/2/3/7) completes.
//
// The SD clock free-runs out of reset: slow (<=400 kHz) until init done, then
// fast (default speed) for transfers. Host outputs change on the SD-clock
// falling edge; inputs are sampled on the rising edge.
//
// Pins are split for IOBUF instantiation by the parent (t=1 => released/input):
//   cmd_o/cmd_t   bidirectional CMD line
//   dat_o/dat_t   bidirectional DAT lines (1-bit mode drives DAT0 only)
//
// SDHC assumption: block addressing (CMD17/24 arg is the sector number) and a
// fixed 512-byte block, so no SET_BLOCKLEN and no byte/block conversion.
module sd_host #(
    parameter [15:0] SLOW_HALF  = 16'd124,    // SD-clk half-period in core
    parameter [15:0] FAST_HALF  = 16'd1,      //   clocks, minus one
    parameter [15:0] INIT_TICKS = 16'd200,    // >=74 SD clocks, CMD released
    parameter [23:0] TIMEOUT    = 24'd2000000 // per-phase SD-tick watchdog
) (
    input  wire        clock,
    input  wire        reset,

    input  wire        req_valid,
    input  wire        req_write,    // 0 = read block, 1 = write block
    input  wire [31:0] req_sector,
    output wire        busy,
    output reg         done,
    output reg         error,
    output reg         ready,
    output reg  [31:0] capacity_sectors,   // from CSD, valid once ready

    // Debug observability (state + last response + command/response counters).
    output wire [15:0] dbg,
    output wire [47:0] dbg_resp,
    output wire [31:0] dbg_diag,
    output wire [31:0] dbg_diag2,   // live line levels + sticky toggle flags

    // Block buffer access (valid only while !busy).
    input  wire [ 5:0] buf_addr,
    input  wire [63:0] buf_wdata,
    input  wire        buf_we,
    output wire [63:0] buf_rdata,

    output reg         sd_clk,
    input  wire        cmd_i,
    output reg         cmd_o,
    output reg         cmd_t,
    input  wire [ 3:0] dat_i,
    output reg  [ 3:0] dat_o,
    output reg  [ 3:0] dat_t
);
   function [63:0] byteswap;
      input [63:0] w;
      byteswap = {w[7:0], w[15:8], w[23:16], w[31:24],
                  w[39:32], w[47:40], w[55:48], w[63:56]};
   endfunction
   function [6:0] crc7_bit;
      input [6:0] crc; input b; reg inv;
      begin inv = b ^ crc[6]; crc7_bit = {crc[5:0],1'b0} ^ (inv ? 7'h09 : 7'h00); end
   endfunction
   function [15:0] crc16_bit;
      input [15:0] crc; input b; reg inv;
      begin inv = b ^ crc[15]; crc16_bit = {crc[14:0],1'b0} ^ (inv ? 16'h1021 : 16'h0000); end
   endfunction
   function [6:0] crc7_cmd;       // CRC7 over the 40-bit command prefix
      input [39:0] data; integer i; reg [6:0] c;
      begin c = 7'd0; for (i=39;i>=0;i=i-1) c = crc7_bit(c, data[i]); crc7_cmd = c; end
   endfunction

   // ---------------------------------------------------------------------
   // 512-byte block buffer: 64 x 64-bit LE words, sync write / async read.
   // Engine owns it while busy; caller owns it while idle.
   // ---------------------------------------------------------------------
   reg [63:0] blockbuf [0:63];
   reg [ 5:0] eng_addr;
   reg [63:0] eng_wdata;
   reg        eng_we;
   wire [ 5:0] wr_addr  = busy ? eng_addr  : buf_addr;
   wire [63:0] wr_data  = busy ? eng_wdata : buf_wdata;
   wire        wr_en    = busy ? eng_we    : buf_we;
   always @(posedge clock) if (wr_en) blockbuf[wr_addr] <= wr_data;
   assign buf_rdata = blockbuf[buf_addr];

   // ---------------------------------------------------------------------
   // Free-running SD clock + edge ticks.
   // ---------------------------------------------------------------------
   reg [15:0] div_cnt;
   wire [15:0] half = ready ? FAST_HALF : SLOW_HALF;
   reg edge_rise, edge_fall;
   always @(posedge clock) begin
      edge_rise <= 1'b0;
      edge_fall <= 1'b0;
      if (reset) begin
         div_cnt <= 16'd0;
         sd_clk  <= 1'b0;
      end else if (div_cnt >= half) begin
         div_cnt   <= 16'd0;
         sd_clk    <= ~sd_clk;
         edge_rise <= ~sd_clk;   // sd_clk about to become 1
         edge_fall <=  sd_clk;   // sd_clk about to become 0
      end else
         div_cnt <= div_cnt + 16'd1;
   end

   localparam [5:0] CMD0=6'd0, CMD2=6'd2, CMD3=6'd3, CMD7=6'd7, CMD8=6'd8,
                    CMD9=6'd9, CMD17=6'd17, CMD24=6'd24, CMD41=6'd41, CMD55=6'd55;

   // ---------------------------------------------------------------------
   // Command engine: send a 48-bit frame, then shift in a response.
   // Always ends by pulsing cmd_done; cmd_failed flags a response timeout.
   // ---------------------------------------------------------------------
   localparam [2:0] C_IDLE=3'd0, C_SEND=3'd1, C_RGAP=3'd2, C_RESP=3'd3,
                    C_BUSY=3'd4, C_DONE=3'd5;
   reg [2:0]   cstate;
   reg         cmd_go, cmd_busy_wait, cmd_done, cmd_failed;
   reg [5:0]   cmd_index;
   reg [31:0]  cmd_arg;
   reg [7:0]   cmd_resp_len;       // 0, 48 or 136
   reg [135:0] cmd_resp;
   reg [47:0]  c_shift;
   reg [7:0]   c_cnt;
   reg [23:0]  c_to;
   reg [4:0]   c_gap;           // Ncc inter-command spacing counter (SD clocks)
   reg         c_pending;       // a command is waiting out the Ncc gap
   localparam [4:0] NCC = 5'd16;
   reg [7:0]   dbg_cmd_count;   // commands issued
   reg [7:0]   dbg_resp_count;  // responses whose start bit was seen
   reg [5:0]   dbg_last_cmd;    // index of the last command issued
   reg         dbg_resp_seen;   // sticky: any response start bit ever seen
   reg         dbg_cmd_low, dbg_cmd_high;   // sticky: cmd_i ever read 0 / 1
   reg         dbg_dat_low, dbg_dat_high;   // sticky: dat_i[0] ever read 0 / 1

   always @(posedge clock) begin
      cmd_done <= 1'b0;
      // Sticky line-activity monitors (every clock, any state).
      if (!cmd_i)    dbg_cmd_low  <= 1'b1;
      if ( cmd_i)    dbg_cmd_high <= 1'b1;
      if (!dat_i[0]) dbg_dat_low  <= 1'b1;
      if ( dat_i[0]) dbg_dat_high <= 1'b1;
      if (reset) begin
         cstate <= C_IDLE; cmd_t <= 1'b1; cmd_o <= 1'b1; cmd_failed <= 1'b0;
         dbg_cmd_count <= 8'd0; dbg_resp_count <= 8'd0;
         dbg_last_cmd <= 6'd0; dbg_resp_seen <= 1'b0;
         dbg_cmd_low <= 1'b0; dbg_cmd_high <= 1'b0;
         dbg_dat_low <= 1'b0; dbg_dat_high <= 1'b0;
         c_pending <= 1'b0; c_gap <= 5'd0;
      end else case (cstate)
        // Accept a command, then hold Ncc (>=8) SD clocks before transmitting
        // it (inter-command spacing). The data phase does not pass through here.
        C_IDLE: begin
           cmd_t <= 1'b1;
           if (cmd_go) begin c_pending <= 1'b1; c_gap <= 5'd0; end
           if (c_pending) begin
              if (c_gap >= NCC) begin
                 c_shift <= {1'b0, 1'b1, cmd_index, cmd_arg,
                             crc7_cmd({1'b0, 1'b1, cmd_index, cmd_arg}), 1'b1};
                 c_cnt      <= 8'd47;
                 c_to       <= TIMEOUT;
                 cmd_failed <= 1'b0;
                 c_pending  <= 1'b0;
                 dbg_last_cmd  <= cmd_index;
                 dbg_cmd_count <= dbg_cmd_count + 8'd1;
                 cstate     <= C_SEND;
              end else if (edge_rise)
                 c_gap <= c_gap + 5'd1;
           end
        end
        C_SEND: if (edge_fall) begin
           cmd_t   <= 1'b0;
           cmd_o   <= c_shift[47];
           c_shift <= {c_shift[46:0], 1'b1};
           if (c_cnt == 8'd0) begin
              if (cmd_resp_len == 8'd0) cstate <= C_DONE;
              else begin cmd_t <= 1'b1; cstate <= C_RGAP; end
           end else
              c_cnt <= c_cnt - 8'd1;
        end
        C_RGAP: begin                       // release CMD, await start bit
           cmd_t <= 1'b1;
           if (edge_fall) begin             // sample card output mid-bit (settled)
              if (!cmd_i) begin
                 cmd_resp <= 136'd0;         // start bit (=0) implied
                 c_cnt    <= cmd_resp_len - 8'd1;
                 dbg_resp_count <= dbg_resp_count + 8'd1;
                 dbg_resp_seen  <= 1'b1;
                 cstate   <= C_RESP;
              end else if (c_to == 24'd0) begin
                 cmd_failed <= 1'b1; cstate <= C_DONE;
              end else
                 c_to <= c_to - 24'd1;
           end
        end
        C_RESP: if (edge_fall) begin
           cmd_resp <= {cmd_resp[134:0], cmd_i};
           if (c_cnt == 8'd1) cstate <= cmd_busy_wait ? C_BUSY : C_DONE;
           else c_cnt <= c_cnt - 8'd1;
        end
        C_BUSY: if (edge_fall) begin         // R1b: DAT0 low while programming
           if (dat_i[0]) cstate <= C_DONE;
        end
        C_DONE: begin cmd_done <= 1'b1; cstate <= C_IDLE; end
        default: cstate <= C_IDLE;
      endcase
   end

   // ---------------------------------------------------------------------
   // Data engine: read or write one 512-byte block on DAT0.
   // Always ends by pulsing dat_done; dat_failed flags CRC/status/timeout.
   // ---------------------------------------------------------------------
   localparam [3:0] D_IDLE=4'd0, D_RWAIT=4'd1, D_RDATA=4'd2, D_RCRC=4'd3,
                    D_WGAP=4'd4, D_WSTART=4'd5, D_WDATA=4'd6, D_WCRC=4'd7,
                    D_WEND=4'd8, D_WREL=4'd9, D_WSTOK=4'd10, D_WSBIT=4'd11,
                    D_WBUSY=4'd12, D_DONE=4'd13;
   reg [3:0]  dstate;
   reg        dat_go, dat_write, dat_done, dat_failed;
   reg [12:0] d_bit;          // 0..4095 data bits
   reg [3:0]  d_crcbit;       // 0..15 CRC bits
   reg [3:0]  d_gap;
   reg [63:0] d_acc;          // read-side bit accumulator
   reg [15:0] d_crc, d_crc_rx;
   reg [2:0]  d_stok;
   reg [1:0]  d_stokcnt;
   reg        d_busyseen;
   reg [23:0] d_to;

   // Write-side: current bit out of the block buffer, MSB-first within a byte.
   wire [63:0] w_word_be = byteswap(blockbuf[d_bit[11:6]]);
   wire        w_bit     = w_word_be[6'd63 - d_bit[5:0]];

   always @(posedge clock) begin
      dat_done <= 1'b0;
      eng_we   <= 1'b0;
      if (reset) begin
         dstate <= D_IDLE; dat_t <= 4'b1111; dat_o <= 4'b1111; dat_failed <= 1'b0;
      end else case (dstate)
        D_IDLE: begin
           dat_t <= 4'b1111;
           if (dat_go) begin
              d_to <= TIMEOUT; d_crc <= 16'd0; d_crc_rx <= 16'd0;
              d_bit <= 13'd0; d_acc <= 64'd0; d_gap <= 4'd8; dat_failed <= 1'b0;
              dstate <= dat_write ? D_WGAP : D_RWAIT;
           end
        end

        // ---------------- READ ----------------
        D_RWAIT: if (edge_fall) begin               // await data start bit
           if (!dat_i[0]) begin d_bit <= 13'd0; dstate <= D_RDATA; end
           else if (d_to == 24'd0) begin dat_failed <= 1'b1; dstate <= D_DONE; end
           else d_to <= d_to - 24'd1;
        end
        D_RDATA: if (edge_fall) begin
           d_acc <= {d_acc[62:0], dat_i[0]};
           d_crc <= crc16_bit(d_crc, dat_i[0]);
           if (d_bit[5:0] == 6'd63) begin           // a 64-bit word completed
              eng_we    <= 1'b1;
              eng_addr  <= d_bit[11:6];
              eng_wdata <= byteswap({d_acc[62:0], dat_i[0]});
           end
           if (d_bit == 13'd4095) begin d_crcbit <= 4'd15; dstate <= D_RCRC; end
           else d_bit <= d_bit + 13'd1;
        end
        D_RCRC: if (edge_fall) begin
           d_crc_rx <= {d_crc_rx[14:0], dat_i[0]};
           if (d_crcbit == 4'd0) begin
              if ({d_crc_rx[14:0], dat_i[0]} != d_crc) dat_failed <= 1'b1;
              dstate <= D_DONE;
           end else d_crcbit <= d_crcbit - 4'd1;
        end

        // ---------------- WRITE ----------------
        D_WGAP: if (edge_fall) begin                // Nwr gap before start bit
           if (d_gap == 4'd0) dstate <= D_WSTART;
           else d_gap <= d_gap - 4'd1;
        end
        D_WSTART: if (edge_fall) begin
           dat_t <= 4'b1110; dat_o[0] <= 1'b0;      // drive DAT0, start bit
           d_bit <= 13'd0;
           dstate <= D_WDATA;
        end
        D_WDATA: if (edge_fall) begin
           dat_o[0] <= w_bit;
           d_crc <= crc16_bit(d_crc, w_bit);
           if (d_bit == 13'd4095) begin d_crcbit <= 4'd15; dstate <= D_WCRC; end
           else d_bit <= d_bit + 13'd1;
        end
        D_WCRC: if (edge_fall) begin
           dat_o[0] <= d_crc[15];
           d_crc <= {d_crc[14:0], 1'b0};
           if (d_crcbit == 4'd0) dstate <= D_WEND;
           else d_crcbit <= d_crcbit - 4'd1;
        end
        D_WEND: if (edge_fall) begin dat_o[0] <= 1'b1; dstate <= D_WREL; end
        D_WREL: if (edge_fall) begin
           dat_t <= 4'b1111; d_to <= TIMEOUT; dstate <= D_WSTOK;
        end
        D_WSTOK: if (edge_fall) begin                // await CRC-status start
           if (!dat_i[0]) begin d_stokcnt <= 2'd2; dstate <= D_WSBIT; end
           else if (d_to == 24'd0) begin dat_failed <= 1'b1; dstate <= D_DONE; end
           else d_to <= d_to - 24'd1;
        end
        D_WSBIT: if (edge_fall) begin                // 3-bit CRC status token
           d_stok <= {d_stok[1:0], dat_i[0]};
           if (d_stokcnt == 2'd0) begin
              d_to <= TIMEOUT; d_busyseen <= 1'b0; dstate <= D_WBUSY;
           end else d_stokcnt <= d_stokcnt - 2'd1;
        end
        // Wait through the end bit (high) for the busy pulse: DAT0 goes low
        // while the card programs, then high again when done.
        D_WBUSY: if (edge_fall) begin
           if (!dat_i[0]) d_busyseen <= 1'b1;
           else if (d_busyseen) begin
              if (d_stok != 3'b010) dat_failed <= 1'b1;   // not "accepted"
              dstate <= D_DONE;
           end else if (d_to == 24'd0) begin
              dat_failed <= 1'b1; dstate <= D_DONE;
           end else d_to <= d_to - 24'd1;
        end

        D_DONE: begin dat_done <= 1'b1; dstate <= D_IDLE; dat_t <= 4'b1111; end
        default: dstate <= D_IDLE;
      endcase
   end

   // ---------------------------------------------------------------------
   // Main sequencer: power-up gap, init handshake, then per-request I/O.
   //   *_S states set up a command and jump through M_ISSUE/M_WAITCMD.
   //   *_C states inspect the response.
   // ---------------------------------------------------------------------
   localparam [4:0]
      M_PWRUP=5'd0, M_ISSUE=5'd1, M_WAITCMD=5'd2,
      M_CMD0=5'd3, M_CMD8=5'd4, M_CMD8C=5'd5,
      M_CMD55=5'd6, M_ACMD41=5'd7, M_ACMD41C=5'd8,
      M_CMD2=5'd9, M_CMD3=5'd10, M_CMD3C=5'd11, M_CMD7=5'd12,
      M_READY=5'd13, M_IO=5'd14, M_DATA=5'd15, M_DATAW=5'd16,
      M_DONE=5'd17, M_ERR=5'd18, M_CMD9=5'd19, M_CMD9C=5'd20;
   reg [4:0]  mstate, m_ret;
   reg [15:0] pwr_cnt;
   reg [15:0] rca;
   reg        op_write;
   reg [31:0] op_sector;
   reg        dbg_err_seen;       // sticky: init hit the error state

   assign busy = (mstate != M_READY);
   assign dbg  = {2'b0, dbg_err_seen, ready, mstate, cstate, dstate};
   assign dbg_resp = cmd_resp[47:0];
   // [22]=resp_seen [21:16]=last_cmd [15:8]=cmd_count [7:0]=resp_count
   assign dbg_diag = {9'd0, dbg_resp_seen, dbg_last_cmd, dbg_cmd_count, dbg_resp_count};
   // [8:5]=dat_i live [4]=cmd_i live [3]=dat_high [2]=dat_low [1]=cmd_high [0]=cmd_low
   assign dbg_diag2 = {23'd0, dat_i, cmd_i, dbg_dat_high, dbg_dat_low, dbg_cmd_high, dbg_cmd_low};

   always @(posedge clock) begin
      done   <= 1'b0;
      error  <= 1'b0;
      cmd_go <= 1'b0;
      dat_go <= 1'b0;
      if (reset) begin
         mstate <= M_PWRUP; pwr_cnt <= 16'd0; ready <= 1'b0; rca <= 16'd0;
         capacity_sectors <= 32'd0; dbg_err_seen <= 1'b0;
      end else case (mstate)
        M_PWRUP: if (edge_rise) begin
           if (pwr_cnt >= INIT_TICKS) mstate <= M_CMD0;
           else pwr_cnt <= pwr_cnt + 16'd1;
        end

        M_ISSUE:   begin cmd_go <= 1'b1; mstate <= M_WAITCMD; end
        M_WAITCMD: if (cmd_done) mstate <= cmd_failed ? M_ERR : m_ret;

        M_CMD0: begin
           cmd_index <= CMD0; cmd_arg <= 32'd0;
           cmd_resp_len <= 8'd0; cmd_busy_wait <= 1'b0;
           m_ret <= M_CMD8; mstate <= M_ISSUE;
        end
        M_CMD8: begin
           cmd_index <= CMD8; cmd_arg <= 32'h0000_01AA;
           cmd_resp_len <= 8'd48; cmd_busy_wait <= 1'b0;
           m_ret <= M_CMD8C; mstate <= M_ISSUE;
        end
        M_CMD8C: mstate <= (cmd_resp[19:8] == 12'h1AA) ? M_CMD55 : M_ERR;

        M_CMD55: begin                       // APP_CMD (RCA=0 pre-ident)
           cmd_index <= CMD55; cmd_arg <= 32'd0;
           cmd_resp_len <= 8'd48; cmd_busy_wait <= 1'b0;
           m_ret <= M_ACMD41; mstate <= M_ISSUE;
        end
        M_ACMD41: begin                      // HCS=1, voltage window
           cmd_index <= CMD41; cmd_arg <= 32'h40FF_8000;
           cmd_resp_len <= 8'd48; cmd_busy_wait <= 1'b0;
           m_ret <= M_ACMD41C; mstate <= M_ISSUE;
        end
        M_ACMD41C: mstate <= cmd_resp[39] ? M_CMD2 : M_CMD55;  // OCR busy bit

        M_CMD2: begin                        // ALL_SEND_CID (R2, ignored)
           cmd_index <= CMD2; cmd_arg <= 32'd0;
           cmd_resp_len <= 8'd136; cmd_busy_wait <= 1'b0;
           m_ret <= M_CMD3; mstate <= M_ISSUE;
        end
        M_CMD3: begin                        // SEND_RELATIVE_ADDR (R6)
           cmd_index <= CMD3; cmd_arg <= 32'd0;
           cmd_resp_len <= 8'd48; cmd_busy_wait <= 1'b0;
           m_ret <= M_CMD3C; mstate <= M_ISSUE;
        end
        M_CMD3C: begin rca <= cmd_resp[39:24]; mstate <= M_CMD9; end
        M_CMD9: begin                        // SEND_CSD (R2) in standby state
           cmd_index <= CMD9; cmd_arg <= {rca, 16'd0};
           cmd_resp_len <= 8'd136; cmd_busy_wait <= 1'b0;
           m_ret <= M_CMD9C; mstate <= M_ISSUE;
        end
        // CSD v2.0: capacity_sectors = (C_SIZE + 1) * 1024, C_SIZE = CSD[69:48].
        // The 127-bit CSD register lands at cmd_resp[127:1], so CSD[n]=resp[n].
        M_CMD9C: begin
           capacity_sectors <= ({10'd0, cmd_resp[69:48]} + 32'd1) << 10;
           mstate <= M_CMD7;
        end
        M_CMD7: begin                        // SELECT_CARD (R1b) -> tran
           cmd_index <= CMD7; cmd_arg <= {rca, 16'd0};
           cmd_resp_len <= 8'd48; cmd_busy_wait <= 1'b1;
           m_ret <= M_READY; mstate <= M_ISSUE;
        end

        M_READY: begin
           ready <= 1'b1;
           if (req_valid) begin
              op_write <= req_write; op_sector <= req_sector;
              mstate <= M_IO;
           end
        end
        M_IO: begin
           cmd_index <= op_write ? CMD24 : CMD17;
           cmd_arg <= op_sector; cmd_resp_len <= 8'd48; cmd_busy_wait <= 1'b0;
           m_ret <= M_DATA; mstate <= M_ISSUE;
        end
        M_DATA: begin dat_go <= 1'b1; dat_write <= op_write; mstate <= M_DATAW; end
        M_DATAW: if (dat_done) mstate <= dat_failed ? M_ERR : M_DONE;

        M_DONE: begin done <= 1'b1; mstate <= M_READY; end
        M_ERR:  begin
           error <= 1'b1;
           dbg_err_seen <= 1'b1;
           mstate <= ready ? M_READY : M_ERR;   // init failure: stay stuck
        end
        default: mstate <= M_ERR;
      endcase
   end
endmodule

`default_nettype wire
