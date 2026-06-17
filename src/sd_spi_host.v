`timescale 1ns / 1ps
`default_nettype none

// SPI-mode SD host controller. Same block interface as sd_host.v (the native
// controller), so virtio_blk can use either — but this one speaks SD over SPI,
// which is what the board's socket is proven to support (the monitor SL command
// boots from it). Modelled directly on workloads/monitor/monitor.c.
//
//   - issue:  pulse req_valid with req_write/req_sector
//   - done:   one-cycle done (or error); busy high meanwhile
//   - data:   512-byte block buffer (64 x 64-bit LE words), caller-addressed
//             while !busy
//   - ready:  rises once init (CMD0/8/ACMD41/CMD58/CMD9) completes
//
// SPI mode 0 (CPOL=0, CPHA=0): MOSI changes on the falling SCK edge, MISO is
// sampled on the rising edge. CS (cs_n) is active-low. SCK is slow (<=400 kHz)
// for init, then fast for transfers.
module sd_spi_host #(
    parameter [15:0] SLOW_HALF  = 16'd416,   // SCK half-period in core clocks-1
    parameter [15:0] FAST_HALF  = 16'd12,    //   (~400 kHz / ~12 MHz @ 333 MHz)
    parameter [15:0] INIT_BYTES = 16'd10     // idle bytes (CS high) before CMD0
) (
    input  wire        clock,
    input  wire        reset,

    // Runtime transfer-clock override (SCK half-period in core clocks - 1);
    // 0 = use the FAST_HALF parameter. Lets the speed be tuned over MMIO.
    input  wire [15:0] fast_half,

    input  wire        req_valid,
    input  wire        req_write,
    input  wire [31:0] req_sector,
    output wire        busy,
    output reg         done,
    output reg         error,
    output reg         ready,
    output reg  [31:0] capacity_sectors,

    input  wire [ 5:0] buf_addr,
    input  wire [63:0] buf_wdata,
    input  wire        buf_we,
    output wire [63:0] buf_rdata,

    output reg         sck,
    output reg         mosi,
    input  wire        miso,
    output reg         cs_n,

    output wire [ 6:0] dbg_state,   // SPI FSM state, for an MMIO debug overlay
    output wire [15:0] dbg_io       // {7'b0, rd_token_timeout, last_io_R1}
);
   // ---------------------------------------------------------------------
   // 512-byte block buffer: 64 x 64-bit LE words, sync write / async read.
   // ---------------------------------------------------------------------
   reg [63:0] blockbuf [0:63];
   reg [ 5:0] eng_addr;
   reg [63:0] eng_wdata;
   reg        eng_we;
   wire [ 5:0] wr_addr = busy ? eng_addr  : buf_addr;
   wire [63:0] wr_data = busy ? eng_wdata : buf_wdata;
   wire        wr_en   = busy ? eng_we    : buf_we;
   always @(posedge clock) if (wr_en) blockbuf[wr_addr] <= wr_data;
   assign buf_rdata = blockbuf[buf_addr];

   // current data byte <-> block buffer (byte b -> word b/8, lane b%8, LE)
   reg  [8:0] byte_idx;          // 0..511 within the block
   wire [5:0] bb_word = byte_idx[8:3];
   wire [2:0] bb_lane = byte_idx[2:0];
   wire [7:0] wr_byte = blockbuf[bb_word][{bb_lane, 3'b000} +: 8];
   reg [63:0] cur_word;          // assembled on read

   // ---------------------------------------------------------------------
   // Byte-SPI engine (mode 0). Pulse bx_go with bx_tx; bx_done pulses with the
   // received bx_rx. One SCK byte = 8 bits, MSB first.
   // ---------------------------------------------------------------------
   reg        bx_go;
   reg [7:0]  bx_tx;
   reg        bx_done;
   reg [7:0]  bx_rx;
   reg        bx_active;
   reg [7:0]  bx_shift;
   reg [3:0]  bx_bits;           // rising edges remaining
   reg        bx_phase;          // 0: SCK low half, 1: SCK high half
   reg [15:0] bx_div;
   wire [15:0] half = ready ? (fast_half != 16'd0 ? fast_half : FAST_HALF) : SLOW_HALF;

   // MISO is asynchronous to clock (driven by the card); synchronize it before
   // sampling so the capture is robust regardless of P&R timing margin.
   reg [1:0]  miso_sync;
   always @(posedge clock) miso_sync <= reset ? 2'b11 : {miso_sync[0], miso};
   wire miso_s = miso_sync[1];

   always @(posedge clock) begin
      bx_done <= 1'b0;
      if (reset) begin
         bx_active <= 1'b0; sck <= 1'b0; mosi <= 1'b1; bx_div <= 16'd0;
      end else if (bx_go && !bx_active) begin
         bx_active <= 1'b1; bx_shift <= bx_tx; mosi <= bx_tx[7];
         bx_bits <= 4'd8; bx_phase <= 1'b0; bx_div <= 16'd0; sck <= 1'b0;
      end else if (bx_active) begin
         if (bx_div >= half) begin
            bx_div <= 16'd0;
            if (!bx_phase) begin                 // rising edge: sample MISO
               sck <= 1'b1;
               bx_rx <= {bx_rx[6:0], miso_s};
               bx_bits <= bx_bits - 4'd1;
               bx_phase <= 1'b1;
            end else begin                        // falling edge: shift MOSI
               sck <= 1'b0;
               if (bx_bits == 4'd0) begin
                  bx_active <= 1'b0; bx_done <= 1'b1;
               end else begin
                  bx_shift <= {bx_shift[6:0], 1'b1};
                  mosi <= bx_shift[6];
                  bx_phase <= 1'b0;
               end
            end
         end else
            bx_div <= bx_div + 16'd1;
      end
   end

   // ---------------------------------------------------------------------
   // Byte-transfer "subroutine": states set bx_tx + bx_ret, jump to S_BX.
   // ---------------------------------------------------------------------
   localparam [6:0]
     S_BX        = 7'd0,  S_BXW      = 7'd1,
     S_RESET     = 7'd2,  S_PWRUP    = 7'd3,
     S_CMD_BUILD = 7'd4,  S_CMD_SEND = 7'd5,  S_CMD_POLL = 7'd6,
     S_CMD_EXTRA = 7'd7,  S_CMD_TAIL = 7'd8,  S_CMD_IDLE = 7'd9,
     S_I_CMD0    = 7'd10, S_I_CMD0D  = 7'd11,
     S_I_CMD8    = 7'd12, S_I_CMD8D  = 7'd13,
     S_I_CMD55   = 7'd14, S_I_CMD55D = 7'd15,
     S_I_ACMD41  = 7'd16, S_I_ACMD41D= 7'd17,
     S_I_CMD58   = 7'd18, S_I_CMD58D = 7'd19,
     S_I_CMD9    = 7'd20, S_I_CMD9D  = 7'd21,
     S_CSD_TOK   = 7'd22, S_CSD_READ = 7'd23, S_CSD_DONE = 7'd24,
     S_READY     = 7'd25,
     S_IO_CMD    = 7'd26, S_IO_CMDD  = 7'd27,
     S_RD_TOK    = 7'd28, S_RD_DATA  = 7'd29, S_RD_CRC   = 7'd30,
     S_WR_TOK    = 7'd31, S_WR_DATA  = 7'd32, S_WR_CRC   = 7'd33,
     S_WR_RESP   = 7'd34, S_WR_BUSY  = 7'd35,
     S_IO_TAIL   = 7'd36, S_IO_IDLE  = 7'd37,
     S_COMPLETE  = 7'd38, S_ERROR    = 7'd39;

   reg [6:0]  state, bx_ret;
   reg [15:0] init_cnt;
   reg        v2_card;
   reg [31:0] ocr;
   reg [9:0]  acmd41_tries;
   reg [19:0] poll_cnt;   // wide: the read-data-token wait must allow tens of ms

   // command engine scratch
   reg [5:0]  c_idx;
   reg [31:0] c_arg;
   reg [7:0]  c_crc;
   reg [2:0]  c_extra;           // extra response bytes after R1
   reg        c_keepcs;          // leave CS asserted (for data commands)
   reg [6:0]  c_ret;             // where to go when the command finishes
   reg [2:0]  c_step;            // byte index 0..6 while sending the frame
   reg [2:0]  c_ecnt;            // extra-byte counter
   reg [7:0]  resp [0:4];        // R1 + up to 4 extra bytes
   reg [7:0]  csd  [0:15];
   reg [3:0]  csd_idx;
   reg        op_write;
   reg [31:0] op_sector;
   reg        io_ok;

   wire [7:0] frame_byte = (c_step == 3'd0) ? {2'b01, c_idx} :
                           (c_step == 3'd1) ? c_arg[31:24] :
                           (c_step == 3'd2) ? c_arg[23:16] :
                           (c_step == 3'd3) ? c_arg[15:8]  :
                           (c_step == 3'd4) ? c_arg[7:0]   : c_crc;
   wire [21:0] csd_c_size = {csd[7][5:0], csd[8], csd[9]};

   reg [7:0] dbg_r1;       // R1 of the last block command
   reg       dbg_rd_to;    // sticky: a read-token poll timed out
   reg       dbg_retried;  // sticky: a read was retried after a failure
   reg [2:0] io_retry;     // remaining read retries
   assign busy = (state != S_READY);
   assign dbg_state = state;
   assign dbg_io = {6'd0, dbg_retried, dbg_rd_to, dbg_r1};

   integer i;
   always @(posedge clock) begin
      done   <= 1'b0;
      error  <= 1'b0;
      bx_go  <= 1'b0;
      eng_we <= 1'b0;
      if (reset) begin
         state <= S_RESET; ready <= 1'b0; cs_n <= 1'b1;
         capacity_sectors <= 32'd0; v2_card <= 1'b0;
         // dbg_r1/dbg_rd_to/dbg_retried deliberately NOT reset here, so the last
         // read's failure mode survives a key[1] soft-reset and can be read from
         // the monitor at 0x10002F08 after a boot failure.
      end else case (state)
        // ---- byte-transfer subroutine ----
        S_BX:  begin bx_go <= 1'b1; state <= S_BXW; end
        S_BXW: if (bx_done) state <= bx_ret;

        // ---- power-up: INIT_BYTES idle bytes, CS high ----
        S_RESET: begin
           cs_n <= 1'b1; init_cnt <= INIT_BYTES; ready <= 1'b0; state <= S_PWRUP;
        end
        S_PWRUP: begin
           if (init_cnt == 16'd0) begin
              // CMD0, arg 0, crc 0x95, no extra
              c_idx<=6'd0; c_arg<=32'd0; c_crc<=8'h95; c_extra<=3'd0;
              c_keepcs<=1'b0; c_ret<=S_I_CMD0D; state<=S_CMD_BUILD;
           end else begin
              bx_tx <= 8'hff; bx_ret <= S_PWRUP; init_cnt <= init_cnt - 16'd1;
              state <= S_BX;
           end
        end

        // ---- generic command: CS low, 0xff, 6 frame bytes, poll R1, extra ----
        S_CMD_BUILD: begin
           cs_n <= 1'b0; c_step <= 3'd0; poll_cnt <= 16'd16; c_ecnt <= 3'd0;
           bx_tx <= 8'hff; bx_ret <= S_CMD_SEND; state <= S_BX;   // leading dummy
        end
        S_CMD_SEND: begin
           bx_tx <= frame_byte; bx_ret <= (c_step == 3'd5) ? S_CMD_POLL : S_CMD_SEND;
           c_step <= c_step + 3'd1; state <= S_BX;
        end
        S_CMD_POLL: begin
           if (!bx_rx[7]) begin                 // R1 received (top bit clear)
              resp[0] <= bx_rx;
              if (c_extra == 3'd0) state <= c_keepcs ? c_ret : S_CMD_TAIL;
              else begin bx_tx<=8'hff; bx_ret<=S_CMD_EXTRA; state<=S_BX; end
           end else if (poll_cnt == 16'd0) begin
              resp[0] <= 8'hff; state <= c_keepcs ? c_ret : S_CMD_TAIL;  // no R1
           end else begin
              poll_cnt <= poll_cnt - 16'd1;
              bx_tx <= 8'hff; bx_ret <= S_CMD_POLL; state <= S_BX;
           end
        end
        S_CMD_EXTRA: begin
           resp[1 + c_ecnt] <= bx_rx; c_ecnt <= c_ecnt + 3'd1;
           if (c_ecnt + 3'd1 == c_extra) state <= c_keepcs ? c_ret : S_CMD_TAIL;
           else begin bx_tx<=8'hff; bx_ret<=S_CMD_EXTRA; state<=S_BX; end
        end
        S_CMD_TAIL: begin cs_n <= 1'b1; bx_tx<=8'hff; bx_ret<=c_ret; state<=S_BX; end

        // ---- init sequence ----
        S_I_CMD0D: begin
           // R1 should be 0x01 (idle). Proceed to CMD8 regardless if responsive.
           c_idx<=6'd8; c_arg<=32'h000001AA; c_crc<=8'h87; c_extra<=3'd4;
           c_keepcs<=1'b0; c_ret<=S_I_CMD8D; state<=S_CMD_BUILD;
        end
        S_I_CMD8D: begin
           v2_card <= !resp[0][2] && resp[3]==8'h01 && resp[4]==8'hAA;
           acmd41_tries <= 10'd1000;
           c_idx<=6'd55; c_arg<=32'd0; c_crc<=8'h01; c_extra<=3'd0;
           c_keepcs<=1'b0; c_ret<=S_I_CMD55D; state<=S_CMD_BUILD;
        end
        S_I_CMD55D: begin
           c_idx<=6'd41; c_arg<=v2_card ? 32'h40000000 : 32'd0; c_crc<=8'h01;
           c_extra<=3'd0; c_keepcs<=1'b0; c_ret<=S_I_ACMD41D; state<=S_CMD_BUILD;
        end
        S_I_ACMD41D: begin
           if (resp[0] == 8'd0) begin            // ready
              c_idx<=6'd58; c_arg<=32'd0; c_crc<=8'h01; c_extra<=3'd4;
              c_keepcs<=1'b0; c_ret<=S_I_CMD58D; state<=S_CMD_BUILD;
           end else if (acmd41_tries == 10'd0) state <= S_ERROR;
           else begin
              acmd41_tries <= acmd41_tries - 10'd1;
              c_idx<=6'd55; c_arg<=32'd0; c_crc<=8'h01; c_extra<=3'd0;
              c_keepcs<=1'b0; c_ret<=S_I_CMD55D; state<=S_CMD_BUILD;
           end
        end
        S_I_CMD58D: begin
           ocr <= {resp[1], resp[2], resp[3], resp[4]};
           // CMD9 SEND_CSD: data command, keep CS, then read the 16-byte CSD.
           c_idx<=6'd9; c_arg<=32'd0; c_crc<=8'h01; c_extra<=3'd0;
           c_keepcs<=1'b1; c_ret<=S_I_CMD9D; state<=S_CMD_BUILD;
        end
        S_I_CMD9D: begin
           if (resp[0] != 8'd0) state <= S_ERROR;
           else begin poll_cnt<=16'd1000; bx_tx<=8'hff; bx_ret<=S_CSD_TOK; state<=S_BX; end
        end
        S_CSD_TOK: begin
           if (bx_rx == 8'hFE) begin csd_idx<=4'd0; bx_tx<=8'hff; bx_ret<=S_CSD_READ; state<=S_BX; end
           else if (poll_cnt==16'd0) state<=S_ERROR;
           else begin poll_cnt<=poll_cnt-16'd1; bx_tx<=8'hff; bx_ret<=S_CSD_TOK; state<=S_BX; end
        end
        S_CSD_READ: begin
           csd[csd_idx] <= bx_rx; csd_idx <= csd_idx + 4'd1;
           if (csd_idx == 4'd15) begin bx_tx<=8'hff; bx_ret<=S_CSD_DONE; state<=S_BX; end  // 1st CRC byte
           else begin bx_tx<=8'hff; bx_ret<=S_CSD_READ; state<=S_BX; end
        end
        S_CSD_DONE: begin
           // 2nd CRC byte, then deassert. CSD v2: cap = (C_SIZE+1) * 1024 sectors.
           capacity_sectors <= ({10'd0, csd_c_size} + 32'd1) << 10;
           cs_n <= 1'b1; bx_tx<=8'hff; bx_ret<=S_READY; state<=S_BX;
        end

        S_READY: begin
           ready <= 1'b1;
           if (req_valid) begin
              op_write <= req_write; op_sector <= req_sector; io_ok <= 1'b1;
              io_retry <= 3'd7;        // up to 7 retries on a failed read
              state <= S_IO_CMD;
           end
        end

        // ---- per-request: CMD17 (read) or CMD24 (write), CS kept low ----
        S_IO_CMD: begin
           c_idx <= op_write ? 6'd24 : 6'd17;
           c_arg <= ocr[30] ? op_sector : (op_sector << 9);  // CCS: block vs byte addr
           c_crc<=8'h01; c_extra<=3'd0; c_keepcs<=1'b1;
           c_ret<=S_IO_CMDD; state<=S_CMD_BUILD;
        end
        S_IO_CMDD: begin
           dbg_r1 <= resp[0];
           if (resp[0] != 8'd0) begin io_ok<=1'b0; state<=S_IO_TAIL; end
           else if (op_write) begin
              byte_idx<=9'd0; bx_tx<=8'hFE; bx_ret<=S_WR_DATA; state<=S_BX;  // start token
           end else begin
              poll_cnt<=20'd100000; bx_tx<=8'hff; bx_ret<=S_RD_TOK; state<=S_BX;
           end
        end

        // ---- read: wait token 0xFE, read 512 bytes, 2 CRC ----
        S_RD_TOK: begin
           if (bx_rx == 8'hFE) begin byte_idx<=9'd0; cur_word<=64'd0; bx_tx<=8'hff; bx_ret<=S_RD_DATA; state<=S_BX; end
           else if (poll_cnt==16'd0) begin io_ok<=1'b0; dbg_rd_to<=1'b1; state<=S_IO_TAIL; end
           else begin poll_cnt<=poll_cnt-16'd1; bx_tx<=8'hff; bx_ret<=S_RD_TOK; state<=S_BX; end
        end
        S_RD_DATA: begin
           cur_word[{bb_lane, 3'b000} +: 8] <= bx_rx;
           if (bb_lane == 3'd7) begin
              eng_we<=1'b1; eng_addr<=bb_word;
              eng_wdata <= {bx_rx, cur_word[55:0]};        // complete the word
           end
           if (byte_idx == 9'd511) begin bx_tx<=8'hff; bx_ret<=S_RD_CRC; poll_cnt<=16'd1; state<=S_BX; end
           else begin byte_idx<=byte_idx+9'd1; bx_tx<=8'hff; bx_ret<=S_RD_DATA; state<=S_BX; end
        end
        S_RD_CRC: begin
           if (poll_cnt==16'd0) state<=S_IO_TAIL;          // both CRC bytes consumed
           else begin poll_cnt<=poll_cnt-16'd1; bx_tx<=8'hff; bx_ret<=S_RD_CRC; state<=S_BX; end
        end

        // ---- write: 0xFE sent, send 512 bytes, 2 CRC, data-response, busy ----
        S_WR_DATA: begin
           bx_tx <= wr_byte; bx_ret <= (byte_idx==9'd511) ? S_WR_CRC : S_WR_DATA;
           if (byte_idx != 9'd511) byte_idx <= byte_idx + 9'd1;
           else poll_cnt <= 16'd2;   // 2 CRC bytes + 1 to capture data-response
           state <= S_BX;
        end
        S_WR_CRC: begin
           bx_tx<=8'hff; bx_ret <= (poll_cnt==16'd0) ? S_WR_RESP : S_WR_CRC;
           if (poll_cnt != 16'd0) poll_cnt <= poll_cnt - 16'd1;
           state<=S_BX;
        end
        S_WR_RESP: begin
           // data response token: xxx00101 = accepted
           if ((bx_rx & 8'h1f) != 8'h05) io_ok <= 1'b0;
           poll_cnt<=20'd200000; bx_tx<=8'hff; bx_ret<=S_WR_BUSY; state<=S_BX;
        end
        S_WR_BUSY: begin                                    // card holds MISO low while writing
           if (bx_rx == 8'hff) state <= S_IO_TAIL;          // busy released
           else if (poll_cnt==20'd0) begin io_ok<=1'b0; state<=S_IO_TAIL; end
           else begin poll_cnt<=poll_cnt-20'd1; bx_tx<=8'hff; bx_ret<=S_WR_BUSY; state<=S_BX; end
        end

        S_IO_TAIL: begin cs_n<=1'b1; bx_tx<=8'hff; bx_ret<=S_IO_IDLE; state<=S_BX; end
        S_IO_IDLE: begin
           if (!io_ok && !op_write && io_retry != 3'd0) begin
              io_retry <= io_retry - 3'd1; io_ok <= 1'b1;
              dbg_retried <= 1'b1; state <= S_IO_CMD;   // retry the read
           end else
              state <= io_ok ? S_COMPLETE : S_ERROR;
        end

        S_COMPLETE: begin done <= 1'b1; state <= S_READY; end
        S_ERROR:    begin error <= 1'b1; state <= ready ? S_READY : S_ERROR; end

        default: state <= S_ERROR;
      endcase
   end
endmodule

`default_nettype wire
