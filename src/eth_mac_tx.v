`timescale 1ns / 1ps
`default_nettype none

// GMII transmit framer: turns an L2 Ethernet frame held in an external buffer
// into a GMII byte stream (preamble + SFD + payload + zero-pad + FCS), with a
// trailing inter-frame gap.  Runs at one byte/clock in the MAC (gmii) clock
// domain.  The FCS path (registered gmii_txd fed to crc32_d8, crc_next for the
// first FCS byte) is replicated bit-for-bit from the board's 12_UDP_TEST design
// so frames are accepted by real hosts.
//
// Buffer read is combinational: rd_data must equal frame_buf[rd_index] in the
// same cycle.  (For a registered/BRAM source, drive the address one cycle
// ahead; the virtio integration will use a dual-port BRAM as the ui_clk->gmii
// CDC and adapt accordingly.)
module eth_mac_tx(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        send,        // 1-cycle pulse: transmit frame_len bytes
    input  wire [10:0] frame_len,   // L2 length, no preamble/FCS (14..1514)
    output wire [10:0] rd_index,    // byte index into the frame buffer
    input  wire [ 7:0] rd_data,     // frame_buf[rd_index], combinational
    output reg         busy,
    output reg         gmii_tx_en,
    output reg  [ 7:0] gmii_txd
    );

   localparam [10:0] MIN_DATA = 11'd60;  // min L2 bytes (=> >=64 with FCS)
   localparam [3:0]  IFG_CNT  = 4'd11;   // 12-byte inter-frame gap

   localparam [2:0] ST_IDLE     = 3'd0,
                    ST_PREAMBLE = 3'd1,
                    ST_DATA     = 3'd2,
                    ST_FCS      = 3'd3,
                    ST_IFG      = 3'd4;

   reg [2:0]  state;
   reg [10:0] data_cnt;     // bytes emitted in ST_DATA so far
   reg [10:0] data_len;     // latched max(frame_len, MIN_DATA)
   reg [3:0]  cnt;          // sub-counter for preamble/fcs/ifg
   reg        crc_en;
   reg        crc_clr;

   wire [31:0] crc_data;
   wire [31:0] crc_next;

   // crc32_d8 folds gmii_txd while crc_en && !crc_clr (preamble/SFD excluded
   // because crc_en is still 0 the cycle gmii_txd holds them).
   crc32_d8 u_crc(
      .clk      (clk),
      .rst_n    (rst_n),
      .data     (gmii_txd),
      .crc_en   (crc_en),
      .crc_clr  (crc_clr),
      .crc_data (crc_data),
      .crc_next (crc_next)
   );

   // Read the byte we are about to register into gmii_txd next cycle.
   assign rd_index = data_cnt;
   wire [7:0] data_byte = (data_cnt < frame_len) ? rd_data : 8'h00; // zero pad

   always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
         state      <= ST_IDLE;
         busy       <= 1'b0;
         gmii_tx_en <= 1'b0;
         gmii_txd   <= 8'h00;
         crc_en     <= 1'b0;
         crc_clr    <= 1'b1;
         cnt        <= 4'd0;
         data_cnt   <= 11'd0;
         data_len   <= 11'd0;
      end else begin
         // defaults
         crc_en     <= 1'b0;
         gmii_tx_en <= 1'b0;

         case (state)
            ST_IDLE: begin
               crc_clr <= 1'b1;          // hold CRC at 0xffffffff until data
               cnt     <= 4'd0;
               data_cnt<= 11'd0;
               if (send) begin
                  busy     <= 1'b1;
                  data_len <= (frame_len < MIN_DATA) ? MIN_DATA : frame_len;
                  state    <= ST_PREAMBLE;
               end else
                  busy <= 1'b0;
            end

            ST_PREAMBLE: begin
               gmii_tx_en <= 1'b1;
               gmii_txd   <= (cnt == 4'd7) ? 8'hd5 : 8'h55;  // 7x55 + SFD
               if (cnt == 4'd7) begin
                  cnt     <= 4'd0;
                  crc_clr <= 1'b0;        // release so first data byte folds
                  state   <= ST_DATA;
               end else
                  cnt <= cnt + 4'd1;
            end

            ST_DATA: begin
               gmii_tx_en <= 1'b1;
               crc_en     <= 1'b1;        // fold data_byte (now in gmii_txd)
               gmii_txd   <= data_byte;
               if (data_cnt == data_len - 11'd1) begin
                  cnt   <= 4'd0;
                  state <= ST_FCS;
               end else
                  data_cnt <= data_cnt + 11'd1;
            end

            ST_FCS: begin
               gmii_tx_en <= 1'b1;
               cnt        <= cnt + 4'd1;
               // Byte 0 uses crc_next (last data byte folds combinationally);
               // bytes 1-3 use the registered crc_data.  Bit-reversed +
               // complemented, exactly as the reference TX.
               case (cnt)
                 4'd0: gmii_txd <= {~crc_next[24],~crc_next[25],~crc_next[26],~crc_next[27],
                                    ~crc_next[28],~crc_next[29],~crc_next[30],~crc_next[31]};
                 4'd1: gmii_txd <= {~crc_data[16],~crc_data[17],~crc_data[18],~crc_data[19],
                                    ~crc_data[20],~crc_data[21],~crc_data[22],~crc_data[23]};
                 4'd2: gmii_txd <= {~crc_data[8], ~crc_data[9], ~crc_data[10],~crc_data[11],
                                    ~crc_data[12],~crc_data[13],~crc_data[14],~crc_data[15]};
                 default: begin
                    gmii_txd <= {~crc_data[0], ~crc_data[1], ~crc_data[2], ~crc_data[3],
                                 ~crc_data[4], ~crc_data[5], ~crc_data[6], ~crc_data[7]};
                    cnt   <= 4'd0;
                    state <= ST_IFG;
                 end
               endcase
            end

            ST_IFG: begin
               crc_clr <= 1'b1;
               if (cnt == IFG_CNT) begin
                  busy  <= 1'b0;
                  state <= ST_IDLE;
               end else
                  cnt <= cnt + 4'd1;
            end

            default: state <= ST_IDLE;
         endcase
      end
   end

endmodule

`default_nettype wire
