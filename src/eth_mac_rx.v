`timescale 1ns / 1ps
`default_nettype none

// GMII receive deframer: skips the preamble, detects the SFD (0xD5), then
// streams the payload out while validating the trailing FCS.  Runs at one
// byte/clock in the PHY RX-clock (gmii) domain.
//
// A 4-byte delay line holds the most recent 4 captured bytes back from the
// payload output: when gmii_rx_dv drops, those 4 bytes are the FCS and the
// bytes already shifted out were the L2 payload.  The same crc32_d8 (and the
// same bit arrangement as eth_mac_tx) recomputes the FCS over the payload and
// compares it to the received FCS, so no residue magic constant is needed and
// frames from any standards-compliant sender validate correctly.
module eth_mac_rx(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        gmii_rx_dv,
    input  wire [ 7:0] gmii_rxd,
    output reg         rx_valid,   // payload byte valid this cycle
    output reg  [ 7:0] rx_data,    // payload byte
    output reg         rx_last,    // 1-cycle pulse: frame complete
    output reg         rx_good     // valid with rx_last: FCS matched
    );

   localparam [1:0] ST_IDLE = 2'd0,  // consume preamble, wait for SFD
                    ST_DATA = 2'd1,  // capturing payload + FCS
                    ST_END  = 2'd2;  // let CRC settle, then check FCS

   reg [1:0]  state;
   reg [7:0]  sr0, sr1, sr2, sr3;    // delay line; sr0 newest, sr3 oldest
   reg [2:0]  fill;                  // how many bytes are in the delay (0..4)

   wire [31:0] crc_data;
   wire [31:0] crc_next;

   // Combinational CRC control, aligned to the byte currently in sr3 (the one
   // being evicted as payload this cycle).  Registering these would fold sr3
   // one cycle late, after it has already shifted.  crc_clr holds the CRC at
   // 0xffffffff during IDLE and the pipeline-fill phase, then releases so the
   // first evicted payload byte folds into the initial value; it stays
   // released through ST_END so crc_data survives for the FCS compare.
   wire crc_en  = (state == ST_DATA) && gmii_rx_dv && (fill == 3'd4);
   wire crc_clr = (state == ST_IDLE) || (state == ST_DATA && fill < 3'd4);

   crc32_d8 u_crc(
      .clk      (clk),
      .rst_n    (rst_n),
      .data     (sr3),               // byte being evicted = a payload byte
      .crc_en   (crc_en),
      .crc_clr  (crc_clr),
      .crc_data (crc_data),
      .crc_next (crc_next)
   );

   // FCS the way eth_mac_tx emits it (complete crc_data, bit-reversed +
   // complemented).  Received FCS order on the wire is byte0..byte3.
   wire [7:0] fcs0 = {~crc_data[24],~crc_data[25],~crc_data[26],~crc_data[27],
                      ~crc_data[28],~crc_data[29],~crc_data[30],~crc_data[31]};
   wire [7:0] fcs1 = {~crc_data[16],~crc_data[17],~crc_data[18],~crc_data[19],
                      ~crc_data[20],~crc_data[21],~crc_data[22],~crc_data[23]};
   wire [7:0] fcs2 = {~crc_data[8], ~crc_data[9], ~crc_data[10],~crc_data[11],
                      ~crc_data[12],~crc_data[13],~crc_data[14],~crc_data[15]};
   wire [7:0] fcs3 = {~crc_data[0], ~crc_data[1], ~crc_data[2], ~crc_data[3],
                      ~crc_data[4], ~crc_data[5], ~crc_data[6], ~crc_data[7]};

   always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
         state    <= ST_IDLE;
         rx_valid <= 1'b0;
         rx_data  <= 8'h00;
         rx_last  <= 1'b0;
         rx_good  <= 1'b0;
         fill     <= 3'd0;
         sr0 <= 8'h00; sr1 <= 8'h00; sr2 <= 8'h00; sr3 <= 8'h00;
      end else begin
         rx_valid <= 1'b0;
         rx_last  <= 1'b0;

         case (state)
            ST_IDLE: begin
               fill    <= 3'd0;
               rx_good <= 1'b0;
               // Consume preamble; SFD starts the frame.
               if (gmii_rx_dv && gmii_rxd == 8'hd5)
                  state <= ST_DATA;
            end

            ST_DATA: begin
               if (gmii_rx_dv) begin
                  // shift new byte into the delay line
                  sr0 <= gmii_rxd;
                  sr1 <= sr0;
                  sr2 <= sr1;
                  sr3 <= sr2;
                  if (fill == 3'd4) begin
                     // delay full: sr3 evicts as a confirmed payload byte
                     // (crc_en folds it combinationally this cycle)
                     rx_valid <= 1'b1;
                     rx_data  <= sr3;
                  end else begin
                     fill <= fill + 3'd1;
                  end
               end else begin
                  state <= ST_END;     // dv dropped: sr3..sr0 hold the FCS
               end
            end

            ST_END: begin
               // crc_data has folded all payload bytes by now.  Received FCS
               // is sr3(byte0) sr2(byte1) sr1(byte2) sr0(byte3).
               rx_last <= 1'b1;
               rx_good <= (sr3 == fcs0) && (sr2 == fcs1) &&
                          (sr1 == fcs2) && (sr0 == fcs3);
               state   <= ST_IDLE;
            end

            default: state <= ST_IDLE;
         endcase
      end
   end

endmodule

`default_nettype wire
