`timescale 1ns / 1ps
`default_nettype none

// Ethernet FCS (CRC-32, polynomial 0x04C11DB7), 8 bits/clock.
// Byte-serial: present each data byte on `data` with crc_en=1; pulse crc_clr
// to reset to 0xffffffff before a frame.  The on-wire FCS bytes are derived
// from crc_data/crc_next as bit-reversed, complemented nibbles (see eth_mac_tx).
//
// Imported verbatim (logic) from the board's 12_UDP_TEST reference design
// (ALINX/"正点原子" crc32_d8), which interoperates with real hosts; only the
// comments were rewritten to ASCII.
module crc32_d8(
    input  wire          clk,
    input  wire          rst_n,
    input  wire  [ 7:0]  data,      // byte to fold in (LSB-first on the wire)
    input  wire          crc_en,    // 1 = accumulate `data` this cycle
    input  wire          crc_clr,   // 1 = reset CRC to 0xffffffff
    output reg   [31:0]  crc_data,  // current CRC register
    output wire  [31:0]  crc_next   // CRC after folding in `data`
    );

   // Ethernet feeds each byte LSB-first, so bit-reverse the input byte.
   wire [7:0] data_t = {data[0],data[1],data[2],data[3],
                        data[4],data[5],data[6],data[7]};

   assign crc_next[0] = crc_data[24] ^ crc_data[30] ^ data_t[0] ^ data_t[6];
   assign crc_next[1] = crc_data[24] ^ crc_data[25] ^ crc_data[30] ^ crc_data[31]
                        ^ data_t[0] ^ data_t[1] ^ data_t[6] ^ data_t[7];
   assign crc_next[2] = crc_data[24] ^ crc_data[25] ^ crc_data[26] ^ crc_data[30]
                        ^ crc_data[31] ^ data_t[0] ^ data_t[1] ^ data_t[2] ^ data_t[6]
                        ^ data_t[7];
   assign crc_next[3] = crc_data[25] ^ crc_data[26] ^ crc_data[27] ^ crc_data[31]
                        ^ data_t[1] ^ data_t[2] ^ data_t[3] ^ data_t[7];
   assign crc_next[4] = crc_data[24] ^ crc_data[26] ^ crc_data[27] ^ crc_data[28]
                        ^ crc_data[30] ^ data_t[0] ^ data_t[2] ^ data_t[3] ^ data_t[4]
                        ^ data_t[6];
   assign crc_next[5] = crc_data[24] ^ crc_data[25] ^ crc_data[27] ^ crc_data[28]
                        ^ crc_data[29] ^ crc_data[30] ^ crc_data[31] ^ data_t[0]
                        ^ data_t[1] ^ data_t[3] ^ data_t[4] ^ data_t[5] ^ data_t[6]
                        ^ data_t[7];
   assign crc_next[6] = crc_data[25] ^ crc_data[26] ^ crc_data[28] ^ crc_data[29]
                        ^ crc_data[30] ^ crc_data[31] ^ data_t[1] ^ data_t[2] ^ data_t[4]
                        ^ data_t[5] ^ data_t[6] ^ data_t[7];
   assign crc_next[7] = crc_data[24] ^ crc_data[26] ^ crc_data[27] ^ crc_data[29]
                        ^ crc_data[31] ^ data_t[0] ^ data_t[2] ^ data_t[3] ^ data_t[5]
                        ^ data_t[7];
   assign crc_next[8] = crc_data[0] ^ crc_data[24] ^ crc_data[25] ^ crc_data[27]
                        ^ crc_data[28] ^ data_t[0] ^ data_t[1] ^ data_t[3] ^ data_t[4];
   assign crc_next[9] = crc_data[1] ^ crc_data[25] ^ crc_data[26] ^ crc_data[28]
                        ^ crc_data[29] ^ data_t[1] ^ data_t[2] ^ data_t[4] ^ data_t[5];
   assign crc_next[10] = crc_data[2] ^ crc_data[24] ^ crc_data[26] ^ crc_data[27]
                        ^ crc_data[29] ^ data_t[0] ^ data_t[2] ^ data_t[3] ^ data_t[5];
   assign crc_next[11] = crc_data[3] ^ crc_data[24] ^ crc_data[25] ^ crc_data[27]
                        ^ crc_data[28] ^ data_t[0] ^ data_t[1] ^ data_t[3] ^ data_t[4];
   assign crc_next[12] = crc_data[4] ^ crc_data[24] ^ crc_data[25] ^ crc_data[26]
                        ^ crc_data[28] ^ crc_data[29] ^ crc_data[30] ^ data_t[0]
                        ^ data_t[1] ^ data_t[2] ^ data_t[4] ^ data_t[5] ^ data_t[6];
   assign crc_next[13] = crc_data[5] ^ crc_data[25] ^ crc_data[26] ^ crc_data[27]
                        ^ crc_data[29] ^ crc_data[30] ^ crc_data[31] ^ data_t[1]
                        ^ data_t[2] ^ data_t[3] ^ data_t[5] ^ data_t[6] ^ data_t[7];
   assign crc_next[14] = crc_data[6] ^ crc_data[26] ^ crc_data[27] ^ crc_data[28]
                        ^ crc_data[30] ^ crc_data[31] ^ data_t[2] ^ data_t[3] ^ data_t[4]
                        ^ data_t[6] ^ data_t[7];
   assign crc_next[15] = crc_data[7] ^ crc_data[27] ^ crc_data[28] ^ crc_data[29]
                        ^ crc_data[31] ^ data_t[3] ^ data_t[4] ^ data_t[5] ^ data_t[7];
   assign crc_next[16] = crc_data[8] ^ crc_data[24] ^ crc_data[28] ^ crc_data[29]
                        ^ data_t[0] ^ data_t[4] ^ data_t[5];
   assign crc_next[17] = crc_data[9] ^ crc_data[25] ^ crc_data[29] ^ crc_data[30]
                        ^ data_t[1] ^ data_t[5] ^ data_t[6];
   assign crc_next[18] = crc_data[10] ^ crc_data[26] ^ crc_data[30] ^ crc_data[31]
                        ^ data_t[2] ^ data_t[6] ^ data_t[7];
   assign crc_next[19] = crc_data[11] ^ crc_data[27] ^ crc_data[31] ^ data_t[3] ^ data_t[7];
   assign crc_next[20] = crc_data[12] ^ crc_data[28] ^ data_t[4];
   assign crc_next[21] = crc_data[13] ^ crc_data[29] ^ data_t[5];
   assign crc_next[22] = crc_data[14] ^ crc_data[24] ^ data_t[0];
   assign crc_next[23] = crc_data[15] ^ crc_data[24] ^ crc_data[25] ^ crc_data[30]
                         ^ data_t[0] ^ data_t[1] ^ data_t[6];
   assign crc_next[24] = crc_data[16] ^ crc_data[25] ^ crc_data[26] ^ crc_data[31]
                         ^ data_t[1] ^ data_t[2] ^ data_t[7];
   assign crc_next[25] = crc_data[17] ^ crc_data[26] ^ crc_data[27] ^ data_t[2] ^ data_t[3];
   assign crc_next[26] = crc_data[18] ^ crc_data[24] ^ crc_data[27] ^ crc_data[28]
                         ^ crc_data[30] ^ data_t[0] ^ data_t[3] ^ data_t[4] ^ data_t[6];
   assign crc_next[27] = crc_data[19] ^ crc_data[25] ^ crc_data[28] ^ crc_data[29]
                         ^ crc_data[31] ^ data_t[1] ^ data_t[4] ^ data_t[5] ^ data_t[7];
   assign crc_next[28] = crc_data[20] ^ crc_data[26] ^ crc_data[29] ^ crc_data[30]
                         ^ data_t[2] ^ data_t[5] ^ data_t[6];
   assign crc_next[29] = crc_data[21] ^ crc_data[27] ^ crc_data[30] ^ crc_data[31]
                         ^ data_t[3] ^ data_t[6] ^ data_t[7];
   assign crc_next[30] = crc_data[22] ^ crc_data[28] ^ crc_data[31] ^ data_t[4] ^ data_t[7];
   assign crc_next[31] = crc_data[23] ^ crc_data[29] ^ data_t[5];

   always @(posedge clk or negedge rst_n) begin
      if (!rst_n)
         crc_data <= 32'hffff_ffff;
      else if (crc_clr)
         crc_data <= 32'hffff_ffff;
      else if (crc_en)
         crc_data <= crc_next;
   end

endmodule

`default_nettype wire
