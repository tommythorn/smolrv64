`timescale 1ns / 1ps
`default_nettype none

// RX engine: bridges eth_mac_rx (gmii = PHY RX-clock domain) to the virtio_net
// backend (ui_clk).  eth_mac_rx streams a received L2 frame in; the engine writes
// it into the next free SLOT of a BRAM ring, and on a good FCS commits the slot to
// the backend, which reads the bytes out (registered read, one cycle), DMAs them
// into a guest RX buffer and pulses frame_ack to release the slot.
//
// SLOTS frames are held, in order: a burst of back-to-back frames from a gigabit
// sender is absorbed instead of lost while the backend drains one (~12 us per
// 1500-byte frame, which is a frame's wire time).  A frame is captured only if its
// FIRST byte arrives while its slot is free -- the decision is made at the start
// and never revisited.  The single-buffered engine (until 2026-09-05) let the ack
// land MID-frame, then captured the frame's tail from offset 0 and committed it as
// a good frame (the FCS is the MAC's, over the whole frame): the kernel dropped
// those as garbage (rx_dropped 20% of frames on the board, one per retransmitted
// TCP segment, FCS-bad 0) and NFS ran at 0.5 MB/s on a 1 Gbit link.
//
// EVERY frame the engine declines is counted: no free slot at its start (busy),
// FCS bad, longer than a slot (oflow).  Counters cross to ui_clk as raw values
// (they change rarely; a torn read is off by one count).
//
// CDC: the byte ring is a two-clock BRAM (gmii write / ui read); each slot has a
// done toggle (gmii -> ui) and an ack toggle (ui -> gmii); len is written before
// the done toggle and read after the synchronised pulse (quasi-static).
module eth_rx_engine #(
    parameter integer SLOTS      = 8,      // frames held; a power of two
    parameter integer SLOT_BYTES = 2048    // bytes per slot (a frame is <= 1536); a power of two
)(
    // gmii_clk (PHY RX-clock) domain -- from eth_mac_rx
    input  wire        gmii_clk,
    input  wire        gmii_rst,
    input  wire        rx_valid,
    input  wire [ 7:0] rx_data,
    input  wire        rx_last,
    input  wire        rx_good,

    // ui_clk (virtio backend) domain
    input  wire        ui_clk,
    input  wire        ui_rst,
    output wire        frame_valid,   // the oldest held frame is ready to read
    output wire [10:0] frame_len,     // its length (stable while frame_valid)
    input  wire [10:0] rd_addr,       // backend reads frame bytes: rd_data one cycle later
    output reg  [ 7:0] rd_data,
    input  wire        frame_ack,     // 1-cycle: done, release the slot
    output reg  [15:0] drop_count,    // frames declined: no free slot at their first byte
    output reg  [15:0] bad_count,     // frames declined: FCS bad
    output reg  [15:0] oflow_count    // frames declined: longer than a slot
    );

   localparam integer SB = $clog2(SLOTS);
   localparam integer OB = $clog2(SLOT_BYTES);

   // ---- the byte ring: gmii write, ui registered read -------------------
   (* ram_style = "block" *)
   reg [7:0] ram [0:SLOTS*SLOT_BYTES-1];

   // ---- gmii-domain capture ---------------------------------------------
   reg  [SB-1:0]    ws;             // slot being written
   reg  [OB-1:0]    wptr;           // bytes captured so far in it
   reg              in_frame;       // a frame is in progress (first byte seen, no rx_last yet)
   reg              cap;            // ...and it is being captured (decided at its first byte)
   reg              ovf;            // ...and it outgrew the slot
   reg  [SLOTS-1:0] held_g;         // slot committed, ui ack not yet seen
   reg  [OB-1:0]    len_g [0:SLOTS-1];
   reg  [SLOTS-1:0] done_tgl;       // toggles on each commit, per slot
   reg  [SLOTS-1:0] ack_tgl;        // ui -> gmii, per slot
   reg  [2:0]       ack_sync [0:SLOTS-1];
   reg  [15:0]      busy_g, bad_g, oflow_g;

   wire start   = rx_valid & ~in_frame;
   wire cap_now = start ? ~held_g[ws] : cap;    // the decision, at the first byte
   wire wr_ok   = rx_valid & cap_now & ~ovf;
   wire commit  = rx_last & rx_good & cap & ~ovf & (wptr != {OB{1'b0}});

   integer i;
   always @(posedge gmii_clk) begin
      if (gmii_rst) begin
         ws <= {SB{1'b0}}; wptr <= {OB{1'b0}}; in_frame <= 1'b0; cap <= 1'b0; ovf <= 1'b0;
         held_g <= {SLOTS{1'b0}}; done_tgl <= {SLOTS{1'b0}};
         busy_g <= 16'd0; bad_g <= 16'd0; oflow_g <= 16'd0;
         for (i = 0; i < SLOTS; i = i + 1) ack_sync[i] <= 3'd0;
      end else begin
         for (i = 0; i < SLOTS; i = i + 1) begin
            ack_sync[i] <= {ack_sync[i][1:0], ack_tgl[i]};
            if (ack_sync[i][2] ^ ack_sync[i][1]) held_g[i] <= 1'b0;   // backend released it
         end
         if (rx_valid) begin
            in_frame <= 1'b1;
            cap      <= cap_now;
            if (wr_ok) begin
               ram[{ws, wptr}] <= rx_data;
               if (wptr == SLOT_BYTES - 1) ovf <= 1'b1;   // no room for another byte
               else wptr <= wptr + 1'b1;
            end
         end
         if (rx_last) begin
            if (commit) begin
               len_g[ws]    <= wptr;
               done_tgl[ws] <= ~done_tgl[ws];
               held_g[ws]   <= 1'b1;
               ws           <= ws + 1'b1;
            end else if (~rx_good)  bad_g   <= bad_g   + 16'd1;
            else if (~cap)          busy_g  <= busy_g  + 16'd1;
            else if (ovf)           oflow_g <= oflow_g + 16'd1;
            in_frame <= 1'b0; cap <= 1'b0; ovf <= 1'b0; wptr <= {OB{1'b0}};
         end
      end
   end

   // rx_last never rides with a byte, and a frame's first byte never lands while another
   // frame is still open: eth_mac_rx pulses rx_last from its own state after dv drops.
   always @(posedge gmii_clk)
      if (!gmii_rst && rx_last && rx_valid)
         $fatal(1, "eth_rx_engine: rx_last together with rx_valid");

   // ---- ui-domain handoff ------------------------------------------------
   reg  [SB-1:0]    rs;             // slot being read
   reg  [SLOTS-1:0] held_ui;        // committed and not yet acked, as the ui side sees it
   reg  [2:0]       done_sync [0:SLOTS-1];

   assign frame_valid = held_ui[rs];
   /* verilator lint_off WIDTHEXPAND */
   assign frame_len   = len_g[rs];            // written before its done toggle; stable while held
   /* verilator lint_on WIDTHEXPAND */

   always @(posedge ui_clk) begin
      if (ui_rst) begin
         rs <= {SB{1'b0}}; held_ui <= {SLOTS{1'b0}}; ack_tgl <= {SLOTS{1'b0}};
         drop_count <= 16'd0; bad_count <= 16'd0; oflow_count <= 16'd0;
         for (i = 0; i < SLOTS; i = i + 1) done_sync[i] <= 3'd0;
      end else begin
         for (i = 0; i < SLOTS; i = i + 1) begin
            done_sync[i] <= {done_sync[i][1:0], done_tgl[i]};
            if (done_sync[i][2] ^ done_sync[i][1]) held_ui[i] <= 1'b1;
         end
         if (frame_ack && held_ui[rs]) begin
            held_ui[rs] <= 1'b0;
            ack_tgl[rs] <= ~ack_tgl[rs];
            rs          <= rs + 1'b1;
         end
         drop_count  <= busy_g;                 // approximate crossings (rarely change)
         bad_count   <= bad_g;
         oflow_count <= oflow_g;
      end
      rd_data <= ram[{rs, rd_addr[OB-1:0]}];    // the registered read; addr leads by a cycle
   end

   // An ack with nothing held is a backend defect, not a no-op.
   always @(posedge ui_clk)
      if (!ui_rst && frame_ack && !held_ui[rs])
         $fatal(1, "eth_rx_engine: frame_ack with no frame held (rs=%0d)", rs);

endmodule

`default_nettype wire
