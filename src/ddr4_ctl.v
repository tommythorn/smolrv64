`default_nettype none
// ddr4_ctl -- the DDR4 controller above the DDR4 IP's PHY-only mode (AMD PG150, "PHY Only
// Interface"): Xilinx's PHY and calibration initialise and train the DRAM, and from calDone every
// command is this controller's. It runs on ui_clk (333 MHz): a cycle is four DRAM clocks, four
// command slots, slot 0 first, and one BL8 burst of data each way.
//
// THE MEMORY: DDR4-2666, 32 bits (two x16 parts), CL19, CWL14, BL8: a burst is 32 bytes and a
// 64-byte line two bursts. Two bank groups of four banks, 17 row bits, a 4 KiB row a bank.
//
// THE ADDRESS MAP, on the line index of the DRAM window (src/dram_window.vh): a line lives in one
// bank, its two bursts at columns c and c+8 of one row; consecutive lines alternate bank groups,
// so a stream keeps the bus busy (tCCD_S between the groups, tCCD_L within one).
//     line[0] bank group   line[6:1] the line in its row   line[25:9] row
//     line[8:7] ^ row[1:0] ^ row[3:2] bank
// The row bits in the bank put a line and the lines 64 and 128 KiB from it, at the same place in
// their rows, in different banks: a D$ set's lines are 4 KiB apart within 128 KiB, and a stream's
// victim is the line 128 KiB back, so its fills and their write-backs do not take turns at a bank.
// A 2 GiB board's parts lack A16 (row bit 16, line[25]): its upper 2 GiB repeats the lower.
//
// REQUESTS carry a line, read or write, and the requester's buffer (4 bits). The PHY's tag is
// {buffer, half}: a read's two bursts come back on rdDataEn/rdDataAddr straight into that
// buffer, and a write's data stays there until wrDataEn/wrDataAddr ask for it (the PHY takes it
// the next cycle). This controller holds no data.
//
// THE SCHEDULE: one queue of QN requests. Column commands go strictly in queue order, the head's
// two bursts and then the next request's, so the PHY's fixed-latency returns come back in
// request order. Rows open and close ahead of their column commands: a bank's OWNER is the oldest
// queued request for it, and only its owner precharges or activates it -- the head's bank first, then
// the lowest, a PRE and an ACT a cycle at most. A bank stays open until a different row's owner or a refresh
// closes it.
//
// SLOTS: CAS in slot 0, ACT in slot 1, PRE, PREA, REF and ZQCS in slot 3. Every timing is a
// down-counter in cycles, rounded up for these slots (each localparam shows its derivation in
// DRAM clocks); the bench's DRAM model, tb_ddr4_phy, checks every rule in DRAM clocks. A counter
// set to T-1 by a command allows the next T cycles after it.
//
// AND: a refresh every tREFI (PREA, REF), a ZQCS every 65536 refreshes, ODT on writes, and the
// PHY's VT tracking: in every PER_INTVL cycles a read, three cycles with no read CAS, read data
// back and then one gt_data_ready pulse. A read is injected (winInjTxn: its data goes to
// per_rd_done, never to a requester) when none has issued by VT_INJ -- the head's own line ahead
// of the head, whose row is open for it, or into an empty queue an open bank's row (line 0 when
// none is open) -- and reads hold from VT_GAP until the gap has come. A refresh waits for its
// interval's read, so it never squeezes out the injected one.
module ddr4_ctl #(
   parameter integer QN = 8                     // the queue: requests whose rows open ahead
) (
   input  wire          clk,                    // ui_clk
   input  wire          reset,
   input  wire          cal_done,               // the PHY's calDone: nothing is issued before it
   // ---- requests ----
   input  wire          rq_valid,
   output wire          rq_ready,
   input  wire          rq_we,
   input  wire [25:0]   rq_line,
   input  wire [3:0]    rq_buf,
   // ---- the PHY-only IP's controller port ----
   output reg  [7:0]    mc_ACT_n,
   output reg  [135:0]  mc_ADR,                 // A16..A0, eight bits (four slots) a pin
   output reg  [15:0]   mc_BA,
   output reg  [7:0]    mc_BG,
   output reg  [7:0]    mc_CS_n,
   output reg  [7:0]    mc_ODT,
   output wire [7:0]    mc_CKE,
   output reg           mcRdCAS,
   output reg           mcWrCAS,
   output wire [1:0]    mcCasSlot,
   output wire          mcCasSlot2,
   output reg  [4:0]    winBuf,
   output reg           winInjTxn,
   output wire          winRmw,
   output wire [1:0]    winRank,
   output reg           gt_data_ready,
   input  wire          rdDataEn,
   input  wire          per_rd_done
);
   // ---- timings: cycles between the edges that latch the two commands ----
   // Every per-bank and per-group spacing fits TW bits; tRFC and tZQCS, which load all eight banks
   // alike, are one shared counter (t_rf)
   localparam integer TW = 4;
   localparam [TW-1:0] T_RCD   = 4'd5;     // ACT (slot 1) -> CAS (slot 0): 4*5 - 1 = 19 >= tRCD 19
   localparam [TW-1:0] T_RAS   = 4'd11;    // ACT (slot 1) -> PRE (slot 3): 4*11 + 2 = 46 >= tRAS 43
   localparam [TW-1:0] T_RP    = 4'd6;     // PRE (slot 3) -> ACT (slot 1): 4*6 - 2 = 22 >= tRP 19
   localparam [TW-1:0] T_RTP   = 4'd2;     // RD -> PRE: 4*2 + 3 = 11 >= tRTP 10
   localparam [TW-1:0] T_WRP   = 4'd9;     // WR -> PRE: 4*9 + 3 = 39 >= CWL + BL/2 + tWR = 38
   localparam [TW-1:0] T_RRD_S = 4'd2;     // ACT -> ACT, the other group: 8 >= tRRD_S 8
   localparam [TW-1:0] T_RRD_L = 4'd3;     // ACT -> ACT, the same group: 12 >= tRRD_L 9
   localparam [TW-1:0] T_FAW   = 4'd10;    // an ACT -> the fourth after it: 40 >= tFAW 40
   localparam [TW-1:0] T_CCD_L = 4'd2;     // CAS -> CAS, the same group: 8 >= tCCD_L 7 (tCCD_S 4: 1)
   localparam [TW-1:0] T_WTR_L = 4'd7;     // WR -> RD, the same group: 28 >= CWL + BL/2 + tWTR_L 28
   localparam [TW-1:0] T_WTR_S = 4'd6;     // WR -> RD, the other group: 24 >= CWL + BL/2 + tWTR_S 22
   localparam [TW-1:0] T_RTW   = 4'd4;     // RD -> WR: 16 >= RL + BL/2 - WL + 4 = 13 (PG150's spacing)
   localparam [TW-1:0] T_RPREF = 4'd5;     // PRE (slot 3) -> REF (slot 3): 20 >= tRP 19
   localparam [7:0] T_RFC   = 8'd184;   // REF (slot 3) -> ACT (slot 1): 4*184 - 2 = 734 >= tRFC 734
   localparam [7:0] T_ZQCS  = 8'd33;    // ZQCS (slot 3) -> ACT (slot 1): 4*33 - 2 = 130 >= tZQCS 128
   localparam [11:0] T_REFI = 12'd2600; // tREFI 7.8 us = 10400 DRAM clocks
   localparam [8:0] PER_INTVL = 9'd320; // VT tracking: within PG150's 1 us (333 cycles)
   localparam [8:0] VT_INJ  = 9'd200;
   localparam [8:0] VT_GAP  = 9'd280;
   localparam integer QB = $clog2(QN);

   assign mc_CKE     = 8'hFF;
   assign mcCasSlot  = 2'b00;           // every CAS in slot 0
   assign mcCasSlot2 = 1'b0;
   assign winRmw     = 1'b0;
   assign winRank    = 2'b00;

   function [2:0]  bank_of(input [25:0] l); bank_of = {l[0], l[8:7] ^ l[10:9] ^ l[12:11]}; endfunction   // {group, bank}
   function [16:0] row_of (input [25:0] l); row_of  = l[25:9]; endfunction
   function [TW-1:0] dn(input [TW-1:0] t); dn = (t == {TW{1'b0}}) ? {TW{1'b0}} : t - 1'b1; endfunction
   function [TW-1:0] mx(input [TW-1:0] x, input [TW-1:0] y); mx = (x > y) ? x : y; endfunction

   // ---- the queue: entry 0 is the oldest; it shifts down as the head completes ----
   reg          q_v   [0:QN-1];
   reg          q_we  [0:QN-1];
   reg          q_inj [0:QN-1];
   reg          q_hit [0:QN-1];               // its row is open in its bank (kept by ACT and PRE)
   reg [25:0]   q_line[0:QN-1];
   reg [3:0]    q_buf [0:QN-1];
   reg [QB:0]   q_n;                         // entries held
   reg          h_half;                      // the head's first burst has issued

   // ---- the banks and their counters ----
   reg          b_open [0:7];
   reg [16:0]   b_row  [0:7];
   reg [TW-1:0] t_rcd  [0:7];                // until a CAS (tRCD)
   reg [TW-1:0] t_pre  [0:7];                // until a PRE (tRAS, tRTP, write recovery)
   reg [TW-1:0] t_rp   [0:7];                // until an ACT (tRP); and until t_rf is zero
   reg [7:0]    t_rf;                        // until an ACT in any bank (tRFC, tZQCS); the ZQCS waits for it
   reg [TW-1:0] t_actg [0:1];                // until an ACT in this group (tRRD)
   reg [TW-1:0] t_casg [0:1];                // until a CAS in this group (tCCD_L)
   reg [TW-1:0] t_rdg  [0:1];                // until a read CAS in this group (tWTR)
   reg [TW-1:0] t_wr;                        // until a write CAS (read to write)
   reg [TW-1:0] faw    [0:3];                // cycles since each of the last four ACTs, to T_FAW
   reg [TW-1:0] lp;                          // cycles since the last PRE, to T_RPREF
   // each counter's "is zero", registered from its next value: the decisions read a flag, not an
   // eight-bit compare behind a bank mux (asserted equal to the counters every cycle)
   reg          z_rcd [0:7], z_pre [0:7], z_act [0:7], z_actg [0:1], z_casg [0:1], z_rdg [0:1], z_wr;

   // ---- each bank's owner: the oldest queued request for it, registered from the queue, so a
   // cycle behind it. That is safe: an owner leaves only after its column command, which needs its
   // row open, so a stale owner never closes a row a request still wants nor opens a wrong one.
   reg          own_v  [0:7];
   reg [16:0]   own_row[0:7];
   reg          n_own_v  [0:7];
   reg [16:0]   n_own_row[0:7];
   integer      i, b;
   reg [2:0]    bo;
   always @* begin
      bo = 3'd0;
      for (b = 0; b < 8; b = b + 1) begin n_own_v[b] = 1'b0; n_own_row[b] = 17'd0; end
      for (i = QN - 1; i >= 0; i = i - 1)
         if (q_v[i]) begin bo = bank_of(q_line[i]); n_own_v[bo] = 1'b1; n_own_row[bo] = row_of(q_line[i]); end
   end
   // ...and whether the bank's open row is another than its owner's, from the same scan: after an
   // ACT it is a cycle stale, while tRAS forbids the PRE anyway
   reg          own_miss [0:7];
   always @(posedge clk) for (b = 0; b < 8; b = b + 1) begin
      own_v[b] <= n_own_v[b] && !reset && cal_done;  own_row[b] <= n_own_row[b];
      own_miss[b] <= (b_row[b] != n_own_row[b]);
   end

   // ---- refresh, ZQCS and VT tracking state ----
   localparam [1:0] R_RUN = 2'd0, R_CLOSE = 2'd1, R_ZQW = 2'd2;
   reg  [1:0]   r_st;
   reg  [11:0]  r_cnt;                       // to the next refresh
   reg  [3:0]   r_owed;                      // refreshes due and not yet issued
   reg  [15:0]  zq_cnt;                      // refreshes to the next ZQCS
   reg  [8:0]   vt_cnt;
   reg          vt_rd, vt_gapd, vt_data, vt_gt, vt_owed;
   reg  [1:0]   vt_quiet;                    // cycles since a read CAS, to 3
   wire         ref_want = (r_owed != 4'd0) && vt_rd;              // a refresh owed, this interval's read issued
   wire         ref_hold = (r_st != R_RUN) || ref_want;              // no new line, no row command
   wire         vt_block = (vt_cnt >= VT_GAP) && !vt_gapd;          // no read CAS: the gap is owed

   wire [2:0]   h_b    = bank_of(q_line[0]);   // the head's bank and group
   wire         h_g    = h_b[2];

   // ---- the row stage: a PRE and an ACT chosen a cycle ahead of their slots (a later command never
   // breaks a minimum spacing), the head's bank first, then the lowest bank that wants one; no new
   // choice of a kind while one of it is pending (two ACTs are tRRD apart anyway), none while a
   // refresh holds ----
   wire         faw_ok = faw[3] >= T_FAW - 1'b1;
   reg          pre_d, act_d;                // this cycle's choices
   reg  [2:0]   pre_db, act_db;
   reg          pre_go, act_go;              // last cycle's choices, issued in this cycle's slots
   reg  [2:0]   pre_b, act_b;
   reg  [16:0]  act_row;
   reg          pre_c [0:7];
   reg          act_c [0:7];
   reg [2:0]    br;
   always @* begin
      for (b = 0; b < 8; b = b + 1) begin
         br = b[2:0];
         pre_c[b] = b_open[b] && own_v[b] && own_miss[b] && z_pre[b];
         act_c[b] = !b_open[b] && own_v[b] && z_act[b] && z_actg[br[2]] && faw_ok;
      end
      pre_d = 1'b0; pre_db = 3'd0; act_d = 1'b0; act_db = 3'd0;
      for (b = 7; b >= 0; b = b - 1) begin
         if (pre_c[b]) begin pre_d = 1'b1; pre_db = b[2:0]; end
         if (act_c[b]) begin act_d = 1'b1; act_db = b[2:0]; end
      end
      if (pre_c[h_b]) pre_db = h_b;
      if (act_c[h_b]) act_db = h_b;
      if (ref_hold || pre_go) pre_d = 1'b0;
      if (ref_hold || act_go) act_d = 1'b0;
   end
   always @(posedge clk) begin
      pre_go  <= pre_d && !reset && cal_done;  pre_b <= pre_db;
      act_go  <= act_d && !reset && cal_done;  act_b <= act_db;  act_row <= own_row[act_db];
   end

   // ---- the column stage: the head's next burst, or an injected read of the head's line first ----
   wire         h_rdy  = q_v[0] && q_hit[0] && z_rcd[h_b] && z_casg[h_g];
   wire         rd_ok  = z_rdg[h_g] && !vt_block;
   wire         inj_cas = vt_owed && h_rdy && !h_half && rd_ok && !ref_hold;
   wire         cas_go = h_rdy && !inj_cas && (q_we[0] ? z_wr : rd_ok) && (h_half || !ref_hold);
   wire         h_done = cas_go && h_half;   // the head's second burst: it leaves the queue
   wire         rd_cas = inj_cas || (cas_go && !q_we[0]);
   wire         wr_cas = cas_go && q_we[0];

   // ---- the refresh FSM's commands ----
   reg          any_open, all_pre_ok;
   always @* begin
      any_open = 1'b0; all_pre_ok = 1'b1;
      for (b = 0; b < 8; b = b + 1) if (b_open[b]) begin any_open = 1'b1; if (!z_pre[b]) all_pre_ok = 1'b0; end
   end
   wire         row_busy = pre_go || act_go;    // a row-stage command in this cycle's slots
   wire         prea_go = (r_st == R_CLOSE) && !h_half && !row_busy && any_open && all_pre_ok;
   wire         ref_go  = (r_st == R_CLOSE) && !h_half && !row_busy && !any_open && (lp >= T_RPREF);
   wire         zq_go   = (r_st == R_ZQW) && !row_busy && (t_rf == 8'd0);

   // ---- entry: an injected read when the queue is empty -- of an open bank's row (the lowest open
   // bank's, its first line), else of line 0 -- else the requester ----
   wire         inj_enq = vt_owed && (q_n == {(QB+1){1'b0}});
   reg  [25:0]  inj_line;
   always @* begin
      inj_line = 26'd0;
      for (b = 7; b >= 0; b = b - 1) if (b_open[b]) inj_line = {b_row[b], b[1:0] ^ b_row[b][1:0] ^ b_row[b][3:2], 6'd0, b[2]};
   end
   assign rq_ready = cal_done && !inj_enq && (q_n < QN[QB:0]);
   wire         enq    = inj_enq || (rq_valid && rq_ready);
   wire [QB:0]  tail_n = q_n - {{QB{1'b0}}, h_done};
   wire [QB-1:0] tail  = tail_n[QB-1:0];

   // ---- each entry's row open or not, after this cycle's PRE, PREA and ACT; an entering request's
   // from the banks as they will stand ----
   reg          n_hit [0:QN-1];
   reg          e_hit;
   reg  [2:0]   e_b;   reg [16:0] e_r;
   always @* begin
      for (i = 0; i < QN; i = i + 1) begin
         n_hit[i] = q_hit[i];
         if (act_go && bank_of(q_line[i]) == act_b && row_of(q_line[i]) == act_row) n_hit[i] = 1'b1;
         if (prea_go || (pre_go && bank_of(q_line[i]) == pre_b)) n_hit[i] = 1'b0;
      end
      // the requester's line and the injected line, side by side; inj_enq picks last
      e_b = bank_of(rq_line);  e_r = row_of(rq_line);
      e_hit = (act_go && act_b == e_b && act_row == e_r) ||
              (b_open[e_b] && b_row[e_b] == e_r && !(pre_go && pre_b == e_b) && !prea_go);
      if (inj_enq) e_hit = (act_go && act_b == bank_of(inj_line) && act_row == row_of(inj_line)) ||
                           (b_open[bank_of(inj_line)] && b_row[bank_of(inj_line)] == row_of(inj_line) &&
                            !(pre_go && pre_b == bank_of(inj_line)) && !prea_go);
   end

   // ---- the slot encoding ----
   // A16/A15/A14 are RAS_n/CAS_n/WE_n when ACT_n is high: REF 001, PRE 010, WR 100, RD 101, ZQC 110
   localparam [2:0] OP_REF = 3'b001, OP_PRE = 3'b010, OP_WR = 3'b100, OP_RD = 3'b101, OP_ZQC = 3'b110;
   function [7:0] lane(input [7:0] cur, input [1:0] slot, input v); begin
      lane = cur; lane[{slot, 1'b0}] = v; lane[{slot, 1'b1}] = v;
   end endfunction
   reg [7:0]   n_act_n, n_cs_n, n_bg, n_odt;
   reg [135:0] n_adr;
   reg [15:0]  n_ba;
   reg [16:0]  a0, a1, a3;                       // the slots' addresses
   reg [2:0]   b0, b1, b3;                       // ...and banks
   integer     p;
   reg         wr_prev;                          // a write CAS last cycle: its ODT runs on
   always @* begin
      a0 = 17'd0;  a0[16:14] = wr_cas ? OP_WR : OP_RD;  a0[12] = 1'b1;   // BC_n high: BL8
      a0[9:0] = {q_line[0][6:1], h_half, 3'b000};  b0 = h_b;
      a1 = act_row;  b1 = act_b;
      a3 = 17'd0;  a3[16:14] = ref_go ? OP_REF : zq_go ? OP_ZQC : OP_PRE;  a3[10] = prea_go;
      b3 = pre_go ? pre_b : 3'd0;
      n_act_n = 8'hFF;  n_cs_n = 8'hFF;  n_adr = {136{1'b1}};  n_ba = 16'hFFFF;  n_bg = 8'hFF;
      n_odt = wr_prev ? 8'h0F : 8'h00;
      if (rd_cas || wr_cas) begin             // slot 0: a burst
         n_cs_n = lane(n_cs_n, 2'd0, 1'b0);
         for (p = 0; p < 17; p = p + 1) n_adr[p*8 +: 8] = lane(n_adr[p*8 +: 8], 2'd0, a0[p]);
         n_ba[7:0] = lane(n_ba[7:0], 2'd0, b0[0]);  n_ba[15:8] = lane(n_ba[15:8], 2'd0, b0[1]);
         n_bg = lane(n_bg, 2'd0, b0[2]);
         if (wr_cas) n_odt = 8'hFF;
      end
      if (act_go) begin                       // slot 1: an ACT
         n_act_n = lane(n_act_n, 2'd1, 1'b0);  n_cs_n = lane(n_cs_n, 2'd1, 1'b0);
         for (p = 0; p < 17; p = p + 1) n_adr[p*8 +: 8] = lane(n_adr[p*8 +: 8], 2'd1, a1[p]);
         n_ba[7:0] = lane(n_ba[7:0], 2'd1, b1[0]);  n_ba[15:8] = lane(n_ba[15:8], 2'd1, b1[1]);
         n_bg = lane(n_bg, 2'd1, b1[2]);
      end
      if (pre_go || prea_go || ref_go || zq_go) begin   // slot 3: PRE, PREA, REF or ZQCS
         n_cs_n = lane(n_cs_n, 2'd3, 1'b0);
         for (p = 0; p < 17; p = p + 1) n_adr[p*8 +: 8] = lane(n_adr[p*8 +: 8], 2'd3, a3[p]);
         n_ba[7:0] = lane(n_ba[7:0], 2'd3, b3[0]);  n_ba[15:8] = lane(n_ba[15:8], 2'd3, b3[1]);
         n_bg = lane(n_bg, 2'd3, b3[2]);
      end
   end

   integer k;
   always @(posedge clk) begin
      mc_ACT_n <= n_act_n;  mc_CS_n <= n_cs_n;  mc_ADR <= n_adr;  mc_BA <= n_ba;  mc_BG <= n_bg;
      mc_ODT   <= n_odt;
      mcRdCAS  <= rd_cas;
      mcWrCAS  <= wr_cas;
      winBuf   <= {q_buf[0], h_half && !inj_cas};
      winInjTxn <= inj_cas || (cas_go && q_inj[0]);
      wr_prev  <= wr_cas;
      if (reset || !cal_done) begin
         mc_CS_n <= 8'hFF;  mc_ACT_n <= 8'hFF;  mc_ODT <= 8'h00;
         mcRdCAS <= 1'b0;  mcWrCAS <= 1'b0;  winInjTxn <= 1'b0;  wr_prev <= 1'b0;
         for (k = 0; k < QN; k = k + 1) q_v[k] <= 1'b0;
         q_n <= {(QB+1){1'b0}};  h_half <= 1'b0;
         for (k = 0; k < 8; k = k + 1) begin b_open[k] <= 1'b0; t_rcd[k] <= {TW{1'b0}}; t_pre[k] <= {TW{1'b0}}; t_rp[k] <= T_RP; end
         for (k = 0; k < 2; k = k + 1) begin t_actg[k] <= {TW{1'b0}}; t_casg[k] <= {TW{1'b0}}; t_rdg[k] <= {TW{1'b0}}; end
         for (k = 0; k < 4; k = k + 1) faw[k] <= T_FAW;
         t_wr <= {TW{1'b0}};  lp <= T_RPREF;  t_rf <= 8'd0;
         r_st <= R_RUN;  r_cnt <= 12'd0;  r_owed <= 4'd1;  zq_cnt <= 16'hFFFF;
         vt_cnt <= 9'd0;  vt_rd <= 1'b0;  vt_gapd <= 1'b0;  vt_data <= 1'b0;  vt_gt <= 1'b0;
         vt_owed <= 1'b0;  vt_quiet <= 2'd0;  gt_data_ready <= 1'b0;
      end else begin
         // the queue: the head leaves after its second burst; a new request enters at the tail
         for (k = 0; k < QN; k = k + 1) q_hit[k] <= n_hit[k];
         if (h_done) begin
            for (k = 0; k < QN - 1; k = k + 1) begin
               q_v[k] <= q_v[k+1];  q_we[k] <= q_we[k+1];  q_inj[k] <= q_inj[k+1];
               q_line[k] <= q_line[k+1];  q_buf[k] <= q_buf[k+1];  q_hit[k] <= n_hit[k+1];
            end
            q_v[QN-1] <= 1'b0;
         end
         if (enq) begin
            q_v[tail] <= 1'b1;  q_we[tail] <= inj_enq ? 1'b0 : rq_we;  q_inj[tail] <= inj_enq;
            q_line[tail] <= inj_enq ? inj_line : rq_line;  q_buf[tail] <= inj_enq ? 4'd0 : rq_buf;
            q_hit[tail] <= e_hit;
         end
         q_n <= tail_n + {{QB{1'b0}}, enq};
         if (cas_go) h_half <= ~h_half;

         // counters count down; a command sets its own
         for (k = 0; k < 8; k = k + 1) begin t_rcd[k] <= dn(t_rcd[k]); t_pre[k] <= dn(t_pre[k]); t_rp[k] <= dn(t_rp[k]); end
         t_rf <= (t_rf == 8'd0) ? 8'd0 : t_rf - 8'd1;
         for (k = 0; k < 2; k = k + 1) begin t_actg[k] <= dn(t_actg[k]); t_casg[k] <= dn(t_casg[k]); t_rdg[k] <= dn(t_rdg[k]); end
         t_wr <= dn(t_wr);
         for (k = 0; k < 4; k = k + 1) if (faw[k] != T_FAW) faw[k] <= faw[k] + 1'b1;
         if (lp != T_RPREF) lp <= lp + 1'b1;

         if (act_go) begin
            b_open[act_b] <= 1'b1;  b_row[act_b] <= act_row;
            t_rcd[act_b]  <= T_RCD - 1'b1;
            t_pre[act_b]  <= T_RAS - 1'b1;
            t_actg[act_b[2]]  <= T_RRD_L - 1'b1;
            t_actg[~act_b[2]] <= mx(dn(t_actg[~act_b[2]]), T_RRD_S - 1'b1);
            faw[0] <= {TW{1'b0}};
            for (k = 1; k < 4; k = k + 1) faw[k] <= (faw[k-1] == T_FAW) ? T_FAW : faw[k-1] + 1'b1;
         end
         if (pre_go) begin b_open[pre_b] <= 1'b0; t_rp[pre_b] <= mx(dn(t_rp[pre_b]), T_RP - 1'b1); lp <= {TW{1'b0}}; end
         if (prea_go) begin
            for (k = 0; k < 8; k = k + 1) begin b_open[k] <= 1'b0; t_rp[k] <= mx(dn(t_rp[k]), T_RP - 1'b1); end
            lp <= {TW{1'b0}};
         end
         if (ref_go) t_rf <= T_RFC - 8'd1;
         if (zq_go)  t_rf <= T_ZQCS - 8'd1;
         if (rd_cas || wr_cas) t_casg[h_g] <= T_CCD_L - 1'b1;
         if (rd_cas) begin
            t_pre[h_b] <= mx(dn(t_pre[h_b]), T_RTP - 1'b1);
            t_wr       <= T_RTW - 1'b1;
         end
         if (wr_cas) begin
            t_pre[h_b]  <= mx(dn(t_pre[h_b]), T_WRP - 1'b1);
            t_rdg[h_g]  <= T_WTR_L - 1'b1;
            t_rdg[~h_g] <= mx(dn(t_rdg[~h_g]), T_WTR_S - 1'b1);
         end

         // refresh: one owed every tREFI; the head finishes its line, the banks close (PREA once
         // every open bank may precharge), tRP, REF; every 65536th REF is followed, tRFC later, by
         // a ZQCS
         r_cnt <= (r_cnt == T_REFI - 12'd1) ? 12'd0 : r_cnt + 12'd1;
         r_owed <= r_owed + {3'd0, r_cnt == T_REFI - 12'd1} - {3'd0, ref_go};
         case (r_st)
            R_RUN:   if (ref_want && !h_half) r_st <= R_CLOSE;
            R_CLOSE: if (ref_go) begin
                        r_st <= (zq_cnt == 16'd0) ? R_ZQW : R_RUN;
                        zq_cnt <= zq_cnt - 16'd1;
                     end
            R_ZQW:   if (zq_go) r_st <= R_RUN;
            default: r_st <= R_RUN;
         endcase

         // VT tracking
         gt_data_ready <= 1'b0;
         vt_quiet <= rd_cas ? 2'd0 : (vt_quiet == 2'd3) ? 2'd3 : vt_quiet + 2'd1;
         if (vt_cnt == PER_INTVL - 9'd1) begin
            vt_cnt <= 9'd0;  vt_rd <= 1'b0;  vt_gapd <= 1'b0;  vt_data <= 1'b0;  vt_gt <= 1'b0;
         end else begin
            vt_cnt <= vt_cnt + 9'd1;
            if (rd_cas) vt_rd <= 1'b1;
            if (vt_quiet == 2'd3) vt_gapd <= 1'b1;
            if (vt_rd && (rdDataEn || per_rd_done)) vt_data <= 1'b1;
            if (vt_rd && vt_gapd && vt_data && !vt_gt) begin gt_data_ready <= 1'b1; vt_gt <= 1'b1; end
         end
         if (vt_cnt == VT_INJ && !vt_rd) vt_owed <= 1'b1;
         else if (inj_cas || inj_enq || rd_cas) vt_owed <= 1'b0;
      end
   end

   // the zero flags: a counter is zero next cycle when no command sets it and it is at most 1 now
   function lo(input [TW-1:0] t); lo = (t[TW-1:1] == {(TW-1){1'b0}}); endfunction
   wire rf_lo = (t_rf[7:1] == 7'd0);
   always @(posedge clk) begin
      for (k = 0; k < 8; k = k + 1) begin
         z_rcd[k] <= lo(t_rcd[k]) && !(act_go && act_b == k[2:0]);
         z_pre[k] <= lo(t_pre[k]) && !(act_go && act_b == k[2:0]) && !((rd_cas || wr_cas) && h_b == k[2:0]);
         z_act[k] <= lo(t_rp[k]) && rf_lo && !(pre_go && pre_b == k[2:0]) && !prea_go && !ref_go && !zq_go;
      end
      for (k = 0; k < 2; k = k + 1) begin
         z_actg[k] <= lo(t_actg[k]) && !act_go;
         z_casg[k] <= lo(t_casg[k]) && !((rd_cas || wr_cas) && h_g == k[0]);
         z_rdg[k]  <= lo(t_rdg[k]) && !wr_cas;
      end
      z_wr <= lo(t_wr) && !rd_cas;
      if (reset || !cal_done) begin
         for (k = 0; k < 8; k = k + 1) begin z_rcd[k] <= 1'b1; z_pre[k] <= 1'b1; z_act[k] <= 1'b0; end
         for (k = 0; k < 2; k = k + 1) begin z_actg[k] <= 1'b1; z_casg[k] <= 1'b1; z_rdg[k] <= 1'b1; end
         z_wr <= 1'b1;
      end
   end

`ifndef SYNTHESIS
   always @(posedge clk) if (!reset && cal_done) begin
      if (enq && q_n == QN[QB:0] && !h_done)   $fatal(1, "ddr4_ctl: a request entered a full queue");
      if ((rd_cas || wr_cas) && !q_v[0])       $fatal(1, "ddr4_ctl: a CAS with no request at the head");
      if (pre_go && act_go && pre_b == act_b)  $fatal(1, "ddr4_ctl: a PRE and an ACT on bank %0d at once", pre_b);
      if ((rd_cas || wr_cas) && pre_go && pre_b == h_b) $fatal(1, "ddr4_ctl: a CAS and a PRE on bank %0d at once", h_b);
      if ({1'b0, pre_go || prea_go} + {1'b0, ref_go} + {1'b0, zq_go} > 2'd1)
         $fatal(1, "ddr4_ctl: two slot-3 commands at once");
      if (r_owed > 4'd8)                       $fatal(1, "ddr4_ctl: %0d refreshes owed (JEDEC allows 8)", r_owed);
      for (k = 0; k < 8; k = k + 1)
         if (z_rcd[k] != (t_rcd[k] == 0) || z_pre[k] != (t_pre[k] == 0) || z_act[k] != (t_rp[k] == 0 && t_rf == 8'd0))
            $fatal(1, "ddr4_ctl: bank %0d's zero flags (rcd %0d pre %0d act %0d) disagree with its counters (%0d %0d %0d/%0d)",
                   k, z_rcd[k], z_pre[k], z_act[k], t_rcd[k], t_pre[k], t_rp[k], t_rf);
      for (k = 0; k < 2; k = k + 1)
         if (z_actg[k] != (t_actg[k] == 0) || z_casg[k] != (t_casg[k] == 0) || z_rdg[k] != (t_rdg[k] == 0))
            $fatal(1, "ddr4_ctl: group %0d's zero flags disagree with its counters", k);
      if (z_wr != (t_wr == 0)) $fatal(1, "ddr4_ctl: z_wr %0d against t_wr %0d", z_wr, t_wr);
      if (vt_cnt == PER_INTVL - 9'd1 && !vt_gt)
         $fatal(1, "ddr4_ctl: a VT-tracking interval ended without its gt_data_ready (read %0d gap %0d data %0d)",
                vt_rd, vt_gapd, vt_data);
   end
`endif
endmodule

`default_nettype wire
