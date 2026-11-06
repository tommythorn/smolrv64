`default_nettype none

// A circular array of DEPTH entries in NB distributed-RAM banks (entry p in bank p % NB, at index
// p / NB), written up to NW entries a cycle and read NR entries a cycle, each at rp plus its own
// offset below NB.
//
// WHY. A queue that takes several entries a cycle, held in flops, pays a mux at EVERY entry (which
// writer lands here?) and a DEPTH:1 mux per read; both are the size of the array (rule I7). Banked,
// the writes of one cycle go to distinct banks, each bank takes one write (one write statement per
// LUTRAM bank), and a bank's single read is the entry at or after rp, so the muxes are one per BANK.
//
// THE WRITES. Writer j writes wd[j] at position wp[j] when wv[j]; the positions of one cycle's
// writers are distinct mod NB (consecutive positions of a push, a queue's tail plus a rank), which
// is asserted. A bank's write is its one matching writer, chosen by an index, so the data side is
// an NW:1 mux per bank and bit. BYBANK=1 is the user that rotated its writers to the banks already
// (NW = NB, writer j's position is in bank j, asserted): no mux at all. WLATE=1 registers the writes
// (NR = 1): they land at the next edge, so the writers' logic -- a dispatch decode, a rank, the
// bank pick -- ends at flops instead of the banks' write pins, and a read of a position written the
// cycle before takes the registered data, so rd reads exactly as without it.
//
// THE WRITE CLOCK. The banks are written on wclk: clk for every queue but the frontend's, whose
// decoders run on clk2x and write each half of a bundle at its own edge (smolrv64_frontend). The
// reads are combinational and their users are on clk.
//
// THE READS. rd[k] is the entry at rp + ro[k], combinational, the value before this cycle's writes.
// Bank b is read once, at the entry at or after rp in it, so any offset below NB names an entry
// some bank holds, and a read port is an NB:1 pick of the banks by (rp + ro[k]) % NB. rp must be
// a register's value or close to it: it is every bank's read address (rule I6); the offsets select
// only among the banks' outputs. PICK=1 is the array read at any one position (NR = 1, no offset):
// every bank reads rp's row, and rp's bank is the one taken.
module smolrv64_bankq
  #(parameter W     = 16,                 // entry width
    parameter DEPTH = 32,                 // entries, a multiple of NB
    parameter NW    = 8,                  // writers a cycle
    parameter NR    = 8,                  // consecutive entries read a cycle
    parameter NB    = 8,                  // banks: a power of two, at least NW and NR
    parameter BYBANK = 0,                 // 1: writer j is bank j's (NW = NB)
    parameter PICK  = 0,                  // 1: one read at any position (NR = 1)
    parameter WLATE = 0,                  // 1: the writes land a cycle late, from registers (NR = 1)
    parameter AB    = $clog2(DEPTH))
   (input  wire               clk,
    input  wire               wclk,       // the banks' write clock (clk, or the frontend's clk2x)
    input  wire               reset,      // only the invariant checks: the array has no reset
    input  wire [NW-1:0]      wv,
    input  wire [NW*AB-1:0]   wp,         // writer j's position at [j*AB +: AB]
    input  wire [NW*W-1:0]    wd,
    input  wire [AB-1:0]      rp,
    input  wire [NR*$clog2(NB)-1:0] ro,   // port k's offset from rp at [k*log2(NB) +: log2(NB)]
    output wire [NR*W-1:0]    rd);

   localparam integer BB = $clog2(NB);
   localparam integer BD = DEPTH / NB;    // entries a bank
   localparam integer IB = (BD > 1) ? $clog2(BD) : 1;
   localparam integer JB = (NW > 1) ? $clog2(NW) : 1;
   initial if ((1 << BB) != NB || NB < NW || DEPTH % NB != 0 || BD < 1 || (1 << AB) != DEPTH
               || (BYBANK && NW != NB) || (PICK && NR != 1) || (WLATE && NR != 1))
      $fatal(1, "smolrv64_bankq: DEPTH=%0d NB=%0d NW=%0d AB=%0d BYBANK=%0d: NB is a power of two >= NW, dividing DEPTH = 2**AB (= NW when BYBANK)",
             DEPTH, NB, NW, AB, BYBANK);

   // the writes as the banks take them: this cycle's, or (WLATE) last cycle's from registers
   wire [NW-1:0]    bv;
   wire [NW*AB-1:0] bp;
   wire [NW*W-1:0]  bw;
   generate if (WLATE) begin: late
      reg [NW-1:0]    v_q;
      reg [NW*AB-1:0] p_q;
      reg [NW*W-1:0]  d_q;
      initial v_q = {NW{1'b0}};
      always @(posedge wclk) begin
         v_q <= reset ? {NW{1'b0}} : wv;
         p_q <= wp;  d_q <= wd;
      end
      assign bv = v_q;  assign bp = p_q;  assign bw = d_q;
   end else begin: now
      assign bv = wv;  assign bp = wp;  assign bw = wd;
   end endgenerate

   wire [W-1:0] brd [0:NB-1];             // each bank's entry at or after rp
   genvar gb, gk;
   generate for (gb = 0; gb < NB; gb = gb + 1) begin: bank
      reg [W-1:0] mem [0:BD-1];
      integer i;
      initial for (i = 0; i < BD; i = i + 1) mem[i] = {W{1'b0}};
      // its writer: the one whose position is in this bank
      wire [NW-1:0] m;
      wire [JB-1:0] sel;
      if (BYBANK) begin: own
         assign m   = bv & ({{(NW-1){1'b0}}, 1'b1} << gb);
         assign sel = gb[JB-1:0];
         always @(posedge wclk) if (!reset & bv[gb] & (bp[gb*AB +: BB] != gb[BB-1:0]))
            $fatal(1, "smolrv64_bankq: bank %0d's writer writes position %0d, of bank %0d", gb, bp[gb*AB +: AB], bp[gb*AB +: BB]);
      end else begin: pick
         reg [NW-1:0] mm;
         reg [JB-1:0] ss;
         integer j;
         always @* begin
            ss = {JB{1'b0}};
            for (j = 0; j < NW; j = j + 1) begin
               mm[j] = bv[j] & (bp[j*AB +: BB] == gb[BB-1:0]);
               if (mm[j]) ss = j[JB-1:0];
            end
         end
         assign m = mm;  assign sel = ss;
      end
      wire [AB-1:0] wpos = bp[sel*AB +: AB];
      wire [IB-1:0] widx = (BD > 1) ? wpos[AB-1 -: IB] : {IB{1'b0}};
      always @(posedge wclk) if (|m) mem[widx] <= bw[sel*W +: W];
      always @(posedge wclk) if (!reset & ((m & (m - 1'b1)) != {NW{1'b0}}))
         $fatal(1, "smolrv64_bankq: writers %b all land in bank %0d", m, gb);
      // its read: the entry at or after rp
      wire [BB-1:0] off  = gb[BB-1:0] - rp[BB-1:0];
      wire [AB-1:0] rpos = rp + {{(AB-BB){1'b0}}, off};
      wire [IB-1:0] ridx = (BD == 1) ? {IB{1'b0}} : PICK ? rp[AB-1 -: IB] : rpos[AB-1 -: IB];
      assign brd[gb] = mem[ridx];
   end endgenerate
   generate for (gk = 0; gk < NR; gk = gk + 1) begin: rk
      wire [BB-1:0] b = rp[BB-1:0] + ro[gk*BB +: BB];
      if (WLATE) begin: byp
         // the position read, against the writes still in the registers (distinct positions)
         wire [AB-1:0] q = PICK ? rp : rp + {{(AB-BB){1'b0}}, ro[gk*BB +: BB]};
         reg           hit;
         reg  [W-1:0]  hd;
         integer j;
         always @* begin
            hit = 1'b0;  hd = {W{1'b0}};
            for (j = 0; j < NW; j = j + 1)
               if (bv[j] & (bp[j*AB +: AB] == q)) begin hit = 1'b1;  hd = bw[j*W +: W]; end
         end
         assign rd[gk*W +: W] = hit ? hd : brd[b];
      end else begin: now
         assign rd[gk*W +: W] = brd[b];
      end
   end endgenerate
endmodule

`default_nettype wire
