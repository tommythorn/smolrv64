`default_nettype none

// Sharded physical register file for smolrv64_core.
//
// WHY SHARDS.  A register file with one write port cannot take the results of units that
// complete out of order in the same cycle, and duplicating the array does not help: every
// copy must receive every write, so replication buys READ ports and never WRITE ports.  The
// cheap way to more write ports is to bank by WRITER, so each bank has exactly one, and that
// requires knowing the destination bank at rename.  That is what renaming buys here.
//
// THE INTEGER SHARDS, one per lane (smolrv64_shards.vh): lane k's shard is written by lane
// k's write register alone, which carries its ALU and multiply results and the landings
// (loads, AMOs, CSR reads, the FP ops' integer results, divides).
//
// THE FP FILE holds f0-f31 and nothing else, in a shard per writer: SH_FL is the load
// landing's (FP loads), SH_FF the F stage's (FP results), each written through the address,
// data and wakeup broadcast its writer already has (we_ld, we_fe). Rename knows the writer of
// every f-register it allocates, so the register's number says which shard holds its value, and
// no write ever waits for the other.
//
// SIZING IS A CORRECTNESS FLOOR, not just performance.  A shard must hold every
// architectural register that can map into it, PLUS at least one free, or rename deadlocks:
// with every register mapped and none free, nothing can be renamed, so nothing can commit,
// so nothing is ever freed.  Every integer shard can hold all 32 integer registers, and each
// FP shard all 32 FP registers, so each must exceed 32.  Above that floor it is a stall/area
// choice, and smolrv64_rename exports per-shard stall counters so the choice can be replaced
// by a measurement.
//
// Physical register number: {shard[2:0], idx[IDXB-1:0]}.  Shard in the HIGH bits so a
// shard's pool is a contiguous range and its free list is a plain counter range.
// Physical register 0 is the architectural zero: never written, always reads 0.
//
// THE READS are combinational and there is no write-through: a consumer reads a register two
// cycles after its producer was selected, and the core forwards from the lanes' write
// registers for the one cycle a write is still in flight.

module smolrv64_prf
  #(parameter IDXB  = 7,                   // index bits within a shard
    parameter PBITS = IDXB + 3,            // physical register number width (3 shard bits)
    parameter NL    = 3,                   // lanes: integer shards 0..NL-1
    parameter N_INT = 64,                  // each lane's shard: > 32 (integer arch regs)
    parameter N_FL  = 64,                  // the FP loads' shard: > 32 (fp arch regs)
    parameter N_FF  = 64)                  // the F stage's shard: > 32 (fp arch regs)
   (input  wire                  clk,
    // ---- the writes: lane k's write register into shard k; the LD and FE streams into SH_FL and SH_FF ----
    input  wire [NL-1:0]         we_l,
    input  wire [NL*PBITS-1:0]   wa_l,
    input  wire [NL*64-1:0]      wd_l,
    input  wire                  we_ld,    // the load landing (an FP load)
    input  wire [PBITS-1:0]      wa_ld,
    input  wire [63:0]           wd_ld,
    input  wire                  we_fe,    // the F stage (an FP result)
    input  wire [PBITS-1:0]      wa_fe,
    input  wire [63:0]           wd_fe,
    // ---- the reads, combinational ----
    input  wire [2*NL*PBITS-1:0] ra_l,     // lane k's two operands at [2k] and [2k+1]: integer shards only
    output wire [2*NL*64-1:0]    rd_l,
    input  wire [PBITS-1:0]      ra_sq,    // the store queue's data: an integer or an FP shard
    output wire [63:0]           rd_sq,
    input  wire [3*PBITS-1:0]    ra_f,     // the FP port's three operands: FP shards only
    output wire [3*64-1:0]       rd_f);

`include "smolrv64_shards.vh"
   localparam integer AB_INT = $clog2(N_INT);
   // the read ports: the lanes' 2*NL, then the store queue's, then the FP port's three. The integer
   // shards serve the first NRI (an int-to-FP op reads its integer rs1 in its lane), the FP shards
   // the last NF.
   localparam integer NR  = 2*NL + 4;
   localparam integer NRI = 2*NL + 1;
   localparam integer NF  = 4;
   wire [NR*PBITS-1:0] ra = {ra_f, ra_sq, ra_l};
   // every integer shard's value at the integer ports' index, and each FP shard's at the FP ports'
   // (a wire per read: no function reads an array, rule F4)
   wire [63:0] iv [0:NL*NRI-1];
   wire [63:0] fv [0:2*NF-1];

   genvar gs, gp;
   generate for (gs = 0; gs < NL; gs = gs + 1) begin: ish
      reg [63:0] mem [0:N_INT-1];
      integer j;
      initial begin
         for (j = 0; j < N_INT; j = j + 1) mem[j] = 64'd0;
         // Boot seed: a1 (x11) = the DTB pointer. x11 maps to {SH_IE, 11} at reset
         // (smolrv64_rename's reset arm), so the seed lands in shard 0's entry 11. Sim-only and
         // inert unless a bench passes +a1=; a harness that resets straight to OpenSBI expects
         // the pointer there.
         if (gs == 0) begin : seed
            reg [63:0] a1v;
            if ($value$plusargs("a1=%h", a1v)) mem[11] = a1v;
         end
      end
      always @(posedge clk) if (we_l[gs]) mem[wa_l[gs*PBITS +: AB_INT]] <= wd_l[gs*64 +: 64];
      for (gp = 0; gp < NRI; gp = gp + 1) begin: r
         assign iv[gs*NRI + gp] = mem[ra[gp*PBITS +: AB_INT]];
      end
   end endgenerate

   // the FP shards: fp[0] is SH_FL, written by the load landing alone, fp[1] SH_FF, by the F stage
   generate for (gs = 0; gs < 2; gs = gs + 1) begin: fp
      localparam integer N  = gs ? N_FF : N_FL;
      localparam integer AB = $clog2(N);
      wire               we = gs ? we_fe : we_ld;
      wire [PBITS-1:0]   wa = gs ? wa_fe : wa_ld;
      wire [63:0]        wd = gs ? wd_fe : wd_ld;
      reg  [63:0]        mem [0:N-1];
      integer k;
      initial for (k = 0; k < N; k = k + 1) mem[k] = 64'd0;
      always @(posedge clk) if (we) mem[wa[AB-1:0]] <= wd;
      for (gp = 0; gp < NF; gp = gp + 1) begin: r
         assign fv[gs*NF + gp] = mem[ra[(NR - NF + gp)*PBITS +: AB]];
      end
      always @(posedge clk) if (we & ({1'b0, wa[IDXB-1:0]} >= N[IDXB:0]))
         $fatal(1, "smolrv64_prf: FP shard %0d's write past its %0d entries", gs ? SH_FF : SH_FL, N);
   end endgenerate

   // each port: its integer shard's value or its FP shard's, as the port reads that file; physical
   // register 0 reads 0
   wire [63:0] rd [0:NR-1];
   generate for (gp = 0; gp < NR; gp = gp + 1) begin: rp
      localparam integer FQ = (gp >= NR - NF) ? gp - (NR - NF) : 0;   // its FP port, if it has one
      localparam integer IQ = (gp < NRI) ? gp : 0;                     // its integer port, if it has one
      wire [2:0] sh = ra[gp*PBITS + IDXB +: 3];
      reg  [63:0] x;
      // an integer port reads lane sh[1:0]'s shard (any other is fatal, below); physical register 0 is
      // shard 0's entry 0, zero from configuration and never written (asserted), so it reads 0 as it is
      always @* begin
         x = 64'd0;
         if ((gp < NRI) && ({1'b0, sh[1:0]} < NL[2:0])) x = iv[sh[1:0]*NRI + IQ];
         if ((gp >= NR - NF) && (sh == SH_FL)) x = fv[FQ];
         if ((gp >= NR - NF) && (sh == SH_FF)) x = fv[NF + FQ];
      end
      // a read of a shard this port has none of would read 0 silently: a lane names an f-register,
      // the FP port's second or third operand an integer one, or any port a shard not in use. The FP
      // port may name a vector register (a vector uop's), which the F stage reads from the vector file.
      always @(posedge clk)
         if ((ra[gp*PBITS +: PBITS] != {PBITS{1'b0}})
             & ~((gp < NRI) & (sh < NL[2:0])) & ~((gp >= NR - NF) & ((sh == SH_FL) | (sh == SH_FF) | (sh == SH_V))))
            $fatal(1, "smolrv64_prf: read port %0d (the lanes' 0..%0d, the store queue's %0d, the FP port's %0d..%0d) names pr=%h, of a shard it does not read",
                   gp, 2*NL - 1, 2*NL, 2*NL + 1, 2*NL + 3, ra[gp*PBITS +: PBITS]);
      assign rd[gp] = x;
   end endgenerate
   generate for (gp = 0; gp < 2*NL; gp = gp + 1) begin: rl
      assign rd_l[gp*64 +: 64] = rd[gp];
   end endgenerate
   assign rd_sq = rd[2*NL];
   assign rd_f  = {rd[2*NL + 3], rd[2*NL + 2], rd[2*NL + 1]};

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   // A write must name its own shard and stay inside that shard's capacity; either would
   // otherwise be a silent wrong-register write, which surfaces far from the mistake. The bound
   // is compared at IDXB+1 bits: a 128-entry shard truncates to 0 in IDXB=7 bits.
   integer i;
   always @(posedge clk) begin
      for (i = 0; i < NL; i = i + 1) begin
         if (we_l[i] && (wa_l[i*PBITS + IDXB +: 3] != i[2:0]))
            $fatal(1, "smolrv64_prf: lane %0d's write to pr=%h, shard %0d is not its own",
                   i, wa_l[i*PBITS +: PBITS], wa_l[i*PBITS + IDXB +: 3]);
         if (we_l[i] && ({1'b0, wa_l[i*PBITS +: IDXB]} >= N_INT[IDXB:0]))
            $fatal(1, "smolrv64_prf: lane %0d's write idx %0d >= N_INT %0d", i, wa_l[i*PBITS +: IDXB], N_INT);
      end
      if (we_l[0] && wa_l[0 +: PBITS] == {PBITS{1'b0}})
         $fatal(1, "smolrv64_prf: write to physical register 0 (architectural zero)");
      // an integer result is its lane's, so the load and F-stage ports write their FP shards only
      if (we_ld && (wa_ld[PBITS-1:IDXB] != SH_FL))
         $fatal(1, "smolrv64_prf: load write to pr=%h, shard %0d is not SH_FL", wa_ld, wa_ld[PBITS-1:IDXB]);
      if (we_fe && (wa_fe[PBITS-1:IDXB] != SH_FF))
         $fatal(1, "smolrv64_prf: fp-exec write to pr=%h, shard %0d is not SH_FF", wa_fe, wa_fe[PBITS-1:IDXB]);
   end

   // The deadlock floor, checked once at elaboration rather than argued in a comment.
   initial begin
      if (N_INT <= 32) $fatal(1, "smolrv64_prf: N_INT=%0d must exceed 32 integer arch regs", N_INT);
      if (N_FL <= 32 || N_FF <= 32) $fatal(1, "smolrv64_prf: N_FL=%0d and N_FF=%0d must exceed 32 fp arch regs", N_FL, N_FF);
      if (N_INT > (1 << IDXB) || N_FL > (1 << IDXB) || N_FF > (1 << IDXB))
         $fatal(1, "smolrv64_prf: a shard exceeds IDXB=%0d addressable", IDXB);
      if (NL > 4) $fatal(1, "smolrv64_prf: NL=%0d lanes: the shard number holds four", NL);
   end
endmodule

`default_nettype wire
