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
// THE FP FILE holds f0-f31 and nothing else, in one slice per rename slot (slot k's is
// shard 4 + k, smolrv64_rename). Its values have two writers, the load landing (FP loads)
// and the F stage (FP results), so each slice has a BANK PER WRITER, written through the
// address, data and wakeup broadcast that writer already has (we_ld, we_fe), and a live-value
// bit per register, set by the write, picks the bank a read takes. A physical register is
// written once per allocation, so the bit is exact, and no write ever waits for the other.
//
// SIZING IS A CORRECTNESS FLOOR, not just performance.  A shard must hold every
// architectural register that can map into it, PLUS at least one free, or rename deadlocks:
// with every register mapped and none free, nothing can be renamed, so nothing can commit,
// so nothing is ever freed.  Every integer shard can hold all 32 integer registers, and every
// FP slice all 32 FP registers, so each must exceed 32.  Above that floor it is a stall/area
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
    parameter NL    = 3,                   // lanes: integer shards 0..NL-1, FP slices 4..4+NL-1
    parameter N_INT = 64,                  // each lane's shard: > 32 (integer arch regs)
    parameter N_FP  = 64)                  // each FP slice: > 32 (fp arch regs)
   (input  wire                  clk,
    // ---- the writes: lane k's write register into shard k; the LD and FE streams into the FP slices ----
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
    input  wire [PBITS-1:0]      ra_sq,    // the store queue's data: an integer shard or an FP slice
    output wire [63:0]           rd_sq,
    input  wire [3*PBITS-1:0]    ra_f,     // the F/CTF port's three operands: either
    output wire [3*64-1:0]       rd_f);

`include "smolrv64_shards.vh"
   localparam integer AB_INT = $clog2(N_INT);
   localparam integer AB_FP  = $clog2(N_FP);
   // the read ports: the lanes' 2*NL, then the store queue's, then the F port's three; the last
   // NF read the FP file too
   localparam integer NR = 2*NL + 4;
   localparam integer NF = 4;
   wire [NR*PBITS-1:0] ra = {ra_f, ra_sq, ra_l};
   // every shard's value at every port's index, and every FP slice's at the FP ports' (a wire per
   // read: no function reads an array, rule F4)
   wire [63:0] iv [0:NL*NR-1];
   wire [63:0] fv [0:NL*NF-1];

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
      for (gp = 0; gp < NR; gp = gp + 1) begin: r
         assign iv[gs*NR + gp] = mem[ra[gp*PBITS +: AB_INT]];
      end
   end endgenerate

   generate for (gs = 0; gs < NL; gs = gs + 1) begin: fp
      reg [63:0]     mem_l [0:N_FP-1];   // written by the load port
      reg [63:0]     mem_f [0:N_FP-1];   // written by the F stage's port
      reg [N_FP-1:0] lvt;                // 1: mem_f holds the register's value
      localparam [2:0] SH = SH_F0 + gs;
      wire wl = we_ld & (wa_ld[PBITS-1:IDXB] == SH);
      wire wf = we_fe & (wa_fe[PBITS-1:IDXB] == SH);
      wire [AB_FP-1:0] il = wa_ld[AB_FP-1:0], jf = wa_fe[AB_FP-1:0];
      integer k;
      initial begin
         for (k = 0; k < N_FP; k = k + 1) begin mem_l[k] = 64'd0; mem_f[k] = 64'd0; end
         lvt = {N_FP{1'b0}};
      end
      always @(posedge clk) begin
         if (wl) mem_l[il] <= wd_ld;
         if (wf) mem_f[jf] <= wd_fe;
         if (wl) lvt[il] <= 1'b0;
         if (wf) lvt[jf] <= 1'b1;
      end
      for (gp = 0; gp < NF; gp = gp + 1) begin: r
         wire [AB_FP-1:0] x = ra[(NR - NF + gp)*PBITS +: AB_FP];
         assign fv[gs*NF + gp] = lvt[x] ? mem_f[x] : mem_l[x];
      end
      always @(posedge clk) begin
         if (wl & wf & (il == jf))
            $fatal(1, "smolrv64_prf: FP slice %0d register %0d written by the load and the F stage at once", gs, il);
         if ((wl & ({1'b0, wa_ld[IDXB-1:0]} >= N_FP[IDXB:0])) | (wf & ({1'b0, wa_fe[IDXB-1:0]} >= N_FP[IDXB:0])))
            $fatal(1, "smolrv64_prf: FP slice %0d write past N_FP %0d", gs, N_FP);
      end
   end endgenerate

   // each port: its shard's value, or (the FP ports) its FP slice's; physical register 0 reads 0
   wire [63:0] rd [0:NR-1];
   generate for (gp = 0; gp < NR; gp = gp + 1) begin: rp
      localparam integer FQ = (gp >= NR - NF) ? gp - (NR - NF) : 0;   // its FP port, if it has one
      wire [2:0] sh = ra[gp*PBITS + IDXB +: 3];
      reg  [63:0] x;
      integer k;
      always @* begin
         x = 64'd0;
         for (k = 0; k < NL; k = k + 1) begin
            if (sh == k[2:0]) x = iv[k*NR + gp];
            if ((gp >= NR - NF) && (sh == SH_F0 + k[2:0])) x = fv[k*NF + FQ];
         end
      end
      assign rd[gp] = (ra[gp*PBITS +: PBITS] == {PBITS{1'b0}}) ? 64'd0 : x;
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
         if (ra_l[(2*i)*PBITS + IDXB + 2] | ra_l[(2*i+1)*PBITS + IDXB + 2])
            $fatal(1, "smolrv64_prf: lane %0d's read port names an FP register", i);
      end
      if (we_l[0] && wa_l[0 +: PBITS] == {PBITS{1'b0}})
         $fatal(1, "smolrv64_prf: write to physical register 0 (architectural zero)");
      // an integer result is its lane's, so the load and F-stage ports write the FP slices only
      if (we_ld && (wa_ld[PBITS-1:IDXB] < SH_F0))
         $fatal(1, "smolrv64_prf: load write to pr=%h, shard %0d is not an FP slice", wa_ld, wa_ld[PBITS-1:IDXB]);
      if (we_fe && (wa_fe[PBITS-1:IDXB] < SH_F0))
         $fatal(1, "smolrv64_prf: fp-exec write to pr=%h, shard %0d is not an FP slice", wa_fe, wa_fe[PBITS-1:IDXB]);
   end

   // The deadlock floor, checked once at elaboration rather than argued in a comment.
   initial begin
      if (N_INT <= 32) $fatal(1, "smolrv64_prf: N_INT=%0d must exceed 32 integer arch regs", N_INT);
      if (N_FP <= 32)  $fatal(1, "smolrv64_prf: N_FP=%0d must exceed 32 fp arch regs", N_FP);
      if (N_INT > (1 << IDXB) || N_FP > (1 << IDXB))
         $fatal(1, "smolrv64_prf: a shard exceeds IDXB=%0d addressable", IDXB);
      if (NL > 4) $fatal(1, "smolrv64_prf: NL=%0d lanes: the shard number holds four", NL);
   end
endmodule

`default_nettype wire
