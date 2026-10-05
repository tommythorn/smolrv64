`default_nettype none

// Sharded physical register file for smolrv64_core.
//
// WHY SHARDS.  rv_regfile is a unified 64-entry array with ONE write port
// (`always @(posedge clk) if (we) r[wa] <= wd;`).  The moment completion goes out of order
// -- which is the whole point of the scoreboard/OoO work -- the LSU, the ALU and the FPU
// contend for it.  Duplicating the array does NOT help: every copy must receive every
// write, so replication buys READ ports and never WRITE ports.  The only cheap way to more
// write ports is to bank by WRITER, so each bank has exactly one, and that requires knowing
// the destination bank at rename.  That is what renaming buys here.
//
// THE INTEGER SHARDS, one writer each:
//
//   SH_IE, SH_IE2, SH_IE3  the lanes A, B, C: each lane's write register, which carries its ALU
//                  and multiply results and the landings (loads, AMOs, CSR reads, the FP ops'
//                  integer results, divides)
//   SH_LD          none: an integer load's, AMO's or CSR read's result lands in its lane's shard
//   SH_FE          none: an FP op's or divide's integer result lands in its lane's shard
//
// THE FP FILE holds f0-f31 and nothing else, in three slices SH_F0..SH_F2, one per rename slot
// (smolrv64_rename). Its values have two writers, the load landing (FP loads) and the F
// stage (FP results), so each slice has a BANK PER WRITER, written through the address, data
// and wakeup broadcast that writer already has (we_ld, we_fe), and a live-value bit per
// register, set by the write, picks the bank a read takes. A physical register is written
// once per allocation, so the bit is exact, and no write ever waits for the other.
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

module smolrv64_prf
  #(parameter IDXB  = 7,                   // index bits within a shard
    parameter PBITS = IDXB + 3,            // physical register number width (3 shard bits: room for 5 shards)
    parameter N_IE  = 64,                  // > 32 (integer arch regs)
    parameter N_LD  = 64,                   // > 32
    parameter N_FE  = 64,                   // > 32
    parameter N_FP  = 64,                   // each FP slice: > 32 (fp arch regs)
    parameter N_IE2 = 64,                   // the SECOND ALU's shard (item 10d-ii): > 32 like SH_IE
    parameter N_IE3 = 64,                   // the THIRD ALU's shard (Stage 3): > 32 like SH_IE
    parameter WRTHRU = 0)                   // see the write-through note below
   (input  wire             clk,

    // ---- write ports: ONE address, THREE data buses -- one per shard's own writer ----
    // THE POINT OF SHARDING BY WRITER IS THAT EACH SHARD TAKES ITS OWN WRITER'S DATA.
    //
    // Each shard is 3 LUTRAM copies (one per read port: rs1/rs2/rs3), so 3 shards is 9
    // arrays.  A single shared write bus reaches all 9 -- gated by enable, but physically
    // connected.  Routing the LSU's load data through the global writeback mux and then to
    // every array is the shape sharding exists to avoid: the D$ read data should reach the
    // 3 load-shard copies and nothing else, and the ALU result should never leave the
    // int-exec shard.
    //
    // The ADDRESS stays shared: there is one writeback per cycle, so one destination
    // register number.  Only the data is per-writer.  When out-of-order writeback lands,
    // each shard's bus is driven by its own unit and they still never contend for a bank.
    input  wire             we_ie,
    input  wire             we_ld,
    input  wire             we_fe,
    input  wire             we_ie2,
    input  wire             we_ie3,   // the third ALU (Stage 3)

    // ONE ADDRESS PER SHARD. They shared a single `wa` while exactly one writeback could
    // happen per cycle; dynamic issue makes simultaneous completions the normal case, and
    // docs/Area-Efficient-Scalar-OoO.md 7 names splitting the file as the way to delete
    // writeback arbitration entirely -- but only if each file has its own port. This is
    // that port. It changes nothing on its own: the M stage still yields the cycle to a
    // landing load or FP result, because the ROB's single completion port has not been
    // widened yet.
    input  wire [PBITS-1:0] wa_ie,
    input  wire [PBITS-1:0] wa_ld,
    input  wire [PBITS-1:0] wa_fe,
    input  wire [PBITS-1:0] wa_ie2,
    input  wire [PBITS-1:0] wa_ie3,

    input  wire [63:0]      wd_ie,    // ALU / CSR result
    input  wire [63:0]      wd_ld,    // LSU load data (an FP load's too), M's CSR result
    input  wire [63:0]      wd_fe,    // the F stage's result (an f-register's too), mul/div, link
    input  wire [63:0]      wd_ie2,   // the second ALU's result
    input  wire [63:0]      wd_ie3,   // the third ALU's result


    // ---- combinational read ports. Only ra2 (M's store data) and ra10-12 (the F/CTF port)
    // read the FP file; the others read integer shards alone. ----
    input  wire [PBITS-1:0] ra1,
    input  wire [PBITS-1:0] ra2,
    input  wire [PBITS-1:0] ra3,
    output wire [63:0]      rd1,
    output wire [63:0]      rd2,
        output wire [63:0]      rd3,
    input  wire [PBITS-1:0] ra4,      // the ALU's own issue port (item 10d-i)
    input  wire [PBITS-1:0] ra5,
    output wire [63:0]      rd4,
        output wire [63:0]      rd5,
    input  wire [PBITS-1:0] ra6,      // the second ALU port (item 10d-ii)
    input  wire [PBITS-1:0] ra7,
    output wire [63:0]      rd6,
    output wire [63:0]      rd7,
    input  wire [PBITS-1:0] ra8,      // the third ALU port (Stage 3)
    input  wire [PBITS-1:0] ra9,
    output wire [63:0]      rd8,
    output wire [63:0]      rd9,
    input  wire [PBITS-1:0] ra10,     // the independent F/CTF port -- its own three reads
    input  wire [PBITS-1:0] ra11,     // (rs1/rs2 for a branch or FMA, rs3 for FMA) so the F
    input  wire [PBITS-1:0] ra12,     // stage no longer borrows M's read ports (CTF-on-FP)
    output wire [63:0]      rd10,
    output wire [63:0]      rd11,
    output wire [63:0]      rd12);

   localparam [2:0] SH_IE = 3'd0, SH_LD = 3'd1, SH_FE = 3'd2, SH_IE2 = 3'd3, SH_IE3 = 3'd4,
                    SH_F0 = 3'd5, SH_F1 = 3'd6, SH_F2 = 3'd7;
   localparam integer AB_FP = $clog2(N_FP);

   // Sized to the largest shard; the smaller shards simply never index above their
   // capacity, which smolrv64_rename's free list enforces and the assertion below checks.
   // EACH ARRAY IS SIZED TO ITS OWN SHARD.  The first cut sized all three to the largest
   // (NMAX), so mem_ie was 128 deep with N_IE=64 -- half of it unreachable, and synthesis
   // duly built it: "mem_ie_reg 128 x 64, RAM64M8 x 60", identical to the 128-entry shards.
   // Pure waste, and area is not free here: the 166 MHz build fails on ROUTING inside the
   // caches (83-85% route on sub-1 ns logic), so congestion costs slack somewhere else.
   localparam integer NMAX  = (N_LD > N_IE) ? ((N_LD > N_FE) ? N_LD : N_FE)
                                            : ((N_IE > N_FE) ? N_IE : N_FE);
   localparam integer AB_IE = $clog2(N_IE), AB_LD = $clog2(N_LD), AB_FE = $clog2(N_FE), AB_IE2 = $clog2(N_IE2), AB_IE3 = $clog2(N_IE3);

   reg [63:0] mem_ie [0:N_IE-1];
   reg [63:0] mem_ie2 [0:N_IE2-1];
   reg [63:0] mem_ie3 [0:N_IE3-1];

   wire [2:0]      sh1 = ra1[PBITS-1:IDXB], sh2 = ra2[PBITS-1:IDXB], sh3 = ra3[PBITS-1:IDXB];
   wire [IDXB-1:0] ix1 = ra1[IDXB-1:0],     ix2 = ra2[IDXB-1:0],     ix3 = ra3[IDXB-1:0];
   wire [2:0]      sh4 = ra4[PBITS-1:IDXB], sh5 = ra5[PBITS-1:IDXB], sh6 = ra6[PBITS-1:IDXB], sh7 = ra7[PBITS-1:IDXB];
   wire [IDXB-1:0] ix4 = ra4[IDXB-1:0],     ix5 = ra5[IDXB-1:0],     ix6 = ra6[IDXB-1:0],     ix7 = ra7[IDXB-1:0];
   wire [2:0]      sh8 = ra8[PBITS-1:IDXB], sh9 = ra9[PBITS-1:IDXB];
   wire [IDXB-1:0] ix8 = ra8[IDXB-1:0],     ix9 = ra9[IDXB-1:0];
   wire [2:0]      sh10 = ra10[PBITS-1:IDXB], sh11 = ra11[PBITS-1:IDXB], sh12 = ra12[PBITS-1:IDXB];
   wire [IDXB-1:0] ix10 = ra10[IDXB-1:0],     ix11 = ra11[IDXB-1:0],     ix12 = ra12[IDXB-1:0];

   // WRITE-THROUGH, and why it is OFF by default.
   //
   // docs/Area-Efficient-Scalar-OoO.md 14.1 makes it load-bearing for the OoO machine: a
   // consumer issuing in the cycle its producer writes back must see the new value, and
   // "an implementation that registers any of these is a different machine".
   //
   // With IN-ORDER issue it is dead code.  The write targets m_prd, the physical register
   // allocated for m_rd; renaming makes physical registers unique, so a source resolves to
   // m_prd only when that source IS m_rd -- which is exactly smolrv64_core's byp1/2/3, and there
   // x_rs takes m_byp_val, never prf_rs.  So the collision can happen but its result is
   // never used.
   //
   // It is not free: 3 read ports x 3 shards of PBITS comparator plus a 64-bit mux, sitting
   // in the operand read path -- the back-to-back ALU loop that must stay fast.  And on this
   // die area is congestion and congestion is slack (docs/rtl-rules.md I1).
   //
   // Turning it on is NOT something to remember: smolrv64_core asserts on a read that collides
   // with the writeback and is not bypassed, so the machine says when this becomes needed.
   function automatic [63:0] rd_shard;
      input [2:0]      sh;
      input [IDXB-1:0] ix;
      input [63:0]     m_ie, m_ie2, m_ie3;
      begin
         case (sh)
           SH_IE: rd_shard = (WRTHRU != 0 && we_ie && wa_ie[IDXB-1:0] == ix
                              && wa_ie[PBITS-1:IDXB] == SH_IE) ? wd_ie : m_ie;
           SH_IE2: rd_shard = (WRTHRU != 0 && we_ie2 && wa_ie2[IDXB-1:0] == ix
                              && wa_ie2[PBITS-1:IDXB] == SH_IE2) ? wd_ie2 : m_ie2;
           SH_IE3: rd_shard = (WRTHRU != 0 && we_ie3 && wa_ie3[IDXB-1:0] == ix
                              && wa_ie3[PBITS-1:IDXB] == SH_IE3) ? wd_ie3 : m_ie3;
           default: rd_shard = 64'd0;
         endcase
      end
   endfunction

   // Physical register 0 reads 0 unconditionally -- it is the architectural zero and is
   // never allocated by smolrv64_rename, so no write can target it.
   // Index each array with only the bits it has.  A read of a shard the operand does not
   // belong to is discarded by rd_shard's case, so a truncated index there is harmless --
   // but it must not be OUT OF RANGE, which for a smaller shard it otherwise would be.
   assign rd1 = (ra1 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh1, ix1, mem_ie[ix1[AB_IE-1:0]],
                          mem_ie2[ix1[AB_IE2-1:0]], mem_ie3[ix1[AB_IE3-1:0]]);
   assign rd2 = (ra2 == {PBITS{1'b0}}) ? 64'd0
              : (sh2 >= SH_F0) ? (sh2 == SH_F0 ? fp_r2[0] : sh2 == SH_F1 ? fp_r2[1] : fp_r2[2])
              : rd_shard(sh2, ix2, mem_ie[ix2[AB_IE-1:0]],
                          mem_ie2[ix2[AB_IE2-1:0]], mem_ie3[ix2[AB_IE3-1:0]]);
   assign rd3 = (ra3 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh3, ix3, mem_ie[ix3[AB_IE-1:0]],
                          mem_ie2[ix3[AB_IE2-1:0]], mem_ie3[ix3[AB_IE3-1:0]]);
   assign rd4 = (ra4 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh4, ix4, mem_ie[ix4[AB_IE-1:0]],
                          mem_ie2[ix4[AB_IE2-1:0]], mem_ie3[ix4[AB_IE3-1:0]]);
   assign rd5 = (ra5 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh5, ix5, mem_ie[ix5[AB_IE-1:0]],
                          mem_ie2[ix5[AB_IE2-1:0]], mem_ie3[ix5[AB_IE3-1:0]]);
   assign rd6 = (ra6 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh6, ix6, mem_ie[ix6[AB_IE-1:0]],
                          mem_ie2[ix6[AB_IE2-1:0]], mem_ie3[ix6[AB_IE3-1:0]]);
   assign rd7 = (ra7 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh7, ix7, mem_ie[ix7[AB_IE-1:0]],
                          mem_ie2[ix7[AB_IE2-1:0]], mem_ie3[ix7[AB_IE3-1:0]]);
   assign rd8 = (ra8 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh8, ix8, mem_ie[ix8[AB_IE-1:0]],
                          mem_ie2[ix8[AB_IE2-1:0]], mem_ie3[ix8[AB_IE3-1:0]]);
   assign rd9 = (ra9 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh9, ix9, mem_ie[ix9[AB_IE-1:0]],
                          mem_ie2[ix9[AB_IE2-1:0]], mem_ie3[ix9[AB_IE3-1:0]]);
   assign rd10 = (ra10 == {PBITS{1'b0}}) ? 64'd0
              : (sh10 >= SH_F0) ? (sh10 == SH_F0 ? fp_r10[0] : sh10 == SH_F1 ? fp_r10[1] : fp_r10[2])
              : rd_shard(sh10, ix10, mem_ie[ix10[AB_IE-1:0]],
                          mem_ie2[ix10[AB_IE2-1:0]], mem_ie3[ix10[AB_IE3-1:0]]);
   assign rd11 = (ra11 == {PBITS{1'b0}}) ? 64'd0
              : (sh11 >= SH_F0) ? (sh11 == SH_F0 ? fp_r11[0] : sh11 == SH_F1 ? fp_r11[1] : fp_r11[2])
              : rd_shard(sh11, ix11, mem_ie[ix11[AB_IE-1:0]],
                          mem_ie2[ix11[AB_IE2-1:0]], mem_ie3[ix11[AB_IE3-1:0]]);
   assign rd12 = (ra12 == {PBITS{1'b0}}) ? 64'd0
              : (sh12 >= SH_F0) ? (sh12 == SH_F0 ? fp_r12[0] : sh12 == SH_F1 ? fp_r12[1] : fp_r12[2])
              : rd_shard(sh12, ix12, mem_ie[ix12[AB_IE-1:0]],
                          mem_ie2[ix12[AB_IE2-1:0]], mem_ie3[ix12[AB_IE3-1:0]]);

   // ---- the FP file: three slices, a bank per writer, a live-value bit per register ----
   wire [63:0] fp_r2 [0:2], fp_r10 [0:2], fp_r11 [0:2], fp_r12 [0:2];
   genvar gF;
   generate for (gF = 0; gF < 3; gF = gF + 1) begin: fp
      reg [63:0]     mem_l [0:N_FP-1];   // written by the load port
      reg [63:0]     mem_f [0:N_FP-1];   // written by the F stage's port
      reg [N_FP-1:0] lvt;                // 1: mem_f holds the register's value
      wire wl = we_ld & (wa_ld[PBITS-1:IDXB] == SH_F0 + gF);
      wire wf = we_fe & (wa_fe[PBITS-1:IDXB] == SH_F0 + gF);
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
      wire [AB_FP-1:0] x2 = ix2[AB_FP-1:0], x10 = ix10[AB_FP-1:0], x11 = ix11[AB_FP-1:0], x12 = ix12[AB_FP-1:0];
      assign fp_r2[gF]  = lvt[x2]  ? mem_f[x2]  : mem_l[x2];
      assign fp_r10[gF] = lvt[x10] ? mem_f[x10] : mem_l[x10];
      assign fp_r11[gF] = lvt[x11] ? mem_f[x11] : mem_l[x11];
      assign fp_r12[gF] = lvt[x12] ? mem_f[x12] : mem_l[x12];
      always @(posedge clk) begin
         if (wl & wf & (il == jf))
            $fatal(1, "smolrv64_prf: FP slice %0d register %0d written by the load and the F stage at once", gF, il);
         if ((wl & ({1'b0, wa_ld[IDXB-1:0]} >= N_FP[IDXB:0])) | (wf & ({1'b0, wa_fe[IDXB-1:0]} >= N_FP[IDXB:0])))
            $fatal(1, "smolrv64_prf: FP slice %0d write past N_FP %0d", gF, N_FP);
      end
   end endgenerate

   integer j;
   initial begin
      for (j = 0; j < N_IE; j = j + 1) mem_ie[j] = 64'd0;
      // Boot seed, mirroring rv_regfile's: a1 (x11) = the DTB pointer.  x11 maps to
      // {SH_IE, 11} at reset (see smolrv64_rename's reset arm), so the seed lands in mem_ie[11].
      // Sim-only and inert unless a TB passes +a1=, but NOT optional: a harness that resets
      // straight to OpenSBI expects the pointer there, and without this the shadow check
      // fires 255 cycles into Linux boot -- which is exactly how this omission was found.
      begin : seed reg [63:0] a1v;
         if ($value$plusargs("a1=%h", a1v)) mem_ie[11] = a1v;
      end
   end

   always @(posedge clk) begin
      if (we_ie) mem_ie[wa_ie[AB_IE-1:0]] <= wd_ie;
      if (we_ie2) mem_ie2[wa_ie2[AB_IE2-1:0]] <= wd_ie2;
      if (we_ie3) mem_ie3[wa_ie3[AB_IE3-1:0]] <= wd_ie3;
   end

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   // The bound is compared at IDXB+1 bits: N_LD=128 truncates to 0 in IDXB=7 bits, which
   // silently turns the check into `idx >= 0` and fires on every write.
   // A write must name its own shard and stay inside that shard's capacity.  Either would
   // otherwise be a silent wrong-register write -- exactly the class of defect that costs
   // days here, because the value surfaces far from the mistake.
   always @(posedge clk) begin
      if (we_ie && (wa_ie[PBITS-1:IDXB] != SH_IE))
         $fatal(1, "smolrv64_prf: int-exec write to pr=%h, shard %0d is not SH_IE",
                wa_ie, wa_ie[PBITS-1:IDXB]);
      // SH_LD and SH_FE have no bank: an integer result is its lane's, so the load and F-stage
      // ports write the FP slices only
      if (we_ld && (wa_ld[PBITS-1:IDXB] < SH_F0))
         $fatal(1, "smolrv64_prf: load write to pr=%h, shard %0d is not an FP slice",
                wa_ld, wa_ld[PBITS-1:IDXB]);
      if (we_fe && (wa_fe[PBITS-1:IDXB] < SH_F0))
         $fatal(1, "smolrv64_prf: fp-exec write to pr=%h, shard %0d is not an FP slice",
                wa_fe, wa_fe[PBITS-1:IDXB]);
      // The integer-only ports never name an f-register.
      if ((sh1 >= SH_F0) | (sh4 >= SH_F0) | (sh5 >= SH_F0) | (sh6 >= SH_F0) | (sh7 >= SH_F0)
          | (sh8 >= SH_F0) | (sh9 >= SH_F0) | (sh3 >= SH_F0))
         $fatal(1, "smolrv64_prf: an integer-only read port names an FP register");
      if (we_ie2 && (wa_ie2[PBITS-1:IDXB] != SH_IE2))
         $fatal(1, "smolrv64_prf: second int-exec write to pr=%h, shard %0d is not SH_IE2",
                wa_ie2, wa_ie2[PBITS-1:IDXB]);
      if (we_ie3 && (wa_ie3[PBITS-1:IDXB] != SH_IE3))
         $fatal(1, "smolrv64_prf: third int-exec write to pr=%h, shard %0d is not SH_IE3",
                wa_ie3, wa_ie3[PBITS-1:IDXB]);
      if (we_ie && ({1'b0, wa_ie[IDXB-1:0]} >= N_IE[IDXB:0]))
         $fatal(1, "smolrv64_prf: int-exec write idx %0d >= N_IE %0d", wa_ie[IDXB-1:0], N_IE);
      if (we_ie2 && ({1'b0, wa_ie2[IDXB-1:0]} >= N_IE2[IDXB:0]))
         $fatal(1, "smolrv64_prf: second int-exec write idx %0d >= N_IE2 %0d", wa_ie2[IDXB-1:0], N_IE2);
      if (we_ie3 && ({1'b0, wa_ie3[IDXB-1:0]} >= N_IE3[IDXB:0]))
         $fatal(1, "smolrv64_prf: third int-exec write idx %0d >= N_IE3 %0d", wa_ie3[IDXB-1:0], N_IE3);
      if (we_ie && wa_ie == {PBITS{1'b0}})
         $fatal(1, "smolrv64_prf: write to physical register 0 (architectural zero)");
   end

   // The deadlock floor, checked once at elaboration rather than argued in a comment.
   initial begin
      if (N_IE <= 32) $fatal(1, "smolrv64_prf: N_IE=%0d must exceed 32 integer arch regs", N_IE);
      if (N_IE3 <= 32) $fatal(1, "smolrv64_prf: N_IE3=%0d must exceed 32 integer arch regs", N_IE3);
      if (N_FE <= 32) $fatal(1, "smolrv64_prf: N_FE=%0d must exceed 32 integer arch regs", N_FE);
      if (N_LD <= 32) $fatal(1, "smolrv64_prf: N_LD=%0d must exceed 32 integer arch regs", N_LD);
      if (N_FP <= 32) $fatal(1, "smolrv64_prf: N_FP=%0d must exceed 32 fp arch regs", N_FP);
      if (NMAX > (1 << IDXB))
         $fatal(1, "smolrv64_prf: NMAX=%0d exceeds IDXB=%0d addressable", NMAX, IDXB);
   end
endmodule

`default_nettype wire
