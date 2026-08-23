`default_nettype none

// Sharded physical register file for the in-order core.
//
// WHY SHARDS.  ino_regfile is a unified 64-entry array with ONE write port
// (`always @(posedge clk) if (we) r[wa] <= wd;`).  The moment completion goes out of order
// -- which is the whole point of the scoreboard/OoO work -- the LSU, the ALU and the FPU
// contend for it.  Duplicating the array does NOT help: every copy must receive every
// write, so replication buys READ ports and never WRITE ports.  The only cheap way to more
// write ports is to bank by WRITER, so each bank has exactly one, and that requires knowing
// the destination bank at rename.  That is what renaming buys here.
//
// THREE SHARDS, one writer each:
//
//   SH_IE  int-exec   ALU                     integer regs last written by the ALU
//   SH_LD  load       LSU + mul/div           int AND fp regs last written by mem or mul/div
//   SH_FE  fp-exec    FPU                     fp AND int regs last written by the FPU
//                                             (fcvt.w.d/fmv.x.w/fclass write x regs)
//
// mul/div ride with the LOAD shard, not the ALU shard.  Measured on the integer half of
// Geekbench 5: multiplies are ~0.1% of instructions and divides a few hundredths of a
// percent (r0302/r0301 stall cycles against a 3-cycle multiplier and an iterative divider).
// A conflict with a load is therefore ~0.05% of cycles, and the loser already waited 3-60
// cycles, so a holding register absorbs it.  Pairing them with the ALU instead would
// collide constantly -- the ALU completes on nearly every cycle it is used.
//
// SIZING IS A CORRECTNESS FLOOR, not just performance.  A shard must hold every
// architectural register that can map into it, PLUS at least one free, or rename deadlocks:
// with every register mapped and none free, nothing can be renamed, so nothing can commit,
// so nothing is ever freed.  Hence N_LD > 64 (the load shard can hold both integer and FP
// mappings), N_FE > 64 (the FPU writes integer regs), and N_IE > 32 (the ALU writes
// only integer regs).  Above that floor it is a stall/area choice -- see
// docs/Area-Efficient-Scalar-OoO.md 9.3 -- and ino_rename exports per-shard stall counters
// so the choice can be replaced by a measurement.
//
// Physical register number: {shard[1:0], idx[IDXB-1:0]}.  Shard in the HIGH bits so a
// shard's pool is a contiguous range and its free list is a plain counter range.
// Physical register 0 is the architectural zero: never written, always reads 0.

module ino_prf
  #(parameter IDXB  = 7,                   // index bits within a shard
    parameter PBITS = IDXB + 2,            // physical register number width
    parameter N_IE  = 64,                  // > 32 (integer arch regs)
    parameter N_LD  = 128,                  // > 64 (integer AND fp arch regs can land here)
    parameter N_FE  = 128,                  // > 64: the FPU writes INTEGER regs too
    parameter WRTHRU = 0)                   // see the write-through note below
   (input  wire             clk,

    // ---- three write ports, one per shard: no arbitration, by construction ----
    input  wire             we_ie,
    input  wire [PBITS-1:0] wa_ie,
    input  wire [63:0]      wd_ie,
    input  wire             we_ld,
    input  wire [PBITS-1:0] wa_ld,
    input  wire [63:0]      wd_ld,
    input  wire             we_fe,
    input  wire [PBITS-1:0] wa_fe,
    input  wire [63:0]      wd_fe,

    // ---- three combinational read ports (rs1, rs2, rs3 for the FMA third operand) ----
    input  wire [PBITS-1:0] ra1,
    input  wire [PBITS-1:0] ra2,
    input  wire [PBITS-1:0] ra3,
    output wire [63:0]      rd1,
    output wire [63:0]      rd2,
    output wire [63:0]      rd3);

   localparam [1:0] SH_IE = 2'd0, SH_LD = 2'd1, SH_FE = 2'd2;

   // Sized to the largest shard; the smaller shards simply never index above their
   // capacity, which ino_rename's free list enforces and the assertion below checks.
   // EACH ARRAY IS SIZED TO ITS OWN SHARD.  The first cut sized all three to the largest
   // (NMAX), so mem_ie was 128 deep with N_IE=64 -- half of it unreachable, and synthesis
   // duly built it: "mem_ie_reg 128 x 64, RAM64M8 x 60", identical to the 128-entry shards.
   // Pure waste, and area is not free here: the 166 MHz build fails on ROUTING inside the
   // caches (83-85% route on sub-1 ns logic), so congestion costs slack somewhere else.
   localparam integer NMAX  = (N_LD > N_IE) ? ((N_LD > N_FE) ? N_LD : N_FE)
                                            : ((N_IE > N_FE) ? N_IE : N_FE);
   localparam integer AB_IE = $clog2(N_IE), AB_LD = $clog2(N_LD), AB_FE = $clog2(N_FE);

   reg [63:0] mem_ie [0:N_IE-1];
   reg [63:0] mem_ld [0:N_LD-1];
   reg [63:0] mem_fe [0:N_FE-1];

   wire [1:0]      sh1 = ra1[PBITS-1:IDXB], sh2 = ra2[PBITS-1:IDXB], sh3 = ra3[PBITS-1:IDXB];
   wire [IDXB-1:0] ix1 = ra1[IDXB-1:0],     ix2 = ra2[IDXB-1:0],     ix3 = ra3[IDXB-1:0];

   // WRITE-THROUGH, and why it is OFF by default.
   //
   // docs/Area-Efficient-Scalar-OoO.md 14.1 makes it load-bearing for the OoO machine: a
   // consumer issuing in the cycle its producer writes back must see the new value, and
   // "an implementation that registers any of these is a different machine".
   //
   // With IN-ORDER issue it is dead code.  The write targets m_prd, the physical register
   // allocated for m_rd; renaming makes physical registers unique, so a source resolves to
   // m_prd only when that source IS m_rd -- which is exactly ino_core's byp1/2/3, and there
   // x_rs takes m_byp_val, never prf_rs.  So the collision can happen but its result is
   // never used.
   //
   // It is not free: 3 read ports x 3 shards of PBITS comparator plus a 64-bit mux, sitting
   // in the operand read path -- the back-to-back ALU loop that must stay fast.  And on this
   // die area is congestion and congestion is slack (docs/rtl-rules.md I1).
   //
   // Turning it on is NOT something to remember: ino_core asserts on a read that collides
   // with the writeback and is not bypassed, so the machine says when this becomes needed.
   function automatic [63:0] rd_shard;
      input [1:0]      sh;
      input [IDXB-1:0] ix;
      input [63:0]     m_ie, m_ld, m_fe;
      begin
         case (sh)
           SH_IE: rd_shard = (WRTHRU != 0 && we_ie && wa_ie[IDXB-1:0] == ix
                              && wa_ie[PBITS-1:IDXB] == SH_IE) ? wd_ie : m_ie;
           SH_LD: rd_shard = (WRTHRU != 0 && we_ld && wa_ld[IDXB-1:0] == ix
                              && wa_ld[PBITS-1:IDXB] == SH_LD) ? wd_ld : m_ld;
           SH_FE: rd_shard = (WRTHRU != 0 && we_fe && wa_fe[IDXB-1:0] == ix
                              && wa_fe[PBITS-1:IDXB] == SH_FE) ? wd_fe : m_fe;
           default: rd_shard = 64'd0;
         endcase
      end
   endfunction

   // Physical register 0 reads 0 unconditionally -- it is the architectural zero and is
   // never allocated by ino_rename, so no write can target it.
   // Index each array with only the bits it has.  A read of a shard the operand does not
   // belong to is discarded by rd_shard's case, so a truncated index there is harmless --
   // but it must not be OUT OF RANGE, which for a smaller shard it otherwise would be.
   assign rd1 = (ra1 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh1, ix1, mem_ie[ix1[AB_IE-1:0]], mem_ld[ix1[AB_LD-1:0]],
                         mem_fe[ix1[AB_FE-1:0]]);
   assign rd2 = (ra2 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh2, ix2, mem_ie[ix2[AB_IE-1:0]], mem_ld[ix2[AB_LD-1:0]],
                         mem_fe[ix2[AB_FE-1:0]]);
   assign rd3 = (ra3 == {PBITS{1'b0}}) ? 64'd0
              : rd_shard(sh3, ix3, mem_ie[ix3[AB_IE-1:0]], mem_ld[ix3[AB_LD-1:0]],
                         mem_fe[ix3[AB_FE-1:0]]);

   integer j;
   initial begin
      for (j = 0; j < N_IE; j = j + 1) mem_ie[j] = 64'd0;
      for (j = 0; j < N_LD; j = j + 1) mem_ld[j] = 64'd0;
      for (j = 0; j < N_FE; j = j + 1) mem_fe[j] = 64'd0;
      // Boot seed, mirroring ino_regfile's: a1 (x11) = the DTB pointer.  x11 maps to
      // {SH_IE, 11} at reset (see ino_rename's reset arm), so the seed lands in mem_ie[11].
      // Sim-only and inert unless a TB passes +a1=, but NOT optional: a harness that resets
      // straight to OpenSBI expects the pointer there, and without this the shadow check
      // fires 255 cycles into Linux boot -- which is exactly how this omission was found.
      begin : seed reg [63:0] a1v;
         if ($value$plusargs("a1=%h", a1v)) mem_ie[11] = a1v;
      end
   end

   always @(posedge clk) begin
      if (we_ie) mem_ie[wa_ie[AB_IE-1:0]] <= wd_ie;
      if (we_ld) mem_ld[wa_ld[AB_LD-1:0]] <= wd_ld;
      if (we_fe) mem_fe[wa_fe[AB_FE-1:0]] <= wd_fe;
   end

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   // The bound is compared at IDXB+1 bits: N_LD=128 truncates to 0 in IDXB=7 bits, which
   // silently turns the check into `idx >= 0` and fires on every write.
   // A write must name its own shard and stay inside that shard's capacity.  Either would
   // otherwise be a silent wrong-register write -- exactly the class of defect that costs
   // days here, because the value surfaces far from the mistake.
   always @(posedge clk) begin
      if (we_ie && (wa_ie[PBITS-1:IDXB] != SH_IE))
         $fatal(1, "ino_prf: int-exec write to pr=%h, shard %0d is not SH_IE",
                wa_ie, wa_ie[PBITS-1:IDXB]);
      if (we_ld && (wa_ld[PBITS-1:IDXB] != SH_LD))
         $fatal(1, "ino_prf: load write to pr=%h, shard %0d is not SH_LD",
                wa_ld, wa_ld[PBITS-1:IDXB]);
      if (we_fe && (wa_fe[PBITS-1:IDXB] != SH_FE))
         $fatal(1, "ino_prf: fp-exec write to pr=%h, shard %0d is not SH_FE",
                wa_fe, wa_fe[PBITS-1:IDXB]);
      if (we_ie && ({1'b0, wa_ie[IDXB-1:0]} >= N_IE[IDXB:0]))
         $fatal(1, "ino_prf: int-exec write idx %0d >= N_IE %0d", wa_ie[IDXB-1:0], N_IE);
      if (we_ld && ({1'b0, wa_ld[IDXB-1:0]} >= N_LD[IDXB:0]))
         $fatal(1, "ino_prf: load write idx %0d >= N_LD %0d", wa_ld[IDXB-1:0], N_LD);
      if (we_fe && ({1'b0, wa_fe[IDXB-1:0]} >= N_FE[IDXB:0]))
         $fatal(1, "ino_prf: fp-exec write idx %0d >= N_FE %0d", wa_fe[IDXB-1:0], N_FE);
      if (we_ie && wa_ie == {PBITS{1'b0}})
         $fatal(1, "ino_prf: write to physical register 0 (architectural zero)");
   end

   // The deadlock floor, checked once at elaboration rather than argued in a comment.
   initial begin
      if (N_IE <= 32) $fatal(1, "ino_prf: N_IE=%0d must exceed 32 integer arch regs", N_IE);
      // The FPU writes fp regs AND integer regs (fcvt.w.d, fmv.x.w, fclass, fcmp), so
      // SH_FE can hold up to 64 mappings -- same floor as the load shard, not 32.
      if (N_FE <= 64) $fatal(1, "ino_prf: N_FE=%0d must exceed 64 (fp AND int land here)", N_FE);
      if (N_LD <= 64) $fatal(1, "ino_prf: N_LD=%0d must exceed 64 (int AND fp map here)", N_LD);
      if (NMAX > (1 << IDXB))
         $fatal(1, "ino_prf: NMAX=%0d exceeds IDXB=%0d addressable", NMAX, IDXB);
   end
endmodule

`default_nettype wire
