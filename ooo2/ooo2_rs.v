`default_nettype none

// The scheduler. docs/Area-Efficient-Scalar-OoO.md 5 ("Scheduler -- RS_SIZE entries") and
// 8.3 ("Issue and execute"): entries hold a readiness bit per source, writeback wakes them,
// and issue selects the OLDEST READY entry -- not the oldest entry.
//
// This is the piece whose absence made everything else cosmetic. A ROB and a physical
// register file give in-order commit and precise state; they do not let an instruction
// start before an older one. Until this module exists, an `add` waiting on a load holds the
// single decode slot and every younger instruction queues behind it in program order, so a
// non-blocking unit only ever buys the instructions BETWEEN a producer and its first
// consumer. Measured: saxpy, whose FP work is independent across iterations, got 1.24x.
//
// WHERE THE EXECUTE PAYLOAD LIVES. Not here as flops, and NOT in a rob_idx-indexed array
// either. It goes in a LUTRAM indexed by THIS MODULE'S ENTRY NUMBER -- written with fsel at
// dispatch, read with sel at select.
//
// Indexing it by rob_idx was the wrong instinct and would have cost twice over. The ROB is
// the LARGE structure (sized by the window) and the scheduler the small one (sized by
// dependency depth), so a payload array indexed by rob_idx is ROB_SIZE deep where NENT
// would do. Worse, it would put a READ PORT AT ISSUE on a ROB-sized array, and doc 5 is
// explicit that this is what the split exists to avoid: "The ROB is written at dispatch and
// writeback, and read only at commit. No ROB field is read at issue; that is why the
// execute-time fields -- opcode, immediate, sources -- live in the scheduler instead."
//
// What the doc keeps in scheduler REGISTERS (opcode, immediate) goes to that LUTRAM instead
// only because RV64GC's execute bundle is ~40 fields against its example ISA's five; the
// structure is the scheduler's either way. Operand VALUES are never stored anywhere -- the
// PRF is read at ISSUE, which is the "values live in one place" property the design rests
// on, and which the current core violates by reading operands in X and carrying them to M.
module ooo2_rs
  #(parameter NENT   = 8,
    parameter IDXB   = 3,             // $clog2(NENT)
    parameter ROBB   = 4,             // ROB index bits
    parameter PBITS  = 9,             // physical register number bits
    parameter NUNIT  = 4,             // functional units, one-hot
    parameter NWB    = 3)             // simultaneous writeback tags (one per PRF shard)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- dispatch: one per cycle, in program order ----
    input  wire                  d_valid,
    output wire                  d_ready,        // a free entry exists
    input  wire [ROBB-1:0]       d_rob,
    input  wire [PBITS-1:0]      d_ps1, d_ps2, d_ps3,
    input  wire                  d_r1, d_r2, d_r3,   // source already available
    input  wire [NUNIT-1:0]      d_unit,             // one-hot

    // ---- wakeup: every PRF write broadcasts its destination ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,

    // ---- issue: oldest ready entry whose unit is free ----
    input  wire [NUNIT-1:0]      unit_busy,
    input  wire [ROBB-1:0]       head,           // ROB head, for age
    output wire                  iss_v,
    output wire [ROBB-1:0]       iss_rob,
    output wire [NUNIT-1:0]      iss_unit,
    input  wire                  iss_take,       // consumer accepted it this cycle

    // ---- recovery: total, at the head of the window ----
    input  wire                  flush,
    output wire [IDXB:0]         occupancy);     // for the stall counters

   reg [NENT-1:0]  v;
   reg [ROBB-1:0]  e_rob  [0:NENT-1];
   reg [PBITS-1:0] e_ps1  [0:NENT-1], e_ps2 [0:NENT-1], e_ps3 [0:NENT-1];
   reg [NENT-1:0]  e_r1, e_r2, e_r3;
   reg [NUNIT-1:0] e_unit [0:NENT-1];

   integer k;
   initial begin
      v = {NENT{1'b0}}; e_r1 = {NENT{1'b0}}; e_r2 = {NENT{1'b0}}; e_r3 = {NENT{1'b0}};
      for (k = 0; k < NENT; k = k + 1) begin
         e_rob[k] = {ROBB{1'b0}}; e_ps1[k] = {PBITS{1'b0}};
         e_ps2[k] = {PBITS{1'b0}}; e_ps3[k] = {PBITS{1'b0}};
         e_unit[k] = {NUNIT{1'b0}};
      end
   end

   // ---- free-slot select: lowest free index (fixed priority, doc 8.4) --------------
   wire [NENT-1:0] freem = ~v;
   assign d_ready = |freem;
   reg [IDXB-1:0] fsel;
   always @* begin
      fsel = {IDXB{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1) if (freem[k]) fsel = k[IDXB-1:0];
   end

   // ---- wakeup ----------------------------------------------------------------------
   // A source is woken by ANY of the writeback ports. Sources are compared as physical
   // register numbers, which is the whole reason rename exists underneath this.
   function automatic hit;
      input [PBITS-1:0] p;
      integer w;
      begin
         hit = 1'b0;
         for (w = 0; w < NWB; w = w + 1)
            if (wb_v[w] && (wb_preg[w*PBITS +: PBITS] == p)) hit = 1'b1;
      end
   endfunction

   // ---- ready and oldest-ready select (doc 8.3) --------------------------------------
   // ready includes the unit check, so a busy unit does not block a DIFFERENT unit's
   // entry -- that is the entire point of the structure.
   wire [NENT-1:0] rdy;
   genvar g;
   generate
      for (g = 0; g < NENT; g = g + 1) begin : g_rdy
         assign rdy[g] = v[g] & (e_r1[g] | hit(e_ps1[g]))
                              & (e_r2[g] | hit(e_ps2[g]))
                              & (e_r3[g] | hit(e_ps3[g]))
                              & ~|(e_unit[g] & unit_busy);
      end
   endgenerate

   // age(x) = (x - head) & (ROB_SIZE-1); 0 is oldest. Minimum-reduction over NENT --
   // a comparator tree, deterministic and starvation-free.
   reg              sel_v;
   reg [IDXB-1:0]   sel;
   reg [ROBB-1:0]   sel_age;
   always @* begin
      sel_v = 1'b0; sel = {IDXB{1'b0}}; sel_age = {ROBB{1'b0}};
      for (k = 0; k < NENT; k = k + 1) begin : g_sel
         if (rdy[k]) begin
            if (!sel_v || ((e_rob[k] - head) < sel_age)) begin
               sel_v   = 1'b1;
               sel     = k[IDXB-1:0];
               sel_age = e_rob[k] - head;
            end
         end
      end
   end

   assign iss_v    = sel_v;
   assign iss_rob  = e_rob[sel];
   assign iss_unit = e_unit[sel];

   reg [IDXB:0] occ;
   always @* begin
      occ = {(IDXB+1){1'b0}};
      for (k = 0; k < NENT; k = k + 1) occ = occ + {{IDXB{1'b0}}, v[k]};
   end
   assign occupancy = occ;

   wire do_disp = d_valid & d_ready & ~flush;
   wire do_iss  = iss_v & iss_take & ~flush;

   always @(posedge clk) begin
      if (reset | flush) begin
         v <= {NENT{1'b0}};
      end else begin
         // Wakeup applies to EVERY live entry, including the one issuing this cycle (it
         // leaves anyway) and the one being dispatched (handled on its own path below).
         for (k = 0; k < NENT; k = k + 1) if (v[k]) begin
            if (hit(e_ps1[k])) e_r1[k] <= 1'b1;
            if (hit(e_ps2[k])) e_r2[k] <= 1'b1;
            if (hit(e_ps3[k])) e_r3[k] <= 1'b1;
         end
         if (do_iss) v[sel] <= 1'b0;
         if (do_disp) begin
            v[fsel]     <= 1'b1;
            e_rob[fsel] <= d_rob;
            e_ps1[fsel] <= d_ps1;  e_ps2[fsel] <= d_ps2;  e_ps3[fsel] <= d_ps3;
            e_unit[fsel]<= d_unit;
            // Dispatch-cycle wakeup: a producer writing back THIS cycle will never
            // broadcast again, so a source that is not yet ready must be checked against
            // the live writeback ports or the entry waits forever.
            e_r1[fsel]  <= d_r1 | hit(d_ps1);
            e_r2[fsel]  <= d_r2 | hit(d_ps2);
            e_r3[fsel]  <= d_r3 | hit(d_ps3);
         end
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) ---------------------------------
   always @(posedge clk) if (!reset) begin
      if (d_valid & ~d_ready & ~flush)
         $fatal(1, "ooo2_rs: dispatch into a full scheduler");
      if (do_disp & v[fsel])
         $fatal(1, "ooo2_rs: dispatch into occupied entry %0d", fsel);
      if (iss_take & ~iss_v)
         $fatal(1, "ooo2_rs: consumer took an issue that was not offered");
      if (do_iss & ~v[sel])
         $fatal(1, "ooo2_rs: issued entry %0d holds nothing", sel);
      if (do_iss & |(iss_unit & unit_busy))
         $fatal(1, "ooo2_rs: issued to a busy unit (unit=%b busy=%b)", iss_unit, unit_busy);
      if (do_disp & (d_unit == {NUNIT{1'b0}}))
         $fatal(1, "ooo2_rs: dispatch with no unit selected (rob=%0d)", d_rob);
   end
endmodule

`default_nettype wire
