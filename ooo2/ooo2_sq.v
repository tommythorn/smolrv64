`default_nettype none

// ooo2_sq -- the store buffer.
//
// WHY. A store currently waits for BOTH its address operand and its data (`rs2`) before it
// may issue, and `u_rs_l` is in-order, so it holds the head of the queue while it waits.
// On workloads/saxpybench (the GB5 Camera loop) the data is an `fadds` result ~21 cycles
// away, and the next element's loads sit behind it -- 22.08 cycles/element for work whose
// elements are completely independent. docs/Area-Efficient-Scalar-OoO.md 11: a store issues
// on its ADDRESS alone and commits when its data arrives.
//
// THE DATA ARRIVES BY SNOOPING, NOT BY A READ PORT. An entry holds `dpreg`, the renamed
// rs2, and watches the writeback ports. When one names `dpreg` the value is captured in
// flight. Holding a register number and reading the PRF at commit would cost a third read
// port, and a read port costs a substantial fraction of the array (doc 15) -- this keeps
// the PRF at 2R1W while still letting address and data arrive apart.
//
// ORDERING. Entries are allocated at DISPATCH in program order. That is not a detail:
// allocating at issue lets younger stores take every slot while an older one waits for its
// operands, and the older store can then never issue to free one (doc 11). Commit is in
// order from the head.
//
// DISAMBIGUATION is a spectrum (doc 11.1) and the cheap end is correct. `ld_block` says an
// older pending store MAY alias:
//   * an entry whose address is not yet computed blocks unconditionally -- nothing can be
//     compared against an unknown address
//   * otherwise the byte ranges are compared, so a load past an adjacent store proceeds
// Byte-range rather than the `|delta| > 8` sketch, because Camera stores y[i] and then
// loads y[i+1] FOUR bytes away: a coarser test would block exactly the case worth winning.
module ooo2_sq
  #(parameter NENT  = 4,
    parameter IDXB  = 2,               // $clog2(NENT)
    parameter PAW   = 56,
    parameter PBITS = 9,
    parameter ROBB  = 4,
    parameter NWB   = 3)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, program order, one per cycle ----
    input  wire                  d_alloc,
    input  wire [ROBB-1:0]       d_rob,       // rides with the store; commit is at the head
    input  wire [PBITS-1:0]      d_dpreg,     // renamed rs2 AT DISPATCH -- see the snoop
    output wire                  d_ready,
    output wire [IDXB-1:0]       d_idx,
    output wire [IDXB-1:0]       d_tag,       // the STORE-SEQNO a load captures at dispatch

    // ---- address (and possibly data) written when the store executes ----
    input  wire                  a_v,
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_addr,
    input  wire [1:0]            a_size,     // 0=B 1=H 2=W 3=D
    input  wire                  a_unc,      // uncached, decided at translate time
    input  wire                  a_data_v,   // the issue-time PRF read of rs2 was valid
    input  wire [63:0]           a_data,     //   (ignored once the snoop has the value)

    // ---- snoop the writeback ports for the data operand ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,
    input  wire [NWB*64-1:0]     wb_data,

    // ---- commit: the head, once its data has arrived ----
    output wire                  c_v,
    output wire [ROBB-1:0]       c_rob,       // core commits only when this IS the ROB head
    output wire [PAW-1:0]        c_addr,
    output wire [63:0]           c_data,
    output wire [1:0]            c_size,
    output wire                  c_unc,
    input  wire                  c_take,

    // ---- load disambiguation ----
    input  wire [PAW-1:0]        ld_addr,
    input  wire [1:0]            ld_size,
    input  wire [IDXB-1:0]       ld_tag,     // this load's captured store-seqno
    output wire                  ld_block,
    output wire                  ld_older,   // an older store is live (aliasing or not) --
                                             // the load is REORDERED past it if it starts

    output wire [IDXB:0]         occupancy,
    input  wire                  flush);

   reg [NENT-1:0]        v, av, dv;          // live / address known / data known
   reg [PAW-1:0]         addr [0:NENT-1];
   reg [63:0]            data [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   reg [NENT-1:0]        unc;
   reg [PBITS-1:0]       dpr  [0:NENT-1];
   reg [ROBB-1:0]        rob  [0:NENT-1];
   reg [IDXB-1:0]        head, tail;
   reg [IDXB:0]          cnt;
   integer               k, w;
   reg                   sn_live;         // snoop scratch, see the writeback loop
   reg [PBITS-1:0]       sn_dpr;

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; dv = {NENT{1'b0}};
                 head = {IDXB{1'b0}}; tail = {IDXB{1'b0}}; cnt = {(IDXB+1){1'b0}}; end

   assign d_ready   = (cnt != NENT[IDXB:0]);
   assign d_idx     = tail;
   assign d_tag     = tail;
   assign occupancy = cnt;

   // c_v says the head is READY (address and data present). Committing in program order
   // is the core's job: it takes the entry only when c_rob is the ROB head, so a store
   // reaches memory exactly where the cosim's retire stream expects it.
   assign c_v    = v[head] & av[head] & dv[head];
   assign c_rob  = rob[head];
   assign c_addr = addr[head];
   assign c_data = data[head];
   assign c_size = sz[head];
   assign c_unc  = unc[head];

   // ---- disambiguation: does any live entry overlap this load? ----
   // A byte range is [addr, addr + (1<<size)). Two ranges overlap unless one ends at or
   // before the other begins. An entry with no address yet cannot be compared, so it
   // blocks -- conservative and correct.
   // ONLY ENTRIES OLDER THAN THE LOAD COUNT, and getting this wrong deadlocks rather than
   // corrupting. Entries are allocated at DISPATCH, so the buffer also holds stores YOUNGER
   // than a load sitting in M. Blocking on those is a circular wait: the load waits for a
   // younger store's address, and that store cannot execute because u_rs_l is in-order and
   // the load is at its head. Measured: it hung 115 of 240 tests.
   //
   // The load carries the store-seqno it captured at dispatch -- `d_tag`, the tail at that
   // moment -- and an entry is older than it exactly when it is nearer the head:
   //     dist(x) = (x - head) mod NENT,   older(g) = dist(g) < dist(ld_tag)
   // No wrap bit is needed: a store younger than the load can never commit before the load
   // retires, so the live region cannot cycle past the load's tag.
   localparam [PAW:0] SQ_ONE = {{PAW{1'b0}}, 1'b1};   // width-matched, not a bare literal
   wire [IDXB-1:0] ld_dist = ld_tag - head;
   wire [NENT-1:0] ovl;
   genvar g;
   generate
      for (g = 0; g < NENT; g = g + 1) begin : g_ovl
         wire [IDXB-1:0] g_dist = g[IDXB-1:0] - head;
         wire         older = v[g] & (g_dist < ld_dist);
         wire [PAW:0] s_lo = {1'b0, addr[g]};
         wire [PAW:0] s_hi = s_lo + (SQ_ONE << sz[g]);
         wire [PAW:0] l_lo = {1'b0, ld_addr};
         wire [PAW:0] l_hi = l_lo + (SQ_ONE << ld_size);
         assign ovl[g] = older & (~av[g] | ~((s_hi <= l_lo) | (l_hi <= s_lo)));
      end
   endgenerate
   assign ld_block = |ovl;
   // Older-and-live, regardless of overlap. ld_block is the subset that actually conflicts,
   // so `ld_older & ~ld_block` at a load's start is precisely a load reordered past an
   // uncommitted store -- the thing this whole structure exists to allow.
   wire [NENT-1:0] oldv;
   genvar go;
   generate
      for (go = 0; go < NENT; go = go + 1) begin : g_old
         assign oldv[go] = v[go] & (((go[IDXB-1:0] - head)) < ld_dist);
      end
   endgenerate
   assign ld_older = |oldv;

   always @(posedge clk) begin
      if (reset | flush) begin
         v <= {NENT{1'b0}}; av <= {NENT{1'b0}}; dv <= {NENT{1'b0}};
         head <= {IDXB{1'b0}}; tail <= {IDXB{1'b0}}; cnt <= {(IDXB+1){1'b0}};
      end else begin
         // commit the head
         if (c_v & c_take) begin
            v[head] <= 1'b0; av[head] <= 1'b0; dv[head] <= 1'b0;
            head <= head + 1'b1;
         end
         // allocate at the tail
         if (d_alloc & d_ready) begin
            v[tail] <= 1'b1; av[tail] <= 1'b0; dv[tail] <= 1'b0;
            rob[tail] <= d_rob;  dpr[tail] <= d_dpreg;
            tail <= tail + 1'b1;
         end
         if ((d_alloc & d_ready) & ~(c_v & c_take)) cnt <= cnt + 1'b1;
         else if (~(d_alloc & d_ready) & (c_v & c_take)) cnt <= cnt - 1'b1;

         // address only. The data operand is NOT captured here -- see the snoop.
         if (a_v) begin
            addr[a_idx] <= a_addr;  sz[a_idx] <= a_size;  av[a_idx] <= 1'b1;
            unc[a_idx]  <= a_unc;
            if (a_data_v & ~dv[a_idx]) begin data[a_idx] <= a_data; dv[a_idx] <= 1'b1; end
         end

         // SNOOP THE WRITEBACK PORTS, ARMED FROM ALLOCATE.
         //
         // Arming at `a_v` instead would be a silent data-loss bug, and it is worth naming
         // because it is not visible from this module alone. A physical register is written
         // back EXACTLY ONCE. Allocate happens at dispatch; the address arrives when the
         // store executes, which is at least one cycle later and unboundedly later behind a
         // PTW. A writeback landing anywhere in that window would find no entry watching for
         // it, and nothing would ever produce it again -- the store would sit in the buffer
         // forever, wedging the ROB head.
         //
         // So `dpr` is captured at ALLOCATE, from the renamer, and the entry watches from
         // the cycle it exists. The `d_alloc` cycle itself is covered by taking the live
         // d_dpreg, because dpr[tail] is written on that same edge.
         //
         // The already-produced case is not the snoop's job: if rs2 was ready when the store
         // read the PRF, `a_data_v` supplies it and no writeback is coming. The two paths are
         // disjoint by construction and `~dv` keeps them so if they ever overlap.
         for (k = 0; k < NENT; k = k + 1) begin
            sn_live = (d_alloc & d_ready & (tail == k[IDXB-1:0])) ? 1'b1     : (v[k] & ~dv[k]);
            sn_dpr  = (d_alloc & d_ready & (tail == k[IDXB-1:0])) ? d_dpreg  : dpr[k];
            for (w = 0; w < NWB; w = w + 1)
               if (sn_live & (sn_dpr != {PBITS{1'b0}})
                   & wb_v[w] & (wb_preg[w*PBITS +: PBITS] == sn_dpr)) begin
                  data[k] <= wb_data[w*64 +: 64];
                  dv[k]   <= 1'b1;
               end
         end
      end
   end

   // Invariants (docs/rtl-rules.md A1): anything the design would otherwise drop silently.
   always @(posedge clk) if (!reset) begin
      if (d_alloc & ~d_ready)
         $fatal(1, "ooo2_sq: allocate into a full buffer");
      if (a_v & ~v[a_idx])
         $fatal(1, "ooo2_sq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "ooo2_sq: address written twice to slot %0d", a_idx);
      // A store whose data can never arrive wedges the ROB head forever -- the one failure
      // of this structure that presents as a hang rather than a wrong answer, so it gets an
      // assertion rather than a comment. dpreg 0 is LEGAL (rs2 = x0, or any value already in
      // the PRF): it means no writeback is coming, which is only a bug if a_data_v did not
      // supply the value either.
      if (a_v & ~a_data_v & ~dv[a_idx] & (dpr[a_idx] == {PBITS{1'b0}}))
         $fatal(1, "ooo2_sq: entry %0d has no data and no producer -- it can never commit", a_idx);
      if (c_take & ~c_v)
         $fatal(1, "ooo2_sq: commit taken with no committable head");
   end
endmodule
`default_nettype wire
