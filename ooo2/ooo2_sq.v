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
    parameter NWB   = 3)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, program order, one per cycle ----
    input  wire                  d_alloc,
    output wire                  d_ready,
    output wire [IDXB-1:0]       d_idx,

    // ---- address (and possibly data) written when the store executes ----
    input  wire                  a_v,
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_addr,
    input  wire [1:0]            a_size,     // 0=B 1=H 2=W 3=D
    input  wire [PBITS-1:0]      a_dpreg,    // renamed rs2; 0 = data already supplied
    input  wire                  a_data_v,
    input  wire [63:0]           a_data,

    // ---- snoop the writeback ports for the data operand ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,
    input  wire [NWB*64-1:0]     wb_data,

    // ---- commit: the head, once its data has arrived ----
    output wire                  c_v,
    output wire [PAW-1:0]        c_addr,
    output wire [63:0]           c_data,
    output wire [1:0]            c_size,
    input  wire                  c_take,

    // ---- load disambiguation ----
    input  wire [PAW-1:0]        ld_addr,
    input  wire [1:0]            ld_size,
    output wire                  ld_block,

    output wire [IDXB:0]         occupancy,
    input  wire                  flush);

   reg [NENT-1:0]        v, av, dv;          // live / address known / data known
   reg [PAW-1:0]         addr [0:NENT-1];
   reg [63:0]            data [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   reg [PBITS-1:0]       dpr  [0:NENT-1];
   reg [IDXB-1:0]        head, tail;
   reg [IDXB:0]          cnt;
   integer               k, w;

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; dv = {NENT{1'b0}};
                 head = {IDXB{1'b0}}; tail = {IDXB{1'b0}}; cnt = {(IDXB+1){1'b0}}; end

   assign d_ready   = (cnt != NENT[IDXB:0]);
   assign d_idx     = tail;
   assign occupancy = cnt;

   assign c_v    = v[head] & av[head] & dv[head];
   assign c_addr = addr[head];
   assign c_data = data[head];
   assign c_size = sz[head];

   // ---- disambiguation: does any live entry overlap this load? ----
   // A byte range is [addr, addr + (1<<size)). Two ranges overlap unless one ends at or
   // before the other begins. An entry with no address yet cannot be compared, so it
   // blocks -- conservative and correct.
   localparam [PAW:0] SQ_ONE = {{PAW{1'b0}}, 1'b1};   // width-matched, not a bare literal
   wire [NENT-1:0] ovl;
   genvar g;
   generate
      for (g = 0; g < NENT; g = g + 1) begin : g_ovl
         wire [PAW:0] s_lo = {1'b0, addr[g]};
         wire [PAW:0] s_hi = s_lo + (SQ_ONE << sz[g]);
         wire [PAW:0] l_lo = {1'b0, ld_addr};
         wire [PAW:0] l_hi = l_lo + (SQ_ONE << ld_size);
         assign ovl[g] = v[g] & (~av[g] | ~((s_hi <= l_lo) | (l_hi <= s_lo)));
      end
   endgenerate
   assign ld_block = |ovl;

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
            tail <= tail + 1'b1;
         end
         if ((d_alloc & d_ready) & ~(c_v & c_take)) cnt <= cnt + 1'b1;
         else if (~(d_alloc & d_ready) & (c_v & c_take)) cnt <= cnt - 1'b1;

         // address (and data, when it was ready at issue)
         if (a_v) begin
            addr[a_idx] <= a_addr;  sz[a_idx] <= a_size;
            dpr[a_idx]  <= a_dpreg; av[a_idx] <= 1'b1;
            if (a_data_v) begin data[a_idx] <= a_data; dv[a_idx] <= 1'b1; end
         end

         // snoop the writeback ports for a pending data operand
         for (k = 0; k < NENT; k = k + 1)
            for (w = 0; w < NWB; w = w + 1)
               if (v[k] & av[k] & ~dv[k] & ~(a_v & (a_idx == k[IDXB-1:0]) & a_data_v)
                   & wb_v[w] & (wb_preg[w*PBITS +: PBITS] == dpr[k])
                   & (dpr[k] != {PBITS{1'b0}})) begin
                  data[k] <= wb_data[w*64 +: 64];
                  dv[k]   <= 1'b1;
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
      if (c_take & ~c_v)
         $fatal(1, "ooo2_sq: commit taken with no committable head");
   end
endmodule
`default_nettype wire
