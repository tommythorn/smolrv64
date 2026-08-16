`default_nettype none

// Blocking load/store unit for the in-order core -- stage M's memory engine.
//
// The OoO LSU exists to let loads and stores execute out of order against each
// other: a store buffer, a load queue, byte-granular store-to-load forwarding,
// seqno-keyed squash, two page-table walkers, commit-gated drain. NONE of that is
// needed here. One memory operation is in flight at a time and it is already the
// oldest instruction in the machine, so:
//
//   * ordering is program order, for free -- no disambiguation, no forwarding;
//   * a store is non-speculative by construction (nothing older can still trap,
//     nothing younger has passed M), so it writes the D$ directly -- no buffer;
//   * one MMU/walker serves loads, stores and atomics.
//
// The request is held stable by ino_core until `done` pulses. `done` is
// combinational in the completing cycle and `rd_val` is valid with it.
//
// Semantics kept bit-identical to probe/lsu.v so the Simmerv cosim agrees:
//   * the D$ port is BYTE-ADDRESS-RELATIVE both ways -- rd_data[7:0] is the byte at
//     rd_addr, and wr_data[7:0] is written to wr_addr+0 under wr_mask -- so a
//     misaligned access needs no lane rotation here; cache.v resolves any
//     line-crossing internally via its two-phase lookup.
//   * a misaligned access that crosses a PAGE boundary is not translatable with one
//     walk, so it raises address-misaligned (cause 4 load / 6 store-AMO), exactly as
//     the OoO LSU does -- software emulates it.
//   * atomics read the containing 8-byte word, compute, and write it back under a
//     0x0F/0xF0/0xFF mask; `.W` selects its half with addr[2].
module ino_lsu
  #(parameter AW = 64,
    parameter [63:0] DRAM_BASE = 64'd0,
    parameter [63:0] DRAM_TOP  = 64'hFFFF_FFFF_FFFF_FFFF)
   (input  wire            clk,
    input  wire            reset,

    // ---- request (held stable until `done`) ----
    input  wire            req_valid,
    input  wire            req_store,      // plain store (not AMO)
    input  wire            req_amo,
    input  wire [4:0]      req_amo_func,   // funct5
    input  wire            req_cbo,        // Zicbom/Zicboz: maintenance, no data write
    input  wire            req_cbo_zero,
    input  wire            req_cbo_keep,
    input  wire [63:0]     req_vaddr,
    input  wire [1:0]      req_size,       // 0=B 1=H 2=W 3=D
    input  wire            req_signed,     // load sign-extends
    input  wire            req_fp,         // FLW -> NaN-box the 32-bit result
    input  wire [63:0]     req_st_data,    // rs2

    // ---- translation context (from csr_file) ----
    input  wire [63:0]     xl_satp,
    input  wire [1:0]      xl_priv,
    input  wire            xl_sum,
    input  wire            xl_mxr,
    input  wire            xl_flush,
    output wire [55:0]     ptw_addr,
    output wire            ptw_read,
    input  wire [63:0]     ptw_rdata,
    input  wire            ptw_rvalid,

    // ---- data memory port ----
    output reg  [AW-1:0]   mem_raddr,
    output reg             mem_ren,
    output reg             mem_runcached,
    input  wire [63:0]     mem_rdata,
    input  wire            mem_rvalid,
    output wire            mem_wen,
    output wire [AW-1:0]   mem_waddr,
    output wire [63:0]     mem_wdata,
    output wire [7:0]      mem_wmask,
    output wire            mem_wuncached,
    output wire            mem_cbo,
    output wire            mem_cbo_zero,
    output wire            mem_cbo_keep,
    input  wire            mem_wready,

    // ---- completion ----
    output wire            done,
    output wire [63:0]     rd_val,
    output wire            fault,
    output wire [3:0]      fault_cause,
    output wire [63:0]     fault_tval,
    output wire            idle);          // no memory op in flight (fence.i drain)

   localparam S_IDLE = 3'd0, S_LD = 3'd1, S_ST = 3'd2,
              S_LD2  = 3'd5, S_ST2 = 3'd6,   // second aligned word
              S_ARD  = 3'd3, S_AWR = 3'd4;

   reg [2:0]   st;
   reg [55:0]  pa_q;                       // translated physical address (WORD-ALIGNED)

   // ---- word-aligned D$ access (see header) --------------------------------------
   wire [2:0]  boff   = req_vaddr[2:0];              // byte offset in the aligned word
   wire [4:0]  wend   = {2'd0, boff} + {1'd0, nb};   // one past the last byte in-word
   // MMIO must keep its EXACT address: devices decode by low address bits, so an
   // aligned-plus-mask access lands on the wrong register (this hung virtio-net at
   // boot). Only DRAM traffic goes through the cache and therefore needs aligning.
   wire        pa_dram = (t_paddr >= DRAM_BASE[55:0]) && (t_paddr < DRAM_TOP[55:0]);
   wire        xl_can = ~req_amo & ~req_cbo & pa_dram; // AMO pre-aligned; CBO is line-wide
   wire        xword  = xl_can & (wend > 5'd8);      // operand straddles two words
   reg         xword_q;
   reg  [2:0]  boff_q;
   reg  [55:0] pa2_q;                                // the next aligned word
   reg  [63:0] ld_lo_q;                              // first word's data
   // boff_q is 1..7 whenever xword_q, so sh_up is 8..56 -- never a 64-bit shift.
   wire [5:0]  sh_dn  = {boff_q, 3'b000};
   wire [5:0]  sh_up  = 6'd0 - {boff_q, 3'b000};     // == 64 - 8*boff (mod 64)
   reg [63:0]  amo_old_q;                  // AMO's rd value, captured at the RMW read
   reg         nc_q;                       // Svpbmt: this access is NC/IO
   initial begin st = S_IDLE; end

   // ---------------------------------------------------------- classification
   wire        is_lr    = req_amo & (req_amo_func == 5'b00010);
   wire        is_sc    = req_amo & (req_amo_func == 5'b00011);
   wire [3:0]  nb       = 4'd1 << req_size;
   wire        wr_class = req_store | (req_amo & ~is_lr);   // store-class for translation/faults

   // ------------------------------------------------------------ translation
   wire        t_ready, t_fault, t_uncached;
   wire [55:0] t_paddr;
   wire [3:0]  t_cause;
   // only ask while a request is actually waiting to be translated (S_IDLE): once
   // started, the access owns pa_q and the walker must be left alone.
   wire        xl_req = req_valid & (st == S_IDLE);

   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_mmu
     (.clk(clk), .reset(reset),
      .req_valid(xl_req), .req_vaddr(req_vaddr),
      .req_access(is_lr ? 2'd1 : req_amo ? 2'd3 : req_store ? 2'd2 : 2'd1),
      .priv(xl_priv), .sum(xl_sum), .mxr(xl_mxr), .satp(xl_satp), .flush(xl_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .t_ready(t_ready), .t_paddr(t_paddr), .t_fault(t_fault), .t_cause(t_cause),
      .t_uncached(t_uncached));

   // A misaligned access whose byte span leaves the page needs a second translation.
   // Raise address-misaligned instead (cause 4 load / 6 store-AMO) and let software
   // emulate -- the OoO LSU makes the same call.
   wire xpage   = (({1'b0, req_vaddr[11:0]} + {9'd0, nb}) > 13'h1000);

   // AMOs (and LR/SC) must be naturally aligned. The RMW datapath is built around
   // the containing 8-byte word -- pa_q is aligned down and a_wmask is word-relative
   // -- so without this check an unaligned AMO corrupts neighbouring bytes silently
   // rather than trapping. nb-1 is the alignment mask (nb=8 truncates to 3'b000, so
   // the 4-bit subtract yields 4'd7 as intended).
   wire [3:0] al_mask = nb - 4'd1;
   wire amo_mis = req_amo & ((req_vaddr[3:0] & al_mask) != 4'd0);

   wire mis_flt = xl_req & (xpage | amo_mis);
   wire xl_flt  = xl_req & t_ready & t_fault;

   assign fault       = req_valid & (mis_flt | xl_flt);
   assign fault_cause = mis_flt ? (wr_class ? 4'd6 : 4'd4) : t_cause;
   assign fault_tval  = req_vaddr;

   // request can start: translated cleanly this cycle
   wire start_ok = xl_req & t_ready & ~t_fault & ~xpage;

   // ------------------------------------------------------- AMO RMW datapath
   wire        a_isw   = (req_size == 2'd2);
   wire        a_half  = req_vaddr[2];
   wire [31:0] a_old32 = a_half ? mem_rdata[63:32] : mem_rdata[31:0];
   wire [31:0] a_d32   = req_st_data[31:0];
   wire [63:0] a_data  = req_st_data;

   // LR/SC reservation: one word-granular reservation register.
   reg         rsv_v;
   reg [35:0]  rsv_w;
   wire [35:0] a_word = req_vaddr[38:3];
   wire        a_scok = rsv_v && (rsv_w == a_word);

   reg [63:0] a_resv;
   always @* begin
      case (req_amo_func)
        5'b00001: a_resv = a_data;                                                     // swap
        5'b00000: a_resv = a_isw ? {32'b0, a_old32 + a_d32}  : mem_rdata + a_data;      // add
        5'b00100: a_resv = a_isw ? {32'b0, a_old32 ^ a_d32}  : mem_rdata ^ a_data;      // xor
        5'b01100: a_resv = a_isw ? {32'b0, a_old32 & a_d32}  : mem_rdata & a_data;      // and
        5'b01000: a_resv = a_isw ? {32'b0, a_old32 | a_d32}  : mem_rdata | a_data;      // or
        5'b10000: a_resv = a_isw ? {32'b0, ($signed(a_old32)<$signed(a_d32))?a_old32:a_d32}
                                 : ($signed(mem_rdata)<$signed(a_data))?mem_rdata:a_data;
        5'b10100: a_resv = a_isw ? {32'b0, ($signed(a_old32)>$signed(a_d32))?a_old32:a_d32}
                                 : ($signed(mem_rdata)>$signed(a_data))?mem_rdata:a_data;
        5'b11000: a_resv = a_isw ? {32'b0, (a_old32<a_d32)?a_old32:a_d32}
                                 : (mem_rdata<a_data)?mem_rdata:a_data;
        5'b11100: a_resv = a_isw ? {32'b0, (a_old32>a_d32)?a_old32:a_d32}
                                 : (mem_rdata>a_data)?mem_rdata:a_data;
        5'b00011: a_resv = a_data;                                                     // SC stores rs2
        default:  a_resv = mem_rdata;                                                  // LR: no write
      endcase
   end
   wire [63:0] a_oldv  = a_isw ? {{32{a_old32[31]}}, a_old32} : mem_rdata;
   wire [63:0] a_rdval = is_sc ? (a_scok ? 64'd0 : 64'd1) : a_oldv;   // SC: 0=ok 1=fail
   wire        a_dowr  = is_lr ? 1'b0 : is_sc ? a_scok : 1'b1;
   wire [63:0] a_wdata = a_isw ? (a_half ? {a_resv[31:0], 32'b0} : {32'b0, a_resv[31:0]}) : a_resv;
   wire [7:0]  a_wmask = a_isw ? (a_half ? 8'hF0 : 8'h0F) : 8'hFF;

   // -------------------------------------------------------- load formatting
   // The port returns the aligned word, so the LSU shifts the operand down itself and
   // splices the second word in when the operand straddled.
   wire [63:0] mem_rdata_eff = (st == S_LD2)
                             ? ((ld_lo_q >> sh_dn) | (mem_rdata << sh_up))
                             : (mem_rdata >> sh_dn);
   wire [63:0] ld_val = (nb == 4'd1) ? (req_signed ? {{56{mem_rdata_eff[7]}},  mem_rdata_eff[7:0]}
                                                   : {56'd0, mem_rdata_eff[7:0]})
                      : (nb == 4'd2) ? (req_signed ? {{48{mem_rdata_eff[15]}}, mem_rdata_eff[15:0]}
                                                   : {48'd0, mem_rdata_eff[15:0]})
                      : (nb == 4'd4) ? (req_fp     ? {32'hffffffff, mem_rdata_eff[31:0]}   // FLW: NaN-box
                                      : req_signed ? {{32{mem_rdata_eff[31]}}, mem_rdata_eff[31:0]}
                                                   : {32'd0, mem_rdata_eff[31:0]})
                      :                mem_rdata_eff;

   // ------------------------------------------------------------ write port
   wire st_go = (st == S_ST) || (st == S_ST2), amo_go = (st == S_AWR);
   wire st2_go = (st == S_ST2);
   wire [7:0] st_mask = (8'd1 << nb) - 8'd1;
   assign mem_wen       = st_go | amo_go;
   assign mem_waddr     = {{(AW-56){1'b0}}, (st2_go ? pa2_q : pa_q)};
   // Store data is placed at its byte offset inside the aligned word; the straddling
   // remainder starts at byte 0 of the next word.
   assign mem_wdata     = amo_go ? a_wdata
                        : st2_go ? (req_st_data >> sh_up)
                                 : (req_st_data << sh_dn);
   assign mem_wmask     = amo_go ? a_wmask
                        : req_cbo ? 8'd0                              // CBO carries no data
                        : st2_go  ? (st_mask >> (4'd8 - {1'b0, boff_q}))
                                  : (st_mask << boff_q);
   assign mem_wuncached = nc_q & ~req_cbo;
   assign mem_cbo       = st_go & req_cbo;
   assign mem_cbo_zero  = mem_cbo & req_cbo_zero;
   assign mem_cbo_keep  = mem_cbo & req_cbo_keep;

   // ------------------------------------------------------------ completion
   assign done   = fault
                 | ((st == S_ST)  & mem_wready & ~xword_q)
                 | ((st == S_ST2) & mem_wready)
                 | ((st == S_LD)  & mem_rvalid & ~xword_q)
                 | ((st == S_LD2) & mem_rvalid)
                 | ((st == S_ARD) & mem_rvalid & ~a_dowr)
                 | (amo_go & mem_wready);
   assign rd_val = (st == S_ARD) ? a_rdval : amo_go ? amo_old_q : ld_val;
   assign idle   = (st == S_IDLE);

   // ------------------------------------------------------------------- FSM
   always @(posedge clk) begin
      if (reset) begin
         st <= S_IDLE; mem_ren <= 1'b0; rsv_v <= 1'b0; mem_runcached <= 1'b0;
      end else begin
         mem_ren <= 1'b0;                                  // one-cycle request pulse
         case (st)
           S_IDLE:
             if (start_ok) begin
                nc_q          <= t_uncached;
                mem_runcached <= t_uncached;
                xword_q <= xword;
                boff_q  <= xl_can ? boff : 3'd0;   // AMO/CBO keep their own addressing
                pa2_q   <= (t_paddr & ~56'd7) + 56'd8;
                if (req_store) begin
                   pa_q <= xl_can ? (t_paddr & ~56'd7) : t_paddr;
                   st   <= S_ST;
                end else if (req_amo) begin
                   // An atomic reads, modifies and writes the CONTAINING 8-BYTE WORD:
                   // a_wdata/a_wmask are built relative to that word (a_half = addr[2]
                   // picks the .W half). The memory port is byte-address-relative, so
                   // the write must use the ALIGNED address too -- using the raw PA
                   // shifts a .W half-word write 4 bytes past its target.
                   pa_q      <= t_paddr & ~56'd7;
                   mem_raddr <= {{(AW-56){1'b0}}, t_paddr} & ~{{(AW-3){1'b0}}, 3'b111};
                   mem_ren   <= 1'b1;
                   st        <= S_ARD;
                end else begin
                   pa_q <= xl_can ? (t_paddr & ~56'd7) : t_paddr;
                   mem_raddr <= xl_can ? ({{(AW-56){1'b0}}, t_paddr} & ~{{(AW-3){1'b0}}, 3'b111})
                                       :  {{(AW-56){1'b0}}, t_paddr};
                   mem_ren   <= 1'b1;
                   st        <= S_LD;
                end
             end
           S_LD:  if (mem_rvalid) begin
                     if (xword_q) begin
                        ld_lo_q   <= mem_rdata;
                        mem_raddr <= {{(AW-56){1'b0}}, pa2_q};
                        mem_ren   <= 1'b1;
                        st        <= S_LD2;
                     end else st <= S_IDLE;
                  end
           S_LD2: if (mem_rvalid) st <= S_IDLE;
           S_ST:  if (mem_wready) st <= (xword_q ? S_ST2 : S_IDLE);
           S_ST2: if (mem_wready) st <= S_IDLE;
           S_ARD: if (mem_rvalid) begin
                     amo_old_q <= a_rdval;
                     if (is_lr) begin rsv_v <= 1'b1; rsv_w <= a_word; end
                     if (is_sc) rsv_v <= 1'b0;
                     st <= a_dowr ? S_AWR : S_IDLE;
                  end
           S_AWR: if (mem_wready) st <= S_IDLE;
           default: st <= S_IDLE;
         endcase
         // a plain store to the reserved word breaks the reservation
         if (st_go && mem_wready && rsv_v && (req_vaddr[38:3] == rsv_w)) rsv_v <= 1'b0;
      end
   end
endmodule

`default_nettype wire
