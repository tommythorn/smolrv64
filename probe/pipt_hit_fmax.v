`default_nettype none

// Timing probe: PIPT D$ HIT PATH at 128 KiB / 2-way skew / 64 B, with the dTLB on
// the address-generation cone (the thing that decides the "+1 load-use cycle").
//
//   lfsr -> din_q -> [ stage1: VA -> dTLB(direct-mapped 16e) -> PA -> phys index ] -> {pa_q, s1_q}
//                 -> [ stage2: pa_q + 2 candidate (valid,ptag) + 2 lines
//                              -> ptag compare -> way mux -> byte/sign-extend ]    -> s2_q
//                 -> probe_out
//
// Faithful to probe/mmu.v's dTLB: direct-mapped, 16 entries, indexed by VA[15:12],
// 27-bit VPN tag compare, 3-way superpage leaf_pa mux, perm check on the PTE bits.
// Modelled as a 16x82 distributed RAM (registered write from stimulus, async read)
// so the 16:1 read mux + compare + leaf_pa form the real address-gen cone. The PA
// is registered (pipeline boundary at the data BRAM) and the post-array compare in
// stage2 uses the registered PA (no second dTLB lookup) -- matching the real
// pipeline: dTLB in cycle1, ptag compare in cycle2.
//
// Compare against vhpr_hit_fmax.v (same geometry): if stage1 here (VA->dTLB->index)
// is the worst path and is slower than VHPR's stage2 compare, the dTLB does NOT fit
// the address-gen cycle and PIPT costs the cycle; if comparable, PIPT is "free".
//
// Usage:  make probe TOP=pipt_hit_fmax PERIOD=2.0 \
//              REGION="CLOCKREGION_X0Y0:CLOCKREGION_X1Y2" SRCS="pipt_hit_fmax.v"
module pipt_hit_fmax (input wire clk, output reg probe_out = 1'b0);

   localparam IDXB  = 10;
   localparam OFFB  = 6;
   localparam PTAGB = 22;        // PA[33:12]
   localparam PAW   = 34;
   localparam LINEB = 512;
   localparam TLBN  = 16, TLBI = 4;
   localparam TLBW  = 1 + 27 + 44 + 2 + 8;   // {v, vpn_tag[26:0], ppn[43:0], lvl[1:0], perm[7:0]}
   localparam METAB = 1 + PTAGB;             // {valid, ptag}

   localparam IN_W = 64          // va
                   + 4           // ctx {access[1:0], sum, mxr}
                   + 2           // priv
                   + 2           // size
                   + TLBI        // tlb write addr
                   + TLBW        // tlb write data
                   + METAB       // way0 meta
                   + METAB       // way1 meta
                   + LINEB       // way0 line
                   + LINEB;      // way1 line
   reg  [IN_W-1:0] lfsr = {IN_W{1'b1}};
   wire fb = lfsr[IN_W-1] ^ lfsr[IN_W-2] ^ lfsr[IN_W-4] ^ lfsr[IN_W-5];
   always @(posedge clk) lfsr <= {lfsr[IN_W-2:0], fb};
   reg  [IN_W-1:0] din_q = {IN_W{1'b0}};
   always @(posedge clk) din_q <= lfsr;

   wire [63:0]      va;
   wire [3:0]       ctx;
   wire [1:0]       priv;
   wire [1:0]       size;
   wire [TLBI-1:0]  tlb_waddr;
   wire [TLBW-1:0]  tlb_wdata;
   wire [METAB-1:0] m0, m1;
   wire [LINEB-1:0] l0, l1;
   assign {va, ctx, priv, size, tlb_waddr, tlb_wdata, m0, m1, l0, l1} = din_q;

   function perm_ok;                       // identical body to vhpr probe
      input [4:0] p; input [3:0] c; input [1:0] pr;
      reg pte_r, pte_w, pte_x, pte_u; reg [1:0] acc; reg s, x, drd, uok;
      begin
         pte_r=p[0]; pte_w=p[1]; pte_x=p[2]; pte_u=p[3];
         acc=c[3:2]; s=c[1]; x=c[0];
         drd = pte_r | (x & pte_x);
         uok = (pr==2'd0) ? pte_u : (pr==2'd1) ? (~pte_u | (acc!=2'd0 & s)) : 1'b1;
         case (acc)
           2'd0: perm_ok = pte_x & uok;
           2'd1: perm_ok = drd & uok;
           2'd2: perm_ok = pte_w & uok;
           default: perm_ok = drd & pte_w & uok;
         endcase
      end
   endfunction

   function [63:0] extract;                // identical body to vhpr probe
      input [63:0] word; input [2:0] boff; input [1:0] sz; input sgn;
      reg [63:0] sh; reg [63:0] r;
      begin
         sh = word >> {boff,3'b000};
         case (sz)
           2'd0: r = sgn ? {{56{sh[7]}},  sh[7:0]}  : {56'd0, sh[7:0]};
           2'd1: r = sgn ? {{48{sh[15]}}, sh[15:0]} : {48'd0, sh[15:0]};
           2'd2: r = sgn ? {{32{sh[31]}}, sh[31:0]} : {32'd0, sh[31:0]};
           default: r = sh;
         endcase
         extract = r;
      end
   endfunction

   // ===== stage 1: direct-mapped dTLB -> PA -> physical skew index =====
   reg [TLBW-1:0] tlb [0:TLBN-1];
   integer ii; initial for (ii=0; ii<TLBN; ii=ii+1) tlb[ii] = 0;
   always @(posedge clk) tlb[tlb_waddr] <= tlb_wdata;     // 1 registered write port

   wire [TLBI-1:0] tidx   = va[12 +: TLBI];               // VPN0[3:0] = VA[15:12]
   wire [TLBW-1:0] ent    = tlb[tidx];                    // async read (distributed RAM)
   wire            e_v    = ent[TLBW-1];
   wire [26:0]     e_tag  = ent[44+2+8 +: 27];
   wire [43:0]     e_ppn  = ent[2+8 +: 44];
   wire [1:0]      e_lvl  = ent[8 +: 2];
   wire [7:0]      e_perm = ent[0 +: 8];
   wire [26:0]     vpn    = va[38:12];
   wire            tlb_hit = e_v & (e_tag == vpn);
   wire            tlb_pok = perm_ok(e_perm[4:0], ctx, priv);
   // superpage leaf_pa (3-way mux on level), then physical cache index
   reg  [PAW-1:0] pa;
   always @* case (e_lvl)
        2'd2: pa = {e_ppn[PAW-13:18], va[29:0]};   // 1 GiB
        2'd1: pa = {e_ppn[PAW-13:9],  va[20:0]};   // 2 MiB
        default: pa = {e_ppn[PAW-13:0], va[11:0]}; // 4 KiB
   endcase
   wire [3:0]     pcolor = pa[15:12];
   wire [IDXB-1:0] pidx0 = {pcolor, pa[11:6]};
   wire [IDXB-1:0] pidx1 = {pcolor ^ pa[19:16] ^ pa[27:24], pa[11:6]};
   reg  [PTAGB-1:0] pa_q = 0;          // PA tag carried to stage2 (pipeline reg)
   reg  [5:0]       off_q = 0;         // PA[5:0] page-offset (== VA[5:0])
   reg  [1:0]       size_q = 0;
   reg  [2*IDXB+1:0] s1_q = 0;
   always @(posedge clk) begin
      pa_q   <= pa[12 +: PTAGB];
      off_q  <= pa[5:0];
      size_q <= size;
      s1_q   <= {pidx0, pidx1, tlb_hit, tlb_pok};
   end

   // ===== stage 2: ptag compare (registered PA) / way-select / extract =====
   wire v0 = m0[METAB-1], v1 = m1[METAB-1];
   wire [PTAGB-1:0] t0 = m0[0 +: PTAGB], t1 = m1[0 +: PTAGB];
   wire hit0 = v0 & (t0 == pa_q);
   wire hit1 = v1 & (t1 == pa_q);
   wire             hit  = hit0 | hit1;
   wire [LINEB-1:0] line = hit1 ? l1 : l0;
   wire [63:0]      word = line[ off_q[5:3]*64 +: 64 ];
   wire [63:0]      data = extract(word, off_q[2:0], size_q, 1'b1);
   reg [64:0]       s2_q = 0;
   always @(posedge clk) s2_q <= {hit, data};

   always @(posedge clk) probe_out <= ^s1_q ^ ^s2_q;

endmodule

`default_nettype wire
