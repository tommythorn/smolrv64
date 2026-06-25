`default_nettype none

// Timing probe: VHPR (virtually-hit) D$ HIT PATH at 128 KiB / 2-way skew / 64 B.
//
//   lfsr -> din_q -> [ stage1: VA -> skewed index ] -> s1_q
//                 -> [ stage2: VA/asid/epoch/perm + 2 candidate lines
//                              -> tag compare -> way mux -> byte/sign-extend ] -> s2_q
//                 -> probe_out
//
// The two stages mirror the real pipeline boundary at the data BRAM: stage1 is
// the address-generation cone (VA -> the index that addresses the array), stage2
// is the post-array cone (the two candidate ways' meta+data are registered inputs
// = "what the array returned"; compare valid/epoch/asid/vtag/perm, pick the way,
// extract+sign-extend the word). Both cones are registered in and out, so the
// worst-setup path the prober reports is whichever stage limits VHPR. The PA is
// produced as a side-effect of the hit (stored ptag), folded into the output.
//
// Compare against pipt_hit_fmax.v (same geometry): VHPR has NO TLB on either cone;
// the question is whether its wider tag compare (vtag+asid+epoch+perm) costs more
// than PIPT's dTLB-on-the-address-cone.
//
// Usage:  make probe TOP=vhpr_hit_fmax PERIOD=2.0 \
//              REGION="CLOCKREGION_X0Y0:CLOCKREGION_X1Y2" SRCS="vhpr_hit_fmax.v"
module vhpr_hit_fmax (input wire clk, output reg probe_out = 1'b0);

   // ---- 128 KiB, 2-way skew-associative, 64 B lines ----
   localparam IDXB   = 10;        // 1024 sets/way (VA[15:6])
   localparam OFFB   = 6;         // 64 B line
   localparam COLORB = 4;         // virtual color VA[15:12]
   localparam VTAGB  = 23;        // Sv39: VA[38:16]
   localparam ASIDB  = 10;
   localparam EPOCHB = 2;
   localparam PERMB  = 5;         // R,W,X,U,G
   localparam PTAGB  = 22;        // PA[33:12]
   localparam LINEB  = 512;       // 64 B data line
   // candidate "meta" = {valid, dirty, epoch, asid, vtag, perm, ptag}
   localparam METAB  = 1 + 1 + EPOCHB + ASIDB + VTAGB + PERMB + PTAGB;

   // ---- stimulus ----
   localparam IN_W = 64           // va
                   + ASIDB        // asid
                   + EPOCHB       // cur epoch
                   + 4            // ctx {access[1:0], sum, mxr} (+priv folded in access)
                   + 2            // priv
                   + 2            // size
                   + METAB        // way0 meta
                   + METAB        // way1 meta
                   + LINEB        // way0 line
                   + LINEB;       // way1 line
   reg  [IN_W-1:0] lfsr = {IN_W{1'b1}};
   wire fb = lfsr[IN_W-1] ^ lfsr[IN_W-2] ^ lfsr[IN_W-4] ^ lfsr[IN_W-5];
   always @(posedge clk) lfsr <= {lfsr[IN_W-2:0], fb};
   reg  [IN_W-1:0] din_q = {IN_W{1'b0}};
   always @(posedge clk) din_q <= lfsr;

   wire [63:0]      va;
   wire [ASIDB-1:0] asid;
   wire [EPOCHB-1:0] cepoch;
   wire [3:0]       ctx;       // {access[1:0], sum, mxr}
   wire [1:0]       priv;
   wire [1:0]       size;
   wire [METAB-1:0] m0, m1;
   wire [LINEB-1:0] l0, l1;
   assign {va, asid, cepoch, ctx, priv, size, m0, m1, l0, l1} = din_q;

   // meta unpack
   function valid_f;   input [METAB-1:0] m; valid_f = m[METAB-1];                         endfunction
   function [EPOCHB-1:0] epoch_f; input [METAB-1:0] m; epoch_f = m[METAB-3 -: EPOCHB];     endfunction
   function [ASIDB-1:0]  asid_f;  input [METAB-1:0] m; asid_f  = m[PTAGB+VTAGB+PERMB +: ASIDB]; endfunction
   function [VTAGB-1:0]  vtag_f;  input [METAB-1:0] m; vtag_f  = m[PTAGB+PERMB +: VTAGB]; endfunction
   function [PERMB-1:0]  perm_f;  input [METAB-1:0] m; perm_f  = m[PTAGB +: PERMB];       endfunction
   function [PTAGB-1:0]  ptag_f;  input [METAB-1:0] m; ptag_f  = m[0 +: PTAGB];           endfunction

   // representative permission re-check (matches cache_perm_allows_ctx shape:
   // access-type case over R/W/X/U + sum/mxr). Identical body used by PIPT probe.
   function perm_ok;
      input [PERMB-1:0] p; input [3:0] c; input [1:0] pr;
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

   // word select + byte align + sign/zero extend (identical body in PIPT probe)
   function [63:0] extract;
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

   // ===== stage 1: VA -> skewed 2-way indices (the address-gen cone) =====
   function [2:0] colormix; input [ASIDB-1:0] a;
      colormix = a[2:0] ^ a[5:3] ^ {2'd0,a[6]} ^ {1'b0,a[8:7]} ^ {2'd0,a[9]};
   endfunction
   wire [COLORB-1:0] color  = va[15:12] ^ {1'b0, colormix(asid)};
   wire [IDXB-1:0]   idx0   = {color, va[11:6]};
   wire [IDXB-1:0]   idx1   = {color ^ va[19:16] ^ va[27:24], va[11:6]};
   reg  [2*IDXB-1:0] s1_q = 0;
   always @(posedge clk) s1_q <= {idx0, idx1};

   // ===== stage 2: post-array compare / way-select / extract =====
   wire [VTAGB-1:0] want_vtag = va[16 +: VTAGB];
   wire hit0 = valid_f(m0) & (epoch_f(m0)==cepoch) & (asid_f(m0)==asid)
             & (vtag_f(m0)==want_vtag) & perm_ok(perm_f(m0), ctx, priv);
   wire hit1 = valid_f(m1) & (epoch_f(m1)==cepoch) & (asid_f(m1)==asid)
             & (vtag_f(m1)==want_vtag) & perm_ok(perm_f(m1), ctx, priv);
   wire             hit  = hit0 | hit1;
   wire [LINEB-1:0] line = hit1 ? l1 : l0;
   wire [63:0]      word = line[ va[5:3]*64 +: 64 ];
   wire [63:0]      data = extract(word, va[2:0], size, 1'b1);
   wire [PTAGB-1:0] pa_tag = hit1 ? ptag_f(m1) : ptag_f(m0);    // PA side-effect
   reg [64+PTAGB:0] s2_q = 0;
   always @(posedge clk) s2_q <= {hit, data, pa_tag};

   always @(posedge clk) probe_out <= ^s1_q ^ ^s2_q;

endmodule

`default_nettype wire
