`timescale 1ns/1ps
`default_nettype none

// Directed tests for the fetch-window aligner. Halfword 0x..03 (==11) is a
// 32-bit opcode low half; 0x..01 (==01) is an RVC. Cases cover: full RVC bundle,
// full 32b bundle, mixed lengths, a 32b op straddling the window end (must NOT be
// consumed -> carries to next window), and short `avail` (fetch bubble / edge).
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBW=4;

   reg  [HW*16-1:0]  hwin;
   reg  [PBW-1:0]    avail;
   reg  [PCW-1:0]    base_pc;
   reg  [SEQW-1:0]   base_seq;
   wire [IW-1:0]     valid;
   wire [IW*32-1:0]  inst;
   wire [IW*PCW-1:0] pc;
   wire [IW*SEQW-1:0] seq;
   wire [PBW-1:0]    consumed;
   integer errs = 0;

   reg  [HW-1:0] st;
   always @* begin : sts
      integer j;
      st[0] = 1'b1;
      for (j = 1; j < HW; j = j + 1) st[j] = ~(st[j-1] & (hwin[(j-1)*16 +: 2] == 2'b11));
   end
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) dut
     (.hwin(hwin), .avail(avail), .base_pc(base_pc), .base_seq(base_seq),
      .solo_all(1'b0), .bytes_late(1'b0), .st(st), .mk({HW{1'b0}}), .mk_tk('0),
      .valid(valid), .inst(inst), .pc(pc), .seq(seq), .consumed(consumed));

   reg [15:0] h [0:HW-1];
   integer i;
   task drive(input [PBW-1:0] av);
      begin for (i=0;i<HW;i=i+1) hwin[i*16 +: 16] = h[i]; avail=av; #1; end
   endtask
   task clr; begin for (i=0;i<HW;i=i+1) h[i]=16'h0000; end endtask
   task ckv(input [127:0] nm, input [IW-1:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s valid=%b exp %b",nm,got,exp); errs=errs+1; end end
   endtask
   task ckc(input [127:0] nm, input [PBW-1:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s consumed=%0d exp %0d",nm,got,exp); errs=errs+1; end end
   endtask
   task cki(input [127:0] nm, input integer k, input [31:0] exp);
      begin if (inst[k*32+:32]!==exp) begin $display("FAIL %0s inst[%0d]=%h exp %h",nm,k,inst[k*32+:32],exp); errs=errs+1; end end
   endtask
   task ckp(input [127:0] nm, input integer k, input [PCW-1:0] exp);
      begin if (pc[k*PCW+:PCW]!==exp) begin $display("FAIL %0s pc[%0d]=%0d exp %0d",nm,k,pc[k*PCW+:PCW],exp); errs=errs+1; end end
   endtask

   // the start-bit aligner against the serial one it replaces, on random windows
   aligner_eq #(.IW(4), .HW(8)) eq_a();
   aligner_eq #(.IW(6), .HW(8)) eq_b();
   aligner_eq #(.IW(6), .HW(2)) eq_c();

   initial begin
      base_pc = 64'h8000_0000; base_seq = 8'd0;
      wait (eq_a.done & eq_b.done & eq_c.done);

      // 1: four RVC
      clr; h[0]=16'h4501; h[1]=16'h4505; h[2]=16'h4509; h[3]=16'h450d; drive(8);
      ckv("rvc4",valid,4'b1111); ckc("rvc4",consumed,4);
      cki("rvc4",0,32'h45054501); ckp("rvc4",1,64'h8000_0002); ckp("rvc4",3,64'h8000_0006);
      if (seq[3*SEQW+:SEQW]!==8'd3) begin $display("FAIL rvc4 seq3"); errs=errs+1; end

      // 2: four 32-bit
      clr; h[0]=16'h0093; h[2]=16'h1093; h[4]=16'h2093; h[6]=16'h3093; drive(8);
      ckv("w4",valid,4'b1111); ckc("w4",consumed,8);
      cki("w4",0,32'h00000093); cki("w4",1,32'h00001093);
      ckp("w4",1,64'h8000_0004); ckp("w4",3,64'h8000_000c);

      // 3: mixed 32, rvc, 32, rvc
      clr; h[0]=16'h0093; h[2]=16'h4501; h[3]=16'h2093; h[5]=16'h4505; drive(8);
      ckv("mix",valid,4'b1111); ckc("mix",consumed,6);
      cki("mix",0,32'h00000093); cki("mix",2,32'h00002093);
      ckp("mix",0,64'h8000_0000); ckp("mix",1,64'h8000_0004);
      ckp("mix",2,64'h8000_0006); ckp("mix",3,64'h8000_000a);

      // 4: three 32b then a 32b straddling the window end (avail=7) -> not consumed
      clr; h[0]=16'h0093; h[2]=16'h1093; h[4]=16'h2093; h[6]=16'h3093; drive(7);
      ckv("straddle",valid,4'b0111); ckc("straddle",consumed,6);

      // 5: empty window
      clr; drive(0);
      ckv("empty",valid,4'b0000); ckc("empty",consumed,0);

      // 6: four RVC but only 3 halfwords available
      clr; h[0]=16'h4501; h[1]=16'h4505; h[2]=16'h4509; h[3]=16'h450d; drive(3);
      ckv("short",valid,4'b0111); ckc("short",consumed,3);

      // 7: RVC then a 32b straddling immediately (avail=2) -> only the RVC, carry 1
      clr; h[0]=16'h4501; h[1]=16'h0093; drive(2);
      ckv("carry",valid,4'b0001); ckc("carry",consumed,1);

      #1;
      errs = errs + eq_a.errs + eq_b.errs + eq_c.errs;
      if (errs==0) $display("aligner: ALL TESTS PASSED");
      else         $display("aligner: %0d FAILURES", errs);
      $finish;
   end
endmodule

// Random equivalence: the start-bit aligner and aligner_ref (the serial length scan it replaced)
// see the same windows, marks, predictions, window ends, bytes_late and solo_all; every output a
// consumer reads must agree (the slots' words, PCs and offsets only where valid).
module aligner_eq #(parameter IW = 4, parameter HW = 8);
   localparam PCW = 64, SEQW = 8, PBW = $clog2(HW+2);
   reg  [HW*16-1:0]  hwin;
   reg  [PBW-1:0]    avail;
   reg               bl, solo;
   reg  [HW-1:0]     mk, st;
   reg  [IW-1:0]     mtk;
   wire [IW-1:0]     v0, v1, smk0, smk1, sbr0, sbr1;
   wire [IW*32-1:0]  i0, i1;
   wire [IW*PCW-1:0] p0, p1;
   wire [IW*PBW-1:0] o0, o1;
   wire [IW*SEQW-1:0] q0, q1;
   wire [PBW-1:0]    c0, c1;
   wire              bt0, bt1, mt0, mt1;
   integer errs = 0, n, j, k;
   reg done = 1'b0;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) a
     (.hwin(hwin), .avail(avail), .base_pc(64'h8000_1000), .base_seq(8'd7), .solo_all(solo),
      .bytes_late(bl), .st(st), .mk(mk), .mk_tk(mtk), .valid(v1), .inst(i1), .pc(p1), .offs(o1),
      .seq(q1), .consumed(c1), .br_term(bt1), .mk_term(mt1), .smk(smk1), .sbr(sbr1));
   aligner_ref #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) r
     (.hwin(hwin), .avail(avail), .base_pc(64'h8000_1000), .base_seq(8'd7), .solo_all(solo),
      .bytes_late(bl), .mk(mk), .mk_tk(mtk), .valid(v0), .inst(i0), .pc(p0), .offs(o0),
      .seq(q0), .consumed(c0), .br_term(bt0), .mk_term(mt0), .smk(smk0), .sbr(sbr0));
   // a halfword drawn from the classes the aligner tells apart
   function [15:0] hw_rand(input integer r);
      reg [15:0] x;
      begin
         x = $random;
         case (r % 12)
            0: x[6:0] = 7'b1100011;                           // BRANCH
            1: x[6:0] = 7'b1101111;                           // JAL
            2: x[6:0] = 7'b1100111;                           // JALR
            3: x[6:0] = 7'b1110011;                           // SYSTEM
            4: x[6:0] = 7'b0101111;                           // AMO
            5: begin x[6:0] = 7'b0001111; x[14:13] = 2'b00; end // FENCE
            6: x[6:0] = 7'b0010011;                           // OP-IMM
            7: begin x[1:0] = 2'b01; x[15:13] = 3'b101; end   // C.J
            8: begin x[1:0] = 2'b01; x[15:13] = {2'b11, x[13]}; end // C.BEQZ/C.BNEZ
            9: begin x[1:0] = 2'b10; x[15:13] = 3'b100; x[6:2] = 5'd0; end // C.JR/C.JALR/C.EBREAK
            default: x[1:0] = x[0] ? 2'b01 : 2'b10;          // another RVC
         endcase
         hw_rand = x;
      end
   endfunction
   initial begin
      for (n = 0; n < 200000; n = n + 1) begin
         for (j = 0; j < HW; j = j + 1) hwin[j*16 +: 16] = hw_rand($unsigned($random));
         avail = $unsigned($random) % (HW + 1);
         mk = $random;  mtk = $random;  bl = $random;  solo = (($unsigned($random) % 16) == 0);
         st[0] = 1'b1;
         for (j = 1; j < HW; j = j + 1) st[j] = ~(st[j-1] & (hwin[(j-1)*16 +: 2] == 2'b11));
         #1;
         if ({v1, c1, bt1, mt1, smk1, sbr1} !== {v0, c0, bt0, mt0, smk0, sbr0}) begin
            errs = errs + 1;
            if (errs < 5) $display("FAIL eq IW=%0d HW=%0d: valid %b/%b consumed %0d/%0d bt %b/%b mt %b/%b smk %b/%b sbr %b/%b",
                                   IW, HW, v1, v0, c1, c0, bt1, bt0, mt1, mt0, smk1, smk0, sbr1, sbr0);
         end
         for (k = 0; k < IW; k = k + 1)
            if (v0[k] && ({i1[k*32 +: 32], p1[k*PCW +: PCW], o1[k*PBW +: PBW], q1[k*SEQW +: SEQW]}
                          !== {i0[k*32 +: 32], p0[k*PCW +: PCW], o0[k*PBW +: PBW], q0[k*SEQW +: SEQW]})) begin
               errs = errs + 1;
               if (errs < 5) $display("FAIL eq IW=%0d HW=%0d slot %0d: inst %h/%h pc %h/%h", IW, HW, k,
                                      i1[k*32 +: 32], i0[k*32 +: 32], p1[k*PCW +: PCW], p0[k*PCW +: PCW]);
            end
      end
      done = 1'b1;
   end
endmodule


// Fetch-window aligner: carves up to IW variable-length instructions (RVC 16b /
// base 32b) out of a window of HW contiguous halfwords whose first halfword is
// at base_pc. Produces exactly the decode_stage input contract
// ({inst[31:0], in_valid, seq} per slot) plus per-slot PC and a `consumed`
// halfword count so fetch can advance.
//
// Length scan: instruction k begins at halfword position p[k]; it is 32-bit iff
// hw[p[k]][1:0]==11 (consumes 2 halfwords) else 16-bit (1). p[0]=0,
// p[k+1]=p[k]+len[k]. This is the classic O(W) dependent prefix scan -- short
// (4 deep) and made of +1/+2 adders on a <=4-bit position.
//
// The data path is uniform: inst[k] is always the 32-bit window {hw[p+1],hw[p]};
// for an RVC the upper half is the next instruction's first halfword, which
// decode_slot ignores (it re-derives is_c from inst[1:0]). So only the *length*
// depends on [1:0] -- no data mux on is32, and no RVC expansion here.
//
// `avail` = how many halfwords of the window are actually present (normally HW;
// fewer at a fetch bubble or buffer edge). An instruction is valid only if all
// its halfwords are present, and validity is a prefix: the first instruction
// that doesn't fit (e.g. a 32-bit op straddling the window end) stops the bundle
// and is NOT counted in `consumed`, so its leading halfword carries to the next
// window. Fixed steering = slot k -> shard k (the slot order IS the steering).
module aligner_ref
  #(parameter IW   = 4,
    parameter HW   = 8,                 // window halfwords (>= 2*IW for a full 32b bundle)
    parameter PCW  = 64,
    parameter SEQW = 8,
    parameter PBW_O = $clog2(HW+2))     // = PBW below; a port width needs it before the body
   (input  wire [HW*16-1:0]        hwin,
    input  wire [$clog2(HW+2)-1:0] avail,    // halfwords present (0..HW)
    input  wire [PCW-1:0]          base_pc,  // PC of hwin[0]
    input  wire [SEQW-1:0]         base_seq, // seq of slot 0
    input  wire                    solo_all, // force one instruction per bundle (fault replay)
    input  wire                    bytes_late, // the window is short of the page end: more is coming
    input  wire [HW-1:0]           mk,       // halfwords that end a CTI the predictor knows (smolrv64_fring):
                                             // an instruction ending on one predicted taken ends the
                                             // bundle, and one whose FIRST halfword is marked does not fit
    input  wire [IW-1:0]           mk_tk,    // the window's marks' predictions in order: taken
    output wire [IW-1:0]           valid,
    output wire [IW*32-1:0]        inst,
    output wire [IW*PCW-1:0]       pc,
    output wire [IW*PBW_O-1:0]     offs,     // each slot's halfword offset from base_pc (fetch builds
                                             // the PCs from precomputed sums: the adder here is
                                             // for the benches, and off the fetch path)
    output wire [IW*SEQW-1:0]      seq,
    output wire [$clog2(HW+2)-1:0] consumed,
    output wire                    br_term,   // bundle ends on a genuine branch/jump (not a
                                              // window cut or a SYSTEM/AMO/FENCE terminator)
    output wire                    mk_term,   // bundle ends on a marked halfword
    output wire [IW-1:0]           smk,       // per slot: it ends on a marked halfword
    output wire [IW-1:0]           sbr);      // per slot: it is a genuine branch/jump

   localparam PBW = $clog2(HW+2);        // holds a position 0..HW (+1 for lookahead)

   // halfword read with out-of-window guard (returns 0 past the window)
   function [15:0] hwr(input [PBW-1:0] idx);
      hwr = (idx < HW) ? hwin[idx*16 +: 16] : 16'b0;
   endfunction
   function mkr(input [PBW-1:0] idx);
      mkr = (idx < HW) ? mk[idx[$clog2(HW)-1:0]] : 1'b0;
   endfunction

   // End-of-bundle predecode (opcode bits only -- no full RVC expansion). A jump, or a
   // serializing op (SYSTEM / AMO / FENCE), *terminates* the bundle: it is the last valid slot,
   // and the rest reappears as slot 0 of the next window (consumed stops at it, like the
   // straddle case). A conditional branch does not, unless it ends on a mark predicted taken:
   // one the predictor calls not taken, or does not know, is predicted to fall through, and the
   // bundle reads on. A branch with younger instructions in its own group is what the core
   // recovers from by its ROB row's columns (smolrv64_rob's kill and walk).
   `include "smolrv64_sysop.vh"
   function is_cti(input [15:0] h0);
      if (h0[1:0] == 2'b11)                 // base 32-bit op (h0[1:0]==11 IS the old `is32` input)
         is_cti = (h0[6:0] == 7'b1100011)   // BRANCH
                | (h0[6:0] == 7'b1101111)   // JAL
                | (h0[6:0] == 7'b1100111)   // JALR
                | sysop(h0[14:0])            // SYSTEM (ecall/ebreak/csr/xret/wfi/sfence; not Zimop)
                | (h0[6:0] == 7'b0101111)   // AMO (atomic: solo, serialized RMW)
                | ((h0[6:0] == 7'b0001111) & (h0[14:13] == 2'b00)); // FENCE / FENCE.I (barrier; excl CBO f3=010)
      else case (h0[1:0])
         2'b01:   is_cti = (h0[15:13] == 3'b101)    // C.J
                         | (h0[15:13] == 3'b110)    // C.BEQZ
                         | (h0[15:13] == 3'b111);   // C.BNEZ
         2'b10:   is_cti = (h0[15:13] == 3'b100)     // C.JR / C.JALR / C.EBREAK
                         & (h0[6:2] == 5'd0);
         default: is_cti = 1'b0;
      endcase
   endfunction

   // Genuine branch/jump (the ops branch_unit's actual_npc != pred_npc compare
   // covers). The predictor may only steer fetch off a bundle ending in one of
   // these: a prediction on any other bundle shape (a stale BTB entry over
   // changed code, a SYSTEM/AMO/FENCE terminator, C.EBREAK) would have NO
   // corrective compare at execute and the wrong path would retire.
   // A conditional branch (BRANCH, C.BEQZ, C.BNEZ).
   function is_cond(input [15:0] h0);
      if (h0[1:0] == 2'b11) is_cond = (h0[6:0] == 7'b1100011);
      else                  is_cond = (h0[1:0] == 2'b01) & ((h0[15:13] == 3'b110) | (h0[15:13] == 3'b111));
   endfunction

   function is_br(input [15:0] h0);
      if (h0[1:0] == 2'b11)
         is_br = (h0[6:0] == 7'b1100011)    // BRANCH
               | (h0[6:0] == 7'b1101111)    // JAL
               | (h0[6:0] == 7'b1100111);   // JALR
      else case (h0[1:0])
         2'b01:   is_br = (h0[15:13] == 3'b101)    // C.J
                        | (h0[15:13] == 3'b110)    // C.BEQZ
                        | (h0[15:13] == 3'b111);   // C.BNEZ
         2'b10:   is_br = (h0[15:13] == 3'b100)    // C.JR / C.JALR
                        & (h0[6:2] == 5'd0)
                        & (h0[11:7] != 5'd0);      // rs1==0 is C.EBREAK
         default: is_br = 1'b0;
      endcase
   endfunction

   reg  [IW-1:0]    v;
   reg  [31:0]      ir   [0:IW-1];
   reg  [PCW-1:0]   pcv  [0:IW-1];
   reg  [PBW-1:0]   ofv  [0:IW-1];
   reg  [SEQW-1:0]  sqv  [0:IW-1];
   reg  [PBW-1:0]   cons;
   reg              bt, mt;
   reg  [IW-1:0]    smv, sbv;
   reg  [$clog2(IW+1)-1:0] mi;            // the marks read so far: the next one's prediction is mk_tk[mi]

   integer k;
   reg [PBW-1:0] pos;
   reg           run;
   reg [15:0]    h0;
   reg           is32, have, is_sys, mkin;
   // Explicit sensitivity: hwin is read via the hwr() function, which iverilog's
   // @* does not pull into the list -- name it so the block re-evaluates on it.
   // A BUNDLE WAITS FOR ITS BYTES RATHER THAN CUTTING AT THEM (item 10e, 2026-09-05). The
   // two-wide fetch's first cut ended a bundle at the 16-byte chunk boundary after slot 0:
   // the chunk beyond may or may not have arrived, so a bundle allowed to span it has a
   // shape that depends on timing, and the predictor, keyed by the bundle's base PC, then
   // meets the same branch under different bases from one iteration to the next (every
   // sha256 loop branch mispredicted, 0.003 -> 1.36 redirects per thousand). The cut made
   // a bundle's shape a function of the code alone -- and 338 of that kernel's 934 bundles
   // singles. The OTHER choice has the same property: when a later slot needs halfwords the
   // window does not have yet but WILL (`bytes_late`: the shortfall is the buffer's, not
   // the page's), the whole bundle waits, and its shape is still the code's. The buffer
   // runs two chunks ahead now (rv_soc_top), so the wait is rare; at the page end the
   // window is cut as before, deterministic because the page boundary is the code's too.
   reg wt;
   always @(hwin or avail or base_pc or base_seq or solo_all or bytes_late or mk or mk_tk) begin
      pos = 0;
      run = 1'b1;
      bt  = 1'b0;
      mt  = 1'b0;
      wt  = 1'b0;
      smv = {IW{1'b0}};
      sbv = {IW{1'b0}};
      mi  = 0;
      for (k = 0; k < IW; k = k + 1) begin
         h0   = hwr(pos);
         is32 = (h0[1:0] == 2'b11);
         // all needed halfwords present?  first always, second only if 32-bit
         have = (pos < avail) && (!is32 || ((pos + 1'b1) < avail));
         mkin = is32 & mkr(pos);              // a mark inside it: what follows is another path's
         // A SYSTEM op (ecall/ebreak/csr/xret), an AMO, or a FENCE is SOLO in its bundle:
         // terminate the bundle BEFORE it (if not slot 0) as well as after (via is_cti). Solo
         // SYSTEM lets a trap roll back TO that checkpoint and precisely annul the faulting op's
         // rd (e.g. a priv-violating csrr). Solo AMO/FENCE means that when it issues (gated to
         // oldest) every older store has drained, so an AMO's direct-memory RMW is coherent and a
         // FENCE is a true ordering point (older writes precede any younger store, incl. a DMA notify).
         is_sys = is32 && (sysop(h0[14:0]) || (h0[6:0] == 7'b0101111)
                           || ((h0[6:0] == 7'b0001111) && (h0[14:13] == 2'b00))); // FENCE / FENCE.I solo
         ir[k]  = {hwr(pos + 1'b1), h0};      // uniform 32-bit window
         pcv[k] = base_pc + (pos << 1);
         ofv[k] = pos;
         sqv[k] = base_seq + k[SEQW-1:0];
         // A solo op begins a fresh bundle only if its halfwords are PRESENT. Testing `is_sys`
         // on a slot beyond the window read the pair register's STALE bytes -- after a restart
         // into a chunk whose successor had not landed, a csrr further down the loop sat at
         // slot 1's position and ended the bundle after slot 0 without the wait below, so the
         // bundle's shape depended on what the buffer used to hold: the predictor, keyed by
         // the bundle base, then met the loop's back edge under a third base and lost its
         // prediction (workloads/brbench near: 15 of 300 iterations, BP_TRACE, 2026-09-10).
         if ((k != 0) && (solo_all || (have && is_sys))) begin
            v[k] = 1'b0; run = 1'b0;           // SYSTEM (or fault replay) begins a fresh (solo) bundle
         end else if (mkin) begin
            v[k] = 1'b0; run = 1'b0;           // it does not fit, and no more of it is coming
         end else begin
            v[k] = run & have;
            if (v[k]) begin
               pos = pos + (is32 ? 2'd2 : 2'd1);
               if (is_cti(h0) & ~is_cond(h0)) run = 1'b0;   // a jump, SYSTEM, AMO or FENCE ends the bundle
               if (is_br(h0))  begin bt = 1'b1; sbv[k] = 1'b1; end   // ... a real branch/jump (is_br implies is_cti)
               if (mkr(pos - 1'b1)) begin                  // it ends on a mark: the mi-th
                  smv[k] = 1'b1;
                  if (mk_tk[mi[$clog2(IW)-1:0]]) begin run = 1'b0; mt = 1'b1; end   // ...predicted taken: the bundle ends
                  mi = mi + 1'b1;
               end
            end else begin                     // prefix: stop at the first that doesn't fit...
               if ((k != 0) && run && bytes_late) wt = 1'b1;   // ...or wait, when it is coming
               run = 1'b0;
            end
         end
      end
      if (wt) begin v = {IW{1'b0}}; bt = 1'b0; mt = 1'b0; smv = {IW{1'b0}}; sbv = {IW{1'b0}}; end
      cons = wt ? {PBW{1'b0}} : pos;          // halfwords consumed (straddler excluded)
   end

   genvar g;
   generate
      for (g = 0; g < IW; g = g + 1) begin : pk
         assign inst[g*32  +: 32]   = ir[g];
         assign pc  [g*PCW +: PCW]  = pcv[g];
         assign offs[g*PBW +: PBW]  = ofv[g];
         assign seq [g*SEQW +: SEQW] = sqv[g];
      end
   endgenerate
   assign valid    = v;
   assign consumed = cons;
   assign br_term  = bt;
   assign mk_term  = mt;
   assign smk      = smv;
   assign sbr      = sbv;
endmodule


`default_nettype wire
