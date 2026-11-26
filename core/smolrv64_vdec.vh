// The vector decode state and what the cracker knows of an instruction, defined once for the
// predecode (smolrv64_frontend), the slot decoders (smolrv64_dslot) and the F stage (smolrv64_core).
// Included inside a module.
//
// THE DECODE STATE (VST_W bits) is {vtype[8:0], vl version[4:0]}: vtype is {vill, vma, vta, vsew[2:0],
// vlmul[2:0]} (vill alone set when it is), the version names the VL file's entry (its low 4 bits)
// that holds vl, one bit wider than the ring so a distance between two is exact (rule B8).
// VLEN is 128.
`include "smolrv64_irec.vh"
localparam integer VST_W = `SMOLRV64_VST_W;
localparam integer UX_W  = `SMOLRV64_UX_W;         // a uop's index within its instruction (up to 32)
localparam [8:0]   VT_VILL = 9'h100;

function automatic is_opv(input [31:0] i);         // OP-V: arithmetic and the configuration ops
   is_opv = (i[6:0] == 7'b1010111);
endfunction
function automatic is_vset(input [31:0] i);        // vsetvli, vsetivli, vsetvl
   is_vset = is_opv(i) & (i[14:12] == 3'b111);
endfunction
function automatic is_vsetvl(input [31:0] i);      // vtype from a register (funct7 1000000)
   is_vsetvl = is_vset(i) & (i[31:25] == 7'b1000000);
endfunction
function automatic is_vseti(input [31:0] i);       // vsetivli: AVL is the 5-bit immediate in rs1
   is_vseti = is_vset(i) & (i[31:30] == 2'b11);
endfunction
function automatic is_vsetx(input [31:0] i);       // vsetvli: vtype is an immediate, AVL a register
   is_vsetx = is_vset(i) & ~i[31];
endfunction

// VLMAX of a legal 8-bit vtype at VLEN 128: (VLEN / SEW) * LMUL, 0 when vlmul is reserved, SEW is
// above 64 or a fractional LMUL leaves no element
function automatic [7:0] vlmax_of(input [7:0] vt);
   reg [7:0] e;                                    // VLEN / SEW
   begin
      e = (vt[5:3] == 3'd0) ? 8'd16 : (vt[5:3] == 3'd1) ? 8'd8 : (vt[5:3] == 3'd2) ? 8'd4
        : (vt[5:3] == 3'd3) ? 8'd2 : 8'd0;
      case (vt[2:0])
        3'd0: vlmax_of = e;        3'd1: vlmax_of = e << 1;
        3'd2: vlmax_of = e << 2;   3'd3: vlmax_of = e << 3;
        3'd5: vlmax_of = e >> 3;   3'd6: vlmax_of = e >> 2;   3'd7: vlmax_of = e >> 1;
        default: vlmax_of = 8'd0;                  // vlmul 100 is reserved
      endcase
   end
endfunction
// the vtype a vsetvli's or vsetivli's immediate asks for, legalized as Simmerv does: vill alone
// when a reserved bit is set or VLMAX would be 0
function automatic [8:0] vset_vtype(input [31:0] i);
   reg [10:0] z;
   begin
      z = is_vseti(i) ? {1'b0, i[29:20]} : i[30:20];
      vset_vtype = ((z[10:8] != 3'd0) || (vlmax_of(z[7:0]) == 8'd0)) ? VT_VILL : {1'b0, z[7:0]};
   end
endfunction
// vsetvli x0, x0 with a legal vtype keeps vl: no version, one uop
function automatic vset_keep(input [31:0] i);
   reg [8:0] t;
   begin
      t = vset_vtype(i);
      vset_keep = is_vsetx(i) & (i[19:15] == 5'd0) & (i[11:7] == 5'd0) & ~t[8];
   end
endfunction
// a vset that writes vl from its immediates allocates the next version
function automatic vset_alloc(input [31:0] i);
   vset_alloc = is_vset(i) & ~is_vsetvl(i) & ~vset_keep(i);
endfunction
// the instruction's uops: a vset that allocates is two (the vl write, then rd), anything else one
// (vsetvl too: it is a SYSQ op, which writes the decode state itself and refetches what follows)
function automatic [UX_W:0] uop_count(input [31:0] i);
   uop_count = vset_alloc(i) ? 2 : 1;
endfunction
// the decode state after the instruction, from the state before it
function automatic [VST_W-1:0] vst_after(input [31:0] i, input [VST_W-1:0] s);
   vst_after = vset_alloc(i) ? {vset_vtype(i), s[4:0] + 5'd1}
             : (is_vset(i) & ~is_vsetvl(i)) ? {vset_vtype(i), s[4:0]} : s;
endfunction

// THE VL UOP's internal word (uop 0 of a vset that allocates): an OP-FP op of a class no
// instruction uses (funct5 10111), which decode_fp calls an in-core op with an integer source, so it
// issues in its lane, which hands rs1 to the F stage. {10111, form, vtype[7:3], rs1, vtype[2:0], 0,
// OP-FP}; form 0 takes AVL from rs1, 1 from the immediate in the rs1 field, 2 is VLMAX (rs1 = x0,
// rd != x0); a vill vtype is vlmul 100, VLMAX 0, so vl is 0. Its destination is the VL file's entry,
// named by the record's rd (VL_ID + the version), never renamed.
localparam [6:0] VL_ID = 7'd112;
function automatic [31:0] vl_word(input [31:0] i);
   reg [1:0] form;
   reg [8:0] t;
   reg [7:0] vt;
   begin
      form = is_vseti(i) ? 2'd1 : (i[19:15] == 5'd0) ? 2'd2 : 2'd0;
      t    = vset_vtype(i);
      vt   = t[8] ? 8'b0000_0100 : t[7:0];
      vl_word = {5'b10111, form, vt[7:3], i[19:15], vt[2:0], 5'd0, 7'b1010011};
   end
endfunction
function automatic is_vlword(input [31:0] i);
   is_vlword = (i[6:0] == 7'b1010011) & (i[31:27] == 5'b10111);
endfunction
// the vl a VL uop writes: min(AVL, VLMAX)
function automatic [7:0] vl_of(input [31:0] w, input [63:0] rs1);
   reg [7:0] m;
   begin
      m = vlmax_of({w[24:20], w[14:12]});
      case (w[26:25])
        2'd0:    vl_of = (rs1 < {56'd0, m}) ? rs1[7:0] : m;
        2'd1:    vl_of = ({3'd0, w[19:15]} < m) ? {3'd0, w[19:15]} : m;
        default: vl_of = m;
      endcase
   end
endfunction
