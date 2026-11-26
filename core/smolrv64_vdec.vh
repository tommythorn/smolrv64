// The vector decode state and what the cracker knows of an instruction, defined once for the
// predecode (smolrv64_frontend), the slot decoders (smolrv64_dslot) and the F stage (smolrv64_core).
// Included inside a module.
//
// THE DECODE STATE (VST_W bits) is {vz, vtype[8:0], vl version[4:0]}: vtype is {vill, vma, vta,
// vsew[2:0], vlmul[2:0]} (vill alone set when it is), the version names the VL file's entry (its low
// 4 bits) that holds vl, one bit wider than the ring so a distance between two is exact (rule B8).
// vz says a vector instruction has been predecoded since the last refetch, so vstart is 0; else
// vstart is the CSR's, which only a refetching write or a trap changes. VLEN is 128.
`include "smolrv64_irec.vh"
localparam integer VST_W = `SMOLRV64_VST_W;
localparam integer UX_W  = `SMOLRV64_UX_W;         // a uop's index within its instruction (up to 32)
localparam integer VS_VZ = VST_W - 1;              // the decode state's vz bit
localparam [8:0]   VT_VILL = 9'h100;
localparam [VST_W-1:0] VST_RESET = {1'b0, VT_VILL, 5'd0};

function automatic is_opv(input [31:0] i);         // OP-V: arithmetic and the configuration ops
   is_opv = (i[6:0] == 7'b1010111);
endfunction
// a vector instruction: OP-V, or a LOAD-FP/STORE-FP of a vector width (not FLW/FLD/FLH/FLQ's)
function automatic is_vecins(input [31:0] i);
   is_vecins = is_opv(i) | (((i[6:2] == 5'b00001) | (i[6:2] == 5'b01001))
                            & ((i[14:12] == 3'b000) | (i[14] & (i[13:12] != 2'b00))));
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

// THE MOVES, the vector unit's instructions (unmasked: vm = 1). vmv.v.v/.x/.i (vs2 is not read, as
// Simmerv), vmv<nr>r.v (nr = 1, 2, 4 or 8, neither group past v31), vmv.s.x (vs2 = 0) and vmv.x.s
// (vs1 = 0). Every other vector arithmetic instruction is illegal.
function automatic is_vmvv(input [31:0] i);        // vmv.v.v (OPIVV), vmv.v.i (OPIVI), vmv.v.x (OPIVX)
   is_vmvv = is_opv(i) & (i[31:25] == 7'b0101111)
           & ((i[14:12] == 3'b000) | (i[14:12] == 3'b011) | (i[14:12] == 3'b100));
endfunction
function automatic is_vmvr(input [31:0] i);        // vmv<nr>r.v: OPIVI, nr - 1 in the immediate
   reg [5:0] nr;
   begin
      nr = {1'b0, i[19:15]} + 6'd1;
      is_vmvr = is_opv(i) & (i[31:25] == 7'b1001111) & (i[14:12] == 3'b011)
              & ((i[19:15] == 5'd0) | (i[19:15] == 5'd1) | (i[19:15] == 5'd3) | (i[19:15] == 5'd7))
              & (({1'b0, i[11:7]} + nr) <= 6'd32) & (({1'b0, i[24:20]} + nr) <= 6'd32);
   end
endfunction
function automatic is_vmvsx(input [31:0] i);       // vmv.s.x: OPMVX
   is_vmvsx = is_opv(i) & (i[31:25] == 7'b0100001) & (i[14:12] == 3'b110) & (i[24:20] == 5'd0);
endfunction
function automatic is_vmvxs(input [31:0] i);       // vmv.x.s: OPMVV
   is_vmvxs = is_opv(i) & (i[31:25] == 7'b0100001) & (i[14:12] == 3'b010) & (i[19:15] == 5'd0);
endfunction
function automatic is_vmv(input [31:0] i);
   is_vmv = is_vmvv(i) | is_vmvr(i) | is_vmvsx(i) | is_vmvxs(i);
endfunction
// a move whose scalar comes from an x register: uop 0 moves it into the hidden register VH0 (an
// FMV.D.X, through a lane like any int-to-FP op), the vector uops read it there
function automatic vmv_x(input [31:0] i);
   vmv_x = (is_vmvv(i) & (i[14:12] == 3'b100)) | is_vmvsx(i);
endfunction
localparam [6:0] VH0 = 7'd96;                      // the first hidden register
// the registers a move writes: nr, LMUL (1 for a fractional one), or 1
function automatic [3:0] vmv_regs(input [31:0] i, input [VST_W-1:0] s);
   vmv_regs = is_vmvr(i) ? {1'b0, i[17:15]} + 4'd1
            : (is_vmvv(i) & ~s[7]) ? (4'd1 << s[6:5]) : 4'd1;
endfunction

// THE WHOLE-REGISTER LOADS AND STORES, vl<nf>re<eew>.v and vs<nf>r.v (nf = 1, 2, 4 or 8, the group
// not past v31; vtype and vl are not read, so vill does not trap them). Each register is two 8-byte
// chunks through VH0, a chunk two uops: a load's FLD of the chunk into VH0 and its pack into the
// register (prestart bytes keep the old ones), a store's extract of the chunk into VH0 and its FSD.
// Position p of the uops is {register, chunk, second}: a fault at a chunk leaves the chunks before
// it written, and a retry cracks from the chunk vstart is in, every chunk before it done.
function automatic is_vwr(input [31:0] i, input st);   // st: a store's
   reg [5:0] nf;
   begin
      nf = {2'b00, i[31:29]} + 6'd1;
      is_vwr = (i[6:0] == (st ? 7'b0100111 : 7'b0000111))
             & ((i[14:12] == 3'b000) | (i[14] & (i[13:12] != 2'b00)))      // a vector width
             & (i[28:26] == 3'b000) & (i[24:20] == 5'b01000)                // mew 0, unit-stride, whole
             & ((i[31:29] == 3'd0) | (i[31:29] == 3'd1) | (i[31:29] == 3'd3) | (i[31:29] == 3'd7))
             & (({1'b0, i[11:7]} + nf) <= 6'd32);
   end
endfunction
function automatic is_vlr(input [31:0] i); is_vlr = is_vwr(i, 1'b0); endfunction
function automatic is_vsr(input [31:0] i); is_vsr = is_vwr(i, 1'b1); endfunction
// the position a whole-register access cracks from, and its uops: four a register, less the two
// of each chunk wholly before vstart (vstart counts elements of the encoding's EEW, i[13:12] its
// log2 bytes). Past the last element it is one uop that changes nothing: the last register's
// pack of nothing, or its second extract.
function automatic [6:0] vwr_c(input [31:0] i, input [6:0] vs);   // vstart's chunk
   reg [9:0] b;                                    // vstart in bytes
   begin
      b = {3'd0, vs} << i[13:12];
      vwr_c = b[9:3];
   end
endfunction
function automatic vwr_past(input [31:0] i, input [6:0] vs);
   vwr_past = vwr_c(i, vs) >= {2'b00, i[31:29], 1'b0} + 7'd2;
endfunction
function automatic [UX_W:0] vwr_s0(input [31:0] i, input [6:0] vs);
   reg [6:0] c;
   begin
      c = vwr_c(i, vs);
      vwr_s0 = ~vwr_past(i, vs) ? {c[UX_W-1:0], 1'b0} : {i[31:29], 2'b00} + (is_vlr(i) ? 6'd3 : 6'd2);
   end
endfunction

// the instruction's uops, from the decode state before it and vstart: a vset that allocates is two
// (the vl write, then rd); a move one a register it writes, after the scalar's move-in; a
// whole-register access four a register from vstart's chunk on; anything else one (vsetvl too: it
// is a SYSQ op, which writes the decode state itself and refetches what follows, and a move at
// vill, which traps)
function automatic [UX_W:0] uop_count(input [31:0] i, input [VST_W-1:0] s, input [6:0] vs);
   reg [UX_W:0] n;
   begin
      n = {i[31:29], 2'b00} + 6'd4;                 // a whole-register access's uops from 0: 4 * nf
      uop_count = vset_alloc(i) ? 2
                : (is_vmv(i) & ~s[13]) ? {2'b00, vmv_regs(i, s)} + {{UX_W{1'b0}}, vmv_x(i)}
                : ((is_vlr(i) | is_vsr(i)) & ~vwr_past(i, vs)) ? n - vwr_s0(i, vs)
                : 1;
   end
endfunction
// the decode state after the instruction, from the state before it
function automatic [VST_W-1:0] vst_after(input [31:0] i, input [VST_W-1:0] s);
   vst_after = {s[VS_VZ] | is_vecins(i),
                vset_alloc(i) ? {vset_vtype(i), s[4:0] + 5'd1}
              : (is_vset(i) & ~is_vsetvl(i)) ? {vset_vtype(i), s[4:0]} : s[VST_W-2:0]};
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
// THE MOVE-IN's internal word: FMV.D.X of the instruction's rs1 (its destination is VH0)
function automatic [31:0] mvin_word(input [31:0] i);
   mvin_word = {5'b11110, 2'b01, 5'd0, i[19:15], 3'b000, 5'd0, 7'b1010011};
endfunction

// A VECTOR UOP keeps its instruction's word (the last retires it); what else the F stage needs rides
// in the record's immediate, the uop's place: {vz, vsew[1:0], the register of the group}, a pack's
// or an extract's {vz, its chunk, 0, the register}
localparam integer VP_W = 6;
function automatic [VP_W-1:0] vpos_of(input [31:0] i, input [UX_W-1:0] ux, input [VST_W-1:0] s);
   reg [UX_W-1:0] r;
   begin
      r = ux - {{(UX_W-1){1'b0}}, vmv_x(i)};
      vpos_of = {s[VS_VZ], s[9:8], r[2:0]};
   end
endfunction
