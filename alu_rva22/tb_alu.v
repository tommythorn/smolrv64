// Self-checking testbench for the RVA22 ALU. Compares the DUT against an
// independent behavioural reference over directed edge cases + random vectors.
// Run with:  verilator --binary -j 0 tb_alu.v alu.v && ./obj_dir/Vtb_alu

`include "../src/alu.v"
`default_nettype none

module tb_alu;
   localparam XLEN = 64;

   reg  [5:0]  op;
   reg         w, uw;
   reg  [63:0] op1, op2;
   wire [63:0] result, sum;
   wire        eq, lt, ltu;

   alu #(.XLEN(XLEN)) dut (.op(op), .w(w), .uw(uw), .op1(op1), .op2(op2),
                           .result(result), .sum(sum), .eq(eq), .lt(lt), .ltu(ltu));

   integer errors = 0;
   integer tests  = 0;

   // ---- reference helpers ----
   function [63:0] sext32; input [63:0] x; sext32 = {{32{x[31]}}, x[31:0]}; endfunction
   function [63:0] zext32; input [63:0] x; zext32 = {32'b0, x[31:0]};       endfunction

   function [6:0] ref_clz; input [63:0] x; input word; integer i,n; begin
      n = word?32:64; ref_clz = n;
      for (i=n-1;i>=0;i=i-1) if (x[i] && ref_clz==n) ref_clz = (n-1)-i; end endfunction
   function [6:0] ref_ctz; input [63:0] x; input word; integer i,n; begin
      n = word?32:64; ref_ctz = n;
      for (i=0;i<n;i=i+1) if (x[i] && ref_ctz==n) ref_ctz = i; end endfunction
   function [6:0] ref_pop; input [63:0] x; input word; integer i,n; begin
      n = word?32:64; ref_pop = 0;
      for (i=0;i<n;i=i+1) ref_pop = ref_pop + x[i]; end endfunction

   function [63:0] ref_alu;
      input [5:0] op; input w, uw; input [63:0] a, b;
      reg [63:0] r; reg [5:0] s; reg [4:0] sw; reg [63:0] au, aw;
      reg [31:0] a32; integer i;
      begin
         s  = b[5:0];          // 64-bit shift amount
         sw = b[4:0];          // word shift amount
         au = uw ? zext32(a) : a;
         a32 = a[31:0];
         case (op)
           `ALU_ADD:    r = au + b;   // add / add.uw (uw modifier)
           `ALU_SUB:    r = a - b;
           `ALU_SH1ADD: r = (au << 1) + b;
           `ALU_SH2ADD: r = (au << 2) + b;
           `ALU_SH3ADD: r = (au << 3) + b;
           `ALU_SLT:    r = {63'b0, ($signed(a)  < $signed(b))};
           `ALU_SLTU:   r = {63'b0, (a < b)};
           `ALU_MIN:    r = ($signed(a) < $signed(b)) ? a : b;
           `ALU_MINU:   r = (a < b) ? a : b;
           `ALU_MAX:    r = ($signed(a) < $signed(b)) ? b : a;
           `ALU_MAXU:   r = (a < b) ? b : a;
           `ALU_SLL:    r = w ? (a32 << sw) : (au << s);
           `ALU_SRL:    r = w ? (a32 >> sw) : (a >> s);
           `ALU_SRA:    r = w ? ($signed(a32) >>> sw) : ($signed(a) >>> s);
           `ALU_ROL:    if (w) r = sw==0 ? a32 : (a32 << sw) | (a32 >> (32-sw));
                        else   r = s ==0 ? a   : (a   << s ) | (a   >> (64-s ));
           `ALU_ROR:    if (w) r = sw==0 ? a32 : (a32 >> sw) | (a32 << (32-sw));
                        else   r = s ==0 ? a   : (a   >> s ) | (a   << (64-s ));
           `ALU_BEXT:   r = {63'b0, a[w ? sw : s]};
           `ALU_AND:    r = a & b;
           `ALU_OR:     r = a | b;
           `ALU_XOR:    r = a ^ b;
           `ALU_ANDN:   r = a & ~b;
           `ALU_ORN:    r = a | ~b;
           `ALU_XNOR:   r = ~(a ^ b);
           `ALU_BCLR:   r = a & ~(64'd1 << (w?sw:s));
           `ALU_BSET:   r = a |  (64'd1 << (w?sw:s));
           `ALU_BINV:   r = a ^  (64'd1 << (w?sw:s));
           `ALU_CLZ:    r = {57'b0, ref_clz(a, w)};
           `ALU_CTZ:    r = {57'b0, ref_ctz(a, w)};
           `ALU_CPOP:   r = {57'b0, ref_pop(a, w)};
           `ALU_REV8:   begin r = 0; for (i=0;i<8;i=i+1) r[8*i +: 8] = a[8*(7-i) +: 8]; end
           `ALU_ORCB:   begin r = 0; for (i=0;i<8;i=i+1) r[8*i +: 8] = (|a[8*i +: 8]) ? 8'hFF : 8'h00; end
           `ALU_SEXTB:  r = {{56{a[7]}},  a[7:0]};
           `ALU_SEXTH:  r = {{48{a[15]}}, a[15:0]};
           `ALU_ZEXTH:  r = {48'b0, a[15:0]};
           `ALU_CZEQZ:  r = (b == 0) ? 64'b0 : a;
           `ALU_CZNEZ:  r = (b != 0) ? 64'b0 : a;
           default:     r = 64'b0;
         endcase
         if (w) r = sext32(r);
         ref_alu = r;
      end
   endfunction

   // BEXT/CLZ/etc are not word ops; ROL/ROR/SLL/SRL/SRA/ADD/SUB/Zba-add are
   // (Zba .uw uses uw, not w). This list drives which modifiers we randomize.
   reg [5:0] ops [0:34];
   integer k;
   initial begin
      ops[0]=`ALU_ADD;   ops[1]=`ALU_SUB;   ops[2]=`ALU_SH1ADD; ops[3]=`ALU_SH2ADD;
      ops[4]=`ALU_SH3ADD;ops[5]=`ALU_SLT;   ops[6]=`ALU_SLTU;   ops[7]=`ALU_MIN;
      ops[8]=`ALU_MINU;  ops[9]=`ALU_MAX;   ops[10]=`ALU_MAXU;  ops[11]=`ALU_SLL;
      ops[12]=`ALU_SRL;  ops[13]=`ALU_SRA;  ops[14]=`ALU_ROL;   ops[15]=`ALU_ROR;
      ops[16]=`ALU_BEXT; ops[17]=`ALU_AND;  ops[18]=`ALU_OR;    ops[19]=`ALU_XOR;
      ops[20]=`ALU_ANDN; ops[21]=`ALU_ORN;  ops[22]=`ALU_XNOR;  ops[23]=`ALU_BCLR;
      ops[24]=`ALU_BSET; ops[25]=`ALU_BINV; ops[26]=`ALU_CLZ;   ops[27]=`ALU_CTZ;
      ops[28]=`ALU_CPOP; ops[29]=`ALU_REV8; ops[30]=`ALU_ORCB;  ops[31]=`ALU_SEXTB;
      ops[32]=`ALU_SEXTH; ops[33]=`ALU_CZEQZ; ops[34]=`ALU_CZNEZ;
   end

   task check; input [63:0] exp; reg [63:0] got; begin
      #1; got = result; tests = tests + 1;
      if (got !== exp) begin
         errors = errors + 1;
         if (errors <= 20)
           $display("MISMATCH op=%0d w=%0d uw=%0d op1=%h op2=%h : dut=%h ref=%h",
                    op, w, uw, op1, op2, got, exp);
      end
   end endtask

   integer iter;
   reg [63:0] vals [0:7];
   initial begin
      vals[0]=64'h0; vals[1]=64'hFFFFFFFFFFFFFFFF; vals[2]=64'h8000000000000000;
      vals[3]=64'h7FFFFFFFFFFFFFFF; vals[4]=64'h00000000FFFFFFFF; vals[5]=64'h0000000080000000;
      vals[6]=64'h0123456789ABCDEF; vals[7]=64'h1;

      // Directed: every op x edge operands x shamt 0..63 x {w,uw}
      for (k=0;k<=34;k=k+1) begin
         op = ops[k];
         for (iter=0; iter<8; iter=iter+1) begin : dvals
            integer j, sh, ww, uu;
            for (j=0;j<8;j=j+1) begin
               for (sh=0; sh<64; sh=(sh<4||sh>59)?sh+1:sh+7) begin
                  for (ww=0; ww<2; ww=ww+1) for (uu=0; uu<2; uu=uu+1) begin
                     op1 = vals[iter]; op2 = (vals[j] & 64'hFFFFFFFFFFFFFFC0) | sh;
                     w = ww; uw = uu;
                     check(ref_alu(op, w, uw, op1, op2));
                  end
               end
            end
         end
      end

      // Random
      for (iter=0; iter<300000; iter=iter+1) begin
         op  = ops[{$random} % 35];
         op1 = {$random, $random};
         op2 = {$random, $random};
         w   = $random & 1;
         uw  = $random & 1;
         check(ref_alu(op, w, uw, op1, op2));
      end

      $display("RVA22 ALU TB: %0d tests, %0d errors", tests, errors);
      if (errors==0) $display("ALL PASS"); else $display("FAIL");
      $finish;
   end
endmodule

`default_nettype wire
