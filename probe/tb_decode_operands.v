`timescale 1ns/1ps
`default_nettype none

// Directed test for decode_operands: one representative per format/opcode plus
// the tricky cases (x0 dest, shift-imm rs2 not a read, CSR vs CSR-imm, illegal).
module tb;
   reg  [31:0] insn;
   wire [5:0]  rd, rs1, rs2;
   wire        rd_v, rs1_v, rs2_v, has_imm, legal;
   wire [63:0] imm;
   integer errs = 0, tid = 0;

   decode_operands dut (.insn(insn), .rd(rd), .rd_v(rd_v), .rs1(rs1), .rs1_v(rs1_v),
                        .rs2(rs2), .rs2_v(rs2_v), .imm(imm), .has_imm(has_imm), .legal(legal));

   // chk: -1 for a field means "don't care"
   task chk(input [31:0] iw,
            input integer erd, input integer erdv,
            input integer ers1, input integer ers1v,
            input integer ers2, input integer ers2v,
            input integer ehimm, input [63:0] eimm, input integer elegal);
      begin
         insn = iw; #1; tid = tid + 1;
         if (elegal>=0 && legal!==elegal[0])         begin $display("T%0d(%08h) legal=%b exp %0d",tid,iw,legal,elegal); errs=errs+1; end
         if (erdv >=0 && rd_v !==erdv[0])             begin $display("T%0d(%08h) rd_v=%b exp %0d",tid,iw,rd_v,erdv); errs=errs+1; end
         if (erd  >=0 && rd   !==erd[5:0])            begin $display("T%0d(%08h) rd=%0d exp %0d",tid,iw,rd,erd); errs=errs+1; end
         if (ers1v>=0 && rs1_v!==ers1v[0])            begin $display("T%0d(%08h) rs1_v=%b exp %0d",tid,iw,rs1_v,ers1v); errs=errs+1; end
         if (ers1 >=0 && rs1  !==ers1[5:0])           begin $display("T%0d(%08h) rs1=%0d exp %0d",tid,iw,rs1,ers1); errs=errs+1; end
         if (ers2v>=0 && rs2_v!==ers2v[0])            begin $display("T%0d(%08h) rs2_v=%b exp %0d",tid,iw,rs2_v,ers2v); errs=errs+1; end
         if (ers2 >=0 && rs2  !==ers2[5:0])           begin $display("T%0d(%08h) rs2=%0d exp %0d",tid,iw,rs2,ers2); errs=errs+1; end
         if (ehimm>=0 && has_imm!==ehimm[0])          begin $display("T%0d(%08h) has_imm=%b exp %0d",tid,iw,has_imm,ehimm); errs=errs+1; end
         if (ehimm==1 && imm  !==eimm)                begin $display("T%0d(%08h) imm=%h exp %h",tid,iw,imm,eimm); errs=errs+1; end
      end
   endtask

   initial begin
      //   insn         rd rdv  rs1 r1v  rs2 r2v  himm imm                legal
      chk(32'h003100b3,  1, 1,   2, 1,    3, 1,    0, 64'd0,              1); // add x1,x2,x3
      chk(32'h00510093,  1, 1,   2, 1,   -1, 0,    1, 64'd5,              1); // addi x1,x2,5
      chk(32'h00000013,  0, 0,   0, 1,   -1, 0,    1, 64'd0,              1); // nop (addi x0,x0,0)
      chk(32'h00832283,  5, 1,   6, 1,   -1, 0,    1, 64'd8,              1); // lw x5,8(x6)
      chk(32'h00312623, -1, 0,   2, 1,    3, 1,    1, 64'd12,             1); // sw x3,12(x2)
      chk(32'h00208063, -1, 0,   1, 1,    2, 1,    1, 64'd0,              1); // beq x1,x2,0
      chk(32'h123452b7,  5, 1,  -1, 0,   -1, 0,    1, 64'h12345000,       1); // lui x5,0x12345
      chk(32'h000000ef,  1, 1,  -1, 0,   -1, 0,    1, 64'd0,              1); // jal x1,0
      chk(32'h004100e7,  1, 1,   2, 1,   -1, 0,    1, 64'd4,              1); // jalr x1,4(x2)
      chk(32'h00311093,  1, 1,   2, 1,   -1, 0,    1, 64'd3,              1); // slli x1,x2,3 (rs2 NOT a read)
      chk(32'h00208033,  0, 0,   1, 1,    2, 1,    0, 64'd0,              1); // add x0,x1,x2 (x0 dest)
      chk(32'h300312f3,  5, 1,   6, 1,   -1, 0,    0, 64'd0,              1); // csrrw x5,mstatus,x6 (no imm)
      chk(32'h3003d2f3,  5, 1,  -1, 0,   -1, 0,    1, 64'd7,              1); // csrrwi x5,mstatus,7
      chk(32'h00000073, -1, 0,  -1, 0,   -1, 0,    0, 64'd0,              1); // ecall (no GPRs)
      chk(32'h00000000, -1, 0,  -1, 0,   -1, 0,   -1, 64'd0,              0); // illegal (legal=0)

      if (errs == 0) $display("decode_operands: ALL TESTS PASSED (%0d)", tid);
      else           $display("decode_operands: %0d FAILURES / %0d", errs, tid);
      $finish;
   end
endmodule

`default_nettype wire
