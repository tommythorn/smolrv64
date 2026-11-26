// smolrv64_sysop.vh -- the one definition of "a SYSTEM-opcode op that serialises".
//
// Everything in the SYSTEM opcode serialises (ecall, ebreak, xRET, wfi, sfence.vma, the CSR
// ops, the interrupt pseudo-op) except funct3 100, Zimop's may-be-operations (mop.r.N,
// mop.rr.N): those are plain ALU ops that write 0 to rd, and any other funct3-100 word is an
// illegal instruction, which serialises as one. It reads bits [14:0] only, so the aligner can
// apply it to an instruction's first halfword.
function automatic sysop(input [14:0] i);
   sysop = (i[6:0] == 7'b1110011) & (i[14:12] != 3'b100);
endfunction

// Zimop's may-be-operations under SYSTEM funct3 100: mop.r.N (insn[25:22] 0111) and mop.rr.N
// (insn[25] set); each writes 0 to rd.
function automatic zimop(input [31:0] i);
   zimop = (i[6:0] == 7'b1110011) & (i[14:12] == 3'b100) & i[31] & (i[29:28] == 2'b00)
         & (i[25] | (i[25:22] == 4'b0111));
endfunction
