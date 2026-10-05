// smolrv64_gclass -- an instruction's dispatch class, from its decoded fields.
//
// The ONE definition of what an instruction is to dispatch: which scheduler it goes to
// (its slot's lane, the ordered memory pipe, or the FP/MD/SYS pipe), whether it may go
// beside anything else (plain), and whether it redirects fetch at decode (an unpredicted
// jal, or a backward conditional the BTB missed, predicted not taken). The frontend
// evaluates it once per instruction at decode and the result rides in the decoupling
// queue's record: the queue head forms dispatch groups from it and dispatch routes by it,
// so the two can never disagree.
//
// `illegal` includes an FP op with mstatus.FS off. Sampled at decode, that is exact: a write
// that changes FS redirects and refetches every younger instruction (csr_file do_fschg).
`default_nettype none

module smolrv64_gclass
  #(parameter integer DCR_BACK = 1)            // a backward conditional the BTB missed redirects at decode
   (input  wire [31:0] insn,                   // the expanded instruction (0 for a fetch fault)
    input  wire        is_mem, is_store, is_amo, is_mul, is_fp, is_csr, is_serialize,
    input  wire        is_fencei, is_cbo, is_branch, is_jump, is_jalr,
    input  wire        illegal, fault,
    input  wire        mis_taken,              // taken would leave the predicted path
    input  wire        imm_neg,                // the branch offset is negative (a back-edge)
    input  wire        btb_hit,                // the BTB named this instruction's fetch block
    output wire [15:0] gc);                    // see the GC_* indices in smolrv64_core.v

   wire fp_valid;
   decode_fp u_dfp
     (.insn(insn), .fp_valid(fp_valid), .use_fpu(), .fp_class(),
      .op(), .op_mod(), .src_fmt(), .dst_fmt(), .int_fmt(),
      .rnd(), .op0_sel(), .op1_sel(), .op2_sel(), .op0_int(), .wr_fp());

   // the interrupt pseudo-op (SYSTEM, funct3 0, imm 0x7F0): M's m_is_irqop one stage earlier
   wire irqop = (insn[6:2] == 5'b11100) & (insn[14:12] == 3'b000) & (insn[31:20] == 12'h7F0)
              & ~illegal & ~fault;
   wire ord   = is_mem | is_amo | is_mul | is_fp | is_csr | is_serialize | is_fencei | is_cbo
              | is_branch | is_jump | is_jalr | illegal | fault | irqop;
   wire cls_f = fp_valid & ~is_mem & ~is_amo & ~illegal & ~fault & ~irqop;
   // control flow is the slot's lane; a fetch-faulted CTI or the irqop stays ordered, so M
   // keeps the single trap site
   wire cls_c = (is_branch | is_jump | is_jalr) & ~illegal & ~fault & ~irqop;
   // a multiply runs in its slot's lane (5.2c); a divide in the shared MD stage
   wire mul_l = is_mul & ~insn[14] & ~illegal & ~fault & ~irqop;
   wire cls_m = is_mul &  insn[14] & ~illegal & ~fault & ~irqop;
   // SYSTEM (the irqop included), FENCE and FENCE.I (MISC-MEM funct3 00x; CBO is 010), and an
   // instruction that traps at dispatch: each serialises and fires at the ROB head from the SYSQ
   wire cls_s = (insn[6:2] == 5'b11100) | ((insn[6:2] == 5'b00011) & (insn[14:13] == 2'b00))
              | illegal | fault;
   wire cls_l  = ord & ~cls_f & ~cls_c & ~cls_m & ~cls_s & ~mul_l;   // the ordered memory pipe (u_iq_l)
   wire cls_i  = ~ord | cls_c | mul_l;                       // ALU op, control flow or multiply: the slot's lane
   wire cls_fc = cls_f | cls_m | cls_s;                      // the FP/MD/SYS pipe (u_iq_f)
   // a serialising op, fence.i, CBO, AMO, CSR, a trap at dispatch or the irqop goes alone
   wire plain = ~(is_serialize | is_fencei | is_cbo | is_amo | is_csr | illegal | fault | irqop);
   wire dcr   = (is_jump & ~is_jalr & mis_taken)
              | ((DCR_BACK != 0) & is_branch & imm_neg & mis_taken & ~btb_hit);

   // the memory queue it allocates in (rule C1: a plain load or store, not an AMO or a CBO), and
   // how it serialises: a CSR op waits only for the CSR op before it; a trap at dispatch or any
   // other serialising op drains the machine first and is alone in flight
   wire ld_nb  = is_mem & ~is_store & ~is_amo & ~is_cbo;
   wire st_nb  = is_store & ~is_amo & ~is_cbo;
   wire csr_op = is_csr & ~illegal & ~fault;
   wire ser    = (is_serialize & ~csr_op) | illegal | fault;

   assign gc = {ser, csr_op, st_nb, ld_nb, dcr, fp_valid, ord, irqop, plain, cls_fc, cls_i, cls_l, cls_s, cls_m, cls_c, cls_f};
endmodule

`default_nettype wire
