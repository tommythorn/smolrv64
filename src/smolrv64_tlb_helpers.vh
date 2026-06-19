// TLB index hashing and SATP field helpers for the smolrv64 core. Pure
// combinational functions; `include inside the module body (after
// smolrv64_defs.vh supplies the `TLB_*_INDEX_BITS macros). The bodies use the
// TLB_CTX_BITS and TLB_SATP_KEY_BITS parameters, so the including module must
// have those in scope.

   function [`TLB_2M_INDEX_BITS-1:0] tlb_2m_index;
      input [63:0] va;
      input [ 1:0] access;
      input [ 1:0] idx_prv;
      input        idx_sum;
      input        idx_mxr;
      reg [TLB_CTX_BITS-1:0] ctx;
      begin
         ctx = {access, idx_prv, idx_sum, idx_mxr};
         tlb_2m_index = va[28:21] ^ va[36:29] ^ {6'd0, va[38:37]} ^
                        {2'd0, ctx};
      end
   endfunction

   function [`TLB_4K_INDEX_BITS-1:0] tlb_4k_index;
      input [63:0] va;
      input [ 1:0] access;
      input [ 1:0] idx_prv;
      input        idx_sum;
      input        idx_mxr;
      reg [TLB_CTX_BITS-1:0] ctx;
      begin
         ctx = {access, idx_prv, idx_sum, idx_mxr};
         tlb_4k_index = va[21:12] ^ {1'b0, va[30:22]} ^
                        {8'd0, va[38:37]} ^ {4'd0, ctx};
      end
   endfunction

   function [TLB_SATP_KEY_BITS-1:0] satp_tlb_key;
      input [63:0] satp;
      begin
         // TLB entries are keyed by ASID only.  Translation invalidation is
         // driven by SFENCE.VMA, not by the SATP CSR write itself.
         satp_tlb_key = satp[53:44];
      end
   endfunction

   function [63:0] satp_warl_value;
      input [63:0] satp;
      begin
         satp_warl_value = satp;
         // RV64 Sv39 permits up to 16 ASID bits; this core implements 10.
         satp_warl_value[59:54] = 6'd0;
      end
   endfunction
