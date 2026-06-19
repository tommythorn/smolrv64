// Cache metadata pack/unpack and index helper functions, shared between the
// smolrv64 core and smolrv64_frontend (which both maintain the VHPR L1 tags).
//
// `include this inside a module body (after smolrv64_defs.vh, which supplies
// the `CACHE_* geometry macros). The function bodies reference the parameters
// TLB_ASID_BITS, CACHE_PERM_BITS, VHPR_EPOCH_BITS, and TLB_CTX_BITS, so the
// including module must declare those in scope.

   function [2:0] cache_asid_color_mix;
      input [TLB_ASID_BITS-1:0] asid;
      begin
         cache_asid_color_mix = asid[2:0] ^ asid[5:3] ^ {2'd0, asid[6]} ^
                                {1'b0, asid[8:7]} ^ {2'd0, asid[9]};
      end
   endfunction

   function [`CACHE_INDEX_BITS-1:0] cache_way0_index;
      input [63:0] va;
      input [TLB_ASID_BITS-1:0] asid;
      begin
         cache_way0_index = {va[14:12] ^ cache_asid_color_mix(asid), va[11:6]};
      end
   endfunction

   function [`CACHE_INDEX_BITS-1:0] cache_way1_index;
      input [63:0] va;
      input [TLB_ASID_BITS-1:0] asid;
      begin
         cache_way1_index = {va[14:12] ^ cache_asid_color_mix(asid) ^
                             va[17:15] ^ va[23:21], va[11:6]};
      end
   endfunction

   function [`CACHE_VTAG_BITS-1:0] cache_vtag;
      input [63:0] va;
      begin
         cache_vtag = va[63:`CACHE_INDEX_BITS+`CACHE_LINE_OFFSET_BITS];
      end
   endfunction

   function [`CACHE_PHYS_TAG_BITS-1:0] cache_ptag;
      input [63:0] pa;
      begin
         cache_ptag = pa[`CACHE_PHYS_BITS-1:`CACHE_PAGE_OFFSET_BITS];
      end
   endfunction

   function cache_meta_valid;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_valid = meta[`CACHE_VALID_BIT];
      end
   endfunction

   function cache_meta_dirty;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_dirty = meta[`CACHE_DIRTY_BIT];
      end
   endfunction

   function [TLB_ASID_BITS-1:0] cache_meta_asid;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_asid = meta[`CACHE_ASID_LSB +: TLB_ASID_BITS];
      end
   endfunction

   function [`CACHE_VTAG_BITS-1:0] cache_meta_vtag;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_vtag = meta[`CACHE_VTAG_LSB +: `CACHE_VTAG_BITS];
      end
   endfunction

   function [`CACHE_PHYS_TAG_BITS-1:0] cache_meta_ptag;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_ptag = meta[`CACHE_PTAG_LSB +: `CACHE_PHYS_TAG_BITS];
      end
   endfunction

   function [CACHE_PERM_BITS-1:0] cache_meta_perm;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_perm = meta[`CACHE_PERM_LSB +: CACHE_PERM_BITS];
      end
   endfunction

   function [VHPR_EPOCH_BITS-1:0] cache_meta_epoch;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_epoch = meta[`CACHE_EPOCH_LSB +: VHPR_EPOCH_BITS];
      end
   endfunction

   function [`CACHE_META_BITS-1:0] cache_make_meta;
      input dirty;
      input valid;
      input [TLB_ASID_BITS-1:0] asid;
      input [CACHE_PERM_BITS-1:0] perm;
      input [`CACHE_VTAG_BITS-1:0] vtag;
      input [`CACHE_PHYS_TAG_BITS-1:0] ptag;
      input [VHPR_EPOCH_BITS-1:0] epoch;
      begin
         cache_make_meta = 0;
         cache_make_meta[`CACHE_DIRTY_BIT] = dirty;
         cache_make_meta[`CACHE_VALID_BIT] = valid;
         cache_make_meta[`CACHE_ASID_LSB +: TLB_ASID_BITS] = asid;
         cache_make_meta[`CACHE_PERM_LSB +: CACHE_PERM_BITS] = perm;
         cache_make_meta[`CACHE_VTAG_LSB +: `CACHE_VTAG_BITS] = vtag;
         cache_make_meta[`CACHE_PTAG_LSB +: `CACHE_PHYS_TAG_BITS] = ptag;
         cache_make_meta[`CACHE_EPOCH_LSB +: VHPR_EPOCH_BITS] = epoch;
      end
   endfunction

   function cache_perm_allows_ctx;
      input [CACHE_PERM_BITS-1:0] perm;
      input [TLB_CTX_BITS-1:0] ctx;
      reg [1:0] access;
      reg [1:0] access_prv;
      reg       access_sum;
      reg       access_mxr;
      reg       pte_r;
      reg       pte_w;
      reg       pte_x;
      reg       pte_u;
      reg       data_read_ok;
      reg       user_ok;
      begin
         access     = ctx[5:4];
         access_prv = ctx[3:2];
         access_sum = ctx[1];
         access_mxr = ctx[0];
         pte_r      = perm[0];
         pte_w      = perm[1];
         pte_x      = perm[2];
         pte_u      = perm[3];

         if (perm[4]) begin
            cache_perm_allows_ctx = 1'b1;
         end else begin
            data_read_ok = pte_r || (access_mxr && pte_x);
            user_ok = access_prv == 0 ? pte_u :
                      access_prv == 1 ? (!pte_u || (access != 2'd0 && access_sum)) :
                                        1'b1;
            case (access)
              2'd0: cache_perm_allows_ctx = pte_x && user_ok;
              2'd1: cache_perm_allows_ctx = data_read_ok && user_ok;
              2'd2: cache_perm_allows_ctx = pte_w && user_ok;
              default: cache_perm_allows_ctx = data_read_ok && pte_w && user_ok;
            endcase
         end
      end
   endfunction
