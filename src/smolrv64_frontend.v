`include "smolrv64_defs.vh"

module smolrv64_frontend #(
   parameter EPOCH_BITS = 2,
   parameter TLB_ASID_BITS = 10,
   parameter TLB_CTX_BITS = 6,
   parameter CACHE_PERM_BITS = 5,
   parameter VHPR_EPOCH_BITS = 2
) (
   input  wire                  clock,
   input  wire                  reset,
   input  wire                  flush,
   input  wire                  fill,
   input  wire [63:0]           fill_pc,
   input  wire [ 1:0]           fill_prv,
   input  wire [TLB_ASID_BITS-1:0] fill_asid,
   input  wire [127:0]          fill_data,

   input  wire                  cmd_valid,
   input  wire [63:0]           cmd_pc,
   input  wire [ 1:0]           cmd_prv,
   input  wire [TLB_ASID_BITS-1:0] cmd_asid,
   input  wire [EPOCH_BITS-1:0] cmd_epoch,

   output wire                  rsp_hit,
   output wire [31:0]           rsp_insn,
   output wire [63:0]           rsp_next_pc,
   output wire [63:0]           rsp_predicted_next_pc,

   input  wire                         icache_invalidate_valid,
   input  wire                         icache_invalidate_way,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_invalidate_idx,
   input  wire                         icache_fill_begin,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_fill_begin_idx,
   input  wire                         icache_fill_begin_way,
   input  wire [TLB_ASID_BITS-1:0]     icache_fill_begin_asid,
   input  wire [CACHE_PERM_BITS-1:0]   icache_fill_begin_perm,
   input  wire [`CACHE_VTAG_BITS-1:0]  icache_fill_begin_vtag,
   input  wire [`CACHE_PHYS_TAG_BITS-1:0] icache_fill_begin_ptag,
   input  wire [VHPR_EPOCH_BITS-1:0]   icache_fill_begin_epoch,
   input  wire                         icache_fill_valid,
   input  wire [2:0]                   icache_fill_beat,
   input  wire [63:0]                  icache_fill_data,
   input  wire [63:0]                  icache_req_va,
   input  wire [63:0]                  icache_req_next_va,
   input  wire [TLB_ASID_BITS-1:0]     icache_req_asid,
   input  wire [`CACHE_VTAG_BITS-1:0]  icache_req_vtag,
   input  wire [`CACHE_VTAG_BITS-1:0]  icache_req_next_vtag,
   input  wire [TLB_CTX_BITS-1:0]      icache_req_ctx,
   input  wire [2:0]                   icache_req_bank,
   input  wire [2:0]                   icache_req_next_bank,
   input  wire                         icache_req_same_line,
   input  wire                         icache_replace_way,
   input  wire [VHPR_EPOCH_BITS-1:0]   icache_vhpr_epoch,
   input  wire                         icache_rsp_capture,
   output reg                          icache_rsp_hit = 0,
   output reg                          icache_rsp_next_valid = 0,
   output reg [127:0]                  icache_rsp_window = 0,
   output reg                          icache_rsp_insn_valid = 0,
   output reg [31:0]                   icache_rsp_insn = 0,
   output reg [63:0]                   icache_rsp_next_pc = 0,
   output wire                         icache_target_way,
   output wire [`CACHE_INDEX_BITS-1:0] icache_target_idx,
   output wire                         icache_target_valid
);
   reg          buf_valid = 0;
   reg  [63:0]  buf_base_va = 0;
   reg  [59:0]  buf_next_va_hi = 0;
   reg  [ 1:0]  buf_prv = 0;
   reg  [TLB_ASID_BITS-1:0] buf_asid = 0;
   reg  [127:0] buf_data = 0;
   reg [`CACHE_INDEX_BITS-1:0] icache_fill_idx = 0;
   reg                         icache_fill_way = 0;
   reg [TLB_ASID_BITS-1:0]     icache_fill_asid = 0;
   reg [CACHE_PERM_BITS-1:0]   icache_fill_perm = 0;
   reg [`CACHE_VTAG_BITS-1:0]  icache_fill_vtag = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] icache_fill_ptag = 0;
   reg [VHPR_EPOCH_BITS-1:0]   icache_fill_epoch = 0;
   wire [`CACHE_META_BITS-1:0] icache_way0_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] icache_way1_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] icache_way0_tag_next_rd_data;
   wire [`CACHE_META_BITS-1:0] icache_way1_tag_next_rd_data;
   wire                        icache_way0_tag_hit;
   wire                        icache_way1_tag_hit;
   wire                        icache_way0_next_tag_hit;
   wire                        icache_way1_next_tag_hit;
   wire                        icache_lookup_hit_way;
   wire                        icache_lookup_next_hit_way;
   wire                        icache_rsp_hit_comb;
   wire                        icache_rsp_next_valid_comb;
   wire [127:0]                icache_rsp_window_comb;
   wire                        icache_rsp_insn_valid_comb;
   wire [31:0]                 icache_rsp_insn_comb;
   wire [63:0]                 icache_rsp_next_pc_comb;

   function [31:0] pick_insn;
      input [127:0] data;
      input [3:0]   byte_offset;
      begin
         case (byte_offset[3:1])
           3'd0:    pick_insn = data[31:0];
           3'd1:    pick_insn = data[47:16];
           3'd2:    pick_insn = data[63:32];
           3'd3:    pick_insn = data[79:48];
           3'd4:    pick_insn = data[95:64];
           3'd5:    pick_insn = data[111:80];
           3'd6:    pick_insn = data[127:96];
           default: pick_insn = {16'd0, data[127:112]};
         endcase
      end
   endfunction

   function [63:0] fallthrough_pc;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         fallthrough_pc = pc + (fetch_insn[1:0] == 2'b11 ? 64'd4 : 64'd2);
      end
   endfunction

   function [63:0] jal_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         jal_target = pc + {{44{fetch_insn[31]}}, fetch_insn[19:12],
                            fetch_insn[20], fetch_insn[30:21], 1'b0};
      end
   endfunction

   function [63:0] branch_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         branch_target = pc + {{52{fetch_insn[31]}}, fetch_insn[7],
                               fetch_insn[30:25], fetch_insn[11:8], 1'b0};
      end
   endfunction

   function [63:0] c_j_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         c_j_target = pc + {{53{fetch_insn[12]}}, fetch_insn[8],
                            fetch_insn[10:9], fetch_insn[6], fetch_insn[7],
                            fetch_insn[2], fetch_insn[11],
                            fetch_insn[5:3], 1'b0};
      end
   endfunction

   function [63:0] c_branch_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         c_branch_target = pc + {{56{fetch_insn[12]}}, fetch_insn[6:5],
                                 fetch_insn[2], fetch_insn[11:10],
                                 fetch_insn[4:3], 1'b0};
      end
   endfunction

   function branch_is_backward;
      input [31:0] fetch_insn;
      begin
         branch_is_backward = fetch_insn[31];
      end
   endfunction

   function c_branch_is_backward;
      input [31:0] fetch_insn;
      begin
         c_branch_is_backward = fetch_insn[12];
      end
   endfunction

   function [63:0] predict_next_pc;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         if ((fetch_insn & 32'h0000007f) == 32'h0000006f) begin
            predict_next_pc = jal_target(pc, fetch_insn);
         end else if ((fetch_insn & 32'h0000007f) == 32'h00000063 &&
                      branch_is_backward(fetch_insn)) begin
            predict_next_pc = branch_target(pc, fetch_insn);
         end else if ((fetch_insn & 32'he003) == 32'ha001) begin
            predict_next_pc = c_j_target(pc, fetch_insn);
         end else if (((fetch_insn & 32'he003) == 32'hc001 ||
                       (fetch_insn & 32'he003) == 32'he001) &&
                      c_branch_is_backward(fetch_insn)) begin
            predict_next_pc = c_branch_target(pc, fetch_insn);
         end else begin
            predict_next_pc = fallthrough_pc(pc, fetch_insn);
         end
      end
   endfunction

   wire        context_hit = cmd_valid && buf_valid &&
                              buf_prv == cmd_prv && buf_asid == cmd_asid;
   wire        addr_same_hi = cmd_pc[63:4] == buf_base_va[63:4];
   wire        addr_next_hi = cmd_pc[63:4] == buf_next_va_hi;
   wire        rsp_addr_hit;
   wire        rsp_full_insn_hit;
   wire [ 3:0] rsp_offset;

   assign rsp_addr_hit = context_hit && !cmd_pc[0] &&
                     ((!buf_base_va[3] && addr_same_hi) ||
                      ( buf_base_va[3] &&
                        ((addr_same_hi &&  cmd_pc[3]) ||
                         (addr_next_hi && !cmd_pc[3]))));
   assign rsp_offset = buf_base_va[3] ?
                   (addr_same_hi ? {1'b0, cmd_pc[2:0]} :
                                   {1'b1, cmd_pc[2:0]}) :
                   cmd_pc[3:0];
   assign rsp_insn = pick_insn(buf_data, rsp_offset);
   assign rsp_full_insn_hit = rsp_insn[1:0] != 2'b11 || rsp_offset <= 4'd12;
   assign rsp_hit = rsp_addr_hit && rsp_full_insn_hit;
   assign rsp_next_pc = fallthrough_pc(cmd_pc, rsp_insn);
   assign rsp_predicted_next_pc = predict_next_pc(cmd_pc, rsp_insn);
   wire [63:0] fill_base_va = {fill_pc[63:3], 3'b000};
   wire        fill_page_ok = fill_base_va[11:0] <= 12'hff0;
   wire [63:0] icache_way0_bank_rd_data [0:7];
   wire [63:0] icache_way1_bank_rd_data [0:7];

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

   function cache_meta_valid;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_valid = meta[`CACHE_VALID_BIT];
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

   assign icache_way0_tag_hit =
      cache_meta_valid(icache_way0_tag_rd_data) &&
      cache_meta_epoch(icache_way0_tag_rd_data) == icache_vhpr_epoch &&
      cache_meta_asid(icache_way0_tag_rd_data) == icache_req_asid &&
      cache_meta_vtag(icache_way0_tag_rd_data) == icache_req_vtag &&
      cache_perm_allows_ctx(cache_meta_perm(icache_way0_tag_rd_data),
                            icache_req_ctx);
   assign icache_way1_tag_hit =
      cache_meta_valid(icache_way1_tag_rd_data) &&
      cache_meta_epoch(icache_way1_tag_rd_data) == icache_vhpr_epoch &&
      cache_meta_asid(icache_way1_tag_rd_data) == icache_req_asid &&
      cache_meta_vtag(icache_way1_tag_rd_data) == icache_req_vtag &&
      cache_perm_allows_ctx(cache_meta_perm(icache_way1_tag_rd_data),
                            icache_req_ctx);
   assign icache_way0_next_tag_hit =
      cache_meta_valid(icache_way0_tag_next_rd_data) &&
      cache_meta_epoch(icache_way0_tag_next_rd_data) == icache_vhpr_epoch &&
      cache_meta_asid(icache_way0_tag_next_rd_data) == icache_req_asid &&
      cache_meta_vtag(icache_way0_tag_next_rd_data) == icache_req_next_vtag &&
      cache_perm_allows_ctx(cache_meta_perm(icache_way0_tag_next_rd_data),
                            icache_req_ctx);
   assign icache_way1_next_tag_hit =
      cache_meta_valid(icache_way1_tag_next_rd_data) &&
      cache_meta_epoch(icache_way1_tag_next_rd_data) == icache_vhpr_epoch &&
      cache_meta_asid(icache_way1_tag_next_rd_data) == icache_req_asid &&
      cache_meta_vtag(icache_way1_tag_next_rd_data) == icache_req_next_vtag &&
      cache_perm_allows_ctx(cache_meta_perm(icache_way1_tag_next_rd_data),
                            icache_req_ctx);

   assign icache_rsp_hit_comb = icache_way0_tag_hit || icache_way1_tag_hit;
   assign icache_lookup_hit_way = icache_way1_tag_hit;
   assign icache_rsp_next_valid_comb =
      (icache_req_same_line || icache_way0_next_tag_hit || icache_way1_next_tag_hit) &&
      (icache_req_same_line || icache_req_va[11:3] != 9'h1ff);
   assign icache_lookup_next_hit_way = icache_way1_next_tag_hit;

   assign icache_target_way =
      !cache_meta_valid(icache_way0_tag_rd_data) ? 1'b0 :
      !cache_meta_valid(icache_way1_tag_rd_data) ? 1'b1 :
      cache_meta_epoch(icache_way0_tag_rd_data) != icache_vhpr_epoch ? 1'b0 :
      cache_meta_epoch(icache_way1_tag_rd_data) != icache_vhpr_epoch ? 1'b1 :
      icache_replace_way;
   wire [`CACHE_META_BITS-1:0] icache_target_meta =
      icache_target_way ? icache_way1_tag_rd_data : icache_way0_tag_rd_data;
   assign icache_target_idx =
      icache_target_way ? icache_way1_rd_idx : icache_way0_rd_idx;
   assign icache_target_valid = cache_meta_valid(icache_target_meta);

   function [63:0] select_icache_bank_data;
      input       way;
      input [2:0] bank;
      begin
         case (bank)
           3'd0: select_icache_bank_data = way ? icache_way1_bank_rd_data[0] : icache_way0_bank_rd_data[0];
           3'd1: select_icache_bank_data = way ? icache_way1_bank_rd_data[1] : icache_way0_bank_rd_data[1];
           3'd2: select_icache_bank_data = way ? icache_way1_bank_rd_data[2] : icache_way0_bank_rd_data[2];
           3'd3: select_icache_bank_data = way ? icache_way1_bank_rd_data[3] : icache_way0_bank_rd_data[3];
           3'd4: select_icache_bank_data = way ? icache_way1_bank_rd_data[4] : icache_way0_bank_rd_data[4];
           3'd5: select_icache_bank_data = way ? icache_way1_bank_rd_data[5] : icache_way0_bank_rd_data[5];
           3'd6: select_icache_bank_data = way ? icache_way1_bank_rd_data[6] : icache_way0_bank_rd_data[6];
           3'd7: select_icache_bank_data = way ? icache_way1_bank_rd_data[7] : icache_way0_bank_rd_data[7];
           default: select_icache_bank_data = 64'd0;
         endcase
      end
   endfunction

   wire [63:0] icache_rsp_data =
      select_icache_bank_data(icache_lookup_hit_way, icache_req_bank);
   wire [63:0] icache_rsp_next_data =
      select_icache_bank_data(icache_req_same_line ? icache_lookup_hit_way :
                                                    icache_lookup_next_hit_way,
                              icache_req_same_line ? icache_req_next_bank : 3'd0);
   assign icache_rsp_window_comb = {icache_rsp_next_data, icache_rsp_data};
   assign icache_rsp_insn_comb =
      pick_insn(icache_rsp_window_comb, {1'b0, icache_req_va[2:0]});
   assign icache_rsp_insn_valid_comb =
      icache_req_va[2:1] != 2'b11 || icache_rsp_insn_comb[1:0] != 2'b11;
   assign icache_rsp_next_pc_comb =
      fallthrough_pc(icache_req_va, icache_rsp_insn_comb);

   wire        icache_fill_finish = icache_fill_valid && icache_fill_beat == 3'd7;
   wire        icache_tag_wr_en = icache_invalidate_valid || icache_fill_finish;
   wire        icache_tag_wr_way = icache_invalidate_valid ? icache_invalidate_way :
                                                            icache_fill_way;
   wire [`CACHE_INDEX_BITS-1:0] icache_tag_wr_idx =
      icache_invalidate_valid ? icache_invalidate_idx : icache_fill_idx;
   wire [`CACHE_META_BITS-1:0] icache_tag_wr_data =
      icache_invalidate_valid ? {`CACHE_META_BITS{1'b0}} :
         cache_make_meta(1'b0, 1'b1,
                         icache_fill_asid,
                         icache_fill_perm,
                         icache_fill_vtag,
                         icache_fill_ptag,
                         icache_fill_epoch);
   wire [7:0] icache_bank_wr_en =
      icache_fill_valid ? (8'd1 << icache_fill_beat) : 8'd0;

   wire [`CACHE_INDEX_BITS-1:0] icache_way0_rd_idx =
      cache_way0_index(icache_req_va, icache_req_asid);
   wire [`CACHE_INDEX_BITS-1:0] icache_way1_rd_idx =
      cache_way1_index(icache_req_va, icache_req_asid);
   wire [`CACHE_INDEX_BITS-1:0] icache_way0_next_rd_idx =
      cache_way0_index(icache_req_next_va, icache_req_asid);
   wire [`CACHE_INDEX_BITS-1:0] icache_way1_next_rd_idx =
      cache_way1_index(icache_req_next_va, icache_req_asid);
   wire icache_bank0_reads_next_line =
      icache_req_bank == 3'd7 && !icache_req_same_line;
   wire [`CACHE_INDEX_BITS-1:0] icache_way0_bank0_rd_idx =
      icache_bank0_reads_next_line ? icache_way0_next_rd_idx : icache_way0_rd_idx;
   wire [`CACHE_INDEX_BITS-1:0] icache_way1_bank0_rd_idx =
      icache_bank0_reads_next_line ? icache_way1_next_rd_idx : icache_way1_rd_idx;

   always @(posedge clock) begin
      if (icache_rsp_capture) begin
         icache_rsp_hit <= icache_rsp_hit_comb;
         icache_rsp_next_valid <= icache_rsp_next_valid_comb;
         icache_rsp_window <= icache_rsp_window_comb;
         icache_rsp_insn_valid <= icache_rsp_insn_valid_comb;
         icache_rsp_insn <= icache_rsp_insn_comb;
         icache_rsp_next_pc <= icache_rsp_next_pc_comb;
      end

      if (reset) begin
         icache_fill_idx   <= 0;
         icache_fill_way   <= 0;
         icache_fill_asid  <= 0;
         icache_fill_perm  <= 0;
         icache_fill_vtag  <= 0;
         icache_fill_ptag  <= 0;
         icache_fill_epoch <= 0;
      end else if (icache_fill_begin) begin
         icache_fill_idx   <= icache_fill_begin_idx;
         icache_fill_way   <= icache_fill_begin_way;
         icache_fill_asid  <= icache_fill_begin_asid;
         icache_fill_perm  <= icache_fill_begin_perm;
         icache_fill_vtag  <= icache_fill_begin_vtag;
         icache_fill_ptag  <= icache_fill_begin_ptag;
         icache_fill_epoch <= icache_fill_begin_epoch;
      end

      if (reset || flush) begin
         buf_valid <= 1'b0;
      end else if (fill && fill_page_ok) begin
         buf_valid      <= 1'b1;
         buf_base_va    <= fill_base_va;
         buf_next_va_hi <= fill_base_va[63:4] + 60'd1;
         buf_prv        <= fill_prv;
         buf_asid       <= fill_asid;
         buf_data       <= fill_data;
      end
   end

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way0_tag_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way0_rd_idx ),
      .rd_data ( icache_way0_tag_rd_data ),
      .wr_en   ( icache_tag_wr_en && !icache_tag_wr_way ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way1_tag_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way1_rd_idx ),
      .rd_data ( icache_way1_tag_rd_data ),
      .wr_en   ( icache_tag_wr_en && icache_tag_wr_way ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way0_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way0_next_rd_idx ),
      .rd_data ( icache_way0_tag_next_rd_data ),
      .wr_en   ( icache_tag_wr_en && !icache_tag_wr_way ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way1_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way1_next_rd_idx ),
      .rd_data ( icache_way1_tag_next_rd_data ),
      .wr_en   ( icache_tag_wr_en && icache_tag_wr_way ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   genvar icache_bank_gen;
   generate
      for (icache_bank_gen = 0; icache_bank_gen < 8; icache_bank_gen = icache_bank_gen + 1) begin : frontend_icache_banks
         wire [63:0] way0_rd_data;
         wire [63:0] way1_rd_data;

         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) icache_way0_bank_ram (
            .clock   ( clock ),
            .rd_addr ( icache_bank_gen == 0 ? icache_way0_bank0_rd_idx : icache_way0_rd_idx ),
            .rd_data ( way0_rd_data ),
            .wr_en   ( icache_bank_wr_en[icache_bank_gen] && !icache_fill_way ),
            .wr_addr ( icache_fill_idx ),
            .wr_data ( icache_fill_data )
         );

         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) icache_way1_bank_ram (
            .clock   ( clock ),
            .rd_addr ( icache_bank_gen == 0 ? icache_way1_bank0_rd_idx : icache_way1_rd_idx ),
            .rd_data ( way1_rd_data ),
            .wr_en   ( icache_bank_wr_en[icache_bank_gen] && icache_fill_way ),
            .wr_addr ( icache_fill_idx ),
            .wr_data ( icache_fill_data )
         );

         assign icache_way0_bank_rd_data[icache_bank_gen] = way0_rd_data;
         assign icache_way1_bank_rd_data[icache_bank_gen] = way1_rd_data;
      end
   endgenerate
endmodule
