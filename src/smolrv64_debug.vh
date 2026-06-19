// Simulation-only debug/introspection helpers for the smolrv64 core:
// human-readable FSM/cache state names and the end-of-run state-time summary.
// `include inside the module body; these reference the core's state_stat_*
// counters and `S_* state codes, so they live with the core rather than as a
// standalone module.

   function [8*24-1:0] state_name;
      input [5:0] s;
      begin
         case (s)
           `S_FETCH1:                state_name = "FETCH1";
           `S_EXECUTE:               state_name = "EXECUTE";
           `S_EXCEPTION:             state_name = "EXCEPTION";
           `S_LOAD_ALIGN:            state_name = "LOAD_ALIGN";
           `S_MMIO_ALIGN:            state_name = "MMIO_ALIGN";
           `S_AMO:                   state_name = "AMO";
           `S_STORE:                 state_name = "STORE";
           `S_HANDLE_CSR:            state_name = "HANDLE_CSR";
           `S_MUL_RUNNING:           state_name = "MUL_RUNNING";
           `S_DIV_RUNNING:           state_name = "DIV_RUNNING";
           `S_PTW_LAUNCH:            state_name = "PTW_LAUNCH";
           `S_FETCH2_HALF:           state_name = "FETCH2_HALF";
           `S_IFETCH_WAIT:           state_name = "IFETCH_WAIT";
           `S_DMEM_LOAD_WAIT:        state_name = "DMEM_LOAD_WAIT";
           `S_PTW_DIRECT_WAIT:       state_name = "PTW_DIRECT_WAIT";
           `S_DMEM_STORE_WAIT:       state_name = "DMEM_STORE_WAIT";
           `S_IFETCH_HALF_WAIT:      state_name = "IFETCH_HALF_WAIT";
           `S_DMEM_LOAD2_WAIT:       state_name = "DMEM_LOAD2_WAIT";
           `S_DMEM_STORE2:           state_name = "DMEM_STORE2";
           `S_EXECUTE2:              state_name = "EXECUTE2";
           `S_PTW_PROCESS:           state_name = "PTW_PROCESS";
           `S_RF:                    state_name = "RF";
           `S_DMEM_STORE_RESP_WAIT:  state_name = "DMEM_STORE_RESP_WAIT";
           `S_DMEM_STORE_RESP_ARM:   state_name = "DMEM_STORE_RESP_ARM";
           `S_STORE_COMMIT:          state_name = "STORE_COMMIT";
           `S_CVFPU_ISSUE:           state_name = "CVFPU_ISSUE";
           `S_CVFPU_WAIT:            state_name = "CVFPU_WAIT";
           `S_TLB_LOOKUP:            state_name = "TLB_LOOKUP";
           `S_TLB_CHECK:             state_name = "TLB_CHECK";
           `S_CBO_EXEC:              state_name = "CBO_EXEC";
           `S_CBO_WAIT:              state_name = "CBO_WAIT";
           `S_IFETCH_RESP:           state_name = "IFETCH_RESP";
           `S_FETCH_BUF_CHECK:       state_name = "FETCH_BUF_CHECK";
           `S_FETCH_BUF_USE:         state_name = "FETCH_BUF_USE";
           `S_MULDIV_START:          state_name = "MULDIV_START";
           `S_FETCH_REQ:             state_name = "FETCH_REQ";
           `S_FRONTEND_MISS_WAIT:    state_name = "FRONTEND_MISS_WAIT";
           `S_INT_COMMIT:            state_name = "INT_COMMIT";
           `S_LOCAL_LOAD:            state_name = "LOCAL_LOAD";
           `S_TLB_INSERT:            state_name = "TLB_INSERT";
           default:                  state_name = "UNKNOWN";
         endcase
      end
   endfunction

   function [8*16-1:0] cache_state_name;
      input [4:0] s;
      begin
         case (s)
           CACHE_IDLE:      cache_state_name = "IDLE";
           CACHE_TAG_READ:  cache_state_name = "TAG_READ";
           CACHE_TAG_CHECK: cache_state_name = "TAG_CHECK";
           CACHE_FILL_REQ:  cache_state_name = "FILL_REQ";
           CACHE_FILL_WAIT: cache_state_name = "FILL_WAIT";
           CACHE_HIT_RESP:  cache_state_name = "HIT_RESP";
           CACHE_WB_REQ:    cache_state_name = "WB_REQ";
           CACHE_WB_WAIT:   cache_state_name = "WB_WAIT";
           CACHE_WB_PREP:   cache_state_name = "WB_PREP";
           CACHE_CBO_TAG_READ:  cache_state_name = "CBO_TAG_READ";
           CACHE_CBO_TAG_CHECK: cache_state_name = "CBO_TAG_CHECK";
           CACHE_CBO_RESP:      cache_state_name = "CBO_RESP";
           CACHE_BRAM_FILL_READ: cache_state_name = "BRAM_FILL_READ";
           CACHE_BRAM_WB_WRITE: cache_state_name = "BRAM_WB_WRITE";
           CACHE_BRAM_FILL_CAPTURE: cache_state_name = "BRAM_FILL_CAP";
           CACHE_BRAM_FILL_COMMIT: cache_state_name = "BRAM_FILL_COMMIT";
           CACHE_HIT_WRITE: cache_state_name = "HIT_WRITE";
           CACHE_TAG_WAIT: cache_state_name = "TAG_WAIT";
           CACHE_CBO_TAG_WAIT: cache_state_name = "CBO_TAG_WAIT";
           CACHE_WB_READ_WAIT: cache_state_name = "WB_READ_WAIT";
           CACHE_PROBE_READ: cache_state_name = "PROBE_READ";
           CACHE_PROBE_WAIT: cache_state_name = "PROBE_WAIT";
           CACHE_PROBE_CHECK: cache_state_name = "PROBE_CHECK";
           CACHE_INVALIDATE: cache_state_name = "INVALIDATE";
           CACHE_FLUSH_READ: cache_state_name = "FLUSH_READ";
           CACHE_FLUSH_WAIT: cache_state_name = "FLUSH_WAIT";
           CACHE_FLUSH_CHECK: cache_state_name = "FLUSH_CHECK";
           CACHE_FILL_LINE_WAIT: cache_state_name = "FILL_LINE_WAIT";
           CACHE_FILL_LINE_INSTALL: cache_state_name = "FILL_LINE_INST";
           default:         cache_state_name = "UNKNOWN";
         endcase
      end
   endfunction

   task dump_state_summary;
      integer summary_i;
      integer tlb_entries_total;
      reg [63:0] ptw_leaf_total;
      begin
         $display("%05d STATE SUMMARY cycles=%0d instret=%0d",
                  $time, state_stat_total_cycles, state_stat_instret);
         for (summary_i = 0; summary_i <= `S_LAST_STATE; summary_i = summary_i + 1) begin
            if (state_stat_cycles[summary_i] != 0) begin
               $display("%05d STATE %0d %-24s cycles=%0d pct_x100=%0d",
                        $time, summary_i, state_name(summary_i[5:0]), state_stat_cycles[summary_i],
                        state_stat_total_cycles == 0 ? 64'd0 :
                        (state_stat_cycles[summary_i] * 64'd10000) / state_stat_total_cycles);
            end
         end
         for (i = 0; i <= CACHE_LAST_STATE; i = i + 1) begin
            if (cache_state_stat_cycles[i] != 0) begin
               $display("%05d CACHE_STATE %0d %-16s cycles=%0d pct_x100=%0d",
                        $time, i, cache_state_name(i[4:0]), cache_state_stat_cycles[i],
                        state_stat_total_cycles == 0 ? 64'd0 :
                        (cache_state_stat_cycles[i] * 64'd10000) / state_stat_total_cycles);
            end
         end

         tlb_entries_total = tlb_stat_entries_4k +
                             tlb_stat_entries_2m +
                             tlb_stat_entries_1g;
         $display("%05d TLB_ENTRY_PAGE_SIZE name=4K entries=%0d pct_x100=%0d",
                  $time, tlb_stat_entries_4k,
                  tlb_entries_total == 0 ? 0 : (tlb_stat_entries_4k * 10000) / tlb_entries_total);
         $display("%05d TLB_ENTRY_PAGE_SIZE name=64K_NAPOT entries=0 pct_x100=0",
                  $time);
         $display("%05d TLB_ENTRY_PAGE_SIZE name=2M entries=%0d pct_x100=%0d",
                  $time, tlb_stat_entries_2m,
                  tlb_entries_total == 0 ? 0 : (tlb_stat_entries_2m * 10000) / tlb_entries_total);
         $display("%05d TLB_ENTRY_PAGE_SIZE name=1G entries=%0d pct_x100=%0d",
                  $time, tlb_stat_entries_1g,
                  tlb_entries_total == 0 ? 0 : (tlb_stat_entries_1g * 10000) / tlb_entries_total);
         $display("%05d TLB_ENTRY_PAGE_SIZE total=%0d capacity=%0d",
                  $time, tlb_entries_total, `TLB_ENTRIES);
         $display("%05d TLB_CAPACITY name=4K entries=%0d", $time, `TLB_4K_ENTRIES);
         $display("%05d TLB_CAPACITY name=2M entries=%0d", $time, `TLB_2M_ENTRIES);
         $display("%05d TLB_STATS lookups=%0d hits=%0d misses=%0d hit_pct_x100=%0d",
                  $time, tlb_stat_lookups, tlb_stat_hits, tlb_stat_misses,
                  tlb_stat_lookups == 0 ? 64'd0 :
                  (tlb_stat_hits * 64'd10000) / tlb_stat_lookups);
         $display("%05d TLB_STATS_4K hits=%0d inserts=%0d evicts=%0d",
                  $time, tlb_stat_hits_4k, tlb_stat_inserts_4k, tlb_stat_evicts_4k);
         $display("%05d TLB_STATS_2M hits=%0d inserts=%0d evicts=%0d",
                  $time, tlb_stat_hits_2m, tlb_stat_inserts_2m, tlb_stat_evicts_2m);
         $display("%05d TLB_UNCACHED page_1g=%0d napot=%0d",
                  $time, tlb_stat_uncached_1g, tlb_stat_uncached_napot);

         ptw_leaf_total = ptw_stat_leaf_4k + ptw_stat_leaf_64k_napot +
                          ptw_stat_leaf_2m + ptw_stat_leaf_1g;
         $display("%05d PTW_PAGE_SIZE name=4K walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_4k,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_4k * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE name=64K_NAPOT walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_64k_napot,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_64k_napot * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE name=2M walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_2m,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_2m * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE name=1G walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_1g,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_1g * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE total=%0d", $time, ptw_leaf_total);
      end
   endtask
