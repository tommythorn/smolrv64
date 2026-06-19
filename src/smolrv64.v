`timescale 1ns/10ps
`default_nettype none

`define insn_rd  [11: 7]
`define insn_rs1 [19:15]
`define insn_rs2 [24:20]
`define insn_csr [31:20]


`ifndef SMOLRV64_BUILD_STAMP
`define SMOLRV64_BUILD_STAMP 64'h0
`endif

// mimpid carries the leading bits of the HEAD commit the bitstream/sim was
// built from. build.tcl passes SMOLRV64_GIT_COMMIT directly; src/Makefile
// defines SMOLRV64_GITVH and generates git_commit.vh (a tracked prerequisite,
// so a new commit forces a relink and mimpid tracks HEAD). Anything else
// falls back to the historical date stamp.
`ifdef SMOLRV64_GITVH
 `include "git_commit.vh"
`endif
`ifndef SMOLRV64_GIT_COMMIT
`define SMOLRV64_GIT_COMMIT 32'h20260518
`endif

module smolrv64(input wire        clock,
                input wire        mem_clock,
                input wire        fpu_clock,
                input wire        reset,
/*
 The eventual memory bus

                output [`BUS_WIDTH-5:0] bus_address, // -1 - 4 for 128b
                output                  bus_read,
                output                  bus_write,
                output [127:0]          bus_writedata,
                output [ 15:0]          bus_byteenable, // Ignored unless bus_write
                input                   bus_waitrequest,
                input                   bus_readdatavalid,
                input  [127:0]          bus_readdata,
*/

// The MMIO bus is 32b only and always aligned access
                output reg [19:0] mmio_address,    // aligned byte addresses, aligned to 32b
                output reg        mmio_read = 0,
                output reg        mmio_write = 0,
                output reg [31:0] mmio_writedata,  // Invalid unless mmio_write
                output reg [ 3:0] mmio_byteenable, // Invalid unless mmio_write

                input wire        mmio_readdatavalid,
                input wire [31:0] mmio_readdata,

                input wire [63:1] ext_irq,         // External interrupt sources for PLIC

                // NS16550A UART (at 0x10000000)
                output reg        uart_tx_valid = 0, // Active for one cycle on THR write
                output reg [ 7:0] uart_tx_data,
                input wire        uart_tx_ready,      // TX holding register empty (from UART)
                input wire        uart_rx_valid,      // Pulse to enqueue a byte
                input wire [ 7:0] uart_rx_data,

                // AXI4 master to DDR4 (0x80000000-0xFFFFFFFF, 64-bit data).
                // Fixed: arsize/awsize=8B, arlen/awlen=0 (single-beat).  At most one
                // read and one write in flight.
                output wire [ 2:0] m_axi_awid,
                output wire [30:0] m_axi_awaddr,
                output wire [ 7:0] m_axi_awlen,
                output wire [ 2:0] m_axi_awsize,
                output wire [ 1:0] m_axi_awburst,
                output wire        m_axi_awlock,
                output wire [ 3:0] m_axi_awcache,
                output wire [ 2:0] m_axi_awprot,
                output wire [ 3:0] m_axi_awqos,
                output wire        m_axi_awvalid,
                input  wire        m_axi_awready,
                output wire [63:0] m_axi_wdata,
                output wire [ 7:0] m_axi_wstrb,
                output wire        m_axi_wlast,
                output wire        m_axi_wvalid,
                input  wire        m_axi_wready,
                input  wire [ 2:0] m_axi_bid,
                input  wire [ 1:0] m_axi_bresp,
                input  wire        m_axi_bvalid,
                output wire        m_axi_bready,
                output wire [ 2:0] m_axi_arid,
                output wire [30:0] m_axi_araddr,
                output wire [ 7:0] m_axi_arlen,
                output wire [ 2:0] m_axi_arsize,
                output wire [ 1:0] m_axi_arburst,
                output wire        m_axi_arlock,
                output wire [ 3:0] m_axi_arcache,
                output wire [ 2:0] m_axi_arprot,
                output wire [ 3:0] m_axi_arqos,
                output wire        m_axi_arvalid,
                input  wire        m_axi_arready,
                input  wire [ 2:0] m_axi_rid,
                input  wire [63:0] m_axi_rdata,
                input  wire [ 1:0] m_axi_rresp,
                input  wire        m_axi_rlast,
                input  wire        m_axi_rvalid,
                output wire        m_axi_rready,

                output reg        halted_o = 0);

// XXX Should I use param/localparam instead?
   `include "smolrv64_trap.vh"

// RISC-V CSR address map + CSR op codes.
`include "smolrv64_csr.vh"

// Smolrv64's state machine: Every instruction (unless an interrupt is
// pending) cycles through the first four: FETCH1, FETCH2, RF, and
// EXECUTE, and most return back to FETCH1.  The register writeback is
// overlapped with FETCH1 (a very modest concession to performance).
//
// All traps and interrupt go to EXCEPTION.  Loads go to LOAD_ALIGN,
// and possibly to MMIO_ALIGN.  AMOs go through
// LOAD_ALIGN, AMO, and STORE.
//
// CSR handling is factored out of EXECUTE into its own state, as are
// multiplication and divisions.
//
   `include "smolrv64_states.vh"

// execute_req_alu_op (EXOP_*) op codes live in smolrv64_defs.vh, shared with
// the extracted smolrv64_alu module.
`include "smolrv64_defs.vh"

// execute_req_mem_op: memory access class pre-decoded in S_RF, consumed in S_EXECUTE.
// Collapses the 22 per-insn load/store/AMO branches into one shared block
// (single mem_addr adder).

   `include "smolrv64_hpm.vh"


   reg [5:0]   state = `S_FETCH1; // XXX We should set this on reset
   reg [1:0]   f_state = `F_IDLE; // free-running frontend FSM; see F_* defines
   reg         ex_state = `EX_IDLE; // back-half EX FSM; see EX_* defines
   reg         f_consumed_hit;    // 1-cycle pulse: F_FETCH_BUF_USE took the hit
   // Set by retire_linear_fetch / retire_prepared_fetch / retire_redirect_fetch
   // (and other real retires) before transitioning to S_FETCH1. Gates retire
   // bookkeeping (csr_minstret, cosim, pc<=npc) so it only fires on real
   // retires, not the extra S_FETCH1 visits introduced by F_FETCH_BUF_USE
   // routing fetches through the queue.
   reg         retire_now_q = 0;
   reg         core_reset_pending = 0;
   wire        core_reset_home;
   wire        core_reset_now;

`ifndef MEM_BASEADDR
`define MEM_BASEADDR    64'h80000000  // override with -DMEM_BASEADDR=64'hXXXXXXXX
`endif
`ifndef MEM_SIZE_LG2
`define MEM_SIZE_LG2    15 // 32 KiB, override with -DMEM_SIZE_LG2=N
`endif
`define MEM_SIZE        (64'd1 << `MEM_SIZE_LG2)
   localparam [63:0] MEM_BASEADDR_VALUE = `MEM_BASEADDR;
   localparam TLB_CTX_BITS = 6;
   localparam TLB_ASID_BITS = 10;
   localparam CACHE_PERM_BITS = 6; // {uncacheable, physical, U, X, W, R} (uncacheable=Svpbmt NC/IO)
   localparam TLB_SATP_KEY_BITS = TLB_ASID_BITS;
   localparam FRONTEND_EPOCH_BITS = 2;
   localparam VHPR_EPOCH_BITS = 2;
   localparam TLB_2M_TAG_BITS = 18;
   localparam TLB_2M_PBASE_BITS = 43;
   localparam TLB_2M_DATA_BITS = TLB_2M_TAG_BITS + TLB_2M_PBASE_BITS +
                                 TLB_SATP_KEY_BITS + TLB_CTX_BITS +
                                 CACHE_PERM_BITS;
   localparam TLB_4K_TAG_BITS = 27;
   localparam TLB_4K_PBASE_BITS = 52;
   localparam TLB_4K_DATA_BITS = TLB_4K_TAG_BITS + TLB_4K_PBASE_BITS +
                                 TLB_SATP_KEY_BITS + TLB_CTX_BITS +
                                 CACHE_PERM_BITS;
   localparam TLB_CTX_LSB = 0;
   localparam TLB_PERM_LSB = TLB_CTX_LSB + TLB_CTX_BITS;
   localparam TLB_SATP_KEY_LSB = TLB_PERM_LSB + CACHE_PERM_BITS;
   localparam TLB_2M_PBASE_LSB = TLB_SATP_KEY_LSB + TLB_SATP_KEY_BITS;
   localparam TLB_2M_TAG_LSB = TLB_2M_PBASE_LSB + TLB_2M_PBASE_BITS;
   localparam TLB_4K_PBASE_LSB = TLB_SATP_KEY_LSB + TLB_SATP_KEY_BITS;
   localparam TLB_4K_TAG_LSB = TLB_4K_PBASE_LSB + TLB_4K_PBASE_BITS;
// Cache / TLB geometry macros moved to smolrv64_defs.vh (included at top),
// shared with the extracted smolrv64_frontend module.
`ifndef RESET_PC
`define RESET_PC        `MEM_BASEADDR  // override with -DRESET_PC=64'hXXXXXXXX
`endif
`ifndef SRAM_EVENHEX
`define SRAM_EVENHEX    "mem.even"
`endif
`ifndef SRAM_ODDHEX
`define SRAM_ODDHEX     "mem.odd"
`endif

   // To enable penalty-free unaligned access, memory is split into
   // even and odd 64b word addresses and striped across them.  Any
   // 64-bit word at address A will then be found in
   // {mem1[A/16],mem0[A/16]} if A/8 is even and
   // {mem0[A/16+1],mem1[A/16]} if A/8 is odd.
   (* ram_style = "block" *) reg  [63:0] mem0[0:`MEM_SIZE/16-1];
   (* ram_style = "block" *) reg  [63:0] mem1[0:`MEM_SIZE/16-1];

`ifdef SIMULATE
   reg [8*200:0] sram_evenhex = 0, sram_oddhex = 0;
`ifdef RISCV_TESTS
   reg [63:0] tohost_phys;
`endif
`endif
   // Forward declaration for init
   reg [ 2:0]  plic_priority [0:63];
   integer i;
   initial begin
`ifdef SIMULATE
      if ($value$plusargs("sram_even=%s", sram_evenhex)) begin
         if (!$value$plusargs("sram_odd=%s", sram_oddhex)) begin
            $display("ERROR: please specify the +sram_odd=<hexfile>");
            $finish;
         end
      end else if ($value$plusargs("sram_odd=%s", sram_oddhex)) begin
         $display("ERROR: please specify the +sram_even=<hexfile>");
         $finish;
      end
`ifdef RISCV_TESTS
      if (!$value$plusargs("tohost=%h", tohost_phys))
         tohost_phys = 64'h80001000;
`endif

       for (i = 0; i < `MEM_SIZE/16; i = i + 1) begin
          mem0[i] = 0;
          mem1[i] = 0;
       end
       for (i = 0; i < 64; i = i + 1) plic_priority[i] = 0;
       if (sram_evenhex != 0)
          $readmemh(sram_evenhex, mem0, 0, `MEM_SIZE/16-1);
       if (sram_oddhex != 0)
          $readmemh(sram_oddhex, mem1, 0, `MEM_SIZE/16-1);
`ifdef DEBUG_SRAM_INIT
       $display("SRAM_INIT even=%0s odd=%0s mem0[0]=%016x mem1[0]=%016x",
                sram_evenhex, sram_oddhex, mem0[0], mem1[0]);
`endif
`else
      $readmemh(`SRAM_EVENHEX, mem0, 0, `MEM_SIZE/16-1);
      $readmemh(`SRAM_ODDHEX,  mem1, 0, `MEM_SIZE/16-1);
`endif
   end


   reg  [63:0] mem_addr;
   reg  [63:0] mem_va;
   reg  [TLB_ASID_BITS-1:0] mem_asid;
   reg  [CACHE_PERM_BITS-1:0] mem_perm;
   reg  [TLB_CTX_BITS-1:0] mem_ctx;
   reg  [15:0] mem_wr_mask;
   // Load/store byte-steering helpers (merge_store_bytes, align_dmem_load_value).
   `include "smolrv64_mem_align.vh"

   /* RISC-V Architectural state: operating mode, pc, and registers*/
   reg  [63:0] pc = 0; // XXX We should set this on reset
   reg  [ 1:0] prv = 3; // XXX We should set this on reset

   // Read ports
   // rs3 is the dedicated FP third-source read address (insn[31:27]) for R4
   // FMADD/FMSUB/FNMSUB/FNMADD.  It has its own FP read port so the FMA no
   // longer repurposes rs1 — repurposing rs1 left the shared read-address
   // register pointing at rs3 and the *following* FP instruction could latch
   // f[rs3] instead of its own rs1 operand (a rare wrong-result hazard).
   reg  [ 4:0] rs1, rs2, rs3;
   wire [63:0] s1_bram;   // BRAM registered output; valid from start of S_RF onwards
   wire [63:0] s2_bram;
   (* max_fanout = 32 *) reg [63:0] execute_req_rs1_value = 0; // flip-flop copy of s1_bram; captured in S_RF, used in S_EXECUTE
   reg  [63:0] execute_req_rs2_value = 0;

   reg  [ 4:0] write_back_register = 0;
   reg  [63:0] write_back_value;
   // Parallel FP writeback path. Unlike the int path, there is no x0-style
   // hardwire: f0 is a real register, so a separate _valid bit gates writes.
   reg         write_back_fp_valid = 0;
   reg  [ 4:0] write_back_fp_register = 0;
   reg  [63:0] write_back_fp_value;
   wire [63:0] f1_bram;     // FP regfile read port 0 (addressed by rs1)
   wire [63:0] f2_bram;     // FP regfile read port 1 (addressed by rs2)
   wire [63:0] f3_bram;     // FP regfile read port 2 (addressed by rs3, FMA only)
   reg  [63:0] execute_req_frs1_value = 0;      // flip-flop copy of f1_bram; captured in S_RF
   reg  [63:0] execute_req_frs2_value = 0;
   reg  [63:0] execute_req_frs3_value = 0;      // flip-flop copy of f3_bram; captured in S_RF
   // Single-precision operand reads: if the f-reg isn't properly NaN-boxed,
   // the spec says single-precision ops see the canonical qNaN 0x7fc00000.
   wire [31:0] execute_req_frs1_s = (&execute_req_frs1_value[63:32]) ? execute_req_frs1_value[31:0] : 32'h7fc00000;
   wire [31:0] execute_req_frs2_s = (&execute_req_frs2_value[63:32]) ? execute_req_frs2_value[31:0] : 32'h7fc00000;
   reg  [63:0] exe_add   = 0;  // execute-stage intermediate (registered at S_EXECUTE→S_EXECUTE2)
   reg         exe_sext32 = 0; // 1 = sign-extend bit 31 of exe_add
   // Pre-decoded ALU control: computed in S_RF, consumed in S_EXECUTE case block.
   // Breaks the ~50-condition priority if-else chain critical path into two pipeline stages.
   reg  [ 3:0] execute_req_alu_op  = 0;  // EXOP_* operation code
   (* max_fanout = 32 *) reg [63:0] execute_req_alu_b = 0; // second operand
   reg         execute_req_alu_sxt = 0;  // 1 → W-type: operate on [31:0], sign-extend result

   // Pre-decoded mem access: computed in S_RF, consumed in S_EXECUTE shared block.
   // Collapses 22 load/store/AMO branches into one; shares a single execute_req_rs1_value+offset adder.
   reg  [ 2:0] execute_req_mem_op       = 0; // MEMOP_* class code (NONE/LOAD/STORE/LR/SC/AMO)
   reg  [63:0] execute_req_mem_offset   = 0; // byte offset added to execute_req_rs1_value to form mem_addr
   reg  [ 2:0] execute_req_load_size_lg2= 0; // size/sign for loads+LR+AMO (matches load_size_lg2)
   reg  [ 7:0] execute_req_mem_wr_mask  = 0; // byte-enable for stores+SC
   reg  [ 4:0] execute_req_mem_wb_reg   = 0; // destination register for loads/LR/SC/AMO (0 for stores)
   reg         execute_req_mem_fp       = 0; // 1 = FP load/store (route via f-regfile, NaN-box FLW)
   reg         execute_req_cbo_zero     = 0; // 1 = Zicboz cbo.zero: write whole line as zeros
   reg  [ 2:0] load_size_lg2; // [1:0] = size (0:B, 1:H, 2:W, 3:D), [2] = sign-extend

   // Execute request boundary. S_RF asserts this after registering operands
   // and predecode outputs; S_EXECUTE clears it when accepted.
   reg         execute_req_valid = 0;
   reg  [63:0] execute_req_pc = `RESET_PC;
   reg  [63:0] execute_req_next_pc = `RESET_PC;
   reg  [63:0] execute_req_predicted_pc = `RESET_PC;
   reg  [ 1:0] execute_req_prv = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] execute_req_epoch = 0;
   (* max_fanout = 16 *) reg [31:0] execute_req_insn = 0;
   reg  [ 4:0] execute_req_rd = 0;
   reg  [ 4:0] execute_req_rs1 = 0;
   reg  [ 4:0] execute_req_rs2 = 0;
   reg  [ 5:0] execute_req_shamt = 0;
   wire        ex_accept_ready = !execute_req_valid;
   wire [63:0] ex_pc = execute_req_pc;
   wire [63:0] ex_next_pc = execute_req_next_pc;
   wire [63:0] ex_predicted_pc = execute_req_predicted_pc;
   wire [ 1:0] ex_prv = execute_req_prv;
   wire [FRONTEND_EPOCH_BITS-1:0] ex_epoch = execute_req_epoch;
   wire [31:0] ex_insn = execute_req_insn;
   wire [ 4:0] ex_rd = execute_req_rd;
   wire [ 4:0] ex_rs1 = execute_req_rs1;
   wire [ 4:0] ex_rs2 = execute_req_rs2;
   wire [ 5:0] ex_shamt = execute_req_shamt;
   reg         execute_res_valid = 0;
`ifdef SIMULATE
   reg         execute_req_valid_q = 0;
   reg  [63:0] execute_req_pc_q = `RESET_PC;
   reg  [63:0] execute_req_next_pc_q = `RESET_PC;
   reg  [63:0] execute_req_predicted_pc_q = `RESET_PC;
   reg  [ 1:0] execute_req_prv_q = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] execute_req_epoch_q = 0;
   reg  [31:0] execute_req_insn_q = 0;
   reg  [ 4:0] execute_req_rd_q = 0;
   reg  [ 4:0] execute_req_rs1_q = 0;
   reg  [ 4:0] execute_req_rs2_q = 0;
   reg  [ 5:0] execute_req_shamt_q = 0;
   reg  [63:0] execute_req_rs1_value_q = 0;
   reg  [63:0] execute_req_rs2_value_q = 0;
   reg  [63:0] execute_req_frs1_value_q = 0;
   reg  [63:0] execute_req_frs2_value_q = 0;
   reg  [63:0] execute_req_frs3_value_q = 0;
   reg  [ 3:0] execute_req_alu_op_q = 0;
   reg  [63:0] execute_req_alu_b_q = 0;
   reg         execute_req_alu_sxt_q = 0;
   reg  [ 2:0] execute_req_mem_op_q = 0;
   reg  [63:0] execute_req_mem_offset_q = 0;
   reg  [ 2:0] execute_req_load_size_lg2_q = 0;
   reg  [ 7:0] execute_req_mem_wr_mask_q = 0;
   reg  [ 4:0] execute_req_mem_wb_reg_q = 0;
   reg         execute_req_mem_fp_q = 0;
`endif

   // FP load completions latch their boxed result into write_back_fp_value
   // before retire. Keeping writeback data independent of execute_req_mem_fp lets an
   // early-launched next instruction predecode without changing the bypassed
   // value from the retiring FP load.
   wire [63:0] fp_writeback_data = write_back_fp_value;

   // Pre-registered interrupt check: computed every cycle, consumed in S_FETCH1.
   // Breaks the mtip_reg → cause_priority_encode → state_reg path (~10 LUT levels)
   // into two shorter stages (~4 LUT levels each).
   reg         pre_intr_pending = 0;  // 1 = interrupt pending (checked in S_FETCH1)
   reg  [11:0] pre_intr_cause   = 0;  // encoded interrupt cause number

   regfile rf_inst(.clock(clock),
                   .write_valid(state == `S_FETCH1 && write_back_register != 0),
                   .write_data(write_back_value),
                   .write_addr(write_back_register),
                   .read_addr_0(rs1),
                   .read_addr_1(rs2),
                   .read_data_0(s1_bram),
                   .read_data_1(s2_bram));

   fregfile f_rf_inst(.clock(clock),
                      .write_valid(state == `S_FETCH1 && write_back_fp_valid),
                      .write_data(fp_writeback_data),
                      .write_addr(write_back_fp_register),
                      .read_addr_0(rs1),
                      .read_addr_1(rs2),
                      .read_addr_2(rs3),
                      .read_data_0(f1_bram),
                      .read_data_1(f2_bram),
                      .read_data_2(f3_bram));

   // Combinational integer ALU. Its registered result lands in exe_add at the
   // S_EXECUTE -> S_EXECUTE2 boundary (see execute_stage below).
   wire [63:0] alu_result;
   smolrv64_alu alu_inst(.op  (execute_req_alu_op),
                         .a   (execute_req_rs1_value),
                         .b   (execute_req_alu_b),
                         .sxt (execute_req_alu_sxt),
                         .result(alu_result));

   reg         cvfpu_in_valid = 0;
   reg  [2:0][63:0] cvfpu_operands = '0;
   reg  [ 2:0] cvfpu_rnd_mode = 0;
   reg  [ 2:0] pre_fp_rnd_mode = 0;
   reg         pre_fp_rmode_ok = 1'b1;
   reg  [ 3:0] cvfpu_op = 0;
   reg         cvfpu_op_mod = 0;
   reg  [ 2:0] cvfpu_src_fmt = 0;
   reg  [ 2:0] cvfpu_dst_fmt = 0;
   reg  [ 1:0] cvfpu_int_fmt = 0;
   reg  [ 7:0] cvfpu_tag_in = 0;
   reg         cvfpu_write_fp = 1'b1;
   wire        cvfpu_in_ready;
   wire [63:0] cvfpu_result;
   wire [ 4:0] cvfpu_fflags;
   wire [ 7:0] cvfpu_tag_out;
   wire        cvfpu_out_valid;
   wire        cvfpu_busy;

   smolrv64_cvfpu cvfpu_inst(
      .clock     ( clock ),
      .fpu_clock ( fpu_clock ),
      .reset     ( core_reset_now ),
      .in_valid  ( cvfpu_in_valid ),
      .in_ready  ( cvfpu_in_ready ),
      .operands  ( cvfpu_operands ),
      .rnd_mode  ( cvfpu_rnd_mode ),
      .op        ( cvfpu_op ),
      .op_mod    ( cvfpu_op_mod ),
      .src_fmt   ( cvfpu_src_fmt ),
      .dst_fmt   ( cvfpu_dst_fmt ),
      .int_fmt   ( cvfpu_int_fmt ),
      .tag_in    ( cvfpu_tag_in ),
      .result    ( cvfpu_result ),
      .fflags    ( cvfpu_fflags ),
      .tag_out   ( cvfpu_tag_out ),
      .out_valid ( cvfpu_out_valid ),
      .out_ready ( 1'b1 ),
      .flush     ( 1'b0 ),
      .busy      ( cvfpu_busy )
   );


   (* max_fanout = 32 *) reg [63:0] npc = `RESET_PC; // XXX We should set this on reset
   reg  [63:0] pre_npc = `RESET_PC;
   reg  [63:0] pre_jalr_target = `RESET_PC;
   reg  [63:0] pre_branch_target = `RESET_PC;
   reg         pre_branch_taken = 0;

   // Pending L1 cache operation signalling.
   // cache_issue_dw_addr is the 8B-aligned doubleword address (= phys[30:3]).
   reg  [27:0]  cache_issue_dw_addr;
   reg          ifetch_read = 0;
   reg          dmem_read = 0;
   reg          dmem_write = 0;
   reg          dmem_write_zero = 0;  // companion to dmem_write: 1 = cbo.zero whole-line zero
   reg  [63:0]  dmem_write_data;
   reg  [ 7:0]  dmem_write_strb;     // 1 = write byte
   wire         dmem_rsp_valid;
   wire [63:0]  dmem_rsp_data;
   wire [63:0]  dmem_rsp_next_data;
   wire         dmem_rsp_next_valid;
   wire         dmem_write_ready;    // L1 cache is ready to accept a store
   wire         dmem_write_done;     // store hit or refill/install completed

   reg  [63:0]  load_latched_data;        // holds first 8B chunk across states
   reg  [127:0] ifetch_latched_window = 0;
   reg  [ 63:0] ifetch_latched_half_data = 0;
   reg          ifetch_latched_next_valid = 0;
   reg          ifetch_latched_insn_valid = 0;
   reg  [31:0]  ifetch_latched_insn = 0;
   reg  [63:0]  ifetch_latched_next_pc = 0;
   reg          fetch_from_ifetch_rsp; // current fetch came from I-fetch response
   reg  [27:0]  dmem_store2_dw_addr; // 8B-doubleword addr for split-store second beat
   reg  [63:0]  dmem_store2_va;      // virtual address for split-store second beat
   reg  [TLB_ASID_BITS-1:0] dmem_store2_asid;
   reg  [CACHE_PERM_BITS-1:0] dmem_store2_perm;
   reg  [TLB_CTX_BITS-1:0] dmem_store2_ctx;
   reg  [63:0]  dmem_store2_data;    // overflow bytes for split store
   reg  [ 7:0]  dmem_store2_strb;    // byte write mask for split-store second beat
   reg          dmem_store_split;    // 1 = second beat pending after S_DMEM_STORE_WAIT
   reg          ptw_direct_read = 0; // physical PTW read bypasses VHPR L1
   reg  [27:0]  ptw_direct_addr = 0;
   reg          ptw_direct_probe_pending = 0;
   reg          ptw_direct_wait_probe = 0;
   reg          ptw_direct_pending = 0;
   reg          ptw_direct_wait_bram = 0;
   reg          ptw_direct_wait_axi = 0;
   reg [`MEM_SIZE_LG2-5:0] ptw_direct_bram_word_idx = 0;
   reg          ptw_direct_bram_word_bank = 0;
   reg          ptw_direct_rsp_valid_r = 0;
   reg  [63:0]  ptw_direct_rsp_data_r = 0;

   // Frontend instruction fetch window.  The old global FSM still launches
   // TLB/cache slow paths; the frontend module owns instruction alignment,
   // the small fetch window, and the registered hit result.
   reg          frontend_buf_flush = 0;
   reg          frontend_buf_fill = 0;
   reg  [63:0]  frontend_buf_fill_pc = `RESET_PC;
   reg  [ 1:0]  frontend_buf_fill_prv = 0;
   reg  [TLB_ASID_BITS-1:0] frontend_buf_fill_asid = 0;
   reg  [127:0] frontend_buf_fill_data = 0;
   wire         frontend_rsp_hit;
   wire [31:0]  frontend_rsp_insn;
   // Free-running frontend's fetch-buffer latch. Written by case(f_state)
   // when f_state == F_FETCH_BUF_CHECK; read by backend's S_FETCH_BUF_USE.
   // Replaces the old backend-owned fetch_buf_latched_* set.
   //
   // The cmd_pc/prv/epoch are also latched here so F_FETCH_BUF_USE consumes
   // a self-consistent (pc, insn) pair: backend can update frontend_cmd_*
   // between F_FETCH_BUF_CHECK and F_FETCH_BUF_USE, and without latching we
   // would feed latch_frontend_decode_pending current frontend_cmd_pc + stale
   // f_latched_insn (different PCs) — a hang.
   reg          f_latched_hit = 0;
   reg  [31:0]  f_latched_insn = 0;
   reg  [63:0]  f_latched_decode_next_pc = `RESET_PC;
   reg  [63:0]  f_latched_next_pc = `RESET_PC;
   reg  [63:0]  f_latched_cmd_pc = `RESET_PC;
   reg  [ 1:0]  f_latched_cmd_prv = 3;
   reg  [TLB_ASID_BITS-1:0] f_latched_cmd_asid = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] f_latched_cmd_epoch = 0;
   wire [63:0]  frontend_rsp_next_pc;
   wire [63:0]  frontend_rsp_predicted_next_pc;
   wire         icache_rsp_hit;
   wire         icache_rsp_next_valid;
   wire [127:0] icache_rsp_window;
   wire         icache_rsp_insn_valid;
   wire [31:0]  icache_rsp_insn;
   wire [63:0]  icache_rsp_next_pc;
   wire         icache_target_way;
   wire [`CACHE_INDEX_BITS-1:0] icache_target_idx;
   wire         icache_target_valid;

   // Frontend command/result boundary.  The backend writes frontend_cmd_* when
   // it wants the frontend to probe or restart at a PC/context; the frontend
   // returns frontend_rsp_* for the registered command while the old FSM still
   // owns slow-path translation and refill sequencing.
   reg          frontend_cmd_valid = 0;
   reg  [63:0]  frontend_cmd_pc = `RESET_PC;
   reg  [ 1:0]  frontend_cmd_prv = 3;
   reg  [TLB_ASID_BITS-1:0] frontend_cmd_asid = 0;
   reg          frontend_cmd_fast_ready = 0;
   reg          frontend_cmd_speculative = 0;
   reg          frontend_cmd_spec_miss_ready = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_cmd_epoch = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] fetch_epoch = 0;
   reg          frontend_redirect_valid = 0;
   reg  [63:0] frontend_redirect_pc = `RESET_PC;
   reg  [ 1:0] frontend_redirect_prv = 3;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_redirect_epoch = 0;
   // Speculative frontend cache miss.  The single global FSM still owns TLB
   // and ordinary fetch misses; this side buffer only overlaps physical
   // cacheable misses with long non-memory backend states.
   reg          frontend_miss_valid = 0;
   reg          frontend_miss_done = 0;
   reg          frontend_flush_this_cycle = 0;
   reg  [63:0]  frontend_miss_pc = `RESET_PC;
   reg  [ 1:0]  frontend_miss_prv = 3;
   reg  [TLB_ASID_BITS-1:0] frontend_miss_asid = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_miss_epoch = 0;
   reg  [127:0] frontend_miss_window = 0;
   reg          frontend_miss_next_valid = 0;
   reg          frontend_miss_insn_valid = 0;
   reg  [31:0]  frontend_miss_insn = 0;
   localparam [1:0] FRONTEND_MISS_WAIT_CONSUME  = 2'd0,
                    FRONTEND_MISS_WAIT_EXCEPTION = 2'd2;
   reg  [ 1:0]  frontend_miss_wait_action = FRONTEND_MISS_WAIT_CONSUME;

   // Register/decode request boundary. The frontend can now run a little
   // ahead of retirement on fetch-window hits; retirement consumes the oldest
   // matching entry and launches the BRAM register-file read.
   localparam RF_DECODE_QUEUE_BITS = 3;
   localparam RF_DECODE_QUEUE_DEPTH = 1 << RF_DECODE_QUEUE_BITS;
   localparam [RF_DECODE_QUEUE_BITS:0] RF_DECODE_QUEUE_DEPTH_COUNT =
      (1 << RF_DECODE_QUEUE_BITS);
   reg  [RF_DECODE_QUEUE_BITS-1:0] rf_decode_head = 0;
   reg  [RF_DECODE_QUEUE_BITS-1:0] rf_decode_tail = 0;
   reg  [RF_DECODE_QUEUE_BITS:0]   rf_decode_count = 0;
   reg  [63:0]  rf_decode_pc_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [63:0]  rf_decode_next_pc_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [63:0]  rf_decode_predicted_pc_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [31:0]  rf_decode_insn_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 1:0]  rf_decode_prv_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [FRONTEND_EPOCH_BITS-1:0] rf_decode_epoch_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg          rf_decode_from_ifetch_rsp_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 4:0]  rf_decode_rd_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 4:0]  rf_decode_rs1_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 4:0]  rf_decode_rs2_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 5:0]  rf_decode_shamt_q [0:RF_DECODE_QUEUE_DEPTH-1];
   wire         rf_decode_valid = rf_decode_count != 0;
   wire         rf_decode_full = rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT;
   wire [63:0]  rf_decode_pc = rf_decode_pc_q[rf_decode_head];
   wire [63:0]  rf_decode_next_pc = rf_decode_next_pc_q[rf_decode_head];
   wire [63:0]  rf_decode_predicted_pc = rf_decode_predicted_pc_q[rf_decode_head];
   wire [31:0]  rf_decode_insn = rf_decode_insn_q[rf_decode_head];
   wire [ 1:0]  rf_decode_prv = rf_decode_prv_q[rf_decode_head];
   wire [FRONTEND_EPOCH_BITS-1:0] rf_decode_epoch = rf_decode_epoch_q[rf_decode_head];
   wire         rf_decode_from_ifetch_rsp = rf_decode_from_ifetch_rsp_q[rf_decode_head];
   wire [ 4:0]  rf_decode_rd = rf_decode_rd_q[rf_decode_head];
   wire [ 4:0]  rf_decode_rs1 = rf_decode_rs1_q[rf_decode_head];
   wire [ 4:0]  rf_decode_rs2 = rf_decode_rs2_q[rf_decode_head];
   wire [ 5:0]  rf_decode_shamt = rf_decode_shamt_q[rf_decode_head];
   reg          rf_decode_pop_this_cycle = 0;
   reg          rf_decode_enqueue_this_cycle = 0;
   reg          rf_decode_prearm_block = 0;
   reg          rf_decode_prearmed = 0;
   reg  [RF_DECODE_QUEUE_BITS-1:0] rf_decode_prearmed_head = 0;
   wire         rf_decode_prearmed_current =
      rf_decode_prearmed && rf_decode_prearmed_head == rf_decode_head;
   reg          frontend_decode_pending_valid = 0;
   reg          frontend_decode_pending_drain = 0;
   reg          frontend_decode_pending_latch_this_cycle = 0;
   reg  [63:0]  frontend_decode_pending_pc = `RESET_PC;
   reg  [63:0]  frontend_decode_pending_next_pc = `RESET_PC;
   reg  [63:0]  frontend_decode_pending_predicted_pc = `RESET_PC;
   reg  [31:0]  frontend_decode_pending_insn = 0;
   reg  [ 1:0]  frontend_decode_pending_prv = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_decode_pending_epoch = 0;

   // ID/RF stage boundary. Dispatch accepts one decoded instruction into this
   // payload and launches the BRAM read; S_RF consumes it into EX.
   reg          id_valid = 0;
   reg          id_rf_ready = 0;
   reg  [63:0]  id_pc = `RESET_PC;
   reg  [63:0]  id_next_pc = `RESET_PC;
   reg  [63:0]  id_predicted_pc = `RESET_PC;
   reg  [ 1:0]  id_prv = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] id_epoch = 0;
   reg  [31:0]  id_insn = 0;
   reg  [ 4:0]  id_rd = 0;
   reg  [ 4:0]  id_rs1 = 0;
   reg  [ 4:0]  id_rs2 = 0;
   reg  [ 5:0]  id_shamt = 0;
   wire         id_ex_fire = id_valid && id_rf_ready && ex_accept_ready;
`ifdef SIMULATE
   reg          rf_decode_valid_q = 0;
   reg  [RF_DECODE_QUEUE_BITS-1:0] rf_decode_head_q = 0;
   reg  [63:0] rf_decode_pc_q_assert = `RESET_PC;
   reg  [63:0] rf_decode_next_pc_q_assert = `RESET_PC;
   reg  [63:0] rf_decode_predicted_pc_q_assert = `RESET_PC;
   reg  [31:0] rf_decode_insn_q_assert = 0;
   reg  [ 1:0] rf_decode_prv_q_assert = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] rf_decode_epoch_q_assert = 0;
   reg          rf_decode_from_ifetch_rsp_q_assert = 0;
   reg  [ 4:0] rf_decode_rd_q_assert = 0;
   reg  [ 4:0] rf_decode_rs1_q_assert = 0;
   reg  [ 4:0] rf_decode_rs2_q_assert = 0;
   reg  [ 5:0] rf_decode_shamt_q_assert = 0;
   reg          id_valid_q = 0;
   reg  [63:0] id_pc_q = `RESET_PC;
   reg  [63:0] id_next_pc_q = `RESET_PC;
   reg  [63:0] id_predicted_pc_q = `RESET_PC;
   reg  [ 1:0] id_prv_q = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] id_epoch_q = 0;
   reg  [31:0] id_insn_q = 0;
   reg  [ 4:0] id_rd_q = 0;
   reg  [ 4:0] id_rs1_q = 0;
   reg  [ 4:0] id_rs2_q = 0;
   reg  [ 5:0] id_shamt_q = 0;
`endif
   wire [63:0]  id_rf_pc = id_pc;
   wire [63:0]  id_rf_next_pc = id_next_pc;
   wire [63:0]  id_rf_predicted_pc = id_predicted_pc;
   wire [ 1:0]  id_rf_prv = id_prv;
   wire [FRONTEND_EPOCH_BITS-1:0] id_rf_epoch = id_epoch;
   wire [31:0]  id_rf_insn = id_insn;
   wire [ 4:0]  id_rf_rd = id_rd;
   wire [ 4:0]  id_rf_rs1 = id_rs1;
   wire [ 4:0]  id_rf_rs2 = id_rs2;
   wire [ 5:0]  id_rf_shamt = id_shamt;
   wire [63:0]  id_rf_rs1_value =
      (write_back_register != 0 && id_rf_rs1 == write_back_register) ?
      write_back_value : s1_bram;
   wire [63:0]  id_rf_rs2_value =
      (write_back_register != 0 && id_rf_rs2 == write_back_register) ?
      write_back_value : s2_bram;
   wire [63:0]  id_rf_frs1_value =
      (write_back_fp_valid && id_rf_rs1 == write_back_fp_register) ?
      fp_writeback_data : f1_bram;
   wire [63:0]  id_rf_frs2_value =
      (write_back_fp_valid && id_rf_rs2 == write_back_fp_register) ?
      fp_writeback_data : f2_bram;
   // FP third source (R4 FMA): insn[31:27], same writeback-bypass as frs1/frs2.
   wire [ 4:0]  id_rf_rs3 = id_rf_insn[31:27];
   wire [63:0]  id_rf_frs3_value =
      (write_back_fp_valid && id_rf_rs3 == write_back_fp_register) ?
      fp_writeback_data : f3_bram;

   // VHPR write-back L1 for BRAM/DRAM.  The hit lookup is virtual
   // (ASID + virtual tag) while miss, writeback, and CBO reconciliation use
   // the stored physical tag to keep a single resident copy per physical line.
   localparam [4:0] CACHE_IDLE      = 5'd0;
   localparam [4:0] CACHE_TAG_READ  = 5'd1;
   localparam [4:0] CACHE_TAG_CHECK = 5'd2;
   localparam [4:0] CACHE_FILL_REQ  = 5'd3;
   localparam [4:0] CACHE_FILL_WAIT = 5'd4;
   localparam [4:0] CACHE_HIT_RESP  = 5'd5;
   localparam [4:0] CACHE_WB_REQ    = 5'd6;
   localparam [4:0] CACHE_WB_WAIT   = 5'd7;
   localparam [4:0] CACHE_WB_PREP   = 5'd8;
   localparam [4:0] CACHE_CBO_TAG_READ  = 5'd9;
   localparam [4:0] CACHE_CBO_TAG_CHECK = 5'd10;
   localparam [4:0] CACHE_CBO_RESP      = 5'd11;
   localparam [4:0] CACHE_BRAM_FILL_READ = 5'd12;
   localparam [4:0] CACHE_BRAM_WB_WRITE = 5'd13;
   localparam [4:0] CACHE_BRAM_FILL_CAPTURE = 5'd14;
   localparam [4:0] CACHE_BRAM_FILL_COMMIT = 5'd15;
   localparam [4:0] CACHE_HIT_WRITE = 5'd16;
   localparam [4:0] CACHE_TAG_WAIT = 5'd17;
   localparam [4:0] CACHE_CBO_TAG_WAIT = 5'd18;
   localparam [4:0] CACHE_WB_READ_WAIT = 5'd19;
   localparam [4:0] CACHE_PROBE_READ = 5'd20;
   localparam [4:0] CACHE_PROBE_WAIT = 5'd21;
   localparam [4:0] CACHE_PROBE_CHECK = 5'd22;
   localparam [4:0] CACHE_INVALIDATE = 5'd23;
   localparam [4:0] CACHE_FLUSH_READ = 5'd24;
   localparam [4:0] CACHE_FLUSH_WAIT = 5'd25;
   localparam [4:0] CACHE_FLUSH_CHECK = 5'd26;
   localparam [4:0] CACHE_FILL_LINE_WAIT = 5'd27;
   localparam [4:0] CACHE_FILL_LINE_INSTALL = 5'd28;
   localparam [4:0] CACHE_LAST_STATE = CACHE_FILL_LINE_INSTALL;

   reg [ 7:0] dcache_bank_wr_en = 0;
   reg [`CACHE_INDEX_BITS-1:0] dcache_bank_wr_idx = 0;
   reg                         dcache_bank_wr_way = 0;
   reg [63:0] dcache_bank_wr_data = 0;

   localparam [CACHE_PERM_BITS-1:0] CACHE_PERM_PHYS = 6'b0_1_1111;

   reg [ 4:0] cache_state = CACHE_IDLE;
   reg [63:0] cache_addr = 0;
   reg [63:0] cache_req_va = 0;
   reg [TLB_ASID_BITS-1:0] cache_req_asid = 0;
   reg [CACHE_PERM_BITS-1:0] cache_req_perm = CACHE_PERM_PHYS;
   reg [TLB_CTX_BITS-1:0] cache_req_ctx = 0;
   reg [63:0] cache_fill_base = 0;
   reg [63:0] cache_wb_base = 0;
   reg [ 2:0] cache_fill_beat = 0;
   reg [ 2:0] cache_wb_beat = 0;
   reg [ 2:0] cache_req_bank = 0;
   reg [ 2:0] cache_req_next_bank = 0;
   reg        cache_req_same_line = 0;
   reg        cache_req_write = 0;
   reg        cache_req_ifetch = 0;
   reg        cache_req_cbo = 0;
   reg        cache_req_zero = 0; // 1 = whole-line zero install/write (Zicboz cbo.zero)
   reg [30:6] cache_req_line_addr = 0;
   reg [`CACHE_VTAG_BITS-1:0] cache_req_vtag = 0;
   reg [`CACHE_VTAG_BITS-1:0] cache_req_next_vtag = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] cache_req_ptag = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] cache_victim_ptag = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_victim_idx = 0;
   reg                         cache_victim_way = 0;
   reg                         cache_replace_way = 0;
   reg [63:0] cache_store_data = 0;
   reg [ 7:0] cache_store_strb = 0;
   reg [63:0] cache_fill_return_data = 0;
   reg [63:0] cache_fill_next_data = 0;
   reg [63:0]  dcache_rsp_data = 0;
   reg [63:0]  dcache_rsp_next_data = 0;
   reg         dcache_rsp_hit = 0;
   reg         dcache_rsp_hit_way = 0;
   reg         dcache_rsp_next_hit = 0;
   reg         dcache_rsp_next_valid = 0;
   reg         dcache_rsp_dirty = 0;
   reg        cache_target_dirty = 0;
   reg        cache_target_valid = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] cache_target_ptag = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_target_idx = 0;
   reg                         cache_target_way = 0;
   reg [2:0]  cache_probe_color = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_probe_idx = 0;
   reg        cache_probe_found = 0;
   reg        cache_probe_found_way = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_probe_found_idx = 0;
   reg        cache_probe_found_dirty = 0;
   reg        cache_probe_active = 0;
   reg        cache_wb_then_fill = 0;
   reg        cache_need_target_wb = 0;
   reg        cache_wb_after_cbo = 0;
   reg        cache_cbo_flush = 0;
   reg        cache_flush_req = 0;
   reg        cache_flush_ack = 0;
   wire       cache_flush_all = cache_flush_req != cache_flush_ack;
   reg [`CACHE_INDEX_BITS-1:0] cache_flush_idx = 0;
   reg        cache_flush_way = 0;
   reg        cache_wb_after_flush = 0;
   reg        cache_wb_after_ncstore = 0; // Svpbmt NC/IO store: writeback then invalidate, signal store done
   reg [30:6] cache_cbo_line_addr = 0;
   reg        cache_cbo_done_r = 0;
   reg [63:0] cache_bram_read_data_stage = 0;
   reg [63:0] cache_bram_fill_data = 0;
   reg        cache_bram_write_done = 0;
   reg [`MEM_SIZE_LG2-5:0] cache_bram_word_idx = 0;
   reg        cache_bram_word_bank = 0;
   reg [63:0] cache_bram_wb_data = 0;
   reg [63:0] cache_issue_va = 0;
   reg [TLB_ASID_BITS-1:0] cache_issue_asid = 0;
   reg [CACHE_PERM_BITS-1:0] cache_issue_perm = CACHE_PERM_PHYS;
   reg [TLB_CTX_BITS-1:0] cache_issue_ctx = 0;
   reg        dmem_rsp_valid_r = 0;
   reg [63:0] dmem_rsp_data_r = 0;
   reg [63:0] dmem_rsp_next_data_r = 0;
   reg        dmem_rsp_next_valid_r = 0;
   reg        ifetch_refill_retry_valid = 0;
   reg        dmem_write_done_r = 0;
   wire       cache_idle = cache_state == CACHE_IDLE;
   wire       cache_cbo_done = cache_cbo_done_r;
   wire       cache_read_req = ifetch_read || dmem_read;
   wire       ifetch_rsp_valid =
      cache_state == CACHE_HIT_RESP && cache_req_ifetch && icache_rsp_hit;

   reg        l2_fill_req_valid = 0;
   wire       l2_fill_req_ready;
   reg [24:0] l2_fill_req_line_addr = 0;
   wire       l2_fill_rsp_valid;
   reg        l2_fill_rsp_ready = 0;
   wire [511:0] l2_fill_rsp_data;
   wire       icache_l2_fill_req_ready;
   wire       dcache_l2_fill_req_ready;
   wire       icache_l2_fill_rsp_valid;
   wire       dcache_l2_fill_rsp_valid;
   wire       icache_l2_fill_rsp_ready;
   wire       dcache_l2_fill_rsp_ready;
   wire [511:0] icache_l2_fill_rsp_data;
   wire [511:0] dcache_l2_fill_rsp_data;
   reg        l2_wb_req_valid = 0;
   wire       l2_wb_req_ready;
   reg [24:0] l2_wb_req_line_addr = 0;
   reg [511:0] l2_wb_req_line_data = 0;
   wire       l2_wb_rsp_valid;
   reg        l2_wb_rsp_ready = 0;
   reg        l2_direct_read_req_valid = 0;
   wire       l2_direct_read_req_ready;
   reg [27:0] l2_direct_read_req_addr = 0;
   wire       l2_direct_read_rsp_valid;
   reg        l2_direct_read_rsp_ready = 0;
   wire [63:0] l2_direct_read_rsp_data;
   wire       mem_engine_idle;
   wire       icache_l2_fill_req_valid = l2_fill_req_valid && cache_req_ifetch;
   wire       dcache_l2_fill_req_valid = l2_fill_req_valid && !cache_req_ifetch;
   assign l2_fill_req_ready = cache_req_ifetch ? icache_l2_fill_req_ready :
                                                 dcache_l2_fill_req_ready;
   assign l2_fill_rsp_valid = cache_req_ifetch ? icache_l2_fill_rsp_valid :
                                                 dcache_l2_fill_rsp_valid;
   assign l2_fill_rsp_data = cache_req_ifetch ? icache_l2_fill_rsp_data :
                                                dcache_l2_fill_rsp_data;
   assign icache_l2_fill_rsp_ready = l2_fill_rsp_ready && cache_req_ifetch;
   assign dcache_l2_fill_rsp_ready = l2_fill_rsp_ready && !cache_req_ifetch;
   wire       l2_fill_req_fire = l2_fill_req_valid && l2_fill_req_ready;
   wire       l2_wb_req_fire = l2_wb_req_valid && l2_wb_req_ready;
   wire       l2_direct_read_req_fire = l2_direct_read_req_valid && l2_direct_read_req_ready;
   wire       icache_fill_begin =
              cache_req_ifetch &&
              ((cache_state == CACHE_FILL_REQ && cache_fill_from_bram &&
                cache_fill_beat == 3'd0) ||
               l2_fill_req_fire);
   wire       cache_bram_fill_commit = cache_state == CACHE_BRAM_FILL_COMMIT;
   reg [511:0] cache_fill_line_data = 0;
   wire       cache_line_install = cache_state == CACHE_FILL_LINE_INSTALL;
   wire       cache_fill_data_valid = cache_line_install || cache_bram_fill_commit;
   wire [63:0] cache_fill_line_word = cache_fill_line_data[cache_fill_beat * 64 +: 64];
   wire [63:0] cache_fill_data = cache_bram_fill_commit ? cache_bram_fill_data
                                                        : cache_fill_line_word;
   wire [63:0] cache_bram_base_addr = {33'd0, MEM_BASEADDR_VALUE[30:0]};
   wire [63:0] cache_bram_window_mask = 64'hffff_ffff_ffff_ffff << `MEM_SIZE_LG2;
   wire       cache_fill_from_bram =
              ((cache_fill_base ^ cache_bram_base_addr) & cache_bram_window_mask) == 0;
   wire       cache_wb_to_bram =
              ((cache_wb_base ^ cache_bram_base_addr) & cache_bram_window_mask) == 0;
   wire [63:0] ptw_direct_addr64 = {33'd0, ptw_direct_addr, 3'b000};
   wire        ptw_direct_from_bram =
              ((ptw_direct_addr64 ^ cache_bram_base_addr) & cache_bram_window_mask) == 0;

   assign     core_reset_home = state == `S_FETCH1 && cache_state == CACHE_IDLE &&
                                !cache_read_req && !dmem_write &&
                                !l2_fill_req_valid && !l2_wb_req_valid &&
                                !l2_direct_read_req_valid &&
                                !ptw_direct_read && !ptw_direct_probe_pending &&
                                !ptw_direct_wait_probe && !ptw_direct_pending &&
                                !ptw_direct_wait_bram && !ptw_direct_wait_axi &&
                                mem_engine_idle;
   assign     core_reset_now  = (reset || core_reset_pending) && core_reset_home;

   wire       hpm_instret_pulse = state == `S_FETCH1 && retire_now_q;
   wire       hpm_icache_read_pulse = cache_state == CACHE_IDLE && ifetch_read;
   wire       hpm_dcache_read_pulse = cache_state == CACHE_IDLE && dmem_read;
   wire       hpm_dcache_write_pulse = cache_state == CACHE_IDLE && dmem_write;
   wire       hpm_icache_hit_pulse =
              cache_state == CACHE_HIT_RESP && cache_req_ifetch &&
              icache_rsp_hit;
   wire       hpm_dcache_hit_pulse =
              cache_state == CACHE_HIT_RESP && !cache_req_ifetch &&
              dcache_rsp_hit;
   wire       hpm_icache_miss_pulse =
              cache_state == CACHE_HIT_RESP && cache_req_ifetch &&
              !icache_rsp_hit;
   wire       hpm_dcache_miss_pulse =
              cache_state == CACHE_HIT_RESP && !cache_req_ifetch &&
              !dcache_rsp_hit;
   wire       hpm_icache_fill_beat_pulse = cache_fill_data_valid && cache_req_ifetch;
   wire       hpm_dcache_fill_beat_pulse = cache_fill_data_valid && !cache_req_ifetch;
   wire       hpm_icache_fill_line_pulse =
              hpm_icache_fill_beat_pulse && cache_fill_beat == 3'd7;
   wire       hpm_dcache_fill_line_pulse =
              hpm_dcache_fill_beat_pulse && cache_fill_beat == 3'd7;
   wire       hpm_dcache_wb_line_pulse = cache_state == CACHE_WB_REQ &&
                                         (cache_wb_to_bram || l2_wb_req_ready) &&
                                         cache_wb_beat == 3'd0;
   wire       hpm_axi_read_pulse = l2_fill_req_fire || l2_direct_read_req_fire;
   wire       hpm_axi_write_pulse = l2_wb_req_fire;
   wire       hpm_bus_wait_cycle = state == `S_IFETCH_WAIT || state == `S_IFETCH_HALF_WAIT ||
                                   state == `S_DMEM_LOAD_WAIT  || state == `S_DMEM_LOAD2_WAIT ||
                                   state == `S_PTW_DIRECT_WAIT   ||
                                   state == `S_DMEM_STORE_WAIT || state == `S_DMEM_STORE2 ||
                                   state == `S_DMEM_STORE_RESP_WAIT || state == `S_DMEM_STORE_RESP_ARM ||
                                   state == `S_MMIO_ALIGN;
   reg        hpm_instret_q = 0;
   reg        hpm_icache_read_q = 0;
   reg        hpm_icache_hit_q = 0;
   reg        hpm_icache_miss_q = 0;
   reg        hpm_icache_fill_line_q = 0;
   reg        hpm_icache_fill_beat_q = 0;
   reg        hpm_dcache_read_q = 0;
   reg        hpm_dcache_write_q = 0;
   reg        hpm_dcache_hit_q = 0;
   reg        hpm_dcache_miss_q = 0;
   reg        hpm_dcache_fill_line_q = 0;
   reg        hpm_dcache_fill_beat_q = 0;
   reg        hpm_dcache_wb_line_q = 0;
   reg        hpm_axi_read_q = 0;
   reg        hpm_axi_write_q = 0;
   reg        hpm_bus_wait_q = 0;
   reg        hpm_tlb_lookup_q = 0;
   reg        hpm_tlb_hit_q = 0;
   reg        hpm_tlb_miss_q = 0;
   reg        hpm_tlb_hit_4k_q = 0;
   reg        hpm_tlb_hit_2m_q = 0;
   reg        hpm_tlb_insert_4k_q = 0;
   reg        hpm_tlb_insert_2m_q = 0;
   reg        hpm_tlb_evict_4k_q = 0;
   reg        hpm_tlb_evict_2m_q = 0;
   reg        hpm_tlb_uncached_1g_q = 0;
   reg        hpm_tlb_uncached_napot_q = 0;
   reg        hpm_ptw_leaf_4k_q = 0;
   reg        hpm_ptw_leaf_2m_q = 0;
   reg        hpm_ptw_leaf_1g_q = 0;
   reg        hpm_ptw_leaf_napot_q = 0;

`ifdef SIMULATE
   reg cache_trace_enabled = 0;
   reg cache_summary_enabled = 0;
   reg fetch_buf_summary_enabled = 0;
   reg state_summary_enabled = 0;
   integer state_summary_interval = 0;
   reg state_summary_interval_seen = 0;
   reg [63:0] state_summary_next = 0;
   reg [63:0] state_stat_total_cycles = 0;
   reg [63:0] state_stat_instret = 0;
   reg [63:0] state_stat_cycles [0:`S_LAST_STATE];
   reg [63:0] cache_state_stat_cycles [0:CACHE_LAST_STATE];
   reg [63:0] icache_stat_reads = 0;
   reg [63:0] icache_stat_hits = 0;
   reg [63:0] icache_stat_misses = 0;
   reg [63:0] icache_stat_fill_lines = 0;
   reg [63:0] dcache_stat_reads = 0;
   reg [63:0] dcache_stat_writes = 0;
   reg [63:0] dcache_stat_hits = 0;
   reg [63:0] dcache_stat_misses = 0;
   reg [63:0] dcache_stat_dirty_misses = 0;
   reg [63:0] dcache_stat_fill_lines = 0;
   reg [63:0] dcache_stat_wb_lines = 0;
   reg [63:0] fetch_buf_stat_hits = 0;
   reg [63:0] fetch_buf_stat_misses = 0;
   reg [63:0] ptw_stat_leaf_4k = 0;
   reg [63:0] ptw_stat_leaf_64k_napot = 0;
   reg [63:0] ptw_stat_leaf_2m = 0;
   reg [63:0] ptw_stat_leaf_1g = 0;
   reg [63:0] tlb_stat_lookups = 0;
   reg [63:0] tlb_stat_hits = 0;
   reg [63:0] tlb_stat_misses = 0;
   reg [63:0] tlb_stat_hits_4k = 0;
   reg [63:0] tlb_stat_hits_2m = 0;
   reg [63:0] tlb_stat_inserts_4k = 0;
   reg [63:0] tlb_stat_inserts_2m = 0;
   reg [63:0] tlb_stat_evicts_4k = 0;
   reg [63:0] tlb_stat_evicts_2m = 0;
   reg [63:0] tlb_stat_uncached_1g = 0;
   reg [63:0] tlb_stat_uncached_napot = 0;
   integer tlb_stat_entries_4k = 0;
   integer tlb_stat_entries_2m = 0;
   integer tlb_stat_entries_1g = 0;
   integer state_stat_i;

   // Simulation-only debug/introspection helpers (state names, run summary).
   `include "smolrv64_debug.vh"

   initial begin
      for (state_stat_i = 0; state_stat_i <= `S_LAST_STATE; state_stat_i = state_stat_i + 1)
         state_stat_cycles[state_stat_i] = 0;
      for (state_stat_i = 0; state_stat_i <= CACHE_LAST_STATE; state_stat_i = state_stat_i + 1)
         cache_state_stat_cycles[state_stat_i] = 0;
      cache_trace_enabled = $test$plusargs("cache_trace");
      cache_summary_enabled = $test$plusargs("cache_summary");
      fetch_buf_summary_enabled = $test$plusargs("fetch_buf_summary");
      state_summary_enabled = $test$plusargs("state_summary");
      state_summary_interval_seen = $value$plusargs("state_summary_interval=%d", state_summary_interval);
      if (state_summary_interval_seen)
         state_summary_enabled = 1;
      else if (state_summary_enabled)
         state_summary_interval = 1000000;
      if (state_summary_interval < 0)
         state_summary_interval = 0;
      state_summary_next = {32'd0, state_summary_interval};
      if (cache_trace_enabled)
         $display("CACHE TRACE ENABLED");
      if (cache_summary_enabled)
         $display("L1 SUMMARY ENABLED");
      if (fetch_buf_summary_enabled)
         $display("FETCH BUFFER SUMMARY ENABLED");
      if (state_summary_enabled) begin
         if (state_summary_interval != 0)
            $display("STATE SUMMARY ENABLED interval=%0d cycles", state_summary_interval);
         else
            $display("STATE SUMMARY ENABLED");
      end
   end
`endif

   wire [63:0] cache_issue_addr = {33'd0, cache_issue_dw_addr, 3'b000};
   wire [63:0] cache_issue_next_va = cache_issue_va + 64'd8;
   wire [`CACHE_PHYS_TAG_BITS-1:0] cache_issue_ptag =
        cache_issue_addr[`CACHE_PHYS_BITS-1:`CACHE_PAGE_OFFSET_BITS];
   wire [`CACHE_PHYS_TAG_BITS-1:0] cache_cbo_ptag =
        cache_cbo_line_addr[30:12];
   wire [`CACHE_INDEX_BITS-1:0] cache_probe_line_index =
        {cache_probe_color, cache_addr[11:6]};
   wire [`CACHE_INDEX_BITS-1:0] cache_req_probe_line_index =
        {cache_probe_color, cache_req_line_addr[11:6]};

   // Cache metadata pack/unpack + index helpers, shared with smolrv64_frontend.
   `include "smolrv64_cache_meta.vh"

   reg [`CACHE_INDEX_BITS-1:0] cache_way0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way1_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way0_bank0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way1_bank0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way0_next_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way1_next_rd_idx = 0;
   reg                         dcache_way0_tag_wr_en = 0;
   reg                         dcache_way1_tag_wr_en = 0;
   reg  [`CACHE_INDEX_BITS-1:0] dcache_tag_wr_idx = 0;
   reg  [`CACHE_META_BITS-1:0]  dcache_tag_wr_data = 0;
   reg                         icache_invalidate_valid = 0;
   reg                         icache_invalidate_way = 0;
   reg  [`CACHE_INDEX_BITS-1:0] icache_invalidate_idx = 0;
   wire [`CACHE_META_BITS-1:0] dcache_way0_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] dcache_way1_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] dcache_way0_tag_next_rd_data;
   wire [`CACHE_META_BITS-1:0] dcache_way1_tag_next_rd_data;
   wire [63:0] dcache_way0_bank_rd_data [0:7];
   wire [63:0] dcache_way1_bank_rd_data [0:7];

   wire dcache_way0_tag_hit = cache_meta_valid(dcache_way0_tag_rd_data) &&
                              cache_meta_epoch(dcache_way0_tag_rd_data) == vhpr_epoch &&
                              cache_meta_asid(dcache_way0_tag_rd_data) == cache_req_asid &&
                              cache_meta_vtag(dcache_way0_tag_rd_data) == cache_req_vtag &&
                              cache_perm_allows_ctx(cache_meta_perm(dcache_way0_tag_rd_data),
                                                    cache_req_ctx);
   wire dcache_way1_tag_hit = cache_meta_valid(dcache_way1_tag_rd_data) &&
                              cache_meta_epoch(dcache_way1_tag_rd_data) == vhpr_epoch &&
                              cache_meta_asid(dcache_way1_tag_rd_data) == cache_req_asid &&
                              cache_meta_vtag(dcache_way1_tag_rd_data) == cache_req_vtag &&
                              cache_perm_allows_ctx(cache_meta_perm(dcache_way1_tag_rd_data),
                                                    cache_req_ctx);
   wire dcache_way0_next_tag_hit = cache_meta_valid(dcache_way0_tag_next_rd_data) &&
                                   cache_meta_epoch(dcache_way0_tag_next_rd_data) == vhpr_epoch &&
                                   cache_meta_asid(dcache_way0_tag_next_rd_data) == cache_req_asid &&
                                   cache_meta_vtag(dcache_way0_tag_next_rd_data) == cache_req_next_vtag &&
                                   cache_perm_allows_ctx(cache_meta_perm(dcache_way0_tag_next_rd_data),
                                                         cache_req_ctx);
   wire dcache_way1_next_tag_hit = cache_meta_valid(dcache_way1_tag_next_rd_data) &&
                                   cache_meta_epoch(dcache_way1_tag_next_rd_data) == vhpr_epoch &&
                                   cache_meta_asid(dcache_way1_tag_next_rd_data) == cache_req_asid &&
                                   cache_meta_vtag(dcache_way1_tag_next_rd_data) == cache_req_next_vtag &&
                                   cache_perm_allows_ctx(cache_meta_perm(dcache_way1_tag_next_rd_data),
                                                         cache_req_ctx);

   function [63:0] dcache_selected_bank_data;
      input       way;
      input [2:0] bank;
      dcache_selected_bank_data = way ? dcache_way1_bank_rd_data[bank]
                                      : dcache_way0_bank_rd_data[bank];
   endfunction

   function [511:0] dcache_selected_line_data;
      input way;
      begin
         dcache_selected_line_data = {
            dcache_selected_bank_data(way, 3'd7),
            dcache_selected_bank_data(way, 3'd6),
            dcache_selected_bank_data(way, 3'd5),
            dcache_selected_bank_data(way, 3'd4),
            dcache_selected_bank_data(way, 3'd3),
            dcache_selected_bank_data(way, 3'd2),
            dcache_selected_bank_data(way, 3'd1),
            dcache_selected_bank_data(way, 3'd0)
         };
      end
   endfunction

   wire dcache_lookup_hit = dcache_way0_tag_hit || dcache_way1_tag_hit;
   wire dcache_lookup_hit_way = dcache_way1_tag_hit;
   wire dcache_lookup_next_hit =
        dcache_way0_next_tag_hit || dcache_way1_next_tag_hit;
   wire dcache_lookup_next_hit_way = dcache_way1_next_tag_hit;
   wire [63:0] dcache_lookup_data =
        dcache_selected_bank_data(dcache_lookup_hit_way, cache_req_bank);
   wire [63:0] dcache_lookup_next_data =
        cache_req_same_line
        ? dcache_selected_bank_data(dcache_lookup_hit_way, cache_req_next_bank)
        : dcache_selected_bank_data(dcache_lookup_next_hit_way, 3'd0);
   wire dcache_lookup_next_valid =
        cache_req_same_line || dcache_lookup_next_hit;
   wire dcache_target_way =
        !cache_meta_valid(dcache_way0_tag_rd_data) ? 1'b0 :
        !cache_meta_valid(dcache_way1_tag_rd_data) ? 1'b1 :
        cache_meta_epoch(dcache_way0_tag_rd_data) != vhpr_epoch ? 1'b0 :
        cache_meta_epoch(dcache_way1_tag_rd_data) != vhpr_epoch ? 1'b1 :
        cache_replace_way;
   wire [`CACHE_META_BITS-1:0] dcache_target_meta =
        dcache_target_way ? dcache_way1_tag_rd_data : dcache_way0_tag_rd_data;
   wire dcache_target_valid = cache_meta_valid(dcache_target_meta);
   wire dcache_target_dirty =
        dcache_target_valid && cache_meta_dirty(dcache_target_meta);
   wire [`CACHE_INDEX_BITS-1:0] dcache_target_idx =
        dcache_target_way ? cache_way1_rd_idx : cache_way0_rd_idx;
   wire [`CACHE_PHYS_TAG_BITS-1:0] dcache_target_ptag =
        cache_meta_ptag(dcache_target_meta);
   wire [`CACHE_META_BITS-1:0] dcache_lookup_meta =
        dcache_lookup_hit_way ? dcache_way1_tag_rd_data : dcache_way0_tag_rd_data;
   wire dcache_lookup_dirty =
        dcache_lookup_hit ? cache_meta_dirty(dcache_lookup_meta) :
                            dcache_target_dirty;

   smolrv64_frontend #(
      .EPOCH_BITS(FRONTEND_EPOCH_BITS),
      .TLB_ASID_BITS(TLB_ASID_BITS),
      .TLB_CTX_BITS(TLB_CTX_BITS),
      .CACHE_PERM_BITS(CACHE_PERM_BITS),
      .VHPR_EPOCH_BITS(VHPR_EPOCH_BITS)
   ) frontend_inst (
      .clock(clock),
      .reset(core_reset_now),
      .flush(frontend_buf_flush),
      .fill(frontend_buf_fill),
      .fill_pc(frontend_buf_fill_pc),
      .fill_prv(frontend_buf_fill_prv),
      .fill_asid(frontend_buf_fill_asid),
      .fill_data(frontend_buf_fill_data),
      .cmd_valid(frontend_cmd_valid),
      .cmd_pc(frontend_cmd_pc),
      .cmd_prv(frontend_cmd_prv),
      .cmd_asid(frontend_cmd_asid),
      .cmd_epoch(frontend_cmd_epoch),
      .rsp_hit(frontend_rsp_hit),
      .rsp_insn(frontend_rsp_insn),
      .rsp_next_pc(frontend_rsp_next_pc),
      .rsp_predicted_next_pc(frontend_rsp_predicted_next_pc),

      .icache_invalidate_valid(icache_invalidate_valid),
      .icache_invalidate_way(icache_invalidate_way),
      .icache_invalidate_idx(icache_invalidate_idx),
      .icache_fill_begin(icache_fill_begin),
      .icache_fill_begin_idx(cache_target_idx),
      .icache_fill_begin_way(cache_target_way),
      .icache_fill_begin_asid(cache_req_asid),
      .icache_fill_begin_perm(cache_req_perm),
      .icache_fill_begin_vtag(cache_req_vtag),
      .icache_fill_begin_ptag(cache_req_ptag),
      .icache_fill_begin_epoch(vhpr_epoch),
      .icache_fill_valid(cache_req_ifetch && cache_fill_data_valid),
      .icache_fill_beat(cache_fill_beat),
      .icache_fill_data(cache_fill_data),
      .icache_req_va(cache_req_va),
      .icache_req_next_va(cache_req_va + 64'd8),
      .icache_req_asid(cache_req_asid),
      .icache_req_vtag(cache_req_vtag),
      .icache_req_next_vtag(cache_req_next_vtag),
      .icache_req_ctx(cache_req_ctx),
      .icache_req_bank(cache_req_bank),
      .icache_req_next_bank(cache_req_next_bank),
      .icache_req_same_line(cache_req_same_line),
      .icache_replace_way(cache_replace_way),
      .icache_vhpr_epoch(vhpr_epoch),
      .icache_rsp_capture(cache_state == CACHE_TAG_CHECK),
      .icache_rsp_hit(icache_rsp_hit),
      .icache_rsp_next_valid(icache_rsp_next_valid),
      .icache_rsp_window(icache_rsp_window),
      .icache_rsp_insn_valid(icache_rsp_insn_valid),
      .icache_rsp_insn(icache_rsp_insn),
      .icache_rsp_next_pc(icache_rsp_next_pc),
      .icache_target_way(icache_target_way),
      .icache_target_idx(icache_target_idx),
      .icache_target_valid(icache_target_valid)
   );

   function hpm_event_active;
      input [15:0] event_code;
      input        instret_pulse;
      input        icache_read_pulse;
      input        icache_hit_pulse;
      input        icache_miss_pulse;
      input        icache_fill_line_pulse;
      input        icache_fill_beat_pulse;
      input        dcache_read_pulse;
      input        dcache_write_pulse;
      input        dcache_hit_pulse;
      input        dcache_miss_pulse;
      input        dcache_fill_line_pulse;
      input        dcache_fill_beat_pulse;
      input        dcache_wb_line_pulse;
      input        axi_read_pulse;
      input        axi_write_pulse;
      input        bus_wait_cycle;
      input        tlb_lookup_pulse;
      input        tlb_hit_pulse;
      input        tlb_miss_pulse;
      input        tlb_hit_4k_pulse;
      input        tlb_hit_2m_pulse;
      input        tlb_insert_4k_pulse;
      input        tlb_insert_2m_pulse;
      input        tlb_evict_4k_pulse;
      input        tlb_evict_2m_pulse;
      input        tlb_uncached_1g_pulse;
      input        tlb_uncached_napot_pulse;
      input        ptw_leaf_4k_pulse;
      input        ptw_leaf_2m_pulse;
      input        ptw_leaf_1g_pulse;
      input        ptw_leaf_napot_pulse;
      input [`VHPRP_WIDTH-1:0] vhpr_pulse;
      begin
         case (event_code)
           `HPM_EVENT_CYCLES:          hpm_event_active = 1'b1;
           `HPM_EVENT_INSTRUCTIONS:    hpm_event_active = instret_pulse;
           `HPM_EVENT_ICACHE_READ:     hpm_event_active = icache_read_pulse;
           `HPM_EVENT_ICACHE_HIT:      hpm_event_active = icache_hit_pulse;
           `HPM_EVENT_ICACHE_MISS:     hpm_event_active = icache_miss_pulse;
           `HPM_EVENT_ICACHE_FILL_LINE: hpm_event_active = icache_fill_line_pulse;
           `HPM_EVENT_ICACHE_FILL_BEAT: hpm_event_active = icache_fill_beat_pulse;
           `HPM_EVENT_DCACHE_READ:     hpm_event_active = dcache_read_pulse;
           `HPM_EVENT_DCACHE_WRITE:    hpm_event_active = dcache_write_pulse;
           `HPM_EVENT_DCACHE_HIT:      hpm_event_active = dcache_hit_pulse;
           `HPM_EVENT_DCACHE_MISS:     hpm_event_active = dcache_miss_pulse;
           `HPM_EVENT_DCACHE_FILL_LINE: hpm_event_active = dcache_fill_line_pulse;
           `HPM_EVENT_DCACHE_FILL_BEAT: hpm_event_active = dcache_fill_beat_pulse;
           `HPM_EVENT_DCACHE_WB_LINE:  hpm_event_active = dcache_wb_line_pulse;
           `HPM_EVENT_AXI_READ:        hpm_event_active = axi_read_pulse;
           `HPM_EVENT_AXI_WRITE:       hpm_event_active = axi_write_pulse;
           `HPM_EVENT_BUS_WAIT_CYCLE:  hpm_event_active = bus_wait_cycle;
           `HPM_EVENT_TLB_LOOKUP:      hpm_event_active = tlb_lookup_pulse;
           `HPM_EVENT_TLB_HIT:         hpm_event_active = tlb_hit_pulse;
           `HPM_EVENT_TLB_MISS:        hpm_event_active = tlb_miss_pulse;
           `HPM_EVENT_TLB_HIT_4K:      hpm_event_active = tlb_hit_4k_pulse;
           `HPM_EVENT_TLB_HIT_2M:      hpm_event_active = tlb_hit_2m_pulse;
           `HPM_EVENT_TLB_INSERT_4K:   hpm_event_active = tlb_insert_4k_pulse;
           `HPM_EVENT_TLB_INSERT_2M:   hpm_event_active = tlb_insert_2m_pulse;
           `HPM_EVENT_TLB_EVICT_4K:    hpm_event_active = tlb_evict_4k_pulse;
           `HPM_EVENT_TLB_EVICT_2M:    hpm_event_active = tlb_evict_2m_pulse;
           `HPM_EVENT_TLB_UNCACHED_1G: hpm_event_active = tlb_uncached_1g_pulse;
           `HPM_EVENT_TLB_UNCACHED_NAPOT: hpm_event_active = tlb_uncached_napot_pulse;
           `HPM_EVENT_PTW_LEAF_4K:     hpm_event_active = ptw_leaf_4k_pulse;
           `HPM_EVENT_PTW_LEAF_2M:     hpm_event_active = ptw_leaf_2m_pulse;
           `HPM_EVENT_PTW_LEAF_1G:     hpm_event_active = ptw_leaf_1g_pulse;
           `HPM_EVENT_PTW_LEAF_NAPOT:  hpm_event_active = ptw_leaf_napot_pulse;
           `HPM_EVENT_VHPR_READS:              hpm_event_active = vhpr_pulse[`VHPRP_READS];
           `HPM_EVENT_VHPR_WRITES:             hpm_event_active = vhpr_pulse[`VHPRP_WRITES];
           `HPM_EVENT_VHPR_READ_HITS:          hpm_event_active = vhpr_pulse[`VHPRP_READ_HITS];
           `HPM_EVENT_VHPR_READ_MISSES:        hpm_event_active = vhpr_pulse[`VHPRP_READ_MISSES];
           `HPM_EVENT_VHPR_WRITE_HITS:         hpm_event_active = vhpr_pulse[`VHPRP_WRITE_HITS];
           `HPM_EVENT_VHPR_WRITE_MISSES:       hpm_event_active = vhpr_pulse[`VHPRP_WRITE_MISSES];
           `HPM_EVENT_VHPR_FILLS:              hpm_event_active = vhpr_pulse[`VHPRP_FILLS];
           `HPM_EVENT_VHPR_VICTIM_EVICTS:      hpm_event_active = vhpr_pulse[`VHPRP_VICTIM_EVICTS];
           `HPM_EVENT_VHPR_DIRTY_VICTIM_EVICTS: hpm_event_active = vhpr_pulse[`VHPRP_DIRTY_VICTIM_EVICTS];
           `HPM_EVENT_VHPR_ALIAS_EVICTS:       hpm_event_active = vhpr_pulse[`VHPRP_ALIAS_EVICTS];
           `HPM_EVENT_VHPR_DIRTY_ALIAS_EVICTS: hpm_event_active = vhpr_pulse[`VHPRP_DIRTY_ALIAS_EVICTS];
           `HPM_EVENT_VHPR_FLUSH_EVICTS:       hpm_event_active = vhpr_pulse[`VHPRP_FLUSH_EVICTS];
           `HPM_EVENT_VHPR_DIRTY_FLUSH_EVICTS: hpm_event_active = vhpr_pulse[`VHPRP_DIRTY_FLUSH_EVICTS];
           `HPM_EVENT_VHPR_CBO_PROBES:         hpm_event_active = vhpr_pulse[`VHPRP_CBO_PROBES];
           `HPM_EVENT_VHPR_PTW_PROBES:         hpm_event_active = vhpr_pulse[`VHPRP_PTW_PROBES];
           `HPM_EVENT_VHPR_EPOCH_BUMPS:        hpm_event_active = vhpr_pulse[`VHPRP_EPOCH_BUMPS];
           `HPM_EVENT_VHPR_EPOCH_ROLLOVERS:    hpm_event_active = vhpr_pulse[`VHPRP_EPOCH_ROLLOVERS];
           default:                    hpm_event_active = 1'b0;
         endcase
      end
   endfunction

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way0_tag_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way0_rd_idx ),
      .rd_data ( dcache_way0_tag_rd_data ),
      .wr_en   ( dcache_way0_tag_wr_en ),
      .wr_addr ( dcache_tag_wr_idx ),
      .wr_data ( dcache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way1_tag_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way1_rd_idx ),
      .rd_data ( dcache_way1_tag_rd_data ),
      .wr_en   ( dcache_way1_tag_wr_en ),
      .wr_addr ( dcache_tag_wr_idx ),
      .wr_data ( dcache_tag_wr_data )
   );

   always @* begin
      dcache_bank_wr_en = 8'd0;
      dcache_bank_wr_idx = cache_target_idx;
      dcache_bank_wr_way = cache_target_way;
      dcache_bank_wr_data = cache_fill_data;

      if ((cache_state == CACHE_FILL_LINE_INSTALL || cache_state == CACHE_BRAM_FILL_COMMIT) &&
          cache_fill_data_valid && !cache_req_ifetch) begin
         dcache_bank_wr_en = 8'd1 << cache_fill_beat;
         dcache_bank_wr_idx = cache_target_idx;
         dcache_bank_wr_way = cache_target_way;
         dcache_bank_wr_data = cache_fill_data;
         if (cache_req_write && cache_fill_beat == cache_req_bank)
            dcache_bank_wr_data = merge_store_bytes(cache_fill_data, cache_store_data, cache_store_strb);
         // Zicboz cbo.zero: overwrite every installed beat with zeros (whole
         // line). The fetched fill data is discarded -- see CBO_ZERO note below
         // for the cold-miss read-for-ownership we still pay here.
         if (cache_req_zero)
            dcache_bank_wr_data = 64'd0;
      end

      if (cache_state == CACHE_HIT_WRITE) begin
         // cbo.zero zeros all 8 banks of the resident line in one cycle via the
         // per-bank write-enable mask (shared write-data bus, uniform zero);
         // ordinary stores write just the addressed bank.
         dcache_bank_wr_en = cache_req_zero ? 8'hFF : (8'd1 << cache_req_bank);
         dcache_bank_wr_idx = dcache_rsp_hit_way ? cache_way1_rd_idx :
                                                   cache_way0_rd_idx;
         dcache_bank_wr_way = dcache_rsp_hit_way;
         dcache_bank_wr_data = cache_req_zero ? 64'd0 :
            merge_store_bytes(dcache_rsp_data, cache_store_data, cache_store_strb);
      end
   end

   genvar cache_bank_gen;
   generate
      for (cache_bank_gen = 0; cache_bank_gen < 8; cache_bank_gen = cache_bank_gen + 1) begin : vhpr_cache_banks
         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) dcache_way0_bank_ram (
            .clock   ( clock ),
            .rd_addr ( cache_bank_gen == 0 ? cache_way0_bank0_rd_idx : cache_way0_rd_idx ),
            .rd_data ( dcache_way0_bank_rd_data[cache_bank_gen] ),
            .wr_en   ( dcache_bank_wr_en[cache_bank_gen] && !dcache_bank_wr_way ),
            .wr_addr ( dcache_bank_wr_idx ),
            .wr_data ( dcache_bank_wr_data )
         );

         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) dcache_way1_bank_ram (
            .clock   ( clock ),
            .rd_addr ( cache_bank_gen == 0 ? cache_way1_bank0_rd_idx : cache_way1_rd_idx ),
            .rd_data ( dcache_way1_bank_rd_data[cache_bank_gen] ),
            .wr_en   ( dcache_bank_wr_en[cache_bank_gen] && dcache_bank_wr_way ),
            .wr_addr ( dcache_bank_wr_idx ),
            .wr_data ( dcache_bank_wr_data )
         );

      end
   endgenerate

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way0_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way0_next_rd_idx ),
      .rd_data ( dcache_way0_tag_next_rd_data ),
      .wr_en   ( dcache_way0_tag_wr_en ),
      .wr_addr ( dcache_tag_wr_idx ),
      .wr_data ( dcache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way1_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way1_next_rd_idx ),
      .rd_data ( dcache_way1_tag_next_rd_data ),
      .wr_en   ( dcache_way1_tag_wr_en ),
      .wr_addr ( dcache_tag_wr_idx ),
      .wr_data ( dcache_tag_wr_data )
   );

`ifndef BUS_TIMEOUT_LG2
`define BUS_TIMEOUT_LG2 24  // ~16M cycles before access fault
`endif
   reg [`BUS_TIMEOUT_LG2-1:0] bus_timeout_ctr = 0;
   // Registered one-cycle-ahead terminal-count, so the cause/state
   // combinational cone sees a FF output instead of a deep AND tree.
   // Fires one cycle after bus_timeout_ctr saturates — negligible at
   // 16M-cycle threshold, but critical for timing closure.
   reg        bus_timeout_expired = 0;

   reg [VHPR_EPOCH_BITS-1:0] vhpr_epoch = 0;
   reg [VHPR_EPOCH_BITS-1:0] vhpr_next_epoch = 0;
   reg                       vhpr_epoch_update_pending = 0;
   reg                       vhpr_epoch_bump_req = 0;
   reg                       vhpr_epoch_bump_ack = 0;
   wire                      vhpr_epoch_bump_pending = vhpr_epoch_bump_req != vhpr_epoch_bump_ack;
   // VHPR cache stats are exposed as selectable HPM events (0x0400+) rather
   // than dedicated always-on CSR counters. One bit per stat is pulsed where
   // the event occurs (block C cache FSM); hpm_vhpr_pulse_q feeds the event mux.
   reg [`VHPRP_WIDTH-1:0] hpm_vhpr_pulse = 0;
   reg [`VHPRP_WIDTH-1:0] hpm_vhpr_pulse_q = 0;

   task cache_flush_next_line;
      reg [`CACHE_INDEX_BITS-1:0] next_idx;
      begin
         if (!cache_flush_way) begin
            cache_flush_way <= 1'b1;
            next_idx = cache_flush_idx;
            cache_way0_rd_idx <= next_idx;
            cache_way1_rd_idx <= next_idx;
            cache_way0_bank0_rd_idx <= next_idx;
            cache_way1_bank0_rd_idx <= next_idx;
            cache_state <= CACHE_FLUSH_READ;
        end else if (cache_flush_idx == {`CACHE_INDEX_BITS{1'b1}}) begin
            cache_flush_ack <= cache_flush_req;
            cache_cbo_done_r <= 1;
            if (vhpr_epoch_update_pending) begin
               vhpr_epoch <= vhpr_next_epoch;
               vhpr_epoch_update_pending <= 1'b0;
            end
            cache_state <= CACHE_IDLE;
         end else begin
            next_idx = cache_flush_idx + 1'b1;
            cache_flush_idx <= next_idx;
            cache_flush_way <= 1'b0;
            cache_way0_rd_idx <= next_idx;
            cache_way1_rd_idx <= next_idx;
            cache_way0_bank0_rd_idx <= next_idx;
            cache_way1_bank0_rd_idx <= next_idx;
            cache_state <= CACHE_FLUSH_READ;
         end
      end
   endtask

   task cache_start_fill_request;
      begin
         cache_fill_base <= {cache_addr[63:6], 6'd0};
         cache_fill_beat <= 0;
         cache_fill_return_data <= 0;
         cache_fill_next_data <= 0;
         cache_replace_way <= ~cache_replace_way;
         cache_probe_color <= 0;
         cache_probe_found <= 0;
         cache_probe_found_way <= 0;
         cache_probe_found_idx <= 0;
         cache_probe_found_dirty <= 0;
         cache_probe_active <= 1;
         cache_way0_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_way1_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_way0_bank0_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_way1_bank0_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_state <= CACHE_PROBE_READ;
      end
   endtask

   task cache_finish_writeback_line;
      begin
         cache_wb_beat <= 0;
         if (cache_wb_after_ncstore) begin
            // Svpbmt NC/IO store flush-around: the filled+merged line was written
            // back to memory; invalidate the slot (it was never installed) and
            // signal store completion so the store reaches DRAM uncached.
            dcache_way0_tag_wr_en <= !cache_victim_way;
            dcache_way1_tag_wr_en <= cache_victim_way;
            dcache_tag_wr_idx <= cache_victim_idx;
            dcache_tag_wr_data <= 0;
            cache_wb_after_ncstore <= 1'b0;
            dmem_write_done_r <= 1;
            cache_state <= CACHE_IDLE;
         end else if (cache_wb_after_cbo) begin
            dcache_way0_tag_wr_en <= !cache_victim_way;
            dcache_way1_tag_wr_en <= cache_victim_way;
            dcache_tag_wr_idx <= cache_victim_idx;
            dcache_tag_wr_data <= 0;
            cache_wb_after_cbo <= 1'b0;
            cache_cbo_done_r <= 1;
            cache_state <= CACHE_IDLE;
         end else if (cache_wb_after_flush) begin
            dcache_way0_tag_wr_en <= !cache_victim_way;
            dcache_way1_tag_wr_en <= cache_victim_way;
            dcache_tag_wr_idx <= cache_victim_idx;
            dcache_tag_wr_data <= 0;
            cache_wb_after_flush <= 1'b0;
            cache_flush_next_line();
         end else if (cache_wb_then_fill) begin
            cache_wb_then_fill <= 1'b0;
            cache_state <= CACHE_INVALIDATE;
         end else begin
            cache_state <= CACHE_FILL_REQ;
         end
      end
   endtask

   task vhpr_request_full_flush;
      input [7:0] reason;
      begin
         cache_flush_req <= ~cache_flush_ack;
         vhpr_epoch_bump_req <= ~vhpr_epoch_bump_ack;
      end
   endtask

   reg [63:0] bus_timeout_tval = 0;
   reg [11:0] bus_timeout_cause = 0;
   reg [63:0] mmio_timeout_tval = 0;

   // Memory transaction latency stats (cycles spent in the long-latency
   // fetch, D-memory, PTW, MMIO, and store-response wait states).
   reg [31:0] mig_latency_ctr = 0;
   reg        mig_prev_waiting = 0;
   reg [31:0] csr_mig_min    = 32'hFFFFFFFF;
   reg [31:0] csr_mig_max    = 0;
   reg [63:0] csr_mig_total  = 0;
   reg [63:0] csr_mig_count  = 0;
   reg [63:0] csr_mig_timeouts = 0;
   // First-timeout context latches (captured once per measurement window)
   reg [63:0] csr_mig_to_pc    = 0;
   reg [63:0] csr_mig_to_tval  = 0;
   reg [63:0] csr_mig_to_state = 0;
   reg [63:0] csr_mig_to_cause = 0;
   reg [63:0] csr_mig_to_addr  = 0;
   reg  [127:0] aligned;
   reg  [127:0] pte_latch = 0;    // registered copy of PTE data used in S_PTW_PROCESS
   // Svpbmt PBMT field is pte[62:61]: 00=PMA, 01=NC, 10=IO, 11=reserved. NC/IO
   // are uncacheable. Reserved/disabled cases page-fault before the success
   // branch, so in the success branch (pbmt != 0) == (bit62 | bit61).
   wire        ptw_pte_uncacheable = pte_latch[62] | pte_latch[61];
   reg  [63:0] imm_i, imm_j, imm_b, imm_u, imm_s, csr_arg, csr_read_val, csr_satp_write_val;
   reg  [63:0] int_commit_result = 0;
   reg  [ 4:0] int_commit_fflags = 0;
   reg         int_commit_prepared_fetch = 0;
   // Forces S_INT_COMMIT to redirect/refetch npc (epoch bump + frontend
   // flush) instead of accepting the prefetch. Set by a value-changing SATP
   // write so the new translation applies to the very next fetch.
   reg         int_commit_redirect_fetch = 0;
   reg  [63:0] c_imm12_8_109_6_7_2_11_53_x2;
   reg  [63:0] c_imm12_65_2_1110_43_x2;
   reg  [ 9:0] c_nzuimm107_1211_5_6_x4;
   reg  [63:0] c_imm12_62;
   reg  [63:0] c_imm12_43_5_2_6_x16;
   reg  [ 6:0] c_uimm5_1210_6_x4;
   reg  [ 8:0] c_uimm42_12_65_x8, c_uimm97_1210_x8;
   reg  [ 7:0] c_uimm32_12_64_x4, c_uimm87_129_x4, c_uimm65_1210_x8;
   reg  [31:0] sext32;
`ifdef SIMULATE
   reg  [127:0] tmp128;
`endif
   (* max_fanout = 16 *) reg [11:0] csrno;
   (* max_fanout = 32 *) reg [31:0] insn = 0; // XXX We should set this on reset
   reg  [ 1:0] csr_op;
   reg  [ 1:0] fcmp_result; // {nv, result} from fcmp_s/fcmp_d

   // CSR state
   reg         deleg, cause_intr;
   reg [63:0]  tval,
               tvec;
   reg [13:0]  csr_mie        = 0, // XXX We should set this on reset
               csr_mideleg    = 0,
               // temporary, will not turn into flop
               cause;
   reg [63:0]  csr_stvec      = 0,
               csr_scounteren = 0,
               csr_mcounteren = 0,
               csr_menvcfg    = 0,
               csr_mcountinhibit = 0,
               csr_mcyclecfg  = 0,
               csr_minstretcfg = 0,
               csr_sscratch   = 0,
               csr_sepc       = 0,
               csr_scause     = 0,
               csr_satp       = 0,
               csr_stval      = 0,
               csr_medeleg    = 0, // XXX We should set this on reset
               csr_mtvec      = 0, // XXX We should set this on reset
               csr_mscratch   = 'hDEADBEEFCAFEF00D,
               csr_mepc       = 0,
               csr_mcause     = 0,
               csr_mtval      = 0,
               csr_mcycle     = 0,
               csr_minstret   = -1; // because we increase it in fetch
   reg [7:0]   csr_senvcfg = 0;
   reg [63:0]  csr_mhpmcounter[0:`HPM_COUNTERS-1];
   reg [63:0]  csr_mhpmevent[0:`HPM_COUNTERS-1];
   reg [63:0]  csr_scountovf_read_val = 0;
   reg         hpm_counter_wr_en = 0;
   reg         hpm_event_wr_en = 0;
   reg [ 3:0]  hpm_wr_idx = 0;

   function [31:0] fetch_buf_pick_insn;
      input [127:0] data;
      input [3:0]   byte_offset;
      begin
         case (byte_offset[3:1])
           3'd0:    fetch_buf_pick_insn = data[31:0];
           3'd1:    fetch_buf_pick_insn = data[47:16];
           3'd2:    fetch_buf_pick_insn = data[63:32];
           3'd3:    fetch_buf_pick_insn = data[79:48];
           3'd4:    fetch_buf_pick_insn = data[95:64];
           3'd5:    fetch_buf_pick_insn = data[111:80];
           3'd6:    fetch_buf_pick_insn = data[127:96];
           default: fetch_buf_pick_insn = {16'd0, data[127:112]};
         endcase
      end
   endfunction

   function [63:0] frontend_fallthrough_pc;
      input [63:0] fetch_pc;
      input [31:0] fetch_insn;
      begin
         frontend_fallthrough_pc =
            fetch_pc + (fetch_insn[1:0] == 2'b11 ? 64'd4 : 64'd2);
      end
   endfunction

   reg [63:0]  hpm_wr_data = 0;
   integer     hpm_i, hpm_j;

   initial begin
      for (hpm_i = 0; hpm_i < `HPM_COUNTERS; hpm_i = hpm_i + 1) begin
         csr_mhpmcounter[hpm_i] = 0;
         csr_mhpmevent[hpm_i] = 0;
      end
   end

   always @(*) begin
      csr_scountovf_read_val = 0;
      for (hpm_j = 0; hpm_j < `HPM_COUNTERS; hpm_j = hpm_j + 1)
         csr_scountovf_read_val[hpm_j + 3] = csr_mhpmevent[hpm_j][`HPM_OF_BIT];
   end

   function counter_access_allowed;
      input [5:0] counter_idx;
      begin
         if (prv == 3)
           counter_access_allowed = 1;
         else if (prv == 1)
           counter_access_allowed = csr_mcounteren[counter_idx];
         else
           counter_access_allowed = csr_mcounteren[counter_idx] && csr_scounteren[counter_idx];
      end
   endfunction

   function hpm_mode_enabled;
      input [63:0] event_sel;
      begin
         case (prv)
           3: hpm_mode_enabled = !event_sel[62]; // MINH
           1: hpm_mode_enabled = !event_sel[61]; // SINH
           default: hpm_mode_enabled = !event_sel[60]; // UINH
         endcase
      end
   endfunction

   function [63:0] csr_modify_value;
      input [63:0] old_val;
      input [63:0] write_arg;
      input [ 1:0] op;
      begin
         case (op)
           `CSR_OP_COPY: csr_modify_value = write_arg;
           `CSR_OP_OR:   csr_modify_value = old_val | write_arg;
           `CSR_OP_ANDN: csr_modify_value = old_val & ~write_arg;
           default:      csr_modify_value = old_val;
         endcase
      end
   endfunction

   function [2:0] legal_frm_value;
      input [2:0] requested_frm;
      begin
         // frm is WARL; coerce reserved rounding modes to the reset mode.
         legal_frm_value = requested_frm <= 3'b100 ? requested_frm : 3'b000;
      end
   endfunction

   // fcsr: fflags[4:0] (NV|DZ|OF|UF|NX) + frm[2:0].  FP arithmetic and
   // conversions OR their IEEE exception flags into fflags as they retire.
   reg [ 4:0]  fflags = 0;
   reg [ 2:0]  frm = 0;

   // CLINT
   reg [63:0]  clint_mtime = 0;
   reg [13:0]  clint_mtime_clock_scaler = 0;
`ifdef VERILATOR_COSIM
   // One-cycle-delayed snapshot of clint_mtime. CSR TIME reads latch a value
   // that retires the cycle after; pass the delayed snapshot to simmerv so
   // its CSR TIME read returns the same value.
   reg [63:0]  clint_mtime_prev = 0;
   always @(posedge clock) clint_mtime_prev <= clint_mtime;
`endif
   reg [63:0]  clint_mtimecmp = ~0;
   reg         clint_msip = 0;

   // NS16550A UART state (at 0x10000000, IRQ 10 on PLIC)
   reg [7:0]   uart_ier = 0;        // Interrupt Enable Register
   reg         uart_fcr_fifo = 0;   // FCR bit 0: FIFO enable
   reg [7:0]   uart_lcr = 0;        // Line Control Register (DLAB = bit 7)
   reg [7:0]   uart_mcr = 0;        // Modem Control Register
   reg [7:0]   uart_scr = 0;        // Scratch Register
   localparam integer UART_FIFO_INDEX_BITS = 10;
   localparam integer UART_FIFO_DEPTH = 1 << UART_FIFO_INDEX_BITS;
   localparam [UART_FIFO_INDEX_BITS:0] UART_FIFO_DEPTH_COUNT =
      {1'b1, {UART_FIFO_INDEX_BITS{1'b0}}};
   (* ram_style = "block" *)
   reg [7:0]   uart_tx_fifo [0:UART_FIFO_DEPTH-1]; // TX FIFO between 16550 model and RS232
   reg [UART_FIFO_INDEX_BITS:0] uart_tx_head = 0, uart_tx_tail = 0;
   reg         uart_thre_pending = 0;
   (* rw_addr_collision = "yes" *)
   reg [7:0]   uart_rx_fifo [0:UART_FIFO_DEPTH-1]; // RX FIFO
   reg [UART_FIFO_INDEX_BITS:0] uart_rx_head = 0, uart_rx_tail = 0;
   reg [7:0]   uart_rx_front = 0;
   reg         uart_rx_front_valid = 0;
   reg         uart_rx_refill_pending = 0;
   reg [UART_FIFO_INDEX_BITS-1:0] uart_rx_refill_addr = 0;
   wire [UART_FIFO_INDEX_BITS:0] uart_tx_count = uart_tx_tail - uart_tx_head;
   wire        uart_tx_empty = uart_tx_head == uart_tx_tail;
   wire        uart_tx_full = uart_tx_count == UART_FIFO_DEPTH_COUNT;
   wire        uart_tx_accept = !uart_tx_full;
   wire        uart_tx_idle = uart_tx_empty && uart_tx_ready;
   wire [UART_FIFO_INDEX_BITS:0] uart_rx_count = uart_rx_tail - uart_rx_head;
   wire        uart_rx_empty = uart_rx_head == uart_rx_tail;
   wire        uart_rx_rbr_read = state == `S_LOCAL_LOAD &&
                                  phys_region(mem_addr) == `REGION_UART &&
                                  mem_addr[2:0] == 3'd0 && !uart_lcr[7];
   wire        uart_rx_pop = uart_rx_rbr_read && uart_rx_front_valid;
   wire        uart_rx_push = uart_rx_valid &&
                              (uart_rx_count < UART_FIFO_DEPTH_COUNT ||
                               uart_rx_pop);
   wire        uart_rx_ip = uart_ier[0] && uart_rx_front_valid;  // RX data available
   wire        uart_thre_ip = uart_ier[1] && uart_thre_pending;
   wire        uart_iir_thre = !uart_rx_ip && uart_thre_ip;
   // IIR: bit 0 = 0 means interrupt pending, 1 = no pending; bits [7:6] = FIFO status
   wire [7:0]  uart_iir = uart_rx_ip   ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h4} :
                          uart_iir_thre ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h2} :
                                         {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h1};
`ifdef PC_TRACE
   // Debug PC tracer (see tracer block below). When armed, SW sees a dummy
   // always-ready UART; the tracer owns the physical TX exclusively.
   reg        dbg_armed   = 0;
   reg [ 7:0] dbg_s_count = 0;
   reg        dbg_busy    = 0;
   reg [63:0] dbg_pc      = 0;
   reg [ 1:0] dbg_mode    = 0;
   reg        dbg_is_trap = 0;
   reg [ 5:0] dbg_cause   = 0;
   reg [ 5:0] dbg_pos     = 0;
   localparam integer DBG_N_RETIRE = 64;
   function [7:0] dbg_hex;
      input [3:0] n;
      dbg_hex = n < 10 ? 8'h30 + {4'd0, n} : 8'h57 + {4'd0, n};
   endfunction
   function [7:0] dbg_byte_at;
      input [ 5:0] pos;
      input [63:0] p;
      input [ 1:0] m;
      input        trap;
      input [ 5:0] cs;
      begin
         case (pos)
           6'd0:  dbg_byte_at = m == 2'd3 ? "M" : m == 2'd1 ? "S" : "U";
           6'd1:  dbg_byte_at = " ";
           6'd2:  dbg_byte_at = dbg_hex(p[63:60]);
           6'd3:  dbg_byte_at = dbg_hex(p[59:56]);
           6'd4:  dbg_byte_at = dbg_hex(p[55:52]);
           6'd5:  dbg_byte_at = dbg_hex(p[51:48]);
           6'd6:  dbg_byte_at = dbg_hex(p[47:44]);
           6'd7:  dbg_byte_at = dbg_hex(p[43:40]);
           6'd8:  dbg_byte_at = dbg_hex(p[39:36]);
           6'd9:  dbg_byte_at = dbg_hex(p[35:32]);
           6'd10: dbg_byte_at = dbg_hex(p[31:28]);
           6'd11: dbg_byte_at = dbg_hex(p[27:24]);
           6'd12: dbg_byte_at = dbg_hex(p[23:20]);
           6'd13: dbg_byte_at = dbg_hex(p[19:16]);
           6'd14: dbg_byte_at = dbg_hex(p[15:12]);
           6'd15: dbg_byte_at = dbg_hex(p[11: 8]);
           6'd16: dbg_byte_at = dbg_hex(p[ 7: 4]);
           6'd17: dbg_byte_at = dbg_hex(p[ 3: 0]);
           6'd18: dbg_byte_at = trap ? " " : "\n";
           6'd19: dbg_byte_at = "c";
           6'd20: dbg_byte_at = "=";
           6'd21: dbg_byte_at = dbg_hex({2'd0, cs[5:4]});
           6'd22: dbg_byte_at = dbg_hex(cs[3:0]);
           6'd23: dbg_byte_at = "\n";
           default: dbg_byte_at = "?";
         endcase
      end
   endfunction
   wire [7:0]  uart_lsr = dbg_armed
        ? {1'b0, 1'b1,           1'b1,           4'b0, uart_rx_front_valid}
        : {1'b0, uart_tx_idle,   uart_tx_accept, 4'b0, uart_rx_front_valid};
`else
   wire [7:0]  uart_lsr = {1'b0, uart_tx_idle, uart_tx_accept, 4'b0, uart_rx_front_valid}; // TEMT|THRE + DR
`endif
   wire        uart_irq_out = uart_rx_ip || uart_thre_ip;

   // PLIC (SiFive layout, base 0x0C000000)
   // Only S-mode context implemented (context 1)
   // plic_priority declared above (before initial block)
   reg [63:0]  plic_pending = 0;       // Interrupt pending bits
   reg [63:0]  plic_enabled = 0;       // Enable bits (S-mode context)
   reg [63:0]  plic_in_service = 0;    // Gateway has delivered source; wait for completion
   reg [ 2:0]  plic_threshold = 0;     // Priority threshold (S-mode)
   wire [63:0] plic_source_level = {ext_irq, 1'b0}
                                  | (uart_irq_out ? (64'd1 << 10) : 64'd0);

   // Highest-priority pending+enabled interrupt (2-cycle pipeline), arbitrated
   // in smolrv64_plic_arbiter. The core still owns the PLIC register state; the
   // arbiter only scans it. Flatten the per-source priority array for the port:
   // source i occupies plic_priority_flat[i*3 +: 3].
   wire [191:0] plic_priority_flat;
   genvar plic_pf;
   generate for (plic_pf = 0; plic_pf < 64; plic_pf = plic_pf + 1)
      assign plic_priority_flat[plic_pf*3 +: 3] = plic_priority[plic_pf];
   endgenerate
   wire [5:0] plic_best_irq;
   wire       plic_has_irq;
   smolrv64_plic_arbiter plic_arbiter_inst
     (.clock         (clock),
      .pending       (plic_pending),
      .enabled       (plic_enabled),
      .priority_flat (plic_priority_flat),
      .threshold     (plic_threshold),
      .best_irq      (plic_best_irq),
      .has_irq       (plic_has_irq));

   // MIP subfields
   // MEIP/SEIP driven by PLIC, MTIP/MSIP driven by CLINT
   reg         ueip = 0,
               lcofip = 0,
               stip_sw = 0, utip = 0,
               ssip = 0, usip = 0;
   wire        meip = plic_has_irq;
   wire        seip = plic_has_irq;
   reg         mtip = 0;
   always @(posedge clock) mtip <= clint_mtime >= clint_mtimecmp;
   wire        msip = clint_msip;

   // Sstc: when menvcfg.STCE is set, STIP is driven by the stimecmp comparator
   // (read-only to software); otherwise STIP keeps its software-written value.
   // The comparator is registered for the same timing reason as mtip.
   reg [63:0]  csr_stimecmp = ~0;
   reg         stip_stc = 0;
   always @(posedge clock) stip_stc <= clint_mtime >= csr_stimecmp;
   wire        stip = csr_menvcfg[63] ? stip_stc : stip_sw;

   wire [13:0] csr_mip = {lcofip, 1'd0, meip, 1'd0, seip, ueip,
                          mtip, 1'd0, stip, utip,
                          msip, 1'd0, ssip, usip};

   // MSTATUS subfields
   // Global interrupt-enable bits
   reg         uie = 0, sie = 0, mie = 0;
   // xPIE holds the value of the interrupt-enable bit active prior to the trap
   reg         upie = 0, spie = 0, mpie = 0;
   // xPP holds the previous privilege mode.
   reg         spp = 0;
   reg [  1:0] mpp;
   // FS encodes the status of the FP unit, including fcsr and FP regs
   reg [  1:0] fs = 0, xs = 0;
   // When MPRV=1, load and store memory addresses are translated and
   // protected as though the current privilege mode were set to MPP.
   reg         mprv = 0;
   // When SUM=0, S-mode memory accesses to pages that are accessible
   // by U-mode (U=1 in Figure 4.15) will fault.
   reg         sum;
   // MXR modifies the privilege with which loads access virtual
   // memory.  When 0, only loads from pages marked readable will
   // succeed.  When 1, loads from pages marked either readable or
   // executable (R=1 or X=1) will succeed.
   reg         mxr = 0;
   // Trap Virtual Memory, Timeout Wait, Trap SRET
   reg         tvm = 0, tw = 0, tsr = 0;
   wire [ 1:0] uxl = 2, sxl = 2; // 64-bit user and supervisor mode
   // "Some Dirty"
   wire        sd = fs == 3 || xs == 3;

   reg [ 63:0] mul_b;
   reg [127:0] mul_a, muldiv_p;
   reg [ 63:0] pre_mul_abs_s1, pre_mul_abs_s2;
   reg [ 31:0] pre_mul_abs_s1w, pre_mul_abs_s2w;
   reg         muldiv_output_sext32;
   reg         muldiv_output_negate;
   reg         muldiv_output_high_part;
   reg [3:0]   muldiv_start_op = `MULDIV_MUL;
   reg [6:0]   div_count;

   reg [63:0]  reservation = ~0;
   // Registered LR/SC reservation hit, computed at S_RF->S_EXECUTE edge
   // (reservation == s1_bram) so S_EXECUTE's SC branch doesn't have to do
   // the 64-bit compare in-flight with the mem_addr next-D mux.
   reg         reservation_match = 0;
   reg         do_atomic;
   reg         csr_access_failure = 0;

   reg [63:0]  store_value;

   // Sv39 page table walk state
   reg [ 1:0]  ptw_level;       // Current walk level (2, 1, 0)
   reg [ 4:0]  ptw_return;      // State to return to after translation
   reg [63:0]  ptw_va;          // Virtual address being translated
   reg [63:0]  ptw_pte_addr;    // Physical address of PTE being read
   reg [ 1:0]  ptw_access;      // 0=fetch, 1=load, 2=store, 3=AMO (R+W)
   reg [ 1:0]  ptw_prv;         // Effective privilege for permission check
   reg [63:0]  ptw_satp;        // SATP value used to launch the walk
   reg         ptw_sum;         // SUM value used for permission check
   reg         ptw_mxr;         // MXR value used for permission check
   reg         translated = 0;  // Set by PTW, cleared by consumer
   reg [11:0]  ptw_fault_cause; // Computed at top of S_PTW_PROCESS
   reg [15:0]  insn_half;       // Saved lower half for cross-page instruction fetch

   reg [`TLB_4K_ENTRIES-1:0] tlb_4k_valid = 0;
   reg [`TLB_2M_ENTRIES-1:0] tlb_2m_valid = 0;
   reg [`TLB_4K_INDEX_BITS-1:0] tlb_4k_rd_idx = 0;
   reg [`TLB_2M_INDEX_BITS-1:0] tlb_2m_rd_idx = 0;
   reg [`TLB_4K_INDEX_BITS-1:0] tlb_4k_wr_idx = 0;
   reg [`TLB_2M_INDEX_BITS-1:0] tlb_2m_wr_idx = 0;
   reg                         tlb_4k_wr_en = 0;
   reg                         tlb_2m_wr_en = 0;
   reg [TLB_4K_DATA_BITS-1:0]  tlb_4k_wr_data = 0;
   reg [TLB_2M_DATA_BITS-1:0]  tlb_2m_wr_data = 0;
   reg [ 1:0]                  tlb_insert_level = 0;
   reg [`TLB_4K_INDEX_BITS-1:0] tlb_insert_4k_idx = 0;
   reg [`TLB_2M_INDEX_BITS-1:0] tlb_insert_2m_idx = 0;
   reg [TLB_4K_DATA_BITS-1:0]  tlb_insert_4k_data = 0;
   reg [TLB_2M_DATA_BITS-1:0]  tlb_insert_2m_data = 0;
   reg [63:0]                  ptw_route_pa = 0;
   reg [CACHE_PERM_BITS-1:0]   ptw_route_perm = 0;
   reg [ 4:0]                  ptw_route_return = 0;
   wire [TLB_4K_DATA_BITS-1:0] tlb_4k_rd_data;
   wire [TLB_2M_DATA_BITS-1:0] tlb_2m_rd_data;
   reg [63:0]  tlb_req_va;
   reg [ 1:0]  tlb_req_access;
   reg [ 1:0]  tlb_req_prv;
   reg         tlb_req_sum;
   reg         tlb_req_mxr;
   reg [ 4:0]  tlb_req_return;
   reg         hpm_tlb_insert_4k_pulse = 0;
   reg         hpm_tlb_insert_2m_pulse = 0;
   reg         hpm_tlb_evict_4k_pulse = 0;
   reg         hpm_tlb_evict_2m_pulse = 0;
   reg         hpm_tlb_uncached_1g_pulse = 0;
   reg         hpm_tlb_uncached_napot_pulse = 0;
   reg         hpm_ptw_leaf_4k_pulse = 0;
   reg         hpm_ptw_leaf_2m_pulse = 0;
   reg         hpm_ptw_leaf_1g_pulse = 0;
   reg         hpm_ptw_leaf_napot_pulse = 0;

   // TLB index hashing + SATP field helpers.
   `include "smolrv64_tlb_helpers.vh"

   wire [TLB_CTX_BITS-1:0] tlb_req_ctx = {tlb_req_access, tlb_req_prv,
                                          tlb_req_sum, tlb_req_mxr};
   wire [TLB_SATP_KEY_BITS-1:0] tlb_current_satp_key =
      satp_tlb_key(csr_satp);
   wire [TLB_ASID_BITS-1:0] current_cache_asid =
      csr_satp[63:60] == 4'd8 ? csr_satp[53:44] : {TLB_ASID_BITS{1'b0}};

   function [TLB_ASID_BITS-1:0] fetch_asid_for_context;
      input [1:0] fetch_prv;
      begin
         fetch_asid_for_context =
            (csr_satp[63:60] == 4'd8 && fetch_prv != 2'd3) ?
            csr_satp[53:44] : {TLB_ASID_BITS{1'b0}};
      end
   endfunction

   wire [TLB_4K_TAG_BITS-1:0]    tlb_4k_rd_tag =
      tlb_4k_rd_data[TLB_4K_TAG_LSB +: TLB_4K_TAG_BITS];
   wire [TLB_4K_PBASE_BITS-1:0]  tlb_4k_rd_pbase =
      tlb_4k_rd_data[TLB_4K_PBASE_LSB +: TLB_4K_PBASE_BITS];
   wire [TLB_SATP_KEY_BITS-1:0]  tlb_4k_rd_satp_key =
      tlb_4k_rd_data[TLB_SATP_KEY_LSB +: TLB_SATP_KEY_BITS];
   wire [CACHE_PERM_BITS-1:0]    tlb_4k_rd_perm =
      tlb_4k_rd_data[TLB_PERM_LSB +: CACHE_PERM_BITS];
   wire [TLB_CTX_BITS-1:0]       tlb_4k_rd_ctx =
      tlb_4k_rd_data[TLB_CTX_LSB +: TLB_CTX_BITS];
   wire [TLB_2M_TAG_BITS-1:0]    tlb_2m_rd_tag =
      tlb_2m_rd_data[TLB_2M_TAG_LSB +: TLB_2M_TAG_BITS];
   wire [TLB_2M_PBASE_BITS-1:0]  tlb_2m_rd_pbase =
      tlb_2m_rd_data[TLB_2M_PBASE_LSB +: TLB_2M_PBASE_BITS];
   wire [TLB_SATP_KEY_BITS-1:0]  tlb_2m_rd_satp_key =
      tlb_2m_rd_data[TLB_SATP_KEY_LSB +: TLB_SATP_KEY_BITS];
   wire [CACHE_PERM_BITS-1:0]    tlb_2m_rd_perm =
      tlb_2m_rd_data[TLB_PERM_LSB +: CACHE_PERM_BITS];
   wire [TLB_CTX_BITS-1:0]       tlb_2m_rd_ctx =
      tlb_2m_rd_data[TLB_CTX_LSB +: TLB_CTX_BITS];
   wire tlb_4k_hit = state == `S_TLB_CHECK &&
                     tlb_4k_valid[tlb_4k_rd_idx] &&
                     tlb_4k_rd_tag == tlb_req_va[38:12] &&
                     tlb_4k_rd_satp_key == tlb_current_satp_key &&
                     tlb_4k_rd_ctx == tlb_req_ctx;
   wire tlb_2m_hit = state == `S_TLB_CHECK &&
                     tlb_2m_valid[tlb_2m_rd_idx] &&
                     tlb_2m_rd_tag == tlb_req_va[38:21] &&
                     tlb_2m_rd_satp_key == tlb_current_satp_key &&
                     tlb_2m_rd_ctx == tlb_req_ctx;
   wire hpm_tlb_lookup_pulse = state == `S_TLB_LOOKUP;
   wire hpm_tlb_hit_pulse = state == `S_TLB_CHECK && (tlb_4k_hit || tlb_2m_hit);
   wire hpm_tlb_miss_pulse = state == `S_TLB_CHECK && !(tlb_4k_hit || tlb_2m_hit);
   wire hpm_tlb_hit_4k_pulse = state == `S_TLB_CHECK && tlb_4k_hit;
   wire hpm_tlb_hit_2m_pulse = state == `S_TLB_CHECK && !tlb_4k_hit && tlb_2m_hit;

   smolrv64_sdpram #(
      .ADDR_WIDTH(`TLB_4K_INDEX_BITS),
      .DATA_WIDTH(TLB_4K_DATA_BITS)
   ) tlb_4k_ram (
      .clock   ( clock ),
      .rd_addr ( tlb_4k_rd_idx ),
      .rd_data ( tlb_4k_rd_data ),
      .wr_en   ( tlb_4k_wr_en ),
      .wr_addr ( tlb_4k_wr_idx ),
      .wr_data ( tlb_4k_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`TLB_2M_INDEX_BITS),
      .DATA_WIDTH(TLB_2M_DATA_BITS)
   ) tlb_2m_ram (
      .clock   ( clock ),
      .rd_addr ( tlb_2m_rd_idx ),
      .rd_data ( tlb_2m_rd_data ),
      .wr_en   ( tlb_2m_wr_en ),
      .wr_addr ( tlb_2m_wr_idx ),
      .wr_data ( tlb_2m_wr_data )
   );

   // Marks the S_FETCH1 cycle immediately after S_EXCEPTION. Used to
   // suppress a stale pre_intr_pending (sampled before S_EXCEPTION's
   // mie:=0 landed). Also used by VERILATOR_COSIM to skip the dummy
   // post-trap retire. Set in S_EXCEPTION, cleared in S_FETCH1.
   reg        just_trapped = 0;

   // Marks the S_FETCH1 cycle(s) of the instruction immediately after an
   // xRET. xRET can re-enable interrupts (via mie/sie <- mpie/spie) and
   // pre_intr_pending may catch that change in time to fire on the very
   // next S_FETCH1 — causing the xRET target to never retire. Spec allows
   // either behavior, but for cosim we pick "let the xRET target retire
   // first" uniformly, regardless of whether the interrupt was pending
   // before xRET or became pending after. Set in MRET/SRET, cleared in
   // S_FETCH1.
   reg        just_xret = 0;

`ifdef VERILATOR_COSIM
   import "DPI-C" function void cosim_retire(
       input longint unsigned pc,
       input longint unsigned next_pc,
       input int    unsigned insn,
       input byte   unsigned rd_kind,   // 0=none, 1=int, 2=fp
       input byte   unsigned rd_idx,
       input byte   unsigned prv,       // pre-retire privilege
       input byte   unsigned trapped,
       input int    unsigned fflags,
       input longint unsigned rd_val,
       input longint unsigned trap_cause,
       input longint unsigned trap_tval,
       input longint unsigned mtime,
       input longint unsigned mtimecmp,
       input longint unsigned mepc,
       input byte     unsigned seip
   );
`endif
   // Pre-retire / pre-trap privilege snapshots. Also used by PC_TRACE.
   reg [1:0]  prv_at_trap  = 0;  // pre-trap privilege, captured in S_EXCEPTION
   reg [1:0]  prv_retire   = 0;  // prv at instruction start (MRET/SRET change prv mid-execute)

   // IEEE 754 FCLASS: 10-bit one-hot classification (bit 0 = -inf, ..., bit 9 = qNaN).
   // Single-precision variant enforces NaN-boxing: unboxed value → canonical qNaN.
   // FP classify/compare helpers.
   `include "smolrv64_fp_ops.vh"

   function frontend_spec_fetch_state;
      input [5:0] s;
      begin
         case (s)
           `S_RF,
           `S_LOAD_ALIGN,
           `S_MMIO_ALIGN,
           `S_AMO,
           `S_STORE,
           `S_STORE_COMMIT,
           `S_MUL_RUNNING,
           `S_DIV_RUNNING,
           `S_EXECUTE,
           `S_EXECUTE2,
           `S_DMEM_LOAD_WAIT,
           `S_DMEM_LOAD2_WAIT,
           `S_DMEM_STORE_WAIT,
           `S_DMEM_STORE2,
           `S_DMEM_STORE_RESP_WAIT,
           `S_DMEM_STORE_RESP_ARM,
           `S_PTW_DIRECT_WAIT,
           `S_CBO_EXEC,
           `S_CBO_WAIT,
           `S_CVFPU_ISSUE,
           `S_CVFPU_WAIT,
           `S_MULDIV_START:
             frontend_spec_fetch_state = 1'b1;
           default:
             frontend_spec_fetch_state = 1'b0;
         endcase
      end
   endfunction

   function [2:0] phys_region;
      input [63:0] addr;
      begin
         if ((addr & 64'hffff_ffff_ffff_fff0) == 64'h0000_0000_1000_0000)
            phys_region = `REGION_UART;
         else if ((addr & 64'hffff_ffff_ffff_0000) == 64'h0000_0000_0200_0000)
            phys_region = `REGION_CLINT;
         else if ((addr & 64'hffff_ffff_ff00_0000) == 64'h0000_0000_0c00_0000)
            phys_region = `REGION_PLIC;
         else if (((addr ^ `MEM_BASEADDR) & (64'hffff_ffff_ffff_ffff << `MEM_SIZE_LG2)) == 0)
            phys_region = `REGION_BRAM;
         else if (addr[63:31] == 0)
            phys_region = `REGION_MMIO;
         else if (addr[63:31] == 1)
            phys_region = `REGION_DRAM;
         else
            phys_region = `REGION_ILLEGAL;
      end
   endfunction

   task flush_tlb;
      begin
         tlb_4k_valid <= 0;
         tlb_2m_valid <= 0;
`ifdef SIMULATE
         tlb_stat_entries_4k = 0;
         tlb_stat_entries_2m = 0;
         tlb_stat_entries_1g = 0;
`endif
      end
   endtask

   task clear_frontend_cmd;
      begin
         frontend_cmd_valid <= 0;
         frontend_cmd_fast_ready <= 0;
         frontend_cmd_speculative <= 0;
         frontend_cmd_spec_miss_ready <= 0;
      end
   endtask

   task arm_frontend_spec_cmd;
      begin
         frontend_cmd_valid <= 1;
         frontend_cmd_fast_ready <= 0;
         frontend_cmd_speculative <= 1;
         frontend_cmd_spec_miss_ready <= 0;
      end
   endtask

   task clear_frontend_fast_cmd;
      begin
         frontend_cmd_fast_ready <= 0;
      end
   endtask

   task clear_decode_queue;
      begin
         rf_decode_head <= 0;
         rf_decode_tail <= 0;
         rf_decode_count <= 0;
         rf_decode_prearmed <= 0;
         rf_decode_prearmed_head <= 0;
         rf_decode_prearm_block = 1;
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 0;
      end
   endtask

   task squash_decode_execute;
      begin
         clear_decode_queue();
         id_valid <= 0;
         id_rf_ready <= 0;
         execute_req_valid <= 0;
      end
   endtask

   task kill_frontend_lookup;
      begin
         f_state <= `F_IDLE;
         f_latched_hit <= 0;
      end
   endtask

   task flush_frontend_speculation;
      begin
         frontend_flush_this_cycle = 1;
         frontend_buf_flush <= 1'b1;
         clear_frontend_cmd();
         kill_frontend_lookup();
         frontend_redirect_valid <= 0;
         squash_decode_execute();
         frontend_miss_valid <= 0;
         frontend_miss_done <= 0;
         frontend_miss_next_valid <= 0;
      end
   endtask

   task trap_frontend_instruction_access_fault;
      begin
`ifdef SIMULATE
`ifdef VERBOSE
         $display("%05d   %1d %x illegal fetch address csr_satp[63:60] = %d",
                  $time, frontend_cmd_prv, frontend_cmd_pc, csr_satp[63:60]);
`endif
`endif
         cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
         tval = 0;
         clear_frontend_cmd();
         kill_frontend_lookup();
         squash_decode_execute();
         if (frontend_miss_valid || frontend_miss_done) begin
            frontend_miss_wait_action <= FRONTEND_MISS_WAIT_EXCEPTION;
            state <= `S_FRONTEND_MISS_WAIT;
         end else begin
            state <= `S_EXCEPTION;
         end
      end
   endtask

   task issue_ifetch_cache_read;
      input [27:0] read_addr;
      input [63:0] read_va;
      input [TLB_ASID_BITS-1:0] read_asid;
      input [CACHE_PERM_BITS-1:0] read_perm;
      input [TLB_CTX_BITS-1:0] read_ctx;
      begin
         cache_issue_dw_addr <= read_addr;
         cache_issue_va      <= read_va;
         cache_issue_asid    <= read_asid;
         cache_issue_perm    <= read_perm;
         cache_issue_ctx     <= read_ctx;
         ifetch_read         <= 1;
      end
   endtask

   task issue_dmem_cache_read;
      input [27:0] read_addr;
      input [63:0] read_va;
      input [TLB_ASID_BITS-1:0] read_asid;
      input [CACHE_PERM_BITS-1:0] read_perm;
      input [TLB_CTX_BITS-1:0] read_ctx;
      begin
         cache_issue_dw_addr <= read_addr;
         cache_issue_va      <= read_va;
         cache_issue_asid    <= read_asid;
         cache_issue_perm    <= read_perm;
         cache_issue_ctx     <= read_ctx;
         dmem_read           <= 1;
      end
   endtask

   task emit_dmem_load_rsp;
      input [63:0] rsp_data;
      input [63:0] rsp_next_data;
      input        rsp_next_valid;
      begin
         dmem_rsp_data_r <= rsp_data;
         dmem_rsp_next_data_r <= rsp_next_data;
         dmem_rsp_next_valid_r <= rsp_next_valid;
         dmem_rsp_valid_r <= 1;
      end
   endtask

   task start_instruction_fetch_miss;
      input [63:0] fetch_va;
      input [ 1:0] fetch_prv;
      begin
         clear_frontend_fast_cmd();
         frontend_cmd_spec_miss_ready <= 0;
         if (csr_satp[63:60] == 4'd8 && fetch_prv != 3) begin
            // Sv39 instruction fetch translation
            start_translation(fetch_va, 2'd0, fetch_prv, `S_FETCH2);
         end else begin
            // Both local BRAM and external memory fetches use the cache response
            // path.  The cache refill engine chooses BRAM or AXI by line address.
            fetch_from_ifetch_rsp <= 1;
            issue_ifetch_cache_read(fetch_va[30:3], fetch_va,
                                    {TLB_ASID_BITS{1'b0}},
                                    CACHE_PERM_PHYS,
                                    {2'd0, fetch_prv, sum, mxr});
            state            <= `S_IFETCH_WAIT;
         end
      end
   endtask

   task accept_instruction_fetch;
      input [63:0] accept_pc;
      input [63:0] accept_next_pc;
      input [63:0] accept_predicted_pc;
      input [31:0] accept_insn;
      input [ 1:0] accept_prv;
      input [FRONTEND_EPOCH_BITS-1:0] accept_epoch;
      input        accept_from_ifetch_rsp;
      begin
         pc <= accept_pc;
         insn <= accept_insn;
         fetch_from_ifetch_rsp <= accept_from_ifetch_rsp;
         translated <= 0;
         clear_frontend_cmd();
         frontend_redirect_valid <= 0;
         // (rf_decode_head/tail/count resets removed: stage_rf_decode_current
         //  in case 3 still resets the queue; case 1/2 slow paths no longer do.)
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 0;
         write_back_register <= 0;
         write_back_fp_valid <= 0;
         if (accept_pc[11:0] == 12'hFFE && accept_insn[1:0] == 2'b11 &&
             csr_satp[63:60] == 4'd8 && accept_prv != 3) begin
            insn_half <= accept_insn[15:0];
            start_translation(accept_pc + 64'd2, 2'd0, accept_prv, `S_FETCH2_HALF);
         end else if (accept_from_ifetch_rsp && accept_pc[2:1] == 2'b11) begin
            insn_half <= accept_insn[15:0];
            if (ifetch_latched_insn_valid) begin
               stage_rf_decode_current(accept_pc,
                                       accept_next_pc,
                                       accept_predicted_pc,
                                       accept_insn,
                                       accept_prv,
                                       accept_epoch,
                                       accept_from_ifetch_rsp);
            end else if (ifetch_latched_next_valid) begin
               ifetch_latched_half_data <= ifetch_latched_window[127:64];
               state <= `S_FETCH2_HALF;
            end else begin
               // For translated fetches, mem_addr still holds the physical
               // address only after a TLB translation.  A VHPR hit deliberately
               // avoids the TLB, so translate the second half instead of
               // deriving it from potentially stale mem_addr state.
               if (csr_satp[63:60] == 4'd8 && accept_prv != 3) begin
                  start_translation(accept_pc + 64'd2, 2'd0, accept_prv, `S_FETCH2_HALF);
               end else begin
                  issue_ifetch_cache_read(accept_pc[30:3] + 1,
                                          accept_pc + 64'd2,
                                          {TLB_ASID_BITS{1'b0}},
                                          CACHE_PERM_PHYS,
                                          {2'd0, accept_prv, sum, mxr});
                  state <= `S_IFETCH_HALF_WAIT;
               end
            end
         end else begin
            // Backend fetch-response path.  Launch directly into RF when the
            // backend boundary is empty; otherwise arbitrate against
            // pending/queue pressure:
            //   - pending empty, queue has room: enqueue directly
            //   - queue full: bail to S_FETCH1 so backend can pop
            //   - pending occupied, queue has room: drain fires this cycle,
            //     wait one cycle and try again from S_FETCH_BUF_USE
            if (!id_valid && !frontend_decode_pending_valid &&
                !rf_decode_valid && !rf_decode_enqueue_this_cycle) begin
               stage_rf_decode_current(accept_pc,
                                       accept_next_pc,
                                       accept_predicted_pc,
                                       accept_insn,
                                       accept_prv,
                                       accept_epoch,
                                       accept_from_ifetch_rsp);
            end else if (!frontend_decode_pending_latch_this_cycle &&
                         !frontend_decode_pending_valid && !rf_decode_full) begin
               enqueue_frontend_decode_hit(
                   accept_pc,
                   accept_next_pc,
                   accept_predicted_pc,
                   accept_insn,
                   accept_prv,
                   accept_epoch);
               state <= `S_FETCH1;
            end else if (rf_decode_full) begin
               state <= `S_FETCH1;
            end else begin
               state <= `S_FETCH_BUF_USE;
            end
         end
      end
   endtask

   task decode_rf_sources;
      input [31:0] decode_insn;
      output [ 4:0] decode_rd;
      output [ 4:0] decode_rs1;
      output [ 4:0] decode_rs2;
      output [ 5:0] decode_shamt;
      begin
         decode_rd = decode_insn`insn_rd;
         case (decode_insn[1:0])
           0: {decode_rs1,decode_rs2} = {{2'd1,decode_insn[9:7]}, {2'd1,decode_insn[4:2]}};
           1: {decode_rs1,decode_rs2} = {decode_insn[11:7],       {2'd1,decode_insn[4:2]}};
           2: {decode_rs1,decode_rs2} = {decode_insn[11:7],       decode_insn[6:2]};
           3: {decode_rs1,decode_rs2} = {decode_insn`insn_rs1,    decode_insn`insn_rs2};
         endcase
         // The exceptions
         if (decode_insn[1:0] == 1 && decode_insn[15])
           decode_rs1 = {2'd1,decode_insn[9:7]};
         if (decode_insn[1:0] == 2 && (decode_insn[15:13] == 3'b001 || decode_insn[15:14] == 2'b01))
           decode_rs1 = 2; // sp
         if (decode_insn[1:0] == 2 && 5 <= decode_insn[15:13])
           decode_rs1 = 2; // sp
         if ((decode_insn & 'he003) == 0)
           decode_rs1 = 2; // sp

         decode_shamt = decode_insn[25:20];
      end
   endtask

   task enqueue_rf_decode;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      input        decode_from_ifetch_rsp;
      reg   [ 4:0] decoded_rd;
      reg   [ 4:0] decoded_rs1;
      reg   [ 4:0] decoded_rs2;
      reg   [ 5:0] decoded_shamt;
      begin
         decode_rf_sources(decode_insn, decoded_rd, decoded_rs1,
                           decoded_rs2, decoded_shamt);
         if (rf_decode_enqueue_this_cycle) begin
`ifdef SIMULATE
            $display("%05d BUG: multiple rf_decode enqueues in one cycle", $time);
            $finish;
`endif
         end else if (rf_decode_full && !rf_decode_pop_this_cycle) begin
`ifdef SIMULATE
            $display("%05d BUG: enqueue into full rf_decode queue", $time);
            $finish;
`endif
         end else begin
            rf_decode_enqueue_this_cycle = 1'b1;
            rf_decode_pc_q[rf_decode_tail] <= decode_pc;
            rf_decode_next_pc_q[rf_decode_tail] <= decode_next_pc;
            rf_decode_predicted_pc_q[rf_decode_tail] <= decode_predicted_pc;
            rf_decode_insn_q[rf_decode_tail] <= decode_insn;
            rf_decode_prv_q[rf_decode_tail] <= decode_prv;
            rf_decode_epoch_q[rf_decode_tail] <= decode_epoch;
            rf_decode_from_ifetch_rsp_q[rf_decode_tail] <= decode_from_ifetch_rsp;
            rf_decode_rd_q[rf_decode_tail] <= decoded_rd;
            rf_decode_rs1_q[rf_decode_tail] <= decoded_rs1;
            rf_decode_rs2_q[rf_decode_tail] <= decoded_rs2;
            rf_decode_shamt_q[rf_decode_tail] <= decoded_shamt;
            rf_decode_tail <= rf_decode_tail + 1'b1;
            rf_decode_count <= rf_decode_pop_this_cycle ?
                               rf_decode_count : rf_decode_count + 1'b1;
         end
      end
   endtask

   function decode_enqueue_leaves_frontend_room;
      begin
         decode_enqueue_leaves_frontend_room =
            !((rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT &&
               rf_decode_pop_this_cycle) ||
              (rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT - 1'b1 &&
               !rf_decode_pop_this_cycle));
      end
   endfunction

   task advance_frontend_cmd_after_decode_accept;
      input [63:0] decode_predicted_pc;
      begin
         rf_decode_prearmed <= 0;
         rf_decode_prearmed_head <= 0;
         rf_decode_prearm_block = 1;
         frontend_cmd_pc <= decode_predicted_pc;
         clear_frontend_fast_cmd();
         frontend_cmd_spec_miss_ready <= 0;
         if (!decode_enqueue_leaves_frontend_room()) begin
            frontend_cmd_valid <= 0;
            frontend_cmd_speculative <= 0;
         end else begin
            frontend_cmd_valid <= 1;
            frontend_cmd_speculative <= 1;
         end
      end
   endtask

   task latch_frontend_decode_pending;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      begin
         if (frontend_decode_pending_latch_this_cycle) begin
`ifdef SIMULATE
            $display("%05d BUG: multiple frontend decode pending latches in one cycle", $time);
            $finish;
`endif
         end else begin
            frontend_decode_pending_latch_this_cycle = 1'b1;
            frontend_decode_pending_valid <= 1;
            frontend_decode_pending_pc <= decode_pc;
            frontend_decode_pending_next_pc <= decode_next_pc;
            frontend_decode_pending_predicted_pc <= decode_predicted_pc;
            frontend_decode_pending_insn <= decode_insn;
            frontend_decode_pending_prv <= decode_prv;
            frontend_decode_pending_epoch <= decode_epoch;
            advance_frontend_cmd_after_decode_accept(decode_predicted_pc);
         end
      end
   endtask

   task enqueue_frontend_decode_pending;
      begin
         enqueue_rf_decode(frontend_decode_pending_pc,
                           frontend_decode_pending_next_pc,
                           frontend_decode_pending_predicted_pc,
                           frontend_decode_pending_insn,
                           frontend_decode_pending_prv,
                           frontend_decode_pending_epoch,
                           1'b0);
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 1'b1;
      end
   endtask

   task enqueue_frontend_decode_hit;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      begin
         enqueue_rf_decode(decode_pc, decode_next_pc, decode_predicted_pc,
                           decode_insn, decode_prv, decode_epoch, 1'b0);
         advance_frontend_cmd_after_decode_accept(decode_predicted_pc);
      end
   endtask

   task stage_rf_decode_current;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      input        decode_from_ifetch_rsp;
      reg   [ 4:0] decoded_rd;
      reg   [ 4:0] decoded_rs1;
      reg   [ 4:0] decoded_rs2;
      reg   [ 5:0] decoded_shamt;
      begin
         if (id_valid) begin
`ifdef SIMULATE
            $display("%05d BUG: stage current decode with busy ID stage", $time);
            $finish;
`endif
            state <= `S_FETCH1;
         end else begin
            decode_rf_sources(decode_insn, decoded_rd, decoded_rs1,
                              decoded_rs2, decoded_shamt);
            write_back_register = 0;
            write_back_fp_valid = 0;
            rf_decode_head <= 0;
            rf_decode_tail <= 0;
            rf_decode_count <= 0;
            rf_decode_prearmed <= 0;
            rf_decode_prearmed_head <= 0;
            rf_decode_prearm_block = 1;
            frontend_decode_pending_valid <= 0;
            frontend_decode_pending_drain = 0;
            id_valid <= 1;
            id_rf_ready <= 0;
            id_pc <= decode_pc;
            id_next_pc <= decode_next_pc;
            id_predicted_pc <= decode_predicted_pc;
            id_prv <= decode_prv;
            id_epoch <= decode_epoch;
            id_insn <= decode_insn;
            id_rd <= decoded_rd;
            id_rs1 <= decoded_rs1;
            id_rs2 <= decoded_rs2;
            id_shamt <= decoded_shamt;
            rs1 <= decoded_rs1;
            rs2 <= decoded_rs2;
            rs3 <= decode_insn[31:27];
            frontend_cmd_pc <= decode_predicted_pc;
            frontend_cmd_prv <= decode_prv;
            arm_frontend_spec_cmd();
            state <= `S_RF;
         end
      end
   endtask

   task enqueue_rf_decode_speculative;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      begin
         enqueue_rf_decode(decode_pc, decode_next_pc, decode_predicted_pc,
                           decode_insn, decode_prv,
                           decode_epoch, 1'b0);
      end
   endtask

   task load_id_from_rf_decode_head;
      begin
         id_valid <= 1;
         id_rf_ready <= rf_decode_prearmed_current;
         id_pc <= rf_decode_pc;
         id_next_pc <= rf_decode_next_pc;
         id_predicted_pc <= rf_decode_predicted_pc;
         id_prv <= rf_decode_prv;
         id_epoch <= rf_decode_epoch;
         id_insn <= rf_decode_insn;
         id_rd <= rf_decode_rd;
         id_rs1 <= rf_decode_rs1;
         id_rs2 <= rf_decode_rs2;
         id_shamt <= rf_decode_shamt;
         rs1 <= rf_decode_rs1;
         rs2 <= rf_decode_rs2;
         rs3 <= rf_decode_insn[31:27];
      end
   endtask

   task pop_rf_decode_head;
      begin
`ifdef SIMULATE
         if (rf_decode_pop_this_cycle) begin
            $display("%05d BUG: multiple rf_decode pops in one cycle", $time);
            $finish;
         end else if (!rf_decode_valid) begin
            $display("%05d BUG: pop empty rf_decode queue", $time);
            $finish;
         end
`endif
         rf_decode_pop_this_cycle = 1'b1;
         rf_decode_head <= rf_decode_head + 1'b1;
         rf_decode_count <= rf_decode_enqueue_this_cycle ?
                            rf_decode_count : rf_decode_count - 1'b1;
         rf_decode_prearmed <= 0;
         rf_decode_prearmed_head <= 0;
         rf_decode_prearm_block = 1;
         if (rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT ||
             rf_decode_enqueue_this_cycle) begin
            arm_frontend_spec_cmd();
         end
      end
   endtask

   task prearm_rf_decode_head;
      begin
         rs1 <= rf_decode_rs1;
         rs2 <= rf_decode_rs2;
         rs3 <= rf_decode_insn[31:27];
         rf_decode_prearmed <= 1;
         rf_decode_prearmed_head <= rf_decode_head;
      end
   endtask

   task launch_rf_decode_read;
      begin
         if (id_valid) begin
`ifdef SIMULATE
            $display("%05d BUG: launch into busy ID stage", $time);
            $finish;
`endif
            state <= `S_FETCH1;
         end else begin
            load_id_from_rf_decode_head();
            pop_rf_decode_head();
            state <= `S_RF;
         end
      end
   endtask

   task launch_rf_decode_read_preserve_state;
      begin
         if (id_valid) begin
`ifdef SIMULATE
            $display("%05d BUG: background launch into busy ID stage", $time);
            $finish;
`endif
         end else begin
            load_id_from_rf_decode_head();
            pop_rf_decode_head();
         end
      end
   endtask

   task prepare_branch_metadata;
      input [63:0] b_pc;
      input [63:0] b_next_pc;
      input [31:0] b_insn;
      input [63:0] b_s1;
      input [63:0] b_s2;
      reg [63:0] d_imm_i, d_imm_j, d_imm_b, d_c_j, d_c_b;
      begin
         d_imm_i = {{52{b_insn[31]}}, b_insn[31:20]};
         d_imm_j = {{44{b_insn[31]}}, b_insn[19:12], b_insn[20], b_insn[30:21], 1'b0};
         d_imm_b = {{52{b_insn[31]}}, b_insn[7], b_insn[30:25], b_insn[11:8], 1'b0};
         d_c_j   = {{53{b_insn[12]}}, b_insn[8], b_insn[10:9], b_insn[6], b_insn[7],
                    b_insn[2], b_insn[11], b_insn[5:3], 1'b0};
         d_c_b   = {{56{b_insn[12]}}, b_insn[6:5], b_insn[2], b_insn[11:10],
                    b_insn[4:3], 1'b0};

         pre_npc <= b_next_pc;
         pre_jalr_target <= (b_s1 + d_imm_i) & ~64'd1;
         pre_branch_target <= b_pc + d_imm_b;
         pre_branch_taken <= 0;

         if ((b_insn & 'he003) == 'ha001) begin // C.J
            pre_npc <= b_pc + d_c_j;
         end else if ((b_insn & 'he003) == 'hc001) begin // C.BEQZ
            pre_branch_target <= b_pc + d_c_b;
            pre_branch_taken <= b_s1 == 0;
         end else if ((b_insn & 'he003) == 'he001) begin // C.BNEZ
            pre_branch_target <= b_pc + d_c_b;
            pre_branch_taken <= b_s1 != 0;
         end else if ((b_insn & 'hf07f) == 'h8002) begin // C.JR
            pre_jalr_target <= b_s1 & ~64'd1;
         end else if ((b_insn & 'hf07f) == 'h9002) begin // C.JALR
            pre_jalr_target <= b_s1 & ~64'd1;
         end else if ((b_insn & 'h0000007f) == 'h0000006f) begin // JAL
            pre_npc <= b_pc + d_imm_j;
         end else if ((b_insn & 'h0000707f) == 'h00000067) begin // JALR
            pre_jalr_target <= (b_s1 + d_imm_i) & ~64'd1;
         end else if ((b_insn & 'h0000707f) == 'h00000063) begin // BEQ
            pre_branch_taken <= b_s1 == b_s2;
         end else if ((b_insn & 'h0000707f) == 'h00001063) begin // BNE
            pre_branch_taken <= b_s1 != b_s2;
         end else if ((b_insn & 'h0000707f) == 'h00004063) begin // BLT
            pre_branch_taken <= $signed(b_s1) < $signed(b_s2);
         end else if ((b_insn & 'h0000707f) == 'h00005063) begin // BGE
            pre_branch_taken <= $signed(b_s1) >= $signed(b_s2);
         end else if ((b_insn & 'h0000707f) == 'h00006063) begin // BLTU
            pre_branch_taken <= b_s1 < b_s2;
         end else if ((b_insn & 'h0000707f) == 'h00007063) begin // BGEU
            pre_branch_taken <= b_s1 >= b_s2;
         end
      end
   endtask


   function id_no_pending_wb_hazard;
      input       pending_int_valid;
      input [4:0] pending_int_rd;
      input       pending_fp_valid;
      input [4:0] pending_fp_rd;
      reg         id_uses_fp_rs3;
      begin
         id_uses_fp_rs3 = id_insn[6:4] == 3'b100 && id_insn[1:0] == 2'b11;
         id_no_pending_wb_hazard =
            (!pending_int_valid || pending_int_rd == 0 ||
             (id_rs1 != pending_int_rd && id_rs2 != pending_int_rd)) &&
            (!pending_fp_valid ||
             (id_rs1 != pending_fp_rd &&
              id_rs2 != pending_fp_rd &&
              (!id_uses_fp_rs3 || id_insn[31:27] != pending_fp_rd)));
      end
   endfunction

   // align_dmem_load_value lives in smolrv64_mem_align.vh (included above).

   task prepare_execute_req_from_id;
      input preserve_state;
      begin
`ifdef SIMULATE
           if (execute_req_valid) begin
              $display("%05d BUG: overwrite busy execute request", $time);
              $finish;
           end
`endif
           // Register RF output into execute_req_rs1_value/execute_req_rs2_value/execute_req_frs1_value/execute_req_frs2_value flip-flops.  Early launch can
           // overlap this read with the previous retire's writeback, so use the
           // local writeback bypass before latching operands.
           execute_req_rs1_value <= id_rf_rs1_value;
           execute_req_rs2_value <= id_rf_rs2_value;
           execute_req_frs1_value <= id_rf_frs1_value;
           execute_req_frs2_value <= id_rf_frs2_value;
           execute_req_frs3_value <= id_rf_frs3_value;
           // Pre-compute SC reservation match one cycle early; S_EXECUTE's
           // SC branch then only sees a 1-bit registered hit.
           reservation_match <= (reservation == id_rf_rs1_value);
           pre_mul_abs_s1  <= id_rf_rs1_value[63] ? -id_rf_rs1_value : id_rf_rs1_value;
           pre_mul_abs_s2  <= id_rf_rs2_value[63] ? -id_rf_rs2_value : id_rf_rs2_value;
           pre_mul_abs_s1w <= id_rf_rs1_value[31] ? -id_rf_rs1_value[31:0] : id_rf_rs1_value[31:0];
           pre_mul_abs_s2w <= id_rf_rs2_value[31] ? -id_rf_rs2_value[31:0] : id_rf_rs2_value[31:0];
           id_valid <= 0;
           id_rf_ready <= 0;
           execute_req_pc <= id_rf_pc;
           execute_req_next_pc <= id_rf_next_pc;
           execute_req_predicted_pc <= id_rf_predicted_pc;
           execute_req_prv <= id_rf_prv;
           execute_req_epoch <= id_rf_epoch;
           execute_req_insn <= id_rf_insn;
           execute_req_rd <= id_rf_rd;
           execute_req_rs1 <= id_rf_rs1;
           execute_req_rs2 <= id_rf_rs2;
           execute_req_shamt <= id_rf_shamt;
           execute_req_valid <= 1;
           prepare_branch_metadata(id_rf_pc, id_rf_next_pc, id_rf_insn,
                                   id_rf_rs1_value, id_rf_rs2_value);
           if (!preserve_state)
              state <= `S_EXECUTE;

           // Pre-decode ALU operation and second operand for S_EXECUTE.
           // id_rf_insn/id_rf_pc are registered FFs; id_rf_rs2_value is read data
           // after same-cycle writeback bypass.
           // All assignments use <= so they register into execute_req_alu_op/execute_req_alu_b/execute_req_alu_sxt.
           // Immediates are computed inline (1-3 LUT from insn_reg) rather than read from
           // the imm_i/imm_u registers (which are only updated with = inside S_EXECUTE).
           begin : id_rf_pre_decode
              reg [63:0] d_imm_i, d_imm_u, d_c_imm;

              d_imm_i = {{52{id_rf_insn[31]}},id_rf_insn[31:20]};
              d_imm_u = {{32{id_rf_insn[31]}},id_rf_insn[31:12],12'd0};
              d_c_imm = {{59{id_rf_insn[12]}},id_rf_insn[6:2]};  // c_imm12_62

              // Default: harmless value (only matters for instructions reaching S_EXECUTE2)
              execute_req_alu_op  <= `EXOP_OPB;
              execute_req_alu_b   <= 64'd0;
              execute_req_alu_sxt <= 0;

              // ---- Compressed instructions (id_rf_insn[1:0] != 2'b11) ----

              // Quadrant 0
              if ((id_rf_insn & 'he003) == 'h0000) begin // C.ADDI4SPN (rd'=rs2)
                 execute_req_alu_op <= `EXOP_ADD;
                 execute_req_alu_b  <= {54'd0, id_rf_insn[10:7], id_rf_insn[12:11], id_rf_insn[5], id_rf_insn[6], 2'd0};
              end

              // Quadrant 1
              else if ((id_rf_insn & 'he003) == 'h0001) begin // C.ADDI / C.NOP
                 execute_req_alu_op <= `EXOP_ADD;
                 execute_req_alu_b  <= d_c_imm;
              end
              else if ((id_rf_insn & 'he003) == 'h2001) begin // C.ADDIW (RV64)
                 execute_req_alu_op  <= `EXOP_ADD;
                 execute_req_alu_b   <= d_c_imm;
                 execute_req_alu_sxt <= 1;
              end
              else if ((id_rf_insn & 'he003) == 'h4001) begin // C.LI
                 execute_req_alu_op <= `EXOP_OPB;
                 execute_req_alu_b  <= d_c_imm;
              end
              else if ((id_rf_insn & 'hef83) == 'h6101) begin // C.ADDI16SP (rd=sp)
                 execute_req_alu_op <= `EXOP_ADD;
                 execute_req_alu_b  <= {{55{id_rf_insn[12]}}, id_rf_insn[4:3], id_rf_insn[5], id_rf_insn[2], id_rf_insn[6], 4'd0};
              end
              else if ((id_rf_insn & 'he003) == 'h6001) begin // C.LUI (rd!=0,2)
                 execute_req_alu_op <= `EXOP_OPB;
                 execute_req_alu_b  <= {{47{id_rf_insn[12]}}, id_rf_insn[6:2], 12'd0};
              end
              else if ((id_rf_insn & 'hec03) == 'h8001) begin // C.SRLI
                 execute_req_alu_op <= `EXOP_SHR;
                 execute_req_alu_b  <= d_c_imm;
              end
              else if ((id_rf_insn & 'hec03) == 'h8401) begin // C.SRAI
                 execute_req_alu_op <= `EXOP_SAR;
                 execute_req_alu_b  <= d_c_imm;
              end
              else if ((id_rf_insn & 'hec03) == 'h8801) begin // C.ANDI
                 execute_req_alu_op <= `EXOP_AND;
                 execute_req_alu_b  <= d_c_imm;
              end
              else if ((id_rf_insn & 'hfc63) == 'h8c01) begin // C.SUB
                 execute_req_alu_op <= `EXOP_SUB;
                 execute_req_alu_b  <= id_rf_rs2_value;
              end
              else if ((id_rf_insn & 'hfc63) == 'h8c21) begin // C.XOR
                 execute_req_alu_op <= `EXOP_XOR;
                 execute_req_alu_b  <= id_rf_rs2_value;
              end
              else if ((id_rf_insn & 'hfc63) == 'h8c41) begin // C.OR
                 execute_req_alu_op <= `EXOP_OR;
                 execute_req_alu_b  <= id_rf_rs2_value;
              end
              else if ((id_rf_insn & 'hfc63) == 'h8c61) begin // C.AND
                 execute_req_alu_op <= `EXOP_AND;
                 execute_req_alu_b  <= id_rf_rs2_value;
              end
              else if ((id_rf_insn & 'hfc63) == 'h9c01) begin // C.SUBW
                 execute_req_alu_op  <= `EXOP_SUB;
                 execute_req_alu_b   <= id_rf_rs2_value;
                 execute_req_alu_sxt <= 1;
              end
              else if ((id_rf_insn & 'hfc63) == 'h9c21) begin // C.ADDW
                 execute_req_alu_op  <= `EXOP_ADD;
                 execute_req_alu_b   <= id_rf_rs2_value;
                 execute_req_alu_sxt <= 1;
              end

              // Quadrant 2
              else if ((id_rf_insn & 'he003) == 'h0002) begin // C.SLLI
                 execute_req_alu_op <= `EXOP_SHL;
                 execute_req_alu_b  <= d_c_imm;
              end
              else if ((id_rf_insn & 'hf07f) == 'h8002) begin // C.JR (no exe_add, default ok)
                 ;
              end
              else if ((id_rf_insn & 'hf003) == 'h8002) begin // C.MV
                 execute_req_alu_op <= `EXOP_OPB;
                 execute_req_alu_b  <= id_rf_rs2_value;
              end
              else if ((id_rf_insn & 'hf07f) == 'h9002) begin // C.JALR (link = id_rf_pc+2)
                 execute_req_alu_op <= `EXOP_OPB;
                 execute_req_alu_b  <= id_rf_next_pc;
              end
              else if ((id_rf_insn & 'hf003) == 'h9002) begin // C.ADD
                 execute_req_alu_op <= `EXOP_ADD;
                 execute_req_alu_b  <= id_rf_rs2_value;
              end

              // ---- 32-bit instructions (id_rf_insn[1:0] == 2'b11) ----
              else if (id_rf_insn[1:0] == 2'b11) begin
                 case (id_rf_insn[6:2])
                    5'b01101: begin // LUI
                       execute_req_alu_op <= `EXOP_OPB;
                       execute_req_alu_b  <= d_imm_u;
                    end
                    5'b00101: begin // AUIPC
                       execute_req_alu_op <= `EXOP_OPB;
                       execute_req_alu_b  <= id_rf_pc + d_imm_u;
                    end
                    5'b11011: begin // JAL (link = id_rf_pc+4)
                       execute_req_alu_op <= `EXOP_OPB;
                       execute_req_alu_b  <= id_rf_next_pc;
                    end
                    5'b11001: begin // JALR (link = id_rf_pc+4)
                       execute_req_alu_op <= `EXOP_OPB;
                       execute_req_alu_b  <= id_rf_next_pc;
                    end
                    5'b00100: begin // OP-IMM: funct3 selects operation
                       execute_req_alu_b <= d_imm_i; // default; shifts override below
                       case (id_rf_insn[14:12])
                          3'b000: execute_req_alu_op <= `EXOP_ADD;   // ADDI
                          3'b001: begin execute_req_alu_op <= `EXOP_SHL; execute_req_alu_b <= {58'd0, id_rf_insn[25:20]}; end  // SLLI
                          3'b010: execute_req_alu_op <= `EXOP_LTS;   // SLTI
                          3'b011: execute_req_alu_op <= `EXOP_LTU;   // SLTIU
                          3'b100: execute_req_alu_op <= `EXOP_XOR;   // XORI
                          3'b101: begin // SRLI / SRAI
                             execute_req_alu_op <= id_rf_insn[30] ? `EXOP_SAR : `EXOP_SHR;
                             execute_req_alu_b  <= {58'd0, id_rf_insn[25:20]};
                          end
                          3'b110: execute_req_alu_op <= `EXOP_OR;    // ORI
                          3'b111: execute_req_alu_op <= `EXOP_AND;   // ANDI
                       endcase
                    end
                    5'b01100: begin // OP-REG: funct3+funct7[5] selects operation
                       execute_req_alu_b <= id_rf_rs2_value;
                       case (id_rf_insn[14:12])
                          3'b000: execute_req_alu_op <= id_rf_insn[30] ? `EXOP_SUB : `EXOP_ADD;  // ADD/SUB
                          3'b001: execute_req_alu_op <= `EXOP_SHL;  // SLL
                          3'b010: execute_req_alu_op <= `EXOP_LTS;  // SLT
                          3'b011: execute_req_alu_op <= `EXOP_LTU;  // SLTU
                          3'b100: execute_req_alu_op <= `EXOP_XOR;  // XOR
                          3'b101: execute_req_alu_op <= id_rf_insn[30] ? `EXOP_SAR : `EXOP_SHR;  // SRL/SRA
                          3'b110: execute_req_alu_op <= `EXOP_OR;   // OR
                          3'b111: execute_req_alu_op <= `EXOP_AND;  // AND
                          // MUL/DIV (funct7[0]=1): exe_add unused; default EXOP_OPB is fine
                       endcase
                    end
                    5'b00110: begin // OP-IMM-32 (W-type immediates)
                       execute_req_alu_sxt <= 1;
                       case (id_rf_insn[14:12])
                          3'b000: begin execute_req_alu_op <= `EXOP_ADD; execute_req_alu_b <= d_imm_i; end  // ADDIW
                          3'b001: begin execute_req_alu_op <= `EXOP_SHL; execute_req_alu_b <= {59'd0, id_rf_insn[24:20]}; end  // SLLIW
                          3'b101: begin  // SRLIW / SRAIW
                             execute_req_alu_op <= id_rf_insn[30] ? `EXOP_SAR : `EXOP_SHR;
                             execute_req_alu_b  <= {59'd0, id_rf_insn[24:20]};
                          end
                          default: ; // other funct3: no exe_add
                       endcase
                    end
                    5'b01110: begin // OP-REG-32 (W-type register)
                       execute_req_alu_sxt <= 1;
                       execute_req_alu_b <= id_rf_rs2_value;
                       case (id_rf_insn[14:12])
                          3'b000: execute_req_alu_op <= id_rf_insn[30] ? `EXOP_SUB : `EXOP_ADD;  // ADDW/SUBW
                          3'b001: execute_req_alu_op <= `EXOP_SHL;  // SLLW
                          3'b101: execute_req_alu_op <= id_rf_insn[30] ? `EXOP_SAR : `EXOP_SHR;  // SRLW/SRAW
                          // MUL/DIV-W: exe_add unused
                          default: ;
                       endcase
                    end
                    5'b01011: begin // AMO — SC.W/D fail path writes exe_add = 1
                       execute_req_alu_op <= `EXOP_ONE;
                    end
                    default: ; // LOAD, STORE, BRANCH, CSR, etc.: exe_add unused
                 endcase
              end
           end // id_rf_pre_decode

           pre_fp_rnd_mode <= id_rf_insn[14:12] == 3'b111 ? frm : id_rf_insn[14:12];
           pre_fp_rmode_ok <= !(id_rf_insn[14:12] == 3'b101 || id_rf_insn[14:12] == 3'b110 ||
                                (id_rf_insn[14:12] == 3'b111 && frm > 3'b100));

           // Mem pre-decode: compute offset/size/op/mask/wb-reg one cycle
           // early so S_EXECUTE can share a single execute_req_rs1_value+offset adder instead of
           // selecting between 22 parallel adders. Immediates are computed
           // inline from id_rf_insn bits (the imm_*/c_uimm* registers are written
           // in S_EXECUTE and therefore stale here).
           begin : id_rf_mem_decode
              reg [63:0] d_imm_i_s, d_imm_s_s;
              reg [63:0] d_clw_off, d_cld_off, d_clwsp_off, d_cldsp_off,
                         d_cswsp_off, d_csdsp_off;

              d_imm_i_s    = {{52{id_rf_insn[31]}}, id_rf_insn[31:20]};
              d_imm_s_s    = {{52{id_rf_insn[31]}}, id_rf_insn[31:25], id_rf_insn[11:7]};
              d_clw_off    = {57'd0, id_rf_insn[5],    id_rf_insn[12:10], id_rf_insn[6],     2'd0};
              d_cld_off    = {56'd0, id_rf_insn[6:5],  id_rf_insn[12:10],              3'd0};
              d_clwsp_off  = {56'd0, id_rf_insn[3:2],  id_rf_insn[12],    id_rf_insn[6:4],   2'd0};
              d_cldsp_off  = {55'd0, id_rf_insn[4:2],  id_rf_insn[12],    id_rf_insn[6:5],   3'd0};
              d_cswsp_off  = {56'd0, id_rf_insn[8:7],  id_rf_insn[12:9],               2'd0};
              d_csdsp_off  = {55'd0, id_rf_insn[9:7],  id_rf_insn[12:10],              3'd0};

              // Defaults: non-mem instruction
              execute_req_mem_op        <= `MEMOP_NONE;
              execute_req_mem_offset    <= 64'd0;
              execute_req_load_size_lg2 <= 3'd0;
              execute_req_mem_wr_mask   <= 8'd0;
              execute_req_mem_wb_reg    <= 5'd0;
              execute_req_mem_fp        <= 1'b0;
              execute_req_cbo_zero      <= 1'b0;

              // Compressed loads / stores (quadrants 0 & 2)
              if ((id_rf_insn & 'he003) == 'h4000) begin // C.LW
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_clw_off;
                 execute_req_load_size_lg2 <= 3'b110; // W, sign-extend
                 execute_req_mem_wb_reg    <= {2'b01, id_rf_insn[4:2]};
              end
              else if ((id_rf_insn & 'he003) == 'h2000) begin // C.FLD
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_cld_off;
                 execute_req_load_size_lg2 <= 3'b011; // D
                 execute_req_mem_wb_reg    <= {2'b01, id_rf_insn[4:2]};
                 execute_req_mem_fp        <= 1'b1;
              end
              else if ((id_rf_insn & 'he003) == 'h6000) begin // C.LD
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_cld_off;
                 execute_req_load_size_lg2 <= 3'b011; // D
                 execute_req_mem_wb_reg    <= {2'b01, id_rf_insn[4:2]};
              end
              else if ((id_rf_insn & 'he003) == 'hc000) begin // C.SW
                 execute_req_mem_op      <= `MEMOP_STORE;
                 execute_req_mem_offset  <= d_clw_off;
                 execute_req_mem_wr_mask <= 8'h0f;
              end
              else if ((id_rf_insn & 'he003) == 'ha000) begin // C.FSD
                 execute_req_mem_op      <= `MEMOP_STORE;
                 execute_req_mem_offset  <= d_cld_off;
                 execute_req_mem_wr_mask <= 8'hff;
                 execute_req_mem_fp      <= 1'b1;
              end
              else if ((id_rf_insn & 'he003) == 'he000) begin // C.SD
                 execute_req_mem_op      <= `MEMOP_STORE;
                 execute_req_mem_offset  <= d_cld_off;
                 execute_req_mem_wr_mask <= 8'hff;
              end
              else if ((id_rf_insn & 'he003) == 'h4002) begin // C.LWSP
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_clwsp_off;
                 execute_req_load_size_lg2 <= 3'b110;
                 execute_req_mem_wb_reg    <= id_rf_insn[11:7];
              end
              else if ((id_rf_insn & 'he003) == 'h2002) begin // C.FLDSP
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_cldsp_off;
                 execute_req_load_size_lg2 <= 3'b011;
                 execute_req_mem_wb_reg    <= id_rf_insn[11:7];
                 execute_req_mem_fp        <= 1'b1;
              end
              else if ((id_rf_insn & 'he003) == 'h6002) begin // C.LDSP
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_cldsp_off;
                 execute_req_load_size_lg2 <= 3'b011;
                 execute_req_mem_wb_reg    <= id_rf_insn[11:7];
              end
              else if ((id_rf_insn & 'he003) == 'hc002) begin // C.SWSP
                 execute_req_mem_op      <= `MEMOP_STORE;
                 execute_req_mem_offset  <= d_cswsp_off;
                 execute_req_mem_wr_mask <= 8'h0f;
              end
              else if ((id_rf_insn & 'he003) == 'ha002) begin // C.FSDSP
                 execute_req_mem_op      <= `MEMOP_STORE;
                 execute_req_mem_offset  <= d_csdsp_off;
                 execute_req_mem_wr_mask <= 8'hff;
                 execute_req_mem_fp      <= 1'b1;
              end
              else if ((id_rf_insn & 'he003) == 'he002) begin // C.SDSP
                 execute_req_mem_op      <= `MEMOP_STORE;
                 execute_req_mem_offset  <= d_csdsp_off;
                 execute_req_mem_wr_mask <= 8'hff;
              end

              // Uncompressed loads / stores / atomics
              else if (id_rf_insn[1:0] == 2'b11 && id_rf_insn[6:2] == 5'b00000) begin // LOAD
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_imm_i_s;
                 // funct3 = id_rf_insn[14:12]: {2:0] = size; [2] = 1 → NO sign-ext (U-variant); invert to match
                 // Current encoding: load_size_lg2 = {sxt, size[1:0]} where sxt=1 means sign-ext.
                 //   LB=0|4, LH=1|4, LW=2|4, LD=3, LBU=0, LHU=1, LWU=2.
                 // RISC-V: funct3[2]=0 is signed (B/H/W), funct3[2]=1 is unsigned (BU/HU/WU); LD has funct3=011 (size=3, no sxt).
                 // So load_size_lg2 = {~funct3[2] & (funct3[1:0] != 2'b11), funct3[1:0]}.
                 execute_req_load_size_lg2 <= {~id_rf_insn[14] & ~(id_rf_insn[13] & id_rf_insn[12]), id_rf_insn[13:12]};
                 execute_req_mem_wb_reg    <= id_rf_insn[11:7];
              end
              else if (id_rf_insn[1:0] == 2'b11 && id_rf_insn[6:2] == 5'b01000) begin // STORE
                 execute_req_mem_op     <= `MEMOP_STORE;
                 execute_req_mem_offset <= d_imm_s_s;
                 // wr_mask = (1 << (1 << funct3[1:0])) - 1
                 case (id_rf_insn[13:12])
                    2'b00: execute_req_mem_wr_mask <= 8'h01; // SB
                    2'b01: execute_req_mem_wr_mask <= 8'h03; // SH
                    2'b10: execute_req_mem_wr_mask <= 8'h0f; // SW
                    2'b11: execute_req_mem_wr_mask <= 8'hff; // SD
                 endcase
              end
              else if ((id_rf_insn & 'hf9f0707f) == 'h1000202f ||  // LR.W
                       (id_rf_insn & 'hf9f0707f) == 'h1000302f) begin // LR.D
                 execute_req_mem_op        <= `MEMOP_LR;
                 execute_req_mem_offset    <= 64'd0;
                 execute_req_load_size_lg2 <= id_rf_insn[12] ? 3'b011 : 3'b110; // D : W(sign-ext)
                 execute_req_mem_wb_reg    <= id_rf_insn[11:7];
              end
              else if ((id_rf_insn & 'hf800707f) == 'h1800202f ||  // SC.W
                       (id_rf_insn & 'hf800707f) == 'h1800302f) begin // SC.D
                 execute_req_mem_op      <= `MEMOP_SC;
                 execute_req_mem_offset  <= 64'd0;
                 execute_req_mem_wr_mask <= id_rf_insn[12] ? 8'hff : 8'h0f;
                 execute_req_mem_wb_reg  <= id_rf_insn[11:7];
              end
              // FP loads: FLW (funct3=010) and FLD (funct3=011); opcode 0000111
              else if (id_rf_insn[1:0] == 2'b11 && id_rf_insn[6:2] == 5'b00001 &&
                       (id_rf_insn[14:12] == 3'b010 || id_rf_insn[14:12] == 3'b011)) begin
                 execute_req_mem_op        <= `MEMOP_LOAD;
                 execute_req_mem_offset    <= d_imm_i_s;
                 // FLW: 32-bit zero-extend (load_size_lg2=010), NaN-box in S_LOAD_ALIGN.
                 // FLD: 64-bit (load_size_lg2=011).
                 execute_req_load_size_lg2 <= {1'b0, id_rf_insn[13:12]};
                 execute_req_mem_wb_reg    <= id_rf_insn[11:7];
                 execute_req_mem_fp        <= 1'b1;
              end
              // FP stores: FSW (funct3=010) and FSD (funct3=011); opcode 0100111
              else if (id_rf_insn[1:0] == 2'b11 && id_rf_insn[6:2] == 5'b01001 &&
                       (id_rf_insn[14:12] == 3'b010 || id_rf_insn[14:12] == 3'b011)) begin
                 execute_req_mem_op        <= `MEMOP_STORE;
                 execute_req_mem_offset    <= d_imm_s_s;
                 execute_req_mem_wr_mask   <= id_rf_insn[12] ? 8'hff : 8'h0f;
                 execute_req_mem_fp        <= 1'b1;
              end
              else if (id_rf_insn[1:0] == 2'b11 && id_rf_insn[6:2] == 5'b01011 &&
                       (id_rf_insn[14:12] == 3'b010 || id_rf_insn[14:12] == 3'b011)) begin // AMO*.W / AMO*.D
                 // funct5 must be one of the 9 defined AMO variants; otherwise
                 // leave execute_req_mem_op = MEMOP_NONE so S_EXECUTE traps illegal instruction.
                 // (LR/SC are funct5 00010/00011, already matched above.)
                 case (id_rf_insn[31:27])
                    5'b00000, 5'b00001, 5'b00100, 5'b01000, 5'b01100,
                    5'b10000, 5'b10100, 5'b11000, 5'b11100: begin
                       execute_req_mem_op        <= `MEMOP_AMO;
                       execute_req_mem_offset    <= 64'd0;
                       execute_req_load_size_lg2 <= id_rf_insn[12] ? 3'b011 : 3'b010; // D : W(no sxt)
                       execute_req_mem_wb_reg    <= id_rf_insn[11:7];
                    end
                    default: ; // illegal AMO funct5: falls through
                 endcase
              end
           end // id_rf_mem_decode
      end
   endtask

   task try_issue_queued_decode_preserve_state;
      input       allow_ex_prepare;
      input       pending_int_valid;
      input [4:0] pending_int_rd;
      input       pending_fp_valid;
      input [4:0] pending_fp_rd;
      begin
         if (allow_ex_prepare && id_ex_fire &&
             id_no_pending_wb_hazard(pending_int_valid, pending_int_rd,
                                     pending_fp_valid, pending_fp_rd)) begin
            prepare_execute_req_from_id(1'b1);
         end else if (!id_valid && !frontend_miss_valid &&
                      rf_decode_matches_retire(npc, prv, fetch_epoch)) begin
            launch_rf_decode_read_preserve_state();
         end
      end
   endtask

   task try_launch_queued_decode_preserve_state;
      begin
         if (!id_valid && !frontend_miss_valid &&
             rf_decode_matches_retire(npc, prv, fetch_epoch))
            launch_rf_decode_read_preserve_state();
      end
   endtask

   task try_prepare_retire_id_preserve_state;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      input       pending_int_valid;
      input [4:0] pending_int_rd;
      input       pending_fp_valid;
      input [4:0] pending_fp_rd;
      begin
         if (id_ex_fire &&
             id_matches_retire(retire_pc, retire_prv, retire_epoch) &&
             id_no_pending_wb_hazard(pending_int_valid, pending_int_rd,
                                     pending_fp_valid, pending_fp_rd)) begin
            prepare_execute_req_from_id(1'b1);
         end
      end
   endtask

   task try_issue_queued_decode_no_pending;
      input allow_ex_prepare;
      begin
         try_issue_queued_decode_preserve_state(allow_ex_prepare,
                                                1'b0, 5'd0,
                                                1'b0, 5'd0);
      end
   endtask

   task try_issue_queued_decode_int_pending;
      input       allow_ex_prepare;
      input [4:0] pending_rd;
      begin
         try_issue_queued_decode_preserve_state(allow_ex_prepare,
                                                1'b1, pending_rd,
                                                1'b0, 5'd0);
      end
   endtask

   task try_issue_queued_decode_current_wb;
      input allow_ex_prepare;
      begin
         try_issue_queued_decode_preserve_state(allow_ex_prepare,
                                                !write_back_fp_valid, write_back_register,
                                                write_back_fp_valid, write_back_fp_register);
      end
   endtask

   task try_issue_queued_decode_tagged_wb;
      input       allow_ex_prepare;
      input       pending_fp_valid;
      input [4:0] pending_rd;
      begin
         try_issue_queued_decode_preserve_state(allow_ex_prepare,
                                                !pending_fp_valid, pending_rd,
                                                pending_fp_valid, pending_rd);
      end
   endtask

   task try_prepare_retire_id_no_pending;
      begin
         try_prepare_retire_id_preserve_state(npc, prv, fetch_epoch,
                                              1'b0, 5'd0,
                                              1'b0, 5'd0);
      end
   endtask

   task try_prepare_retire_id_int_pending;
      input [4:0] pending_rd;
      begin
         try_prepare_retire_id_preserve_state(npc, prv, fetch_epoch,
                                              1'b1, pending_rd,
                                              1'b0, 5'd0);
      end
   endtask

   task try_prepare_retire_id_fp_pending;
      input [4:0] pending_rd;
      begin
         try_prepare_retire_id_preserve_state(npc, prv, fetch_epoch,
                                              1'b0, 5'd0,
                                              1'b1, pending_rd);
      end
   endtask

   task try_prepare_retire_id_current_wb;
      begin
         try_prepare_retire_id_preserve_state(npc, prv, fetch_epoch,
                                              !write_back_fp_valid, write_back_register,
                                              write_back_fp_valid, write_back_fp_register);
      end
   endtask

   task try_prepare_retire_id_tagged_wb;
      input       pending_fp_valid;
      input [4:0] pending_rd;
      begin
         try_prepare_retire_id_preserve_state(npc, prv, fetch_epoch,
                                              !pending_fp_valid, pending_rd,
                                              pending_fp_valid, pending_rd);
      end
   endtask

   function frontend_physical_fetch_ok;
      input [63:0] fetch_pc;
      begin
         frontend_physical_fetch_ok =
            (((fetch_pc ^ `MEM_BASEADDR) &
              (64'hffff_ffff_ffff_ffff << `MEM_SIZE_LG2)) == 0) ||
            fetch_pc[63:31] == 1;
      end
   endfunction

   function frontend_speculative_fetch_ok;
      input [63:0] fetch_pc;
      input [ 1:0] fetch_prv;
      begin
         frontend_speculative_fetch_ok =
            (csr_satp[63:60] == 4'd8 && fetch_prv != 3) ||
            frontend_physical_fetch_ok(fetch_pc);
      end
   endfunction

   task try_frontend_speculative_fetch_buf_enqueue;
      begin
         if (frontend_cmd_speculative &&
             (!rf_decode_full || rf_decode_pop_this_cycle) &&
             !frontend_decode_pending_latch_this_cycle &&
             (!frontend_decode_pending_valid ||
              frontend_decode_pending_drain) &&
             !frontend_miss_valid && !frontend_miss_done &&
             !frontend_cmd_spec_miss_ready &&
             frontend_speculative_fetch_ok(frontend_cmd_pc, frontend_cmd_prv) &&
             frontend_rsp_hit) begin
            if (!frontend_decode_pending_valid &&
                !rf_decode_enqueue_this_cycle) begin
               enqueue_frontend_decode_hit(frontend_cmd_pc,
                                           frontend_rsp_next_pc,
                                           frontend_rsp_predicted_next_pc,
                                           frontend_rsp_insn,
                                           frontend_cmd_prv,
                                           frontend_cmd_epoch);
            end else begin
               latch_frontend_decode_pending(frontend_cmd_pc,
                                             frontend_rsp_next_pc,
                                             frontend_rsp_predicted_next_pc,
                                             frontend_rsp_insn,
                                             frontend_cmd_prv,
                                             frontend_cmd_epoch);
            end
         end else if (frontend_cmd_speculative && frontend_cmd_valid &&
                      (!rf_decode_full || rf_decode_pop_this_cycle) &&
                      !rf_decode_enqueue_this_cycle &&
                      !frontend_decode_pending_latch_this_cycle &&
                      (!frontend_decode_pending_valid ||
                       frontend_decode_pending_drain) &&
                      !frontend_miss_valid && !frontend_miss_done &&
                      !frontend_cmd_spec_miss_ready &&
                      frontend_speculative_fetch_ok(frontend_cmd_pc, frontend_cmd_prv)) begin
            frontend_cmd_spec_miss_ready <= 1;
         end
      end
   endtask

   function frontend_miss_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         frontend_miss_matches_retire =
            frontend_miss_epoch == retire_epoch &&
            frontend_miss_pc == retire_pc &&
            frontend_miss_prv == retire_prv;
      end
   endfunction

   function id_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         id_matches_retire =
            id_valid &&
            id_pc == retire_pc &&
            id_prv == retire_prv &&
            id_epoch == retire_epoch;
      end
   endfunction

   function rf_decode_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         rf_decode_matches_retire =
            rf_decode_valid &&
            rf_decode_pc == retire_pc &&
            rf_decode_prv == retire_prv &&
            rf_decode_epoch == retire_epoch;
      end
   endfunction

   function execute_req_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         execute_req_matches_retire =
            execute_req_valid &&
            execute_req_pc == retire_pc &&
            execute_req_prv == retire_prv &&
            execute_req_epoch == retire_epoch;
      end
   endfunction

   function frontend_spec_miss_state;
      input [5:0] s;
      begin
         case (s)
           `S_MULDIV_START,
           `S_MUL_RUNNING,
           `S_DIV_RUNNING,
           `S_CVFPU_ISSUE,
           `S_CVFPU_WAIT:
             frontend_spec_miss_state = 1'b1;
         default:
           frontend_spec_miss_state = 1'b0;
      endcase
      end
   endfunction

   function rf_prearm_safe_state;
      input [5:0] s;
      begin
         case (s)
           `S_FETCH1,
           `S_EXECUTE2,
           `S_FRONTEND_MISS_WAIT,
           `S_LOAD_ALIGN,
           `S_TLB_LOOKUP,
           `S_TLB_CHECK,
           `S_PTW_LAUNCH,
           `S_DMEM_LOAD_WAIT,
           `S_DMEM_LOAD2_WAIT,
           `S_PTW_PROCESS,
           `S_DMEM_STORE_WAIT,
           `S_DMEM_STORE2,
           `S_DMEM_STORE_RESP_WAIT,
           `S_DMEM_STORE_RESP_ARM,
           `S_PTW_DIRECT_WAIT,
           `S_TLB_INSERT,
           `S_CBO_WAIT,
           `S_MUL_RUNNING,
           `S_DIV_RUNNING,
           `S_CVFPU_ISSUE,
           `S_CVFPU_WAIT:
             rf_prearm_safe_state = 1'b1;
           default:
             rf_prearm_safe_state = 1'b0;
         endcase
      end
   endfunction

   task try_frontend_speculative_miss_start;
      begin
         if (frontend_cmd_speculative && frontend_cmd_valid &&
             !frontend_miss_valid && !frontend_miss_done &&
             frontend_cmd_spec_miss_ready && cache_idle &&
             (csr_satp[63:60] != 4'd8 || frontend_cmd_prv == 3) &&
             frontend_physical_fetch_ok(frontend_cmd_pc) &&
             frontend_cmd_pc[2:1] != 2'b11) begin
            frontend_miss_valid      <= 1;
            frontend_miss_done       <= 0;
            frontend_miss_pc         <= frontend_cmd_pc;
            frontend_miss_prv        <= frontend_cmd_prv;
            frontend_miss_asid       <= frontend_cmd_asid;
            frontend_miss_epoch      <= frontend_cmd_epoch;
            frontend_miss_next_valid <= 0;
            frontend_miss_insn_valid <= 0;
            frontend_cmd_spec_miss_ready <= 0;
            issue_ifetch_cache_read(frontend_cmd_pc[30:3],
                                    frontend_cmd_pc,
                                    {TLB_ASID_BITS{1'b0}},
                                    CACHE_PERM_PHYS,
                                    {2'd0, frontend_cmd_prv, sum, mxr});
         end
      end
   endtask

   task consume_frontend_miss;
      reg [127:0] miss_aligned;
      reg [31:0]  miss_insn;
      reg [63:0]  miss_next_pc;
      begin
         miss_aligned = frontend_miss_next_valid ?
                        frontend_miss_window :
                        {64'bx, frontend_miss_window[63:0]};
         miss_insn = frontend_miss_insn;
         if (frontend_miss_next_valid) begin
            frontend_buf_fill      <= 1'b1;
            frontend_buf_fill_pc   <= frontend_miss_pc;
            frontend_buf_fill_prv  <= frontend_miss_prv;
            frontend_buf_fill_asid <= frontend_miss_asid;
            frontend_buf_fill_data <= miss_aligned;
         end
         if (!frontend_miss_insn_valid)
            miss_insn = fetch_buf_pick_insn(miss_aligned, {1'b0, frontend_miss_pc[2:0]});
         miss_next_pc = frontend_fallthrough_pc(frontend_miss_pc, miss_insn);
         frontend_miss_valid <= 0;
         frontend_miss_done  <= 0;
         pc <= frontend_miss_pc;
         insn <= miss_insn;
         fetch_from_ifetch_rsp <= 1;
         translated <= 0;
         frontend_redirect_valid <= 0;
         write_back_register <= 0;
         write_back_fp_valid <= 0;
         if (!frontend_decode_pending_latch_this_cycle &&
             !frontend_decode_pending_valid &&
             !rf_decode_full && !rf_decode_enqueue_this_cycle) begin
            enqueue_frontend_decode_hit(frontend_miss_pc,
                                        miss_next_pc,
                                        miss_next_pc,
                                        miss_insn,
                                        frontend_miss_prv,
                                        frontend_miss_epoch);
            state <= `S_FETCH1;
         end else if (!frontend_decode_pending_latch_this_cycle &&
                      (!frontend_decode_pending_valid ||
                       frontend_decode_pending_drain)) begin
            latch_frontend_decode_pending(frontend_miss_pc,
                                          miss_next_pc,
                                          miss_next_pc,
                                          miss_insn,
                                          frontend_miss_prv,
                                          frontend_miss_epoch);
            state <= `S_FETCH1;
         end else begin
            accept_instruction_fetch(frontend_miss_pc,
                                     miss_next_pc,
                                     miss_next_pc,
                                     miss_insn,
                                     frontend_miss_prv,
                                     frontend_miss_epoch,
                                     1'b1);
         end
      end
   endtask

   task consume_latched_ifetch_response;
      reg [127:0] rsp_aligned;
      reg [31:0]  rsp_insn;
      reg [63:0]  rsp_next_pc;
      begin
         // Cross-doubleword responses keep the old second-half path for now;
         // within-doubleword hits may use the frontend-produced instruction.
         rsp_aligned = ifetch_latched_next_valid
                       ? ifetch_latched_window
                       : {64'bx, ifetch_latched_window[63:0]};
         if (ifetch_latched_next_valid) begin
            frontend_buf_fill      <= 1'b1;
            frontend_buf_fill_pc   <= frontend_cmd_pc;
            frontend_buf_fill_prv  <= frontend_cmd_prv;
            frontend_buf_fill_asid <= frontend_cmd_asid;
            frontend_buf_fill_data <= rsp_aligned;
         end
         rsp_insn = ifetch_latched_insn_valid
                  ? ifetch_latched_insn
                  : fetch_buf_pick_insn(rsp_aligned,
                                        {1'b0, frontend_cmd_pc[2:0]});
         rsp_next_pc = ifetch_latched_insn_valid
                     ? ifetch_latched_next_pc
                     : frontend_fallthrough_pc(frontend_cmd_pc, rsp_insn);
         accept_instruction_fetch(frontend_cmd_pc,
                                  rsp_next_pc,
                                  rsp_next_pc,
                                  rsp_insn,
                                  frontend_cmd_prv,
                                  frontend_cmd_epoch,
                                  1'b1);
      end
   endtask

   task prepare_retire_fetch;
      input [63:0] prepare_pc;
      input [ 1:0] prepare_prv;
      input [FRONTEND_EPOCH_BITS-1:0] prepare_epoch;
      begin
         frontend_cmd_valid <= 1;
         frontend_cmd_pc <= prepare_pc;
         frontend_cmd_prv <= prepare_prv;
         frontend_cmd_asid <= fetch_asid_for_context(prepare_prv);
         frontend_cmd_epoch <= prepare_epoch;
         frontend_cmd_fast_ready <= 1;
         frontend_cmd_speculative <= 0;
         frontend_cmd_spec_miss_ready <= 0;
         f_state <= `F_FETCH_BUF_CHECK;
      end
   endtask

   task prepare_current_epoch_fetch;
      input [63:0] prepare_pc;
      input [ 1:0] prepare_prv;
      begin
         prepare_retire_fetch(prepare_pc, prepare_prv, fetch_epoch);
      end
   endtask

   task issue_frontend_redirect;
      begin
         prepare_retire_fetch(frontend_redirect_pc,
                              frontend_redirect_prv,
                              frontend_redirect_epoch);
         frontend_redirect_valid <= 0;
      end
   endtask

   task redirect_retire_fetch;
      input [63:0] redirect_pc;
      input [ 1:0] redirect_prv;
      reg   [FRONTEND_EPOCH_BITS-1:0] redirect_epoch;
      begin
         frontend_flush_this_cycle = 1;
         redirect_epoch = fetch_epoch + 1'b1;
         fetch_epoch <= redirect_epoch;
         squash_decode_execute();
         clear_frontend_cmd();
         kill_frontend_lookup();
         frontend_redirect_valid <= 1;
         frontend_redirect_pc <= redirect_pc;
         frontend_redirect_prv <= redirect_prv;
         frontend_redirect_epoch <= redirect_epoch;
      end
   endtask

   task retire_queued_decode_or_refetch;
      begin
         if (rf_decode_matches_retire(npc, prv, fetch_epoch)) begin
            insn <= rf_decode_insn;
            fetch_from_ifetch_rsp <= rf_decode_from_ifetch_rsp;
            translated <= 0;
            write_back_register = 0;
            write_back_fp_valid = 0;
            launch_rf_decode_read();
         end else if (rf_decode_pc == pc && rf_decode_epoch == fetch_epoch) begin
            // Stale duplicate of the just-retired instruction: drop and
            // re-consume next cycle instead of squashing the lead (see
            // try_early_launch_queued_decode for the safety argument).
            pop_rf_decode_head();
            state <= `S_FETCH1;
         end else begin
            redirect_retire_fetch(npc, prv);
            state <= `S_FETCH_REQ;
         end
      end
   endtask

   task try_early_launch_queued_decode;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      output       launched;
      begin
         launched = 1'b0;
         if (!id_valid && !frontend_miss_valid &&
             rf_decode_matches_retire(retire_pc, retire_prv, fetch_epoch)) begin
            launch_rf_decode_read_preserve_state();
            launched = 1'b1;
         end else if (!id_valid && rf_decode_valid && !frontend_miss_valid) begin
            if (rf_decode_pc == pc && rf_decode_epoch == fetch_epoch) begin
               // Stale duplicate of the just-retired instruction at the head
               // (seeded by an earlier squash->refetch double-enqueue). Drop it
               // and re-consume next cycle instead of squashing the whole lead.
               // Safe: head==pc can only be the legitimate next insn in a self
               // loop, and that case matches npc above and takes the consume path.
               pop_rf_decode_head();
               launched = 1'b1;
            end else begin
               redirect_retire_fetch(retire_pc, retire_prv);
               launched = 1'b1;
            end
         end
      end
   endtask

   task retire_prepared_fetch;
      reg early_launched;
      begin
         execute_res_valid <= 0;
         retire_now_q <= 1;
         if (npc == ex_predicted_pc) begin
            if (execute_req_matches_retire(npc, prv, fetch_epoch)) begin
               clear_frontend_fast_cmd();
            end else begin
               try_early_launch_queued_decode(npc, prv, early_launched);
               if (!early_launched)
                  prepare_current_epoch_fetch(npc, prv);
            end
         end else begin
            redirect_retire_fetch(npc, prv);
         end
         state <= `S_FETCH1;
      end
   endtask

   task retire_linear_fetch;
      reg early_launched;
      begin
         retire_now_q <= 1;
         if (execute_req_matches_retire(npc, prv, fetch_epoch)) begin
            clear_frontend_fast_cmd();
         end else if (id_matches_retire(npc, prv, fetch_epoch)) begin
            clear_frontend_fast_cmd();
         end else begin
            try_early_launch_queued_decode(npc, prv, early_launched);
            if (!early_launched)
               prepare_current_epoch_fetch(npc, prv);
         end
         state <= `S_FETCH1;
      end
   endtask

   task finish_load_writeback;
      begin
         if (write_back_fp_valid)
            write_back_fp_value = load_size_lg2[0] ?
                                  write_back_value :
                                  {32'hffffffff, write_back_value[31:0]};
      end
   endtask

   task retire_redirect_fetch;
      begin
         execute_res_valid <= 0;
         retire_now_q <= 1;
         redirect_retire_fetch(npc, prv);
         state <= `S_FETCH1;
      end
   endtask

   task retire_int_value_prepared_fetch;
      input [63:0] retire_value;
      begin
         try_prepare_retire_id_int_pending(write_back_register);
         write_back_value <= retire_value;
         retire_prepared_fetch();
      end
   endtask

   task retire_execute_req_alu_b;
      begin
         retire_int_value_prepared_fetch(execute_req_alu_b);
      end
   endtask

   task retire_no_wb_prepared_fetch;
      begin
         try_prepare_retire_id_no_pending();
         retire_prepared_fetch();
      end
   endtask

   task retire_current_wb_prepared_fetch;
      begin
         try_prepare_retire_id_current_wb();
         retire_prepared_fetch();
      end
   endtask

   task retire_no_wb_linear_fetch;
      begin
         try_prepare_retire_id_no_pending();
         retire_linear_fetch();
      end
   endtask

   task retire_current_wb_linear_fetch;
      begin
         try_prepare_retire_id_current_wb();
         retire_linear_fetch();
      end
   endtask

   task retire_current_wb_redirect_fetch;
      begin
         try_prepare_retire_id_current_wb();
         // A value-changing SATP write invalidates the fetch buffer's
         // translation context. A plain redirect (redirect_retire_fetch, as
         // used by branch mispredicts) leaves frontend_buf intact, so a
         // redirect to the sequential npc would hit the stale line prefetched
         // under the old satp and be accepted WITHOUT re-translation. Flush the
         // frontend like SFENCE.VMA so the refetch of npc misses the buffer and
         // re-walks under the new satp; then layer the redirect target on top.
         flush_frontend_speculation();
         retire_redirect_fetch();
      end
   endtask

   task retire_int_pending_linear_fetch;
      input [4:0] pending_rd;
      begin
         try_prepare_retire_id_int_pending(pending_rd);
         retire_linear_fetch();
      end
   endtask

   task retire_fp_pending_linear_fetch;
      input [4:0] pending_rd;
      begin
         try_prepare_retire_id_fp_pending(pending_rd);
         retire_linear_fetch();
      end
   endtask

   task retire_fp_value_linear_fetch;
      input [ 4:0] retire_rd;
      input [63:0] retire_value;
      begin
         write_back_fp_valid    = 1;
         write_back_fp_register = retire_rd;
         write_back_fp_value    <= retire_value;
         retire_fp_pending_linear_fetch(retire_rd);
      end
   endtask

   task retire_tagged_wb_linear_fetch;
      input       pending_fp_valid;
      input [4:0] pending_rd;
      begin
         try_prepare_retire_id_tagged_wb(pending_fp_valid, pending_rd);
         retire_linear_fetch();
      end
   endtask

   task stage_int_commit_result;
      input [63:0] result;
      input [ 4:0] result_fflags;
      input        use_prepared_fetch;
      begin
         int_commit_result <= result;
         int_commit_fflags <= result_fflags;
         int_commit_prepared_fetch <= use_prepared_fetch;
         int_commit_redirect_fetch <= 1'b0;
         state <= `S_INT_COMMIT;
      end
   endtask

   task start_cvfpu_issue;
      input [63:0] operand0;
      input [63:0] operand1;
      input [63:0] operand2;
      input [ 2:0] rnd_mode;
      input [ 3:0] op;
      input        op_mod;
      input [ 2:0] src_fmt;
      input [ 2:0] dst_fmt;
      input [ 1:0] int_fmt;
      input [ 7:0] tag;
      input        write_fp;
      begin
         cvfpu_operands[0] <= operand0;
         cvfpu_operands[1] <= operand1;
         cvfpu_operands[2] <= operand2;
         cvfpu_rnd_mode    <= rnd_mode;
         cvfpu_op          <= op;
         cvfpu_op_mod      <= op_mod;
         cvfpu_src_fmt     <= src_fmt;
         cvfpu_dst_fmt     <= dst_fmt;
         cvfpu_int_fmt     <= int_fmt;
         cvfpu_tag_in      <= tag;
         cvfpu_write_fp    <= write_fp;
         cvfpu_in_valid    <= 1'b1;
         state             <= `S_CVFPU_ISSUE;
      end
   endtask

   task retire_cvfpu_output;
      begin
         if (cvfpu_write_fp) begin
            write_back_fp_valid    = 1;
            write_back_fp_register = cvfpu_tag_out[4:0];
            write_back_fp_value    <= cvfpu_result;
         end else begin
            write_back_register = cvfpu_tag_out[4:0];
            write_back_value    <= cvfpu_result;
         end
         fflags = fflags | cvfpu_fflags;
         retire_tagged_wb_linear_fetch(cvfpu_write_fp, cvfpu_tag_out[4:0]);
      end
   endtask

/* verilator lint_off WIDTHTRUNC */
   task route_translated_addr;
      input [63:0] req_pa;
      input [CACHE_PERM_BITS-1:0] req_perm;
      input [ 4:0] req_return;
      begin
         mem_addr = req_pa;
         mem_va = tlb_req_va;
         mem_asid <= current_cache_asid;
         mem_perm <= req_perm;
         mem_ctx <= tlb_req_ctx;
         translated <= 1;
         if (req_return == `S_FETCH2 || req_return == `S_FETCH2_HALF) begin
            if (phys_region(mem_addr) == `REGION_BRAM ||
                phys_region(mem_addr) == `REGION_DRAM) begin
               // Cacheable instruction fetch.  Local BRAM is a cache refill
               // source now; it is no longer consumed directly by fetch.
               fetch_from_ifetch_rsp <= 1;
               issue_ifetch_cache_read(mem_addr[30:3], tlb_req_va,
                                       current_cache_asid, req_perm,
                                       tlb_req_ctx);
               state           <= (req_return == `S_FETCH2) ?
                                  `S_IFETCH_WAIT : `S_IFETCH_HALF_WAIT;
            end else begin
               cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
               tval = req_pa;
               state <= `S_EXCEPTION;
            end
         end else begin
            state     <= {1'b0, req_return};
         end
      end
   endtask
/* verilator lint_on WIDTHTRUNC */

   task start_ptw;
      input [63:0] req_va;
      input [ 1:0] req_access;
      input [ 1:0] req_prv;
      input [ 4:0] req_return;
      begin
         ptw_va       = req_va;
         ptw_level    = 2;
         ptw_access   = req_access;
         ptw_prv      = req_prv;
         ptw_satp     = csr_satp;
         ptw_sum      = sum;
         ptw_mxr      = mxr;
         ptw_return   = req_return;
         ptw_pte_addr <= {8'd0, csr_satp[43:0], 12'd0} + {52'd0, req_va[38:30], 3'd0};
         state        <= `S_PTW_LAUNCH;
      end
   endtask

   task start_translation;
      input [63:0] req_va;
      input [ 1:0] req_access;
      input [ 1:0] req_prv;
      input [ 4:0] req_return;
      begin
         tlb_req_va     <= req_va;
         tlb_req_access <= req_access;
         tlb_req_prv    <= req_prv;
         tlb_req_sum    <= sum;
         tlb_req_mxr    <= mxr;
         tlb_req_return <= req_return;
         tlb_4k_rd_idx  <= tlb_4k_index(req_va, req_access, req_prv, sum, mxr);
         tlb_2m_rd_idx  <= tlb_2m_index(req_va, req_access, req_prv, sum, mxr);
         state          <= `S_TLB_LOOKUP;
      end
   endtask

   task stage_tlb_insert;
      input [63:0] req_va;
      input [63:0] req_pa;
      input [ 1:0] req_level;
      input [ 1:0] req_access;
      input [ 1:0] req_prv;
      input [CACHE_PERM_BITS-1:0] req_perm;
      input [63:0] req_satp;
      input        req_sum;
      input        req_mxr;
      begin
         tlb_insert_level <= req_level;
         tlb_insert_4k_idx <= tlb_4k_index(req_va, req_access, req_prv,
                                           req_sum, req_mxr);
         tlb_insert_2m_idx <= tlb_2m_index(req_va, req_access, req_prv,
                                           req_sum, req_mxr);
         tlb_insert_4k_data <= {req_va[38:12], req_pa[63:12],
                                satp_tlb_key(req_satp),
                                req_perm,
                                {req_access, req_prv, req_sum, req_mxr}};
         tlb_insert_2m_data <= {req_va[38:21], req_pa[63:21],
                                satp_tlb_key(req_satp),
                                req_perm,
                                {req_access, req_prv, req_sum, req_mxr}};
      end
   endtask

   task commit_staged_tlb_insert;
      begin
         if (tlb_insert_level == 1) begin
`ifdef SIMULATE
            if (!tlb_2m_valid[tlb_insert_2m_idx])
               tlb_stat_entries_2m = tlb_stat_entries_2m + 1;
`endif
            hpm_tlb_insert_2m_pulse <= 1;
            if (tlb_2m_valid[tlb_insert_2m_idx])
               hpm_tlb_evict_2m_pulse <= 1;
            tlb_2m_valid[tlb_insert_2m_idx] <= 1;
            tlb_2m_wr_en <= 1;
            tlb_2m_wr_idx <= tlb_insert_2m_idx;
            tlb_2m_wr_data <= tlb_insert_2m_data;
         end else if (tlb_insert_level == 0) begin
`ifdef SIMULATE
            if (!tlb_4k_valid[tlb_insert_4k_idx])
               tlb_stat_entries_4k = tlb_stat_entries_4k + 1;
`endif
            hpm_tlb_insert_4k_pulse <= 1;
            if (tlb_4k_valid[tlb_insert_4k_idx])
               hpm_tlb_evict_4k_pulse <= 1;
            tlb_4k_valid[tlb_insert_4k_idx] <= 1;
            tlb_4k_wr_en <= 1;
            tlb_4k_wr_idx <= tlb_insert_4k_idx;
            tlb_4k_wr_data <= tlb_insert_4k_data;
         end else begin
            hpm_tlb_uncached_1g_pulse <= 1;
         end
      end
   endtask

   always @(posedge clock) begin
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`ifdef SIMULATE
      if (core_reset_now) begin
         rf_decode_valid_q <= 0;
         rf_decode_head_q <= 0;
         rf_decode_pc_q_assert <= `RESET_PC;
         rf_decode_next_pc_q_assert <= `RESET_PC;
         rf_decode_predicted_pc_q_assert <= `RESET_PC;
         rf_decode_insn_q_assert <= 0;
         rf_decode_prv_q_assert <= 0;
         rf_decode_epoch_q_assert <= 0;
         rf_decode_from_ifetch_rsp_q_assert <= 0;
         rf_decode_rd_q_assert <= 0;
         rf_decode_rs1_q_assert <= 0;
         rf_decode_rs2_q_assert <= 0;
         rf_decode_shamt_q_assert <= 0;
         id_valid_q <= 0;
         id_pc_q <= `RESET_PC;
         id_next_pc_q <= `RESET_PC;
         id_predicted_pc_q <= `RESET_PC;
         id_prv_q <= 0;
         id_epoch_q <= 0;
         id_insn_q <= 0;
         id_rd_q <= 0;
         id_rs1_q <= 0;
         id_rs2_q <= 0;
         id_shamt_q <= 0;
         execute_req_valid_q <= 0;
         execute_req_pc_q <= `RESET_PC;
         execute_req_next_pc_q <= `RESET_PC;
         execute_req_predicted_pc_q <= `RESET_PC;
         execute_req_prv_q <= 0;
         execute_req_epoch_q <= 0;
         execute_req_insn_q <= 0;
         execute_req_rd_q <= 0;
         execute_req_rs1_q <= 0;
         execute_req_rs2_q <= 0;
         execute_req_shamt_q <= 0;
         execute_req_rs1_value_q <= 0;
         execute_req_rs2_value_q <= 0;
         execute_req_frs1_value_q <= 0;
         execute_req_frs2_value_q <= 0;
         execute_req_alu_op_q <= 0;
         execute_req_alu_b_q <= 0;
         execute_req_alu_sxt_q <= 0;
         execute_req_mem_op_q <= 0;
         execute_req_mem_offset_q <= 0;
         execute_req_load_size_lg2_q <= 0;
         execute_req_mem_wr_mask_q <= 0;
         execute_req_mem_wb_reg_q <= 0;
         execute_req_mem_fp_q <= 0;
      end else begin
         if (rf_decode_count > RF_DECODE_QUEUE_DEPTH_COUNT) begin
            $display("%05d BUG: rf_decode queue count out of range", $time);
            $finish;
         end

         if (rf_decode_valid_q && rf_decode_valid &&
             !rf_decode_pop_this_cycle &&
             (rf_decode_head != rf_decode_head_q ||
              rf_decode_pc != rf_decode_pc_q_assert ||
              rf_decode_next_pc != rf_decode_next_pc_q_assert ||
              rf_decode_predicted_pc != rf_decode_predicted_pc_q_assert ||
              rf_decode_insn != rf_decode_insn_q_assert ||
              rf_decode_prv != rf_decode_prv_q_assert ||
              rf_decode_epoch != rf_decode_epoch_q_assert ||
              rf_decode_from_ifetch_rsp != rf_decode_from_ifetch_rsp_q_assert ||
              rf_decode_rd != rf_decode_rd_q_assert ||
              rf_decode_rs1 != rf_decode_rs1_q_assert ||
              rf_decode_rs2 != rf_decode_rs2_q_assert ||
              rf_decode_shamt != rf_decode_shamt_q_assert)) begin
            $display("%05d BUG: rf_decode head payload changed without pop", $time);
            $finish;
         end

         if (id_valid_q && id_valid &&
             (id_pc != id_pc_q ||
              id_next_pc != id_next_pc_q ||
              id_predicted_pc != id_predicted_pc_q ||
              id_prv != id_prv_q ||
              id_epoch != id_epoch_q ||
              id_insn != id_insn_q ||
              id_rd != id_rd_q ||
              id_rs1 != id_rs1_q ||
              id_rs2 != id_rs2_q ||
              id_shamt != id_shamt_q)) begin
            $display("%05d BUG: ID payload changed while valid", $time);
            $finish;
         end

         if (execute_req_valid_q && execute_req_valid &&
             (execute_req_pc != execute_req_pc_q ||
              execute_req_next_pc != execute_req_next_pc_q ||
              execute_req_predicted_pc != execute_req_predicted_pc_q ||
              execute_req_prv != execute_req_prv_q ||
              execute_req_epoch != execute_req_epoch_q ||
              execute_req_insn != execute_req_insn_q ||
              execute_req_rd != execute_req_rd_q ||
              execute_req_rs1 != execute_req_rs1_q ||
              execute_req_rs2 != execute_req_rs2_q ||
              execute_req_shamt != execute_req_shamt_q ||
              execute_req_rs1_value != execute_req_rs1_value_q ||
              execute_req_rs2_value != execute_req_rs2_value_q ||
              execute_req_frs1_value != execute_req_frs1_value_q ||
              execute_req_frs2_value != execute_req_frs2_value_q ||
              execute_req_frs3_value != execute_req_frs3_value_q ||
              execute_req_alu_op != execute_req_alu_op_q ||
              execute_req_alu_b != execute_req_alu_b_q ||
              execute_req_alu_sxt != execute_req_alu_sxt_q ||
              execute_req_mem_op != execute_req_mem_op_q ||
              execute_req_mem_offset != execute_req_mem_offset_q ||
              execute_req_load_size_lg2 != execute_req_load_size_lg2_q ||
              execute_req_mem_wr_mask != execute_req_mem_wr_mask_q ||
              execute_req_mem_wb_reg != execute_req_mem_wb_reg_q ||
              execute_req_mem_fp != execute_req_mem_fp_q)) begin
            $display("%05d BUG: execute request payload changed while valid", $time);
            $finish;
         end

         rf_decode_valid_q <= rf_decode_valid;
         rf_decode_head_q <= rf_decode_head;
         rf_decode_pc_q_assert <= rf_decode_pc;
         rf_decode_next_pc_q_assert <= rf_decode_next_pc;
         rf_decode_predicted_pc_q_assert <= rf_decode_predicted_pc;
         rf_decode_insn_q_assert <= rf_decode_insn;
         rf_decode_prv_q_assert <= rf_decode_prv;
         rf_decode_epoch_q_assert <= rf_decode_epoch;
         rf_decode_from_ifetch_rsp_q_assert <= rf_decode_from_ifetch_rsp;
         rf_decode_rd_q_assert <= rf_decode_rd;
         rf_decode_rs1_q_assert <= rf_decode_rs1;
         rf_decode_rs2_q_assert <= rf_decode_rs2;
         rf_decode_shamt_q_assert <= rf_decode_shamt;

         id_valid_q <= id_valid;
         id_pc_q <= id_pc;
         id_next_pc_q <= id_next_pc;
         id_predicted_pc_q <= id_predicted_pc;
         id_prv_q <= id_prv;
         id_epoch_q <= id_epoch;
         id_insn_q <= id_insn;
         id_rd_q <= id_rd;
         id_rs1_q <= id_rs1;
         id_rs2_q <= id_rs2;
         id_shamt_q <= id_shamt;

         execute_req_valid_q <= execute_req_valid;
         execute_req_pc_q <= execute_req_pc;
         execute_req_next_pc_q <= execute_req_next_pc;
         execute_req_predicted_pc_q <= execute_req_predicted_pc;
         execute_req_prv_q <= execute_req_prv;
         execute_req_epoch_q <= execute_req_epoch;
         execute_req_insn_q <= execute_req_insn;
         execute_req_rd_q <= execute_req_rd;
         execute_req_rs1_q <= execute_req_rs1;
         execute_req_rs2_q <= execute_req_rs2;
         execute_req_shamt_q <= execute_req_shamt;
         execute_req_rs1_value_q <= execute_req_rs1_value;
         execute_req_rs2_value_q <= execute_req_rs2_value;
         execute_req_frs1_value_q <= execute_req_frs1_value;
         execute_req_frs2_value_q <= execute_req_frs2_value;
         execute_req_frs3_value_q <= execute_req_frs3_value;
         execute_req_alu_op_q <= execute_req_alu_op;
         execute_req_alu_b_q <= execute_req_alu_b;
         execute_req_alu_sxt_q <= execute_req_alu_sxt;
         execute_req_mem_op_q <= execute_req_mem_op;
         execute_req_mem_offset_q <= execute_req_mem_offset;
         execute_req_load_size_lg2_q <= execute_req_load_size_lg2;
         execute_req_mem_wr_mask_q <= execute_req_mem_wr_mask;
         execute_req_mem_wb_reg_q <= execute_req_mem_wb_reg;
         execute_req_mem_fp_q <= execute_req_mem_fp;
      end
`endif
      if (!csr_mcountinhibit[0] && hpm_mode_enabled(csr_mcyclecfg))
         csr_mcycle <= csr_mcycle + 1;
      if (core_reset_now) begin
         hpm_instret_q <= 0;
         hpm_icache_read_q <= 0;
         hpm_icache_hit_q <= 0;
         hpm_icache_miss_q <= 0;
         hpm_icache_fill_line_q <= 0;
         hpm_icache_fill_beat_q <= 0;
         hpm_dcache_read_q <= 0;
         hpm_dcache_write_q <= 0;
         hpm_dcache_hit_q <= 0;
         hpm_dcache_miss_q <= 0;
         hpm_dcache_fill_line_q <= 0;
         hpm_dcache_fill_beat_q <= 0;
         hpm_dcache_wb_line_q <= 0;
         hpm_axi_read_q <= 0;
         hpm_axi_write_q <= 0;
         hpm_bus_wait_q <= 0;
         hpm_tlb_lookup_q <= 0;
         hpm_tlb_hit_q <= 0;
         hpm_tlb_miss_q <= 0;
         hpm_tlb_hit_4k_q <= 0;
         hpm_tlb_hit_2m_q <= 0;
         hpm_tlb_insert_4k_q <= 0;
         hpm_tlb_insert_2m_q <= 0;
         hpm_tlb_evict_4k_q <= 0;
         hpm_tlb_evict_2m_q <= 0;
         hpm_tlb_uncached_1g_q <= 0;
         hpm_tlb_uncached_napot_q <= 0;
         hpm_ptw_leaf_4k_q <= 0;
         hpm_ptw_leaf_2m_q <= 0;
         hpm_ptw_leaf_1g_q <= 0;
         hpm_ptw_leaf_napot_q <= 0;
         hpm_vhpr_pulse_q <= 0;
      end else begin
         hpm_instret_q <= hpm_instret_pulse;
         hpm_icache_read_q <= hpm_icache_read_pulse;
         hpm_icache_hit_q <= hpm_icache_hit_pulse;
         hpm_icache_miss_q <= hpm_icache_miss_pulse;
         hpm_icache_fill_line_q <= hpm_icache_fill_line_pulse;
         hpm_icache_fill_beat_q <= hpm_icache_fill_beat_pulse;
         hpm_dcache_read_q <= hpm_dcache_read_pulse;
         hpm_dcache_write_q <= hpm_dcache_write_pulse;
         hpm_dcache_hit_q <= hpm_dcache_hit_pulse;
         hpm_dcache_miss_q <= hpm_dcache_miss_pulse;
         hpm_dcache_fill_line_q <= hpm_dcache_fill_line_pulse;
         hpm_dcache_fill_beat_q <= hpm_dcache_fill_beat_pulse;
         hpm_dcache_wb_line_q <= hpm_dcache_wb_line_pulse;
         hpm_axi_read_q <= hpm_axi_read_pulse;
         hpm_axi_write_q <= hpm_axi_write_pulse;
         hpm_bus_wait_q <= hpm_bus_wait_cycle;
         hpm_tlb_lookup_q <= hpm_tlb_lookup_pulse;
         hpm_tlb_hit_q <= hpm_tlb_hit_pulse;
         hpm_tlb_miss_q <= hpm_tlb_miss_pulse;
         hpm_tlb_hit_4k_q <= hpm_tlb_hit_4k_pulse;
         hpm_tlb_hit_2m_q <= hpm_tlb_hit_2m_pulse;
         hpm_tlb_insert_4k_q <= hpm_tlb_insert_4k_pulse;
         hpm_tlb_insert_2m_q <= hpm_tlb_insert_2m_pulse;
         hpm_tlb_evict_4k_q <= hpm_tlb_evict_4k_pulse;
         hpm_tlb_evict_2m_q <= hpm_tlb_evict_2m_pulse;
         hpm_tlb_uncached_1g_q <= hpm_tlb_uncached_1g_pulse;
         hpm_tlb_uncached_napot_q <= hpm_tlb_uncached_napot_pulse;
         hpm_ptw_leaf_4k_q <= hpm_ptw_leaf_4k_pulse;
         hpm_ptw_leaf_2m_q <= hpm_ptw_leaf_2m_pulse;
         hpm_ptw_leaf_1g_q <= hpm_ptw_leaf_1g_pulse;
         hpm_ptw_leaf_napot_q <= hpm_ptw_leaf_napot_pulse;
         hpm_vhpr_pulse_q <= hpm_vhpr_pulse;
      end
      for (hpm_i = 0; hpm_i < `HPM_COUNTERS; hpm_i = hpm_i + 1) begin
         if (core_reset_now) begin
            csr_mhpmcounter[hpm_i] <= 0;
            csr_mhpmevent[hpm_i] <= 0;
         end else begin
            if (hpm_event_wr_en && hpm_wr_idx == hpm_i[3:0]) begin
               csr_mhpmevent[hpm_i] <= hpm_wr_data;
            end
            if (hpm_counter_wr_en && hpm_wr_idx == hpm_i[3:0]) begin
               csr_mhpmcounter[hpm_i] <= hpm_wr_data;
            end else if (!csr_mcountinhibit[hpm_i + 3] &&
                         hpm_mode_enabled(csr_mhpmevent[hpm_i]) &&
                         hpm_event_active(csr_mhpmevent[hpm_i][15:0],
                                          hpm_instret_q,
                                          hpm_icache_read_q,
                                          hpm_icache_hit_q,
                                          hpm_icache_miss_q,
                                          hpm_icache_fill_line_q,
                                          hpm_icache_fill_beat_q,
                                          hpm_dcache_read_q,
                                          hpm_dcache_write_q,
                                          hpm_dcache_hit_q,
                                          hpm_dcache_miss_q,
                                          hpm_dcache_fill_line_q,
                                          hpm_dcache_fill_beat_q,
                                          hpm_dcache_wb_line_q,
                                          hpm_axi_read_q,
                                          hpm_axi_write_q,
                                          hpm_bus_wait_q,
                                          hpm_tlb_lookup_q,
                                          hpm_tlb_hit_q,
                                          hpm_tlb_miss_q,
                                          hpm_tlb_hit_4k_q,
                                          hpm_tlb_hit_2m_q,
                                          hpm_tlb_insert_4k_q,
                                          hpm_tlb_insert_2m_q,
                                          hpm_tlb_evict_4k_q,
                                          hpm_tlb_evict_2m_q,
                                          hpm_tlb_uncached_1g_q,
                                          hpm_tlb_uncached_napot_q,
                                          hpm_ptw_leaf_4k_q,
                                          hpm_ptw_leaf_2m_q,
                                          hpm_ptw_leaf_1g_q,
                                          hpm_ptw_leaf_napot_q,
                                          hpm_vhpr_pulse_q)) begin
               if (csr_mhpmcounter[hpm_i] == 64'hffff_ffff_ffff_ffff &&
                   !csr_mhpmevent[hpm_i][`HPM_OF_BIT] &&
                   !(hpm_event_wr_en && hpm_wr_idx == hpm_i[3:0])) begin
                  csr_mhpmevent[hpm_i] <= csr_mhpmevent[hpm_i] | 64'h8000_0000_0000_0000;
                  lcofip <= 1;
               end
               csr_mhpmcounter[hpm_i] <= csr_mhpmcounter[hpm_i] + 1;
            end
         end
      end
      if (reset)
         core_reset_pending <= 1;
      // XXX This isn't very portable
      if (clint_mtime_clock_scaler[13]) begin
         clint_mtime_clock_scaler <= 3333 - 2; // 333.3333.. MHz / 3333 ~ 100.01 kHz
         clint_mtime <= clint_mtime + 1;
      end else
        clint_mtime_clock_scaler <= clint_mtime_clock_scaler - 1;
      uart_tx_valid <= 0;

      if (uart_rx_refill_pending) begin
         uart_rx_front <= uart_rx_fifo[uart_rx_refill_addr];
         uart_rx_front_valid <= 1;
         uart_rx_refill_pending <= 0;
      end

      // Enqueue UART RX data
      if (uart_rx_push) begin
         uart_rx_fifo[uart_rx_tail[UART_FIFO_INDEX_BITS-1:0]] <= uart_rx_data;
         uart_rx_tail <= uart_rx_tail + 1;
         if (uart_rx_empty || (uart_rx_pop && uart_rx_count == 1)) begin
            uart_rx_front <= uart_rx_data;
            uart_rx_front_valid <= 1;
            uart_rx_refill_pending <= 0;
         end
      end

`ifdef PC_TRACE
      if (!dbg_armed && uart_tx_ready && !uart_tx_valid && !uart_tx_empty) begin
`else
      if (uart_tx_ready && !uart_tx_valid && !uart_tx_empty) begin
`endif
         uart_tx_valid <= 1;
         uart_tx_data  <= uart_tx_fifo[uart_tx_head[UART_FIFO_INDEX_BITS-1:0]];
         uart_tx_head  <= uart_tx_head + 1;
         if (uart_tx_count == 1 || uart_tx_full)
            uart_thre_pending <= 1;
      end

      // PLIC gateway model: a source can become pending only while it is not
      // already pending and not in service. Claim clears pending and marks the
      // source in service; completion rearms the gateway.
      plic_pending <= plic_pending | (plic_source_level & ~plic_in_service);

      mmio_write = 0;
      mmio_read = 0;
      frontend_flush_this_cycle = 0;
      f_consumed_hit = 0;
      rf_decode_pop_this_cycle = 0;
      rf_decode_enqueue_this_cycle = 0;
      rf_decode_prearm_block = 0;
      frontend_decode_pending_drain = 0;
      frontend_decode_pending_latch_this_cycle = 0;
      frontend_buf_flush <= 1'b0;
      frontend_buf_fill <= 1'b0;
      ifetch_read <= 0;
      dmem_read   <= 0;
      dmem_write <= 0;
      dmem_write_zero <= 0;
      ptw_direct_read <= 0;
      cache_cbo_flush <= 0;
      hpm_counter_wr_en <= 0;
      hpm_event_wr_en <= 0;
      tlb_4k_wr_en <= 0;
      tlb_2m_wr_en <= 0;
      hpm_tlb_insert_4k_pulse <= 0;
      hpm_tlb_insert_2m_pulse <= 0;
      hpm_tlb_evict_4k_pulse <= 0;
      hpm_tlb_evict_2m_pulse <= 0;
      hpm_tlb_uncached_1g_pulse <= 0;
      hpm_tlb_uncached_napot_pulse <= 0;
      hpm_ptw_leaf_4k_pulse <= 0;
      hpm_ptw_leaf_2m_pulse <= 0;
      hpm_ptw_leaf_1g_pulse <= 0;
      hpm_ptw_leaf_napot_pulse <= 0;

      if (!core_reset_now && frontend_decode_pending_valid &&
          !rf_decode_full) begin
         enqueue_frontend_decode_pending();
      end

      // Pre-register interrupt pending for S_FETCH1 timing closure.
      // Computed from current FFs so the result is available as a stable FF in
      // the NEXT cycle (adding ≤1 cycle of interrupt detection latency, which
      // is architecturally legal).
      begin : pre_intr_precompute
         reg [13:0] pi_pm, pi_ps, pi_raw;
         pi_pm = csr_mip & csr_mie & ~csr_mideleg;
         pi_ps = csr_mip & csr_mie &  csr_mideleg;
         if ((prv == 3 ? mie : 1'b1) && pi_pm != 0)
            pi_raw = pi_pm;
         else if (((prv == 1) ? sie : (prv == 0)) && pi_ps != 0)
            pi_raw = pi_ps;
         else
            pi_raw = 0;
         pre_intr_pending <= pi_raw != 0;
         pre_intr_cause   <= pi_raw[`LOCAL_COUNTER_OVERFLOW_INTERRUPT] ? `LOCAL_COUNTER_OVERFLOW_INTERRUPT :
                             pi_raw[`MACHINE_EXTERNAL_INTERRUPT] ? `MACHINE_EXTERNAL_INTERRUPT :
                             pi_raw[`MACHINE_SOFTWARE_INTERRUPT] ? `MACHINE_SOFTWARE_INTERRUPT :
                             pi_raw[`MACHINE_TIMER_INTERRUPT]    ? `MACHINE_TIMER_INTERRUPT :
                             pi_raw[`SUPERVISOR_EXTERNAL_INTERRUPT] ? `SUPERVISOR_EXTERNAL_INTERRUPT :
                             pi_raw[`SUPERVISOR_SOFTWARE_INTERRUPT] ? `SUPERVISOR_SOFTWARE_INTERRUPT :
                             pi_raw[`SUPERVISOR_TIMER_INTERRUPT] ? `SUPERVISOR_TIMER_INTERRUPT :
                             pi_raw[`USER_EXTERNAL_INTERRUPT]    ? `USER_EXTERNAL_INTERRUPT :
                             pi_raw[`USER_SOFTWARE_INTERRUPT]    ? `USER_SOFTWARE_INTERRUPT :
                                                                   `USER_TIMER_INTERRUPT;
      end

`ifdef PC_TRACE
      if (dbg_busy) begin
         // Gate on !uart_tx_valid so we don't re-assert valid before the UART
         // has latched the previous byte (rs232tx needs ready & valid to
         // coincide for exactly one cycle; ready stays high until it sees valid).
         if (uart_tx_ready && !uart_tx_valid) begin
            uart_tx_valid <= 1;
            uart_tx_data  <= dbg_byte_at(dbg_pos, dbg_pc, dbg_mode, dbg_is_trap, dbg_cause);
            dbg_pos       <= dbg_pos + 1;
            if (dbg_pos == (dbg_is_trap ? 6'd23 : 6'd18))
               dbg_busy <= 0;
         end
      end else begin
`endif

      // Free-running frontend FSM. Runs BEFORE case(state) so f_consumed_hit
      // (blocking assign) propagates to the backend's S_FETCH_BUF_USE arm in
      // the same cycle. F_FETCH_BUF_USE owns the simple cache-hit path:
      // enqueue the just-fetched instruction directly into rf_decode_*.
      // Backend just retires from the queue.
      case (f_state)
        `F_IDLE: begin
           // Self-kick: any cycle where a fetch command is pending and the
           // queue/pending have room, start the lookup. No state-of-backend
           // gating — frontend runs independently. Vintage hazards are
           // handled by latching frontend_cmd_* into f_latched_cmd_* in
           // F_FETCH_BUF_CHECK below.
           if (!core_reset_now && !frontend_flush_this_cycle &&
               frontend_cmd_valid &&
               (!frontend_decode_pending_valid || frontend_decode_pending_drain) &&
               !rf_decode_full && !frontend_miss_valid && !frontend_miss_done &&
               !frontend_redirect_valid) begin
              f_state <= `F_FETCH_BUF_CHECK;
           end
        end
        `F_FETCH_BUF_CHECK: begin
           // Latch the frontend's cache-buffer response AND the cmd context
           // it corresponds to. Backend may mutate frontend_cmd_* in any
           // subsequent cycle; F_FETCH_BUF_USE consumes only f_latched_*.
           f_latched_hit             <= frontend_rsp_hit;
           f_latched_insn            <= frontend_rsp_insn;
           f_latched_decode_next_pc  <= frontend_rsp_next_pc;
           f_latched_next_pc         <= frontend_rsp_predicted_next_pc;
           f_latched_cmd_pc          <= frontend_cmd_pc;
           f_latched_cmd_prv         <= frontend_cmd_prv;
           f_latched_cmd_asid        <= frontend_cmd_asid;
           f_latched_cmd_epoch       <= frontend_cmd_epoch;
           f_state                   <= `F_FETCH_BUF_USE;
        end
        `F_FETCH_BUF_USE: begin
           f_state <= `F_IDLE;
           // Simple hit, using only the latched data — page-boundary
           // translated case and queue/pending pressure fall through to the
           // backend's arm.
           if (f_latched_hit &&
               !(f_latched_cmd_pc[11:0] == 12'hFFE && f_latched_insn[1:0] == 2'b11 &&
                 csr_satp[63:60] == 4'd8 && f_latched_cmd_prv != 3) &&
               !frontend_decode_pending_latch_this_cycle &&
               (!frontend_decode_pending_valid ||
                frontend_decode_pending_drain)) begin
              f_consumed_hit = 1;
              if (!rf_decode_full && !rf_decode_enqueue_this_cycle) begin
                 enqueue_frontend_decode_hit(
                     f_latched_cmd_pc,
                     f_latched_decode_next_pc,
                     f_latched_next_pc,
                     f_latched_insn,
                     f_latched_cmd_prv,
                     f_latched_cmd_epoch);
                 if (decode_enqueue_leaves_frontend_room() &&
                     !frontend_miss_valid && !frontend_miss_done &&
                     !frontend_redirect_valid)
                    f_state <= `F_FETCH_BUF_CHECK;
              end else begin
                 latch_frontend_decode_pending(
                     f_latched_cmd_pc,
                     f_latched_decode_next_pc,
                     f_latched_next_pc,
                     f_latched_insn,
                     f_latched_cmd_prv,
                     f_latched_cmd_epoch);
              end
           end
        end
        default: f_state <= `F_IDLE;
      endcase

      // Back-half EX FSM. Placed lexically before case(state) so future
      // blocking-write signals propagate.
      case (ex_state)
        `EX_IDLE: ;
        `EX_EXECUTE2: begin
           // Compute the integer-ALU writeback value from the pre-decoded
           // exe_add / exe_sext32 set in S_EXECUTE. Kicked at the same
           // cycle as state <= S_EXECUTE2 (default at top of S_EXECUTE).
           // Gate on state == S_EXECUTE2 so we don't clobber write_back_value
           // for branches whose S_EXECUTE arm overrode state to something
           // else (e.g. SC.W/D sets state to a memory path and writes its
           // own reservation-based result).
           if (state == `S_EXECUTE2 && execute_res_valid)
              write_back_value <= exe_sext32 ? {{32{exe_add[31]}}, exe_add[31:0]} : exe_add;
           ex_state <= `EX_IDLE;
        end
        default: ex_state <= `EX_IDLE;
      endcase

      case (state)
        `S_FETCH1: begin
           if (retire_now_q && !csr_mcountinhibit[2] && hpm_mode_enabled(csr_minstretcfg))
              csr_minstret <= csr_minstret + 1;
           retire_now_q <= 0;

           // Reset to default values
           muldiv_p = 0;
           muldiv_output_negate = 0;
           muldiv_output_high_part = 0;
           muldiv_output_sext32 = 0;
           do_atomic = 0;
           cvfpu_in_valid <= 0;
           cvfpu_write_fp <= 1'b1;

`ifdef VERILATOR_COSIM
           // Normal retire: pc/insn/write_back_* still hold the just-completed
           // instruction's data; npc is its post-retire pc. Skip the first
           // fetch (csr_mcycle == 0) and the dummy fetch right after a trap
           // (just_trapped set by S_EXCEPTION). retire_now_q gates out extra
           // S_FETCH1 visits introduced by F_FETCH_BUF_USE's queue path.
           if (retire_now_q && csr_mcycle != 0 && !just_trapped) begin
              cosim_retire(
                  pc,
                  npc,
                  insn,
                  // rd_kind: 2=fp (takes priority — FP writes never coexist with int),
                  //          1=int (nonzero write_back_register), 0=none.
                  write_back_fp_valid        ? 8'd2 :
                  (write_back_register != 0) ? 8'd1 : 8'd0,
                  write_back_fp_valid        ? {3'd0, write_back_fp_register}
                                             : {3'd0, write_back_register},
                  {6'd0, prv_retire},
                  8'd0,
                  {27'd0, fflags},
                  write_back_fp_valid ? write_back_fp_value : write_back_value,
                  64'd0,
                  64'd0,
                  clint_mtime_prev,
                  clint_mtimecmp,
                  csr_mepc,
                  {7'd0, seip}
              );
           end
`endif
`ifdef PC_TRACE
           if (retire_now_q && csr_mcycle != 0 && !just_trapped) begin
              // Arm on first M→S transition (OpenSBI's mret into Linux).
              if (!dbg_armed && prv_retire == 2'd3 && prv != 2'd3) begin
                 dbg_armed <= 1;
              end else if (dbg_armed && dbg_s_count < DBG_N_RETIRE[7:0] && !dbg_busy) begin
                 dbg_busy    <= 1;
                 dbg_pc      <= pc;
                 dbg_mode    <= prv_retire;
                 dbg_is_trap <= 0;
                 dbg_pos     <= 0;
                 dbg_s_count <= dbg_s_count + 1;
              end
           end
`endif
           just_trapped <= 0;
           just_xret <= 0;
           execute_res_valid <= 0;

`ifdef DISASS
`include "disass.vh"
`endif
`ifdef TRACE
           if (retire_now_q && csr_mcycle) begin
              if ((insn & 3) == 3)
                $write("%0d %0d %016x %08x", csr_minstret - 1, prv, pc, insn);
              else
                $write("%0d %0d %016x %04x", csr_minstret - 1, prv, pc, insn[15:0]);
              if (write_back_register != 0)
                $display(" %0d %016x", write_back_register, write_back_value);
              else
                $display("");
           end
`endif

           if (retire_now_q) pc <= npc;

           state <= `S_FETCH_REQ;

           // Use pre-registered interrupt check (computed previous cycle) for timing closure.
           // pre_intr_pending/pre_intr_cause are stable FFs; the path to state_reg is short.
           // Suppress on the S_FETCH1 right after a trap or xRET: in both cases
           // the interrupt-enable/mask was just changed and we want the target
           // instruction to retire before any newly-unmasked interrupt fires.
           cause_intr = 0;
           if (pre_intr_pending && !just_trapped && !just_xret) begin
              clear_frontend_cmd();
              kill_frontend_lookup();
              squash_decode_execute();
              frontend_redirect_valid <= 0;
              cause = pre_intr_cause;
              cause_intr = 1;
              tval = 0;
              if (frontend_miss_valid || frontend_miss_done) begin
                 frontend_miss_wait_action <= FRONTEND_MISS_WAIT_EXCEPTION;
                 state <= `S_FRONTEND_MISS_WAIT;
              end else begin
                 state <= `S_EXCEPTION;
              end
           end else if (frontend_miss_done &&
                        frontend_miss_matches_retire(npc, prv, fetch_epoch)) begin
              frontend_miss_wait_action <= FRONTEND_MISS_WAIT_CONSUME;
              consume_frontend_miss();
           end else if (execute_req_matches_retire(npc, prv, fetch_epoch)) begin
              state <= `S_EXECUTE;
           end else if (execute_req_valid) begin
              redirect_retire_fetch(npc, prv);
              state <= `S_FETCH_REQ;
           end else if (id_matches_retire(npc, prv, fetch_epoch)) begin
              state <= `S_RF;
           end else if (id_valid) begin
              redirect_retire_fetch(npc, prv);
              state <= `S_FETCH_REQ;
           end else if (rf_decode_valid && !frontend_miss_valid) begin
              retire_queued_decode_or_refetch();
           end else if (frontend_miss_valid || frontend_miss_done) begin
              frontend_miss_wait_action <= FRONTEND_MISS_WAIT_CONSUME;
              state <= `S_FRONTEND_MISS_WAIT;
           end else if (frontend_redirect_valid) begin
              clear_frontend_cmd();
              state <= `S_FETCH_REQ;
           end else if (f_consumed_hit) begin
              // The free-running frontend already queued/latch-buffered the
              // matching fetch hit this cycle. Leave the backend in FETCH1 so
              // it can retire from rf_decode next cycle while the frontend
              // starts the next lookup on its own.
              state <= `S_FETCH1;
           end else if (frontend_cmd_fast_ready && frontend_cmd_valid) begin
              clear_frontend_fast_cmd();
              state <= `S_FETCH_BUF_CHECK;
           end else begin
              prepare_current_epoch_fetch(npc, prv);
           end
        end

        `S_FETCH_REQ: begin
`ifdef SIMULATE
           if (fetch_buf_summary_enabled) begin
              if (frontend_rsp_hit) begin
                 fetch_buf_stat_hits <= fetch_buf_stat_hits + 1;
              end else begin
                 fetch_buf_stat_misses <= fetch_buf_stat_misses + 1;
                 if (fetch_buf_stat_misses[17:0] == 18'h3ffff)
                    $display("%05d FETCHBUF SUMMARY hits=%0d misses=%0d",
                             $time,
                             fetch_buf_stat_hits + (frontend_rsp_hit ? 64'd1 : 64'd0),
                             fetch_buf_stat_misses + 64'd1);
              end
           end
`endif

           if (frontend_redirect_valid) begin
              issue_frontend_redirect();
              state <= `S_FETCH_REQ;
           end else if (!frontend_cmd_valid) begin
              clear_frontend_fast_cmd();
              state <= `S_FETCH1;
           end else if ((csr_satp[63:60] != 4'd8 || frontend_cmd_prv == 2'd3) &&
                        !frontend_physical_fetch_ok(frontend_cmd_pc)) begin
              trap_frontend_instruction_access_fault();
           end else begin
              state <= `S_FETCH_BUF_CHECK;
           end
        end

        `S_FETCH_BUF_CHECK: begin
           if ((csr_satp[63:60] != 4'd8 || frontend_cmd_prv == 2'd3) &&
               !frontend_physical_fetch_ok(frontend_cmd_pc)) begin
              trap_frontend_instruction_access_fault();
           end else begin
              // Latching now happens in case(f_state) F_FETCH_BUF_CHECK arm.
              state                    <= `S_FETCH_BUF_USE;
           end
        end

        `S_FETCH_BUF_USE: begin
`ifdef SIMULATE
           if (fetch_buf_summary_enabled) begin
              if (f_latched_hit)
                 fetch_buf_stat_hits <= fetch_buf_stat_hits + 1;
              else begin
                 fetch_buf_stat_misses <= fetch_buf_stat_misses + 1;
                 if (fetch_buf_stat_misses[17:0] == 18'h3ffff)
                    $display("%05d FETCHBUF SUMMARY hits=%0d misses=%0d",
                             $time,
                             fetch_buf_stat_hits +
                             (f_latched_hit ? 64'd1 : 64'd0),
                             fetch_buf_stat_misses + 64'd1);
              end
           end
`endif
           if (f_consumed_hit) begin
              // Frontend just enqueued this fetch via case(f_state) earlier in
              // the cycle; backend returns to retire from the queue.
              state <= `S_FETCH1;
           end else if (f_latched_hit) begin
              accept_instruction_fetch(f_latched_cmd_pc,
                                       f_latched_decode_next_pc,
                                       f_latched_next_pc,
                                       f_latched_insn,
                                       f_latched_cmd_prv,
                                       f_latched_cmd_epoch,
                                       1'b0);
           end else if (!cache_idle) begin
              state <= `S_FETCH_BUF_USE;
           end else begin
              // The miss response path still packages through frontend_cmd_*.
              // Restore the exact command observed by F_FETCH_BUF_CHECK before
              // launching the slow path.
              frontend_cmd_pc <= f_latched_cmd_pc;
              frontend_cmd_prv <= f_latched_cmd_prv;
              frontend_cmd_asid <= f_latched_cmd_asid;
              frontend_cmd_epoch <= f_latched_cmd_epoch;
              start_instruction_fetch_miss(f_latched_cmd_pc, f_latched_cmd_prv);
           end
        end

        `S_FRONTEND_MISS_WAIT: begin
           if (frontend_miss_done) begin
              if (frontend_miss_wait_action == FRONTEND_MISS_WAIT_EXCEPTION) begin
                 frontend_miss_valid <= 0;
                 frontend_miss_done  <= 0;
                 state <= `S_EXCEPTION;
              end else if (frontend_miss_wait_action == FRONTEND_MISS_WAIT_CONSUME &&
                           frontend_miss_matches_retire(npc, prv, fetch_epoch)) begin
                 consume_frontend_miss();
              end else begin
                 frontend_miss_valid <= 0;
                 frontend_miss_done  <= 0;
                 redirect_retire_fetch(npc, prv);
                 state <= `S_FETCH_REQ;
              end
           end else begin
              try_issue_queued_decode_no_pending(1'b1);
           end
        end

        `S_IFETCH_RESP: consume_latched_ifetch_response();

        `S_RF: begin
           if (!id_valid) begin
              state <= `S_FETCH1;
           end else if (!id_rf_ready) begin
              id_rf_ready <= 1;
              state <= `S_RF;
           end else if (!id_ex_fire) begin
              state <= `S_RF;
           end else begin
              prepare_execute_req_from_id(1'b0);
           end
        end

        `S_EXECUTE: begin
           if (!execute_req_valid) begin
`ifdef SIMULATE
              $display("%05d BUG: S_EXECUTE without execute request", $time);
              $finish;
`endif
              state <= `S_FETCH1;
           end else begin
           pc <= ex_pc;
           insn <= ex_insn;
           execute_req_valid <= 0;
           execute_res_valid <= 1;
           state <= `S_EXECUTE2; // Default: complete write_back_value
           ex_state <= `EX_EXECUTE2; // case(ex_state) owns write_back_value compute
           prv_retire <= prv;    // snapshot pre-execution prv (MRET/SRET mutate prv below)

           imm_i = {{52{ex_insn[31]}},ex_insn[31:20]};
           imm_j = {{44{ex_insn[31]}},ex_insn[19:12],ex_insn[20],ex_insn[30:21],1'd0};
           imm_b = {{52{ex_insn[31]}},ex_insn[7],ex_insn[30:25],ex_insn[11:8],1'd0};
           imm_u = {{32{ex_insn[31]}},ex_insn[31:12],12'd0};
           imm_s = {{52{ex_insn[31]}},ex_insn[31:25],ex_insn[11:7]};

           c_nzuimm107_1211_5_6_x4 = {ex_insn[10:7],ex_insn[12:11],ex_insn[5],ex_insn[6],2'd0};
           c_uimm5_1210_6_x4       = {ex_insn[5],ex_insn[12:10],ex_insn[6],2'd0};
           c_imm12_62              = {{59{ex_insn[12]}},ex_insn[6:2]};
           c_imm12_43_5_2_6_x16    = {{55{ex_insn[12]}},ex_insn[4:3],ex_insn[5],ex_insn[2],ex_insn[6],4'd0};
           c_imm12_8_109_6_7_2_11_53_x2
                                   = {{53{ex_insn[12]}},ex_insn[8],ex_insn[10:9],ex_insn[6],ex_insn[7],ex_insn[2],ex_insn[11],ex_insn[5:3],
                                      1'd0};
           c_imm12_65_2_1110_43_x2 = {{56{ex_insn[12]}},ex_insn[6:5],ex_insn[2],ex_insn[11:10],ex_insn[4:3],1'd0};
           c_uimm42_12_65_x8       = {ex_insn[4:2],ex_insn[12],ex_insn[6:5],3'd0};
           c_uimm32_12_64_x4       = {ex_insn[3:2],ex_insn[12],ex_insn[6:4],2'd0};
           c_uimm97_1210_x8        = {ex_insn[9:7],ex_insn[12:10],3'd0};
           c_uimm87_129_x4         = {ex_insn[8:7],ex_insn[12:9],2'd0};

           c_uimm65_1210_x8        = {ex_insn[6:5],ex_insn[12:10],3'd0};

           csrno                   = ex_insn[31:20];

           npc = pre_npc;

           // Default: this instruction targets no register. Each writing
           // instruction sets these in its arm below; non-writing ones
           // (branches, JALR x0, MRET/SRET, fences, ...) leave them cleared so
           // they don't inherit the previous instruction's writeback, which
           // would cause a redundant regfile write (write_valid is gated on
           // write_back_register != 0) and a cosim divergence.
           write_back_register = 0;
           write_back_fp_valid = 0;

           // RV64IC decoding
           //
           // The order of instructions [mostly] follows
           // simmerv for ease of reference, who in turn took the
           // ordering from the RISC-V spec.  There is intentionally
           // _no_ overlap in patterns so the order is not important,
           // but we keep the if-else chain in order to catch the
           // unhandled instructions.

           // Shared mem-access block — collapses all load/store/LR/SC/AMO
           // branches using the pre-decoded signals from id_rf_mem_decode.
           // One execute_req_rs1_value+execute_req_mem_offset adder replaces 22 parallel copies,
           // shrinking the mem_addr critical path from ~14 LUT levels to ~7.
           if (execute_req_mem_op != `MEMOP_NONE) begin
              if (execute_req_mem_fp && fs == 0) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
              if (execute_req_mem_fp) fs = 3;
              write_back_register    = execute_req_mem_fp ? 5'd0 : execute_req_mem_wb_reg;
              write_back_fp_valid    = execute_req_mem_fp && execute_req_mem_op == `MEMOP_LOAD;
              write_back_fp_register = execute_req_mem_wb_reg;
              mem_addr      = execute_req_rs1_value + execute_req_mem_offset;
              mem_va        = execute_req_rs1_value + execute_req_mem_offset;
              mem_asid      <= {TLB_ASID_BITS{1'b0}};
              mem_perm      <= CACHE_PERM_PHYS;
              mem_ctx       <= {((execute_req_mem_op == `MEMOP_STORE || execute_req_mem_op == `MEMOP_SC) ? 2'd2 :
                                 (execute_req_mem_op == `MEMOP_AMO ? 2'd3 : 2'd1)),
                                (mprv ? mpp : prv), sum, mxr};
              load_size_lg2 = execute_req_load_size_lg2;
              begin : mem_access_dispatch
                 reg [12:0] mem_access_bytes;

                 case (execute_req_mem_op)
                   `MEMOP_STORE,
                   `MEMOP_SC: begin
                      case (execute_req_mem_wr_mask)
                        8'hff: mem_access_bytes = 13'd8;
                        8'h0f: mem_access_bytes = 13'd4;
                        8'h03: mem_access_bytes = 13'd2;
                        default: mem_access_bytes = 13'd1;
                      endcase
                   end
                   default: mem_access_bytes = 13'd1 << execute_req_load_size_lg2[1:0];
                 endcase

                 if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 &&
                     (execute_req_mem_op != `MEMOP_SC || reservation_match) &&
                     ({1'b0, mem_addr[11:0]} + mem_access_bytes > 13'd4096)) begin
                    cause = (execute_req_mem_op == `MEMOP_STORE || execute_req_mem_op == `MEMOP_SC || execute_req_mem_op == `MEMOP_AMO)
                            ? `TRAP_STORE_ADDRESS_MISALIGNED
                            : `TRAP_LOAD_ADDRESS_MISALIGNED;
                    tval = mem_addr;
                    write_back_register = 0;
                    write_back_fp_valid = 0;
                    state <= `S_EXCEPTION;
                 end else begin
                    case (execute_req_mem_op)
                       `MEMOP_LOAD: state <= `S_LOAD_ALIGN;
                       `MEMOP_STORE: begin
                          mem_wr_mask = execute_req_mem_wr_mask;
                          store_value = execute_req_mem_fp ? execute_req_frs2_value : execute_req_rs2_value;
                          state <= `S_STORE;
                       end
                       `MEMOP_LR: begin
                          reservation <= execute_req_rs1_value;
                          state <= `S_LOAD_ALIGN;
                       end
                       `MEMOP_SC: begin
                          if (reservation_match) begin
                             write_back_value <= 0;
                             mem_wr_mask = execute_req_mem_wr_mask;
                             store_value = execute_req_rs2_value;
                             state <= `S_STORE;
                          end
                          // SC fail: write_back_value = 1 from EXOP_ONE in id_rf_pre_decode;
                          // default state <= S_EXECUTE2 at top of S_EXECUTE retires it.
                       end
                       `MEMOP_AMO: begin
                          do_atomic <= 1;
                          state <= `S_LOAD_ALIGN;
                       end
                       default: ;
                    endcase
                 end
              end
              end // else: !(execute_req_mem_fp && fs == 0)
           end

           // Quadrant 0
           else if ((ex_insn & 'he003) == 'h0000) begin // C.ADDI4SPN/illegal
              write_back_register = ex_rs2;
              if ((ex_insn & 'hffff) == 0) begin
                 write_back_register = 0;
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end
           end

           // Compressed integer/FP loads and stores are handled by the shared mem block above.

              // Quadrant 1
           else if (ex_insn == 1) begin // C.NOP
             // NOP
             retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'he003) == 'h0001) begin // C.ADDI
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'h2001) begin // C.ADDIW
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'h4001) begin // C.LI
              write_back_register = ex_insn[11:7];
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'hef83) == 'h6101) begin // C.ADDI16SP
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'h6001) begin // C.LUI
              write_back_register = ex_rs1;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'hec03) == 'h8001) begin // C.SRLI
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hec03) == 'h8401) begin // C.SRAI
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hec03) == 'h8801) begin // C.ANDI
              write_back_register = ex_rs1;
              retire_int_value_prepared_fetch(execute_req_rs1_value & execute_req_alu_b);
           end

           else if ((ex_insn & 'hfc63) == 'h8c01) begin // C.SUB
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hfc63) == 'h8c21) begin // C.XOR
              write_back_register = ex_rs1;
              retire_int_value_prepared_fetch(execute_req_rs1_value ^ execute_req_alu_b);
           end

           else if ((ex_insn & 'hfc63) == 'h8c41) begin // C.OR
              write_back_register = ex_rs1;
              retire_int_value_prepared_fetch(execute_req_rs1_value | execute_req_alu_b);
           end

           else if ((ex_insn & 'hfc63) == 'h8c61) begin // C.AND
              write_back_register = ex_rs1;
              retire_int_value_prepared_fetch(execute_req_rs1_value & execute_req_alu_b);
           end

           else if ((ex_insn & 'hfc63) == 'h9c01) begin // C.SUBW
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hfc63) == 'h9c21) begin // C.ADDW
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'ha001) begin // C.J
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'he003) == 'hc001) begin // C.BEQZ
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'he003) == 'he001) begin // C.BNEZ
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end


              // Quadrant 2
           else if ((ex_insn & 'he003) == 'h0002) begin // C.SLLI
              write_back_register = ex_rs1;
           end

           // C.LWSP / C.LDSP / C.FLDSP handled by shared mem block above.

           else if ((ex_insn & 'hf07f) == 'h8002) begin // C.JR
              npc = pre_jalr_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'hf003) == 'h8002) begin // C.MV
              write_back_register = ex_rs1;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'hffff) == 'h9002) begin // C.EBREAK
              cause = `TRAP_BREAKPOINT;
              tval = 0;
              state <= `S_EXCEPTION;
           end

           else if ((ex_insn & 'hf07f) == 'h9002) begin // C.JALR
              write_back_register = 1;
              npc = pre_jalr_target;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'hf003) == 'h9002) begin // C.ADD
              write_back_register = ex_rs1;
           end

           // C.SWSP / C.SDSP / C.FSDSP handled by shared mem block above.

           // Quadrant 3, uncompressed
           else if ((ex_insn & 'h0000007f) == 'h00000037) begin // LUI
              write_back_register = ex_rd;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'h0000007f) == 'h00000017) begin // AUIPC
              write_back_register = ex_rd;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'h0000007f) == 'h0000006f) begin // JAL
              write_back_register = ex_rd;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'h0000707f) == 'h00000067) begin // JALR
              write_back_register = ex_rd;
              npc = pre_jalr_target;
              retire_execute_req_alu_b();
           end

           else if ((ex_insn & 'h0000707f) == 'h00000063) begin // BEQ
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00001063) begin // BNE
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00004063) begin // BLT
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00005063) begin // BGE
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00006063) begin // BLTU
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00007063) begin // BGEU
              if (pre_branch_taken) npc = pre_branch_target;
              retire_no_wb_prepared_fetch();
           end

           // LB/LH/LW/LD/LBU/LHU/LWU and SB/SH/SW/SD handled by shared mem block above.

           else if ((ex_insn & 'h0000707f) == 'h00000013) begin // ADDI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h00002013) begin // SLTI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h00003013) begin // SLTIU
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h00004013) begin // XORI
              write_back_register = ex_rd;
              retire_int_value_prepared_fetch(execute_req_rs1_value ^ execute_req_alu_b);
           end

           else if ((ex_insn & 'h0000707f) == 'h00006013) begin // ORI
              write_back_register = ex_rd;
              retire_int_value_prepared_fetch(execute_req_rs1_value | execute_req_alu_b);
           end

           else if ((ex_insn & 'h0000707f) == 'h00007013) begin // ANDI
              write_back_register = ex_rd;
              retire_int_value_prepared_fetch(execute_req_rs1_value & execute_req_alu_b);
           end

           else if ((ex_insn & 'hfe00707f) == 'h00000033) begin // ADD
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h40000033) begin // SUB
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00001033) begin // SLL
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00002033) begin // SLT
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00003033) begin // SLTU
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00004033) begin // XOR
              write_back_register = ex_rd;
              retire_int_value_prepared_fetch(execute_req_rs1_value ^ execute_req_alu_b);
           end

           else if ((ex_insn & 'hfe00707f) == 'h00005033) begin // SRL
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h40005033) begin // SRA
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00006033) begin // OR
              write_back_register = ex_rd;
              retire_int_value_prepared_fetch(execute_req_rs1_value | execute_req_alu_b);
           end

           else if ((ex_insn & 'hfe00707f) == 'h00007033) begin // AND
              write_back_register = ex_rd;
              retire_int_value_prepared_fetch(execute_req_rs1_value & execute_req_alu_b);
           end

           else if ((ex_insn & 'hf000707f) == 'h0000000f) begin // FENCE
              // Nothing to do here
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'hf000707f) == 'h8000000f) begin // FENCE.TSO
              // Nothing to do here
              retire_no_wb_prepared_fetch();
           end

           else if ((ex_insn & 'hfff0707f) == 'h0000200f || // CBO.INVAL
                    (ex_insn & 'hfff0707f) == 'h0010200f || // CBO.CLEAN
                    (ex_insn & 'hfff0707f) == 'h0020200f) begin // CBO.FLUSH
              mem_addr = execute_req_rs1_value;
              mem_va = execute_req_rs1_value;
              mem_asid <= {TLB_ASID_BITS{1'b0}};
              mem_perm <= CACHE_PERM_PHYS;
              mem_ctx <= {2'd2, (mprv ? mpp : prv), sum, mxr};
              translated <= 0;
              if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3)
                 start_translation(mem_addr, 2'd2, mprv ? mpp : prv, `S_CBO_EXEC);
              else
                 state <= `S_CBO_EXEC;
           end

           else if ((ex_insn & 'hfff0707f) == 'h0040200f) begin // CBO.ZERO (Zicboz)
              // Whole-line zero. Implemented as a cacheable write of zeros to
              // the 64-byte block: the cache zeroes all 8 banks in one cycle via
              // cache_req_zero (per-bank write-enable + uniform zero), so the
              // ordinary store path handles translation, install/hit, dirty
              // marking, and retire. A 64-aligned 64-byte block never crosses a
              // page, so one translation covers it. Translated as a write
              // (access 2'd2) so it faults on read-only pages and sets D.
              //   COLD-MISS TODO: a cold miss still fetches the line from memory
              //   before overwriting it with zeros (read-for-ownership). The
              //   future optimization is to skip the fill on cache_req_zero and
              //   jump straight to a zeroed dirty install. See OPTIMIZATIONS.md.
              // Ungated on menvcfg/senvcfg CBZE, matching cbo.clean/flush/inval
              // and the simmerv cosim model (OpenSBI sets CBZE before S-mode).
              mem_addr = {execute_req_rs1_value[63:6], 6'd0};
              mem_va   = {execute_req_rs1_value[63:6], 6'd0};
              mem_asid <= {TLB_ASID_BITS{1'b0}};
              mem_perm <= CACHE_PERM_PHYS;
              mem_ctx  <= {2'd2, (mprv ? mpp : prv), sum, mxr};
              mem_wr_mask = 8'hff;
              store_value = 64'd0;
              translated <= 0;
              execute_req_cbo_zero <= 1'b1;
              state <= `S_STORE;
           end

           else if ((ex_insn & 'hffffffff) == 'h00000073) begin // ECALL
              cause = `TRAP_ENVIRONMENT_CALL_FROM_U_MODE + prv;
              tval = 0;
              state <= `S_EXCEPTION;
`ifdef SIMULATE
`ifdef VERBOSE
              $display("ECALL: pc %x prv %d time %0t", ex_pc, prv, $time);
`endif
`endif
           end

           else if ((ex_insn & 'hffffffff) == 'h00100073) begin // EBREAK
              cause = `TRAP_BREAKPOINT;
              tval = 0;
              state <= `S_EXCEPTION;
           end

           else if ((ex_insn & 'hfc00707f) == 'h00001013) begin // SLLI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfc00707f) == 'h00005013) begin // SRLI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfc00707f) == 'h40005013) begin // SRAI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h0000001b) begin // ADDIW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000101b) begin // SLLIW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000501b) begin // SRLIW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h4000501b) begin // SRAIW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000003b) begin // ADDW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h4000003b) begin // SUBW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000103b) begin // SLLW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000503b) begin // SRLW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h4000503b) begin // SRAW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hffffffff) == 'h0000100f) begin // FENCE.I
              flush_frontend_speculation;
              fetch_epoch <= fetch_epoch + 1'b1;
              vhpr_request_full_flush(8'd1);
              execute_res_valid <= 0;
              state <= `S_CBO_WAIT;
           end

           else if ((ex_insn & 'h0000707f) == 'h00001073) begin // CSRRW
              // CSRRW and CSRRWI (and only those) do not read the CSR
              // if rd == 0 This matters [only] if the read has side
              // effects (I'm guilty of this part of RISC-V semantics).
              csr_op = `CSR_OP_COPY;
              csr_arg = execute_req_rs1_value;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00002073) begin // CSRRS
              csr_op = `CSR_OP_OR;
              csr_arg = execute_req_rs1_value;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00003073) begin // CSRRC
              csr_op = `CSR_OP_ANDN;
              csr_arg = execute_req_rs1_value;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00005073) begin // CSRRWI
              csr_op = `CSR_OP_COPY;
              csr_arg = ex_rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00006073) begin // CSRRSI
              csr_op = `CSR_OP_OR;
              csr_arg = ex_rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00007073) begin // CSRRCI
              csr_op = `CSR_OP_ANDN;
              csr_arg = ex_rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02000033) begin // MUL
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MUL;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02001033) begin // MULH
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULH;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02002033) begin // MULHSU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULHSU;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02003033) begin // MULHU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULHU;
              state <= `S_MULDIV_START;
           end


           else if ((ex_insn & 'hfe00707f) == 'h02004033) begin // DIV
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIV;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02005033) begin // DIVU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIVU;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02006033) begin // REM
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REM;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02007033) begin // REMU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REMU;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200003b) begin // MULW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200403b) begin // DIVW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIVW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200503b) begin // DIVUW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIVUW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200603b) begin // REMW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REMW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200703b) begin // REMUW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REMUW;
              state <= `S_MULDIV_START;
           end

           // LR.W/D, SC.W/D, and all AMO*.W/D variants handled by shared mem block above.

           else if ((ex_insn & 'hffffffff) == 'h30200073) begin // MRET
              frontend_buf_flush <= 1'b1;
              kill_frontend_lookup();
              if (mpp != 3) mprv = 0;
              prv = mpp;
              mpp = 0;
              mie = mpie;
              mpie = 1;
              npc = csr_mepc;
              just_xret <= 1;
              retire_now_q <= 1; // MRET retires; bypasses retire helpers
              state <= `S_FETCH1;
           end

           else if ((ex_insn & 'hffffffff) == 'h10200073) begin // SRET
              if (prv == 0 || prv == 1 && tsr) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 frontend_buf_flush <= 1'b1;
                 kill_frontend_lookup();
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("SRET: pc %x prv %d->%d sepc %x time %0t", ex_pc, prv, spp, csr_sepc, $time);
`endif
`endif
                 mprv = 0; // sret can only return to S or U, never M
                 prv = spp;
                 spp = 0;
                 sie = spie;
                 spie = 1;
                 npc = csr_sepc;
                 just_xret <= 1;
              end
           end

           else if ((ex_insn & 'hfe007fff) == 'h12000073) begin // SFENCE.VMA
              if (prv < 1 || prv == 1 && tvm) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
             end else begin
                 flush_frontend_speculation;
                 fetch_epoch <= fetch_epoch + 1'b1;
                 flush_tlb;
                 vhpr_request_full_flush(8'd2);
                 execute_res_valid <= 0;
                 state <= `S_CBO_WAIT;
              end
           end

           else if ((ex_insn & 'hffffffff) == 'h10500073) begin // WFI
              if (prv == 0 || prv == 1 && tw) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 retire_no_wb_prepared_fetch(); // treat as NOP (no real sleep in simulation)
              end
           end

           // OP-FP (opcode 0x53): full F/D — arithmetic, conversions,
           // sign-injection, compares, classify, and FMV, via the CV-FPU.
           // Unrecognized funct7/rm encodings fall through to illegal.
           else if (ex_insn[6:0] == 7'b1010011) begin
              if (fs == 0) begin
                 // FP state disabled by mstatus.FS: any FP instruction traps.
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 // Reaching an FP instruction dirties the FP state.
                 fs = 3;
                 case (ex_insn[31:25])
                   // FADD.S / FSUB.S
                   7'b0000000,
                   7'b0000100: begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(64'd0, execute_req_frs1_value, execute_req_frs2_value, pre_fp_rnd_mode,
                                           4'd2, ex_insn[27],
                                           3'd0, 3'd0, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FADD.D / FSUB.D
                   7'b0000001,
                   7'b0000101: begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(64'd0, execute_req_frs1_value, execute_req_frs2_value, pre_fp_rnd_mode,
                                           4'd2, ex_insn[27],
                                           3'd1, 3'd1, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FMUL.S
                   7'b0001000: begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, execute_req_frs2_value, 64'd0, pre_fp_rnd_mode,
                                           4'd3, 1'b0,
                                           3'd0, 3'd0, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FMUL.D
                   7'b0001001: begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, execute_req_frs2_value, 64'd0, pre_fp_rnd_mode,
                                           4'd3, 1'b0,
                                           3'd1, 3'd1, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FDIV.S
                   7'b0001100: begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, execute_req_frs2_value, 64'd0, pre_fp_rnd_mode,
                                           4'd4, 1'b0,
                                           3'd0, 3'd0, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FDIV.D
                   7'b0001101: begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, execute_req_frs2_value, 64'd0, pre_fp_rnd_mode,
                                           4'd4, 1'b0,
                                           3'd1, 3'd1, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FSQRT.S
                   7'b0101100: if (ex_insn[24:20] == 5'd0) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd5, 1'b0,
                                           3'd0, 3'd0, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FSQRT.D
                   7'b0101101: if (ex_insn[24:20] == 5'd0) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd5, 1'b0,
                                           3'd1, 3'd1, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMIN.S / FMAX.S
                   7'b0010100: begin
                      if (ex_insn[14:12] > 3'b001) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, execute_req_frs2_value, 64'd0,
                                           {2'b00, ex_insn[12]},
                                           4'd7, 1'b0,
                                           3'd0, 3'd0, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FMIN.D / FMAX.D
                   7'b0010101: begin
                      if (ex_insn[14:12] > 3'b001) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, execute_req_frs2_value, 64'd0,
                                           {2'b00, ex_insn[12]},
                                           4'd7, 1'b0,
                                           3'd1, 3'd1, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end
                   // FCVT.S.D
                   7'b0100000: if (ex_insn[24:20] == 5'd1) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd10, 1'b0,
                                           3'd1, 3'd0, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.D.S
                   7'b0100001: if (ex_insn[24:20] == 5'd0) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd10, 1'b0,
                                           3'd0, 3'd1, 2'd3,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FSGNJ/N/X .S — NaN-box-check operands; NaN-box result.
                   7'b0010000: begin
                      case (ex_insn[14:12])
                        3'b000: retire_fp_value_linear_fetch(
                           ex_rd, {32'hffffffff, execute_req_frs2_s[31], execute_req_frs1_s[30:0]});
                        3'b001: retire_fp_value_linear_fetch(
                           ex_rd, {32'hffffffff, ~execute_req_frs2_s[31], execute_req_frs1_s[30:0]});
                        3'b010: retire_fp_value_linear_fetch(
                           ex_rd, {32'hffffffff, execute_req_frs2_s[31] ^ execute_req_frs1_s[31], execute_req_frs1_s[30:0]});
                        default: begin
                           cause = `TRAP_ILLEGAL_INSTRUCTION;
                           tval = ex_insn;
                           state <= `S_EXCEPTION;
                        end
                      endcase
                   end
                   // FSGNJ/N/X .D — no boxing check; 64-bit direct.
                   7'b0010001: begin
                      case (ex_insn[14:12])
                        3'b000: retire_fp_value_linear_fetch(
                           ex_rd, {execute_req_frs2_value[63], execute_req_frs1_value[62:0]});
                        3'b001: retire_fp_value_linear_fetch(
                           ex_rd, {~execute_req_frs2_value[63], execute_req_frs1_value[62:0]});
                        3'b010: retire_fp_value_linear_fetch(
                           ex_rd, {execute_req_frs2_value[63] ^ execute_req_frs1_value[63], execute_req_frs1_value[62:0]});
                        default: begin
                           cause = `TRAP_ILLEGAL_INSTRUCTION;
                           tval = ex_insn;
                           state <= `S_EXCEPTION;
                        end
                      endcase
                   end
                   // FEQ.S / FLT.S / FLE.S: integer rd; NV flag on NaN per op.
                   7'b1010000: if (ex_insn[14:12] <= 3'b010) begin
                      fcmp_result = fcmp_s(ex_insn[14:12], execute_req_frs1_s, execute_req_frs2_s);
                      write_back_register = ex_rd;
                      stage_int_commit_result({63'd0, fcmp_result[0]},
                                              fcmp_result[1] ? 5'b10000 : 5'd0,
                                              1'b0);
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FEQ.D / FLT.D / FLE.D
                   7'b1010001: if (ex_insn[14:12] <= 3'b010) begin
                      fcmp_result = fcmp_d(ex_insn[14:12], execute_req_frs1_value, execute_req_frs2_value);
                      write_back_register = ex_rd;
                      stage_int_commit_result({63'd0, fcmp_result[0]},
                                              fcmp_result[1] ? 5'b10000 : 5'd0,
                                              1'b0);
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.W[U].S / FCVT.L[U].S
                   7'b1100000: if (ex_insn[24:20] <= 5'd3) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd11, ex_insn[20],
                                           3'd0, 3'd0,
                                           ex_insn[21] ? 2'd3 : 2'd2,
                                           {3'd0, ex_rd}, 1'b0);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.W[U].D / FCVT.L[U].D
                   7'b1100001: if (ex_insn[24:20] <= 5'd3) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_frs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd11, ex_insn[20],
                                           3'd1, 3'd0,
                                           ex_insn[21] ? 2'd3 : 2'd2,
                                           {3'd0, ex_rd}, 1'b0);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.S.W[U] / FCVT.S.L[U]
                   7'b1101000: if (ex_insn[24:20] <= 5'd3) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_rs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd12, ex_insn[20],
                                           3'd0, 3'd0,
                                           ex_insn[21] ? 2'd3 : 2'd2,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.D.W[U] / FCVT.D.L[U]
                   7'b1101001: if (ex_insn[24:20] <= 5'd3) begin
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         start_cvfpu_issue(execute_req_rs1_value, 64'd0, 64'd0, pre_fp_rnd_mode,
                                           4'd12, ex_insn[20],
                                           3'd0, 3'd1,
                                           ex_insn[21] ? 2'd3 : 2'd2,
                                           {3'd0, ex_rd}, 1'b1);
                      end
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.X.W (rs2=0, rm=0) or FCLASS.S (rs2=0, rm=1).
                   7'b1110000: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      write_back_register = ex_rd;
                      stage_int_commit_result({{32{execute_req_frs1_value[31]}}, execute_req_frs1_value[31:0]}, 5'd0, 1'b0);
                   end else if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b001) begin
                      write_back_register = ex_rd;
                      stage_int_commit_result(fclass_s(execute_req_frs1_value), 5'd0, 1'b0);
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.X.D (rs2=0, rm=0) or FCLASS.D (rs2=0, rm=1).
                   7'b1110001: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      write_back_register = ex_rd;
                      stage_int_commit_result(execute_req_frs1_value, 5'd0, 1'b0);
                   end else if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b001) begin
                      write_back_register = ex_rd;
                      stage_int_commit_result(fclass_d(execute_req_frs1_value), 5'd0, 1'b0);
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.W.X (rs2=0, rm=0): NaN-box execute_req_rs1_value[31:0] into f[rd].
                   7'b1111000: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      retire_fp_value_linear_fetch(ex_rd, {32'hffffffff, execute_req_rs1_value[31:0]});
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.D.X (rs2=0, rm=0): full 64-bit move.
                   7'b1111001: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      retire_fp_value_linear_fetch(ex_rd, execute_req_rs1_value);
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   default: begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                 endcase
              end
           end

           // R4 fused multiply-add/subtract family: FMADD/FMSUB/FNMSUB/FNMADD.
           else if (ex_insn[6:4] == 3'b100 && ex_insn[1:0] == 2'b11) begin
              if (fs == 0) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 fs = 3;
                 if (ex_insn[26:25] > 2'b01 || !pre_fp_rmode_ok) begin
                    cause = `TRAP_ILLEGAL_INSTRUCTION;
                    tval = ex_insn;
                    state <= `S_EXCEPTION;
                 end else begin
                    // rs3 was read on its own FP port and latched in S_RF, so
                    // issue the fused op directly (no rs1-repurpose detour).
                    start_cvfpu_issue(execute_req_frs1_value,
                                      execute_req_frs2_value,
                                      execute_req_frs3_value,
                                      pre_fp_rnd_mode,
                                      ex_insn[3] ? 4'd1 : 4'd0,
                                      ex_insn[2],
                                      {2'd0, ex_insn[25]},
                                      {2'd0, ex_insn[25]},
                                      2'd3,
                                      {3'd0, ex_rd},
                                      1'b1);
                 end
              end
           end

           else begin
`ifdef SIMULATE
`ifdef VERBOSE
              if (ex_insn[1:0] == 3)
                $display("%05d   %1d %x %x illegal unknown instruction", $time, prv, ex_pc, ex_insn);
              else
                $display("%05d   %1d %x     %x illegal unknown instruction (%1d,%1d)",
                         $time, prv, ex_pc, ex_insn[15:0], ex_insn[15:13], ex_insn[1:0]);
              $finish;
`endif
`endif
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              tval = ex_insn;
              state <= `S_EXCEPTION;
           end

           // Pre-registered ALU computation: smolrv64_alu (alu_inst) combinationally
           // decodes execute_req_alu_op/rs1/alu_b/sxt; we register its result here.
           // Both operands and selector are flip-flops, so the critical path is
           // only ~7 LUT levels (vs ~15 with the old inline if-else).
           exe_add <= alu_result;
           exe_sext32 <= execute_req_alu_sxt;

           end
        end

        `S_EXECUTE2: begin : execute2_stage
           if (execute_res_valid) begin
              // write_back_value compute moved to case(ex_state) EX_EXECUTE2.
              execute_res_valid <= 0;
              retire_current_wb_linear_fetch();
           end else begin
              prepare_current_epoch_fetch(npc, prv);
              state <= `S_FETCH1;
           end
        end

        `S_INT_COMMIT: begin
           write_back_value <= int_commit_result;
           fflags = fflags | int_commit_fflags;
           if (int_commit_redirect_fetch) begin
              retire_current_wb_redirect_fetch();
           end else if (int_commit_prepared_fetch) begin
              execute_res_valid <= 0;
              retire_current_wb_prepared_fetch();
           end else begin
              retire_current_wb_linear_fetch();
           end
        end

        `S_CVFPU_ISSUE: begin
           if (cvfpu_in_ready) begin
              cvfpu_in_valid <= 1'b0;
              if (cvfpu_out_valid) begin
                 retire_cvfpu_output();
              end else begin
                 state <= `S_CVFPU_WAIT;
              end
           end else begin
              try_launch_queued_decode_preserve_state();
           end
        end

        `S_CVFPU_WAIT: begin
           if (cvfpu_out_valid) begin
              retire_cvfpu_output();
           end else begin
              try_issue_queued_decode_tagged_wb(1'b1, cvfpu_write_fp, cvfpu_tag_in[4:0]);
           end
        end

        `S_CBO_EXEC: begin
           translated <= 0;
           if (phys_region(mem_addr) == `REGION_BRAM ||
               phys_region(mem_addr) == `REGION_DRAM) begin
              if (cache_idle) begin
                 cache_cbo_line_addr <= mem_addr[30:6];
                 cache_cbo_flush <= 1;
                 state <= `S_CBO_WAIT;
              end
           end else begin
              retire_no_wb_linear_fetch();
           end
        end

        `S_CBO_WAIT: begin
           if (cache_cbo_done) begin
              retire_no_wb_linear_fetch();
           end else begin
              try_issue_queued_decode_no_pending(1'b1);
           end
        end

        `S_STORE: begin

           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 store address translation
              start_translation(mem_addr, 2'd2, mprv ? mpp : prv, `S_STORE);
           end else if (phys_region(mem_addr) == `REGION_UART) begin
              translated <= 0;
              retire_no_wb_linear_fetch();
              reservation <= ~0;

              // Keep UART writes on the original store cycle; only BRAM writes
              // need the delayed commit state for SRAM WE timing.
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // THR (when DLAB=0)
`ifdef PC_TRACE
                      if (!dbg_armed) begin
                         if (uart_tx_accept) begin
                            uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                            uart_tx_tail <= uart_tx_tail + 1;
                            uart_thre_pending <= 0;
                         end
                      end
`else
                      if (uart_tx_accept) begin
                         uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                         uart_tx_tail <= uart_tx_tail + 1;
                         uart_thre_pending <= 0;
                      end
`endif
                   end
                1: if (!uart_lcr[7]) begin // Only bits [3:0] valid
                   uart_ier <= store_value[3:0];
                   if (!store_value[1])
                      uart_thre_pending <= 0;
                   else if (!uart_ier[1] && uart_tx_accept)
                      uart_thre_pending <= 1;
                end
                2: begin // FCR (write-only)
                   uart_fcr_fifo <= store_value[0];
                   if (store_value[1]) begin
                      uart_rx_head <= 0;
                      uart_rx_tail <= 0;
                      uart_rx_front_valid <= 0;
                      uart_rx_refill_pending <= 0;
                   end
                   if (store_value[2]) begin
                      uart_tx_head <= 0;
                      uart_tx_tail <= 0;
                      if (uart_ier[1])
                         uart_thre_pending <= 1;
                   end
                end
                3: uart_lcr <= store_value[7:0];
                4: uart_mcr <= store_value[4:0];
                7: uart_scr <= store_value[7:0];
                default: begin end
              endcase
              mem_wr_mask = 0;
           end else begin
              state <= `S_STORE_COMMIT;
           end
        end

        `S_STORE_COMMIT: begin
           translated <= 0;
           retire_no_wb_linear_fetch();
           reservation <= ~0;

`ifdef RISCV_TESTS
           // riscv-tests signal completion by storing gp to "tohost". In the
           // FPGA-like sim config the program image lives behind AXI, so this
           // terminating store must be recognized before the store is routed.
           if (mem_addr == tohost_phys && mem_wr_mask[0] && store_value != 0) begin
              if (store_value == 1)
                 $display("Test Passed");
              else
                 $display("Test Failed with %3d", store_value);
              $finish;
           end
`endif

           case (phys_region(mem_addr))
             `REGION_UART: begin
              // NS16550A UART write (0x10000000-0x1000000F)
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // THR (when DLAB=0)
`ifdef PC_TRACE
                      if (!dbg_armed) begin
                         if (uart_tx_accept) begin
                            uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                            uart_tx_tail <= uart_tx_tail + 1;
                            uart_thre_pending <= 0;
                         end
                      end
`else
                      if (uart_tx_accept) begin
                         uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                         uart_tx_tail <= uart_tx_tail + 1;
                         uart_thre_pending <= 0;
                      end
`endif
                   end
                1: if (!uart_lcr[7]) begin // Only bits [3:0] valid
                   uart_ier <= store_value[3:0];
                   if (!store_value[1])
                      uart_thre_pending <= 0;
                   else if (!uart_ier[1] && uart_tx_accept)
                      uart_thre_pending <= 1;
                end
                2: begin // FCR (write-only)
                   uart_fcr_fifo <= store_value[0];
                   if (store_value[1]) begin
                      uart_rx_head <= 0;
                      uart_rx_tail <= 0;
                      uart_rx_front_valid <= 0;
                      uart_rx_refill_pending <= 0;
                   end
                   if (store_value[2]) begin
                      uart_tx_head <= 0;
                      uart_tx_tail <= 0;
                      if (uart_ier[1])
                         uart_thre_pending <= 1;
                   end
                end
                3: uart_lcr <= store_value[7:0];
                4: uart_mcr <= store_value[4:0];
                7: uart_scr <= store_value[7:0];
                default: begin end
              endcase
              mem_wr_mask = 0;
             end
             `REGION_CLINT: begin
              // CLINT: 0x02000000 msip, 0x02004000 mtimecmp, 0x0200BFF8 mtime
              case (mem_addr[15:0])
                16'h0000: clint_msip <= store_value[0];
                16'h4000: if (mem_wr_mask[4])
                             clint_mtimecmp <= store_value;
                          else
                             clint_mtimecmp[31:0] <= store_value[31:0];
                16'h4004: clint_mtimecmp[63:32] <= store_value[31:0];
                16'hBFF8: if (mem_wr_mask[4])
                             clint_mtime <= store_value;
                          else
                             clint_mtime[31:0] <= store_value[31:0];
                16'hBFFC: clint_mtime[63:32] <= store_value[31:0];
                default: begin end
              endcase
              mem_wr_mask = 0;
             end
             `REGION_PLIC: begin
              // PLIC write (base 0x0C000000)
              if (mem_addr[23:0] <= 24'h0000FF)
                 plic_priority[mem_addr[7:2]] <= store_value[2:0];
              else if (mem_addr[23:0] >= 24'h002080 && mem_addr[23:0] <= 24'h002087) begin
                 if (mem_wr_mask[4])
                    plic_enabled <= store_value;
                 else if (mem_addr[2])
                    plic_enabled[63:32] <= store_value[31:0];
                 else
                    plic_enabled[31:0] <= store_value[31:0];
              end else if (mem_addr[23:0] >= 24'h201000 && mem_addr[23:0] <= 24'h201003)
                 plic_threshold <= store_value[2:0];
              else if (mem_addr[23:0] >= 24'h201004 && mem_addr[23:0] <= 24'h201007) begin
                 // Completion: rearm the source gateway. If the source is
                 // still asserted, it will become pending again next cycle.
                 if (store_value[5:0] != 0)
                    plic_in_service[store_value[5:0]] <= 0;
              end
              mem_wr_mask = 0;
             end
             `REGION_BRAM,
             `REGION_DRAM: begin
              // Cacheable store.  The cache refill/writeback engine chooses
              // BRAM or AXI backing by line address; MMIO never reaches here.
             begin : cacheable_store_calc
                 reg [127:0] wide_data;
                 reg  [15:0] wide_mask;
                 wide_data = {64'd0, store_value} << (mem_addr[2:0] * 8);
                 wide_mask = {8'd0, mem_wr_mask[7:0]} << mem_addr[2:0];
                 cache_issue_dw_addr <= mem_addr[30:3];
                 cache_issue_va      <= mem_va;
                 cache_issue_asid    <= mem_asid;
                 cache_issue_perm    <= mem_perm;
                 cache_issue_ctx     <= mem_ctx;
                 dmem_write_data    <= wide_data[63:0];
                 dmem_write_strb    <= wide_mask[7:0];
                 mem_wr_mask        = 0;
                 if (|wide_mask[15:8]) begin
                    // Overflow into next 8-byte chunk: save for S_DMEM_STORE2
                    dmem_store2_dw_addr <= mem_addr[30:3] + 1;
                    dmem_store2_va      <= {mem_va[63:3], 3'b000} + 64'd8;
                    dmem_store2_asid    <= mem_asid;
                    dmem_store2_perm    <= mem_perm;
                    dmem_store2_ctx     <= mem_ctx;
                    dmem_store2_data    <= wide_data[127:64];
                    dmem_store2_strb    <= wide_mask[15:8];
                    dmem_store_split    <= 1;
                    if (dmem_write_ready) begin
                       dmem_write <= 1;
                       dmem_write_zero <= execute_req_cbo_zero;
                       state      <= `S_DMEM_STORE_RESP_ARM;
                    end else begin
                       state      <= `S_DMEM_STORE_WAIT;
                    end
                 end else begin
                    dmem_store_split  <= 0;
                    if (dmem_write_ready) begin
                       dmem_write <= 1;
                       dmem_write_zero <= execute_req_cbo_zero;
                       state      <= `S_DMEM_STORE_RESP_ARM;
                    end else begin
                       state      <= `S_DMEM_STORE_WAIT;
                    end
                 end
              end
             end
             `REGION_MMIO: begin
`ifdef TRACE_MMIO
             $display("%05d  MMIO WRITE %x/%x <- %x", $time, mem_addr, mem_wr_mask, store_value);
`endif
              mmio_address = mem_addr;
              mmio_write = 1;
              mmio_writedata = store_value << (8 * (mem_addr % 4));
              mmio_byteenable = mem_wr_mask << (mem_addr % 4);
              mem_wr_mask = 0;
             end
             default: begin
`ifdef SIMULATE
`ifdef VERBOSE
              $display("%05d   %x xxxxxxxx illegal store address %x", $time, prv, mem_addr);
`endif
`endif
              cause = `TRAP_STORE_ACCESS_FAULT;
              tval = mem_addr;
              mem_wr_mask = 0;
              state <= `S_EXCEPTION;
             end
           endcase

        end

        `S_LOAD_ALIGN: begin
           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 load/AMO address translation
              start_translation(mem_addr, do_atomic ? 2'd3 : 2'd1, mprv ? mpp : prv, `S_LOAD_ALIGN);
           end else begin
              if (!do_atomic) translated <= 0;

              state <= `S_FETCH1;

              if (do_atomic)
                state <= `S_AMO;

              case (phys_region(mem_addr))
                `REGION_UART,
                `REGION_CLINT,
                `REGION_PLIC: begin
                 state <= `S_LOCAL_LOAD;
                end
                `REGION_MMIO: begin
`ifdef TRACE_MMIO
                 $display("%05d  MMIO READ FROM %x/%x", $time, mem_addr, load_size_lg2);
`endif
                 state <= `S_MMIO_ALIGN;
                 mmio_address = mem_addr;
                 mmio_read = 1;
                 mmio_timeout_tval <= mem_va;
                end
                `REGION_BRAM,
                `REGION_DRAM: begin
                 // Cacheable load.  The cache refill engine chooses BRAM or
                 // AXI by line address; MMIO stays on the explicit slow path.
                 issue_dmem_cache_read(mem_addr[30:3], mem_va, mem_asid,
                                       mem_perm, mem_ctx);
                 state           <= `S_DMEM_LOAD_WAIT;
                end
                default: begin
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %x xxxxxxxx illegal load address %x", $time, prv, mem_addr);
`endif
`endif
                 write_back_register = 0;
                 cause = `TRAP_LOAD_ACCESS_FAULT;
                 tval = mem_addr;
                 state <= `S_EXCEPTION;
                end
              endcase
          end
        end

        `S_LOCAL_LOAD: begin
           if (!do_atomic)
              retire_current_wb_linear_fetch();
           if (do_atomic)
              state <= `S_AMO;

           case (phys_region(mem_addr))
             `REGION_UART: begin
              // NS16550A UART read (0x10000000-0x1000000F)
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // RBR (when DLAB=0)
                      write_back_value = uart_rx_front_valid ? uart_rx_front : 0;
                      if (uart_rx_pop) begin
                         uart_rx_head <= uart_rx_head + 1;
                         if (uart_rx_count > 1) begin
                            uart_rx_front_valid <= 0;
                            uart_rx_refill_pending <= 1;
                            uart_rx_refill_addr <= uart_rx_head[UART_FIFO_INDEX_BITS-1:0] + 1'b1;
                         end else if (!uart_rx_push) begin
                            uart_rx_front_valid <= 0;
                         end
                      end
                   end else
                      write_back_value = 0; // DLL (divisor, ignored)
                1: write_back_value = uart_lcr[7] ? 0 : {4'd0, uart_ier[3:0]};
                2: begin
                   write_back_value = uart_iir;
                   if (uart_iir_thre)
                      uart_thre_pending <= 0;
                end
                3: write_back_value = uart_lcr;
                4: write_back_value = uart_mcr;
                5: write_back_value = uart_lsr;
                6: write_back_value = 8'hB0; // MSR: CTS+DSR+CD asserted
                7: write_back_value = uart_scr;
                default: write_back_value = 0;
              endcase
              // Byte/half sign extension for LB/LH (load_size_lg2 = {sxt, size}).
              if (load_size_lg2 == 4) // LB
                 write_back_value = {{56{write_back_value[7]}}, write_back_value[7:0]};
              else if (load_size_lg2 == 5) // LH
                 write_back_value = {{48{write_back_value[15]}}, write_back_value[15:0]};
             end
             `REGION_CLINT: begin
              // CLINT read: return value directly, no MMIO bus
              case (mem_addr[15:0])
                16'h0000: write_back_value = {63'd0, clint_msip};
                16'h4000: write_back_value = clint_mtimecmp;
                16'h4004: write_back_value = clint_mtimecmp[63:32];
                16'hBFF8: write_back_value = clint_mtime;
                16'hBFFC: write_back_value = clint_mtime[63:32];
                default:  write_back_value = 0;
              endcase
              // 32-bit loads need sign/zero extension
              if (load_size_lg2 == 2)
                 write_back_value = write_back_value[31:0];
              else if (load_size_lg2 == 6)
                 write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};
             end
             `REGION_PLIC: begin
              // PLIC read (base 0x0C000000)
              if (mem_addr[23:0] <= 24'h0000FF)
                 write_back_value = plic_priority[mem_addr[7:2]];
              else if (mem_addr[23:0] >= 24'h001000 && mem_addr[23:0] <= 24'h00107F)
                 write_back_value = plic_pending >> ((mem_addr[6:0] - 7'h00) * 8);
              else if (mem_addr[23:0] >= 24'h002080 && mem_addr[23:0] <= 24'h002087)
                 write_back_value = plic_enabled;
              else if (mem_addr[23:0] >= 24'h201000 && mem_addr[23:0] <= 24'h201003)
                 write_back_value = plic_threshold;
              else if (mem_addr[23:0] >= 24'h201004 && mem_addr[23:0] <= 24'h201007) begin
                 write_back_value = plic_best_irq;
                 if (plic_best_irq != 0) begin
                    plic_pending[plic_best_irq] <= 0;
                    plic_in_service[plic_best_irq] <= 1;
                 end
              end
              else
                 write_back_value = 0;
              if (load_size_lg2 == 2)
                 write_back_value = write_back_value[31:0];
              else if (load_size_lg2 == 6)
                 write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};
             end
             default: begin
              write_back_register = 0;
              cause = `TRAP_LOAD_ACCESS_FAULT;
              tval = mem_addr;
              state <= `S_EXCEPTION;
             end
           endcase
           finish_load_writeback();
        end

        `S_MMIO_ALIGN: begin
           if (mmio_readdatavalid) begin

              aligned = mmio_readdata >> (mem_addr[1:0] * 8);

/*
              $display("%x >> %d = %x ==? %x",
                       mmio_readdata,
                       (mem_addr[1:0] * 8),
                       mmio_readdata >> (mem_addr[1:0] * 8),
                       aligned);
*/

              case (load_size_lg2)
                0: write_back_value = aligned[ 7:0];
                1: write_back_value = aligned[15:0];
                2: write_back_value = mmio_readdata;
                3: write_back_value = 64'h 6464DEADDEAD6464;
                4: write_back_value = {{56{aligned[ 7]}},aligned[ 7:0]};
                5: write_back_value = {{48{aligned[15]}},aligned[15:0]};
                6: write_back_value = {{32{mmio_readdata[31]}},mmio_readdata[31:0]};
                7: write_back_value = 64'h DEAD_BEEF_C0DE_CAFE;
              endcase

`ifdef TRACE_MMIO
              $display("%05d  MMIO READ GOT %x (aligned %x)", $time, mmio_readdata, write_back_value);
`endif

              finish_load_writeback();
              if (do_atomic) begin
                 retire_linear_fetch();
`ifdef SIMULATE
                 $display("Sorry, atomics to MMIO aren't supported yet");
                 $finish;
`endif
                 state <= `S_AMO;
              end else begin
                 retire_current_wb_linear_fetch();
              end
           end else begin
              try_issue_queued_decode_current_wb(!do_atomic);
           end
        end

        `S_AMO: begin
           // Note: translated stays set from S_LOAD_ALIGN PTW (ptw_access=3
           // already checked both read and write permission), so S_STORE
           // will skip re-translation and use the physical mem_addr directly.
           mem_wr_mask = 255;
           store_value = execute_req_rs2_value;
           if (!ex_insn[12]) begin
              write_back_value = {{32{write_back_value[31]}},write_back_value[31:0]};
              store_value = {{32{execute_req_rs2_value[31]}},execute_req_rs2_value[31:0]};
              mem_wr_mask = 15;
           end

           // ex_insn[31:27]=funct5, ex_insn[26]=aq, ex_insn[25]=rl, ex_insn[24]=rs2[4]
           // Mask aq/rl/rs2[4] so any register and any ordering variant is handled.
           case ({ex_insn[31:27], 3'b0})
             'h08: begin end // AMOSWAP
             'h00: store_value = store_value + write_back_value; // AMOADD
             'h20: store_value = store_value ^ write_back_value; // AMOXOR
             'h60: store_value = store_value & write_back_value; // AMOAND
             'h40: store_value = store_value | write_back_value; // AMOOR
             'h80: store_value = $signed(store_value) < $signed(write_back_value) ? store_value : write_back_value; // AMOMIN
             'ha0: store_value = $signed(store_value) < $signed(write_back_value) ? write_back_value : store_value; // AMOMAX
             'hc0: store_value = store_value < write_back_value ? store_value : write_back_value; // AMOMINU
             'he0: store_value = store_value < write_back_value ? write_back_value : store_value; // AMOMAXU
             default: begin
`ifdef SIMULATE
                $display("Impossible AMO"); $finish;
`endif
                store_value = 64'hX;
             end
           endcase

           state <= `S_STORE;
        end


        `S_HANDLE_CSR: begin : handle_csr_state
           reg csr_write_failure;
           state <= `S_INT_COMMIT;
           int_commit_fflags <= 5'd0;
           int_commit_prepared_fetch <= 1'b1;
           int_commit_redirect_fetch <= 1'b0;
           csr_access_failure = 0;
           csr_write_failure = 0;
           csr_read_val = 0;
           tval = ex_insn;
           write_back_register = ex_rd;

           if (ex_rd != 0 || csr_op != `CSR_OP_COPY) begin
              // read the CSR

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, reads zero
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) csr_read_val = 0;
              // mhpmevent3..31 / mhpmcounter3..31 / hpmcounter3..31: the full
              // architectural range exists. Counters/events 3..(2+HPM_COUNTERS)
              // are implemented; the rest are hardwired 0 (read 0, no trap) per
              // the privileged spec.
              else if (`CSR_MHPMEVENT3 <= csrno && csrno <= `CSR_MHPMEVENT3 + 12'd28) begin
                 if (csrno - `CSR_MHPMEVENT3 < `HPM_COUNTERS)
                    csr_read_val = csr_mhpmevent[(csrno - `CSR_MHPMEVENT3)];
                 else
                    csr_read_val = 0;
              end else if (`CSR_MHPMCOUNTER3 <= csrno && csrno <= `CSR_MHPMCOUNTER3 + 12'd28) begin
                 if (csrno - `CSR_MHPMCOUNTER3 < `HPM_COUNTERS)
                    csr_read_val = csr_mhpmcounter[(csrno - `CSR_MHPMCOUNTER3)];
                 else
                    csr_read_val = 0;
              end else if (`CSR_HPMCOUNTER3 <= csrno && csrno <= `CSR_HPMCOUNTER3 + 12'd28) begin
                 if (csrno - `CSR_HPMCOUNTER3 < `HPM_COUNTERS)
                    csr_read_val = csr_mhpmcounter[(csrno - `CSR_HPMCOUNTER3)];
                 else
                    csr_read_val = 0;
              end
              else case (csrno)
                `CSR_FFLAGS:    csr_read_val = {59'd0, fflags};
                `CSR_FRM:       csr_read_val = {61'd0, frm};
                `CSR_FCSR:      csr_read_val = {56'd0, frm, fflags};
                `CSR_SSTATUS:
                  csr_read_val = {sd, 29'd0,            uxl, 12'd0,  // 63:20
                                                    mxr, sum, 1'd0,  // 19:17
                                  xs,   fs,         4'd0,      spp,  // 16: 8
                                  2'd0, spie, upie, 2'd0, sie, uie}; //  7: 0
                `CSR_SIE:       csr_read_val = csr_mie & 14'h2222;
                `CSR_STVEC:     csr_read_val = csr_stvec;
                `CSR_SCOUNTEREN:csr_read_val = csr_scounteren;
                `CSR_SENVCFG:   csr_read_val = {56'd0, csr_senvcfg};
                `CSR_SSCRATCH:  csr_read_val = csr_sscratch;
                `CSR_SEPC:      csr_read_val = csr_sepc;
                `CSR_SCAUSE:    csr_read_val = csr_scause;
                `CSR_STVAL:     csr_read_val = csr_stval;
                `CSR_SIP:       csr_read_val = csr_mip & csr_mideleg;
                `CSR_STIMECMP:  csr_read_val = csr_stimecmp;
                `CSR_SCOUNTOVF: csr_read_val = csr_scountovf_read_val;

                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      state <= `S_EXCEPTION;
                   end else
                     csr_read_val = csr_satp;
                end
                `CSR_MSTATUS:
                  csr_read_val = {sd, 27'd0,  sxl,  uxl, 9'd0,                  // 63:23
                                              tsr,  tw,   tvm, mxr, sum, mprv,  // 22:17
                                  xs,         fs,         mpp, 2'd0,      spp,  // 16: 8
                                  mpie, 1'd0, spie, upie, mie, 1'd0, sie, uie}; //  7: 0
                // RV64GC = I M A F D C, plus S and U.  F and D are fully
                // implemented via the CV-FPU.
                `CSR_MISA:     csr_read_val = 64'h800000000014112d;
                // Hardwired 1 0100 0001 0001 0010 1101
                //    ZY XWV U TSRQ PONM LKJI HGFE DCBA
                //           U  S      M    I F  DC A
                //    SUIMAFDC
                `CSR_MEDELEG:  csr_read_val = csr_medeleg;
                `CSR_MIDELEG:  csr_read_val = csr_mideleg;
                `CSR_MIE:      csr_read_val = csr_mie;
                `CSR_MTVEC:    csr_read_val = csr_mtvec;
                `CSR_MCOUNTEREN: csr_read_val = csr_mcounteren;
                `CSR_MENVCFG:  csr_read_val = csr_menvcfg;
                `CSR_MCOUNTINHIBIT: csr_read_val = csr_mcountinhibit & `HPM_INHIBIT_MASK;
                `CSR_MCYCLECFG: csr_read_val = csr_mcyclecfg;
                `CSR_MINSTRETCFG: csr_read_val = csr_minstretcfg;
                `CSR_MSCRATCH: csr_read_val = csr_mscratch;
                `CSR_MEPC:     csr_read_val = csr_mepc;
                `CSR_MCAUSE:   csr_read_val = csr_mcause;
                `CSR_MTVAL:    csr_read_val = csr_mtval;
                `CSR_MIP:      csr_read_val = csr_mip;
                `CSR_TSELECT:  csr_read_val = 0;
                `CSR_TDATA1:   csr_read_val = 0;
                `CSR_TDATA2:   csr_read_val = 0;
                `CSR_TDATA3:   csr_read_val = 0;
                `CSR_TINFO:    csr_read_val = 0;
                `CSR_MCYCLE:   csr_read_val = csr_mcycle;
                `CSR_MTIME:    csr_read_val = clint_mtime; // mtime mirrored as a CSR (non-standard)
                `CSR_MINSTRET: csr_read_val = csr_minstret;
                `CSR_CYCLE:    csr_read_val = csr_mcycle;
                `CSR_TIME:     csr_read_val = clint_mtime; // time: the CLINT mtime (rdtime source)
                `CSR_INSTRET:  csr_read_val = csr_minstret;
                `CSR_MHARTID:  csr_read_val = 0;
                `CSR_MVENDORID:csr_read_val = 0;
                `CSR_MARCHID:  csr_read_val = 9; // YARVI, Smolrv64 = YARVI4
                `CSR_MIMPID:   csr_read_val = `SMOLRV64_GIT_COMMIT;
                `CSR_MIG_MIN:  csr_read_val = {32'd0, csr_mig_min};
                `CSR_MIG_MAX:  csr_read_val = {32'd0, csr_mig_max};
                `CSR_MIG_TOTAL:csr_read_val = csr_mig_total;
                `CSR_MIG_COUNT:csr_read_val = csr_mig_count;
                `CSR_MIG_TIMEOUTS:csr_read_val = csr_mig_timeouts;
                `CSR_MIG_TO_PC:   csr_read_val = csr_mig_to_pc;
                `CSR_MIG_TO_TVAL: csr_read_val = csr_mig_to_tval;
                `CSR_MIG_TO_STATE:csr_read_val = csr_mig_to_state;
                `CSR_MIG_TO_CAUSE:csr_read_val = csr_mig_to_cause;
                `CSR_MIG_TO_ADDR: csr_read_val = csr_mig_to_addr;
                `CSR_VHPR_EPOCH: csr_read_val = {{64-VHPR_EPOCH_BITS{1'b0}}, vhpr_epoch};
                `CSR_BUILD_STAMP: csr_read_val = `SMOLRV64_BUILD_STAMP;
                default: begin
`ifdef SIMULATE
`ifdef VERBOSE
                   $display("%05d   %1d %x %x illegal CSR %x (read)", $time, prv, pc, insn, csrno);
`endif
`endif
                   cause = `TRAP_ILLEGAL_INSTRUCTION;
                   state <= `S_EXCEPTION;
                end
              endcase

              // As there are no side effects (besides exceptions) yet, we
              // can postpone the privilege check to here
              if (prv < csrno[9:8]) begin
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %1d %x %x mode %d isn't priviledged to read CSR %x", $time,
                          prv, pc, insn, prv, csrno);
`endif
`endif
                 csr_access_failure = 1;
              end
              // Sstc: stimecmp is accessible from S-mode only when menvcfg.STCE=1
              if (csrno == `CSR_STIMECMP && prv == 1 && !csr_menvcfg[63])
                 csr_access_failure = 1;
              if (!csr_access_failure) begin
                 if (csrno == `CSR_CYCLE && !counter_access_allowed(6'd0))
                    csr_access_failure = 1;
                 else if (csrno == `CSR_TIME && !counter_access_allowed(6'd1))
                    csr_access_failure = 1;
                 else if (csrno == `CSR_INSTRET && !counter_access_allowed(6'd2))
                    csr_access_failure = 1;
                 else if (`CSR_HPMCOUNTER3 <= csrno && csrno <= `CSR_HPMCOUNTER3 + (`HPM_COUNTERS - 1) &&
                          !counter_access_allowed({1'b0, csrno[4:0]}))
                    csr_access_failure = 1;
              end
           end

           // Write privilege check
           if (ex_rs1 != 0 || csr_op == `CSR_OP_COPY) begin
              if (prv < csrno[9:8]) begin
                 csr_write_failure = 1;
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %1d %x %x mode isn't priviledged to write CSR %x", $time,
                          prv, pc, insn, csrno);
`endif
`endif
              end

              if (csrno[11:10] == 3) begin
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %1d %x %x write attempt to Read Only CSR %x", $time,
                          prv, pc, insn, csrno);
`endif
`endif
                 csr_write_failure = 1;
              end

              // Sstc: stimecmp is accessible from S-mode only when menvcfg.STCE=1
              if (csrno == `CSR_STIMECMP && prv == 1 && !csr_menvcfg[63])
                 csr_write_failure = 1;
           end



           // CSRRS, CSRRC, CSRRSI, and CSRRCI don't write the CSR if rs1 == 0
           if (!csr_write_failure && (ex_rs1 != 0 || csr_op == `CSR_OP_COPY)) begin
              // write the CSR

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, writes silently ignored
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) begin end
              // mhpmevent3..31 / mhpmcounter3..31: writes to implemented
              // counters/events take effect; writes to the hardwired-0 ones
              // (16..31) are silently ignored (no trap) per the spec.
              else if (`CSR_MHPMEVENT3 <= csrno && csrno <= `CSR_MHPMEVENT3 + 12'd28) begin
                 if (csrno - `CSR_MHPMEVENT3 < `HPM_COUNTERS) begin
                    hpm_event_wr_en <= 1;
                    hpm_wr_idx <= (csrno - `CSR_MHPMEVENT3);
                    hpm_wr_data <= csr_modify_value(csr_mhpmevent[(csrno - `CSR_MHPMEVENT3)],
                                                     csr_arg, csr_op);
                 end
              end else if (`CSR_MHPMCOUNTER3 <= csrno && csrno <= `CSR_MHPMCOUNTER3 + 12'd28) begin
                 if (csrno - `CSR_MHPMCOUNTER3 < `HPM_COUNTERS) begin
                    hpm_counter_wr_en <= 1;
                    hpm_wr_idx <= (csrno - `CSR_MHPMCOUNTER3);
                    hpm_wr_data <= csr_modify_value(csr_mhpmcounter[(csrno - `CSR_MHPMCOUNTER3)],
                                                     csr_arg, csr_op);
                 end
              end
              else case (csrno)
                // fcsr: fflags aliased at [4:0], frm aliased at [7:5].
                // Writing any of these is an implicit "FP state touched"
                // event, so we mark FS=Dirty at the same time.
                `CSR_FFLAGS: begin : csr_write_fflags
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({59'd0, fflags}, csr_arg, csr_op);
                   fflags = csr_next[4:0];
                   fs = 3;
                end
                `CSR_FRM: begin : csr_write_frm
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({61'd0, frm}, csr_arg, csr_op);
                   frm = legal_frm_value(csr_next[2:0]);
                   fs = 3;
                end
                `CSR_FCSR: begin : csr_write_fcsr
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({56'd0, frm, fflags}, csr_arg, csr_op);
                   frm = legal_frm_value(csr_next[7:5]);
                   fflags = csr_next[4:0];
                   fs = 3;
                end
                `CSR_SSTATUS: begin : csr_write_sstatus
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({sd, 29'd0, uxl, 12'd0, mxr, sum, 1'd0,
                                                xs, fs, 4'd0, spp, 2'd0, spie, upie, 2'd0, sie, uie},
                                               csr_arg, csr_op);
`ifdef SIMULATE
`ifdef VERBOSE
                   if (sum != csr_next[18])
                      $display("SSTATUS: sum %d->%d pc %x time %0t", sum, csr_next[18], pc, $time);
`endif
`endif
                   {mxr, sum}        = csr_next[19:18];
                   fs                = csr_next[14:13];
                   spp               = csr_next[8];
                   {spie, upie}      = csr_next[5:4];
                   {sie, uie}        = csr_next[1:0];
                end
                `CSR_SIE:       csr_mie    = csr_modify_value(csr_mie & 14'h2222, csr_arg, csr_op) & 14'h2222 | csr_mie & ~14'h2222;
                `CSR_STVEC:     csr_stvec  = csr_modify_value(csr_stvec, csr_arg, csr_op);
                `CSR_SCOUNTEREN:csr_scounteren = csr_modify_value(csr_scounteren, csr_arg, csr_op) & `HPM_COUNTER_MASK;
                `CSR_SENVCFG: begin : csr_write_senvcfg
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({56'd0, csr_senvcfg}, csr_arg, csr_op);
                   csr_senvcfg = csr_next[7:0] & 8'hf1;
                end
                `CSR_SSCRATCH:  csr_sscratch = csr_modify_value(csr_sscratch, csr_arg, csr_op);
                `CSR_SEPC:      csr_sepc   = csr_modify_value(csr_sepc, csr_arg, csr_op) & ~1;
                `CSR_SCAUSE:    csr_scause = csr_modify_value(csr_scause, csr_arg, csr_op);
                `CSR_STVAL:     csr_stval  = csr_modify_value(csr_stval, csr_arg, csr_op);
                `CSR_SIP:       begin : csr_write_sip
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value(csr_mip & csr_mideleg, csr_arg, csr_op);
                   // Only SSIP (bit 1) is writable via SIP; SEIP/STIP are read-only
                   if (csr_mideleg[13]) lcofip = csr_next[13];
                   if (csr_mideleg[1]) ssip = csr_next[1];
                   if (csr_mideleg[0]) usip = csr_next[0];
                end
                `CSR_STIMECMP:  csr_stimecmp = csr_modify_value(csr_stimecmp, csr_arg, csr_op);
                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      state <= `S_EXCEPTION;
                   end else begin
                     case (csr_op)
                       `CSR_OP_COPY: csr_satp_write_val = csr_arg;
                       `CSR_OP_OR:   csr_satp_write_val = csr_satp | csr_arg;
                       default:      csr_satp_write_val = csr_satp & ~csr_arg;
                     endcase

                     csr_satp_write_val = satp_warl_value(csr_satp_write_val);
                     if (csr_satp_write_val[63:60] == 4'd0 ||
                         csr_satp_write_val[63:60] == 4'd8) begin
                       // WARL: only Bare and Sv39 are supported; unsupported
                       // MODE values cause the entire write to have no effect.
                       // Treat a value-changing SATP write as fetch-serializing:
                       // the new translation must apply to the next fetch, so
                       // redirect/refetch npc under it rather than accepting the
                       // frontend's prefetch made under the old satp. The ASID-
                       // tagged TLB is left intact (no TLB flush needed).
                       if (csr_satp_write_val != csr_satp)
                          int_commit_redirect_fetch <= 1'b1;
                       csr_satp = csr_satp_write_val;
                     end
                   end
                end
                `CSR_MSTATUS: begin : csr_write_mstatus
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({sd, 27'd0, sxl, uxl, 9'd0,
                                                tsr, tw, tvm, mxr, sum, mprv,
                                                xs, fs, mpp, 2'd0, spp,
                                                mpie, 1'd0, spie, upie, mie, 1'd0, sie, uie},
                                               csr_arg, csr_op);
                   {sie, uie}        = csr_next[1:0];
                   {spie, upie, mie} = csr_next[5:3];
                   {spp, mpie}       = csr_next[8:7];
                   mpp               = csr_next[12:11];
                   fs                = csr_next[14:13];
                   {tsr, tw, tvm, mxr, sum, mprv} = csr_next[22:17];
                end
                `CSR_MISA:     begin end
                `CSR_MEDELEG:  csr_medeleg  = csr_modify_value(csr_medeleg, csr_arg, csr_op);
                `CSR_MIDELEG:  csr_mideleg  = csr_modify_value(csr_mideleg, csr_arg, csr_op);
                `CSR_MIE:      csr_mie      = csr_modify_value(csr_mie, csr_arg, csr_op);
                `CSR_MTVEC:    csr_mtvec    = csr_modify_value(csr_mtvec, csr_arg, csr_op); // XXX enforce 256-byte alignment for vectored interrupts
                `CSR_MCOUNTEREN: csr_mcounteren = csr_modify_value(csr_mcounteren, csr_arg, csr_op) & `HPM_COUNTER_MASK;
                // menvcfg: stored WARL, matching simmerv's raw store. The gb5
                // DT advertises no sstc/zicboz, so STCE/CBZE are never enabled.
                `CSR_MENVCFG:  csr_menvcfg  = csr_modify_value(csr_menvcfg, csr_arg, csr_op);
                `CSR_MCOUNTINHIBIT: csr_mcountinhibit = csr_modify_value(csr_mcountinhibit, csr_arg, csr_op) & `HPM_INHIBIT_MASK;
                `CSR_MCYCLECFG: csr_mcyclecfg = csr_modify_value(csr_mcyclecfg, csr_arg, csr_op);
                `CSR_MINSTRETCFG: csr_minstretcfg = csr_modify_value(csr_minstretcfg, csr_arg, csr_op);
                `CSR_MSCRATCH: csr_mscratch = csr_modify_value(csr_mscratch, csr_arg, csr_op);
                `CSR_MEPC:     csr_mepc     = csr_modify_value(csr_mepc, csr_arg, csr_op) & ~1;
                `CSR_MCAUSE:   csr_mcause   = csr_modify_value(csr_mcause, csr_arg, csr_op);
                `CSR_MTVAL:    csr_mtval    = csr_modify_value(csr_mtval, csr_arg, csr_op);
                `CSR_MIP:      begin : csr_write_mip
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value(csr_mip, csr_arg, csr_op);
                   // MEIP/SEIP (bits 11,9) are read-only, driven by PLIC
                   // MTIP/MSIP (bits 7,3) are read-only, driven by CLINT
                   lcofip = csr_next[13];
                   ueip = csr_next[8];
                   stip_sw = csr_next[5];
                   utip = csr_next[4];
                   ssip = csr_next[1];
                   usip = csr_next[0];
                end
                `CSR_TSELECT:  begin end
                `CSR_TDATA1:   begin end
                `CSR_TDATA2:   begin end
                `CSR_TDATA3:   begin end
                `CSR_MCYCLE:   csr_mcycle   <= csr_modify_value(csr_mcycle, csr_arg, csr_op);
                `CSR_MINSTRET: csr_minstret <= csr_modify_value(csr_minstret, csr_arg, csr_op);
                // Any write to any mig_* CSR clears all four to their initial
                // sentinels (fresh measurement window).  The written value is
                // ignored; this is the "clear stats" knob for the monitor.
                `CSR_MIG_MIN,
                `CSR_MIG_MAX,
                `CSR_MIG_TOTAL,
                `CSR_MIG_COUNT,
                `CSR_MIG_TIMEOUTS,
                `CSR_MIG_TO_PC,
                `CSR_MIG_TO_TVAL,
                `CSR_MIG_TO_STATE,
                `CSR_MIG_TO_CAUSE,
                `CSR_MIG_TO_ADDR: begin
                   csr_mig_min      <= 32'hFFFFFFFF;
                   csr_mig_max      <= 0;
                   csr_mig_total    <= 0;
                   csr_mig_count    <= 0;
                   csr_mig_timeouts <= 0;
                   csr_mig_to_pc    <= 0;
                   csr_mig_to_tval  <= 0;
                   csr_mig_to_state <= 0;
                   csr_mig_to_cause <= 0;
                   csr_mig_to_addr  <= 0;
                end
                default: begin
                 csr_write_failure = 1;
`ifdef SIMULATE
`ifdef VERBOSE
                   $display("%05d   %1d %x %x illegal CSR %x (write)", $time, prv, pc, insn, csrno);
`endif
`endif
                end
              endcase
           end

           int_commit_result <= csr_read_val;
           if (csr_access_failure || csr_write_failure) begin
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              state <= `S_EXCEPTION;
           end

           // Any CSR write may change interrupt-enable/pending state
           // (sstatus/mstatus/sie/mie/mip/mideleg). pre_intr_pending
           // is a FF sampled the cycle BEFORE S_FETCH1 from current CSR
           // FF values, so it is stale for one cycle after any CSR write.
           // Reuse just_xret as a generic one-cycle suppress flag.
           just_xret <= 1;
        end

        `S_EXCEPTION: begin
`ifdef SIMULATE
`ifdef VERBOSE
           $display("%05d  ** Exception, cause %x, pc %x, tval %x, prv %d", $time, cause, pc, tval, prv);
           $fflush(0);
`endif
`endif

           prv_at_trap = prv;  // capture before the mutation below

           write_back_register = 0;

           // XXX We would probably save gates by factoring the deleg
           // calculation out to where cause is set as it's usually a
           // constant.
           deleg = prv <= 1 && (cause_intr ? csr_mideleg[cause[3:0]] : csr_medeleg[cause[3:0]]);
           if (deleg) begin
              csr_scause = {cause_intr, 63'd0} | {50'd0, cause};
              csr_sepc = pc;
              csr_stval = tval;
              spie = sie;
              sie = 0;
              spp = prv;
              tvec = csr_stvec;
              prv = 1;
           end else begin
              csr_mcause = {cause_intr, 63'd0} | {50'd0, cause};
              csr_mepc = pc;
              csr_mtval = tval;
              mpie = mie;
              mie = 0;
              mpp = prv;
              tvec = csr_mtvec;
              prv = 3;
           end

           // Handle vectored interrupts, just to be compatible
           npc = (tvec[0] && cause_intr) ? (tvec & ~3) + cause[11:0] * 4 : tvec & ~3;
`ifdef SIMULATE
`ifdef VERBOSE
           $display("%05d  ** Exception resuming at %x", $time, (tvec[0] && cause_intr) ? (tvec & ~3) + cause[11:0] * 4 : tvec & ~3);
`endif
`endif

`ifdef VERILATOR_COSIM
           // Trap retire: pc/insn still hold the trapping instruction; npc is
           // the trap vector we just computed. prv_at_trap is pre-trap prv.
           // Report insn=0 whenever no instruction actually retired: async
           // interrupts, and instruction-side faults (misaligned/access/page)
           // where the fetch never completed. Matches simmerv's convention.
           cosim_retire(
               pc,
               npc,
               (cause_intr || cause == `TRAP_INSTRUCTION_ADDRESS_MISALIGNED ||
                              cause == `TRAP_INSTRUCTION_ACCESS_FAULT ||
                              cause == `TRAP_INSTRUCTIONPAGE_FAULT)
                   ? 32'd0 : insn,
               8'd0,                         // no writeback on trap
               8'd0,
               {6'd0, prv_at_trap},
               8'd1,                         // trapped
               32'd0,
               64'd0,
               ({cause_intr, 63'd0} | {50'd0, cause}),   // architectural mcause/scause form
               tval,
               clint_mtime_prev,
               clint_mtimecmp,
               csr_mepc,
               {7'd0, seip}
           );
`endif
`ifdef PC_TRACE
           if (dbg_armed && !dbg_busy) begin
              dbg_busy    <= 1;
              dbg_pc      <= pc;
              dbg_mode    <= prv_at_trap;
              dbg_is_trap <= 1;
              dbg_cause   <= cause[5:0];
              dbg_pos     <= 0;
           end
`endif
           just_trapped <= 1;
           frontend_buf_flush <= 1'b1;
           fetch_epoch <= fetch_epoch + 1'b1;
           clear_frontend_cmd();
           kill_frontend_lookup();
           frontend_redirect_valid <= 0;
           frontend_miss_valid <= 0;
           frontend_miss_done <= 0;
           squash_decode_execute();

           state <= `S_FETCH1;
        end

        `S_MULDIV_START: begin
           muldiv_output_sext32 = 0;
           muldiv_output_negate = 0;
           muldiv_output_high_part = 0;

           case (muldiv_start_op)
             `MULDIV_MUL: begin
                mul_a = execute_req_rs1_value;
                mul_b = execute_req_rs2_value;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_MULH: begin
                muldiv_output_negate = execute_req_rs1_value[63] != execute_req_rs2_value[63];
                mul_a = {64'd0, pre_mul_abs_s1};
                mul_b = pre_mul_abs_s2;
                muldiv_output_high_part = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_MULHSU: begin
                muldiv_output_negate = execute_req_rs1_value[63];
                mul_a = {64'd0, pre_mul_abs_s1};
                mul_b = execute_req_rs2_value;
                muldiv_output_high_part = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_MULHU: begin
                mul_a = {64'd0, execute_req_rs1_value};
                mul_b = execute_req_rs2_value;
                muldiv_output_high_part = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_DIV: begin
                muldiv_output_negate = execute_req_rs1_value[63] != execute_req_rs2_value[63];
                if (execute_req_rs2_value == 0)
                  // No matter execute_req_rs1_value, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 64;
                muldiv_p = {64'd0, pre_mul_abs_s1};
                mul_a = {pre_mul_abs_s2, 63'd0};
                mul_b = 0;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_DIVU: begin
                div_count = 64;
                muldiv_p = {64'd0, execute_req_rs1_value};
                mul_a = {execute_req_rs2_value, 63'd0};
                mul_b = 0;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REM: begin
                // "For REM, the sign of a nonzero result equals the sign of the dividend."
                muldiv_output_negate = execute_req_rs1_value[63];
                if (execute_req_rs2_value == 0)
                  // Preserve existing divide-by-zero behavior.
                  muldiv_output_negate = 0;
                div_count = 64;
                muldiv_p = {64'd0, pre_mul_abs_s1};
                mul_a = {pre_mul_abs_s2, 63'd0};
                mul_b = 0;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REMU: begin
                if (execute_req_rs2_value == 0)
                  // No matter execute_req_rs1_value, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 64;
                muldiv_p = {64'd0, execute_req_rs1_value};
                mul_a = {execute_req_rs2_value, 63'd0};
                mul_b = 0;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_MULW: begin
                mul_a = execute_req_rs1_value[31:0];
                mul_b = execute_req_rs2_value[31:0];
                muldiv_output_sext32 = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_DIVW: begin
                muldiv_output_negate = execute_req_rs1_value[31] != execute_req_rs2_value[31];
                if (execute_req_rs2_value == 0)
                  // No matter execute_req_rs1_value, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 32;
                muldiv_p = {96'd0, pre_mul_abs_s1w};
                mul_a = {pre_mul_abs_s2w, 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_DIVUW: begin
                if (execute_req_rs2_value == 0)
                  // No matter execute_req_rs1_value, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 32;
                muldiv_p = {96'd0, execute_req_rs1_value[31:0]};
                mul_a = {execute_req_rs2_value[31:0], 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REMW: begin
                // "For REM, the sign of a nonzero result equals the sign of the dividend."
                muldiv_output_negate = execute_req_rs1_value[31];
                div_count = 32;
                muldiv_p = {96'd0, pre_mul_abs_s1w};
                mul_a = {pre_mul_abs_s2w, 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REMUW: begin
                div_count = 32;
                muldiv_p = {96'd0, execute_req_rs1_value[31:0]};
                mul_a = {execute_req_rs2_value[31:0], 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             default: begin
                cause = `TRAP_ILLEGAL_INSTRUCTION;
                tval = ex_insn;
                state <= `S_EXCEPTION;
             end
           endcase
        end

        `S_MUL_RUNNING: begin
           if (mul_b != 0) begin
              if (mul_b[0])
                muldiv_p = muldiv_p + mul_a;
              mul_a = mul_a << 1;
              mul_b = mul_b >> 1;
              try_issue_queued_decode_int_pending(1'b1, write_back_register);
           end else begin
              if (muldiv_output_sext32)
                write_back_value = {{32{muldiv_p[31]}}, muldiv_p[31:0]};
              else begin
                 if (muldiv_output_negate)
                   muldiv_p = -muldiv_p;
                 else
                   muldiv_p = muldiv_p;

                 if (muldiv_output_high_part)
                   write_back_value = muldiv_p[127:64];
                 else
                   write_back_value = muldiv_p[63:0];
              end

              retire_int_pending_linear_fetch(write_back_register);
           end
        end

        `S_DIV_RUNNING: begin
           if (div_count != 0) begin
              mul_b = mul_b << 1;
              if (muldiv_p >= mul_a) begin
                 muldiv_p = muldiv_p - mul_a;
                 mul_b = mul_b | 1;
              end
              mul_a = mul_a >> 1;
              div_count = div_count  - 1;
              try_issue_queued_decode_int_pending(1'b1, write_back_register);
           end else begin
              write_back_value = muldiv_output_negate ? -mul_b : mul_b;
              if (muldiv_output_high_part)
                // REM
                write_back_value = muldiv_output_negate ? -muldiv_p[63:0] : muldiv_p[63:0];

              if (muldiv_output_sext32)
                write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};

              retire_int_pending_linear_fetch(write_back_register);
           end
        end

        `S_TLB_LOOKUP: begin
           try_launch_queued_decode_preserve_state();
           state <= `S_TLB_CHECK;
        end

        `S_TLB_CHECK: begin
           try_launch_queued_decode_preserve_state();
           if (tlb_4k_hit || tlb_2m_hit) begin
              route_translated_addr(tlb_4k_hit ? {tlb_4k_rd_pbase, tlb_req_va[11:0]} :
                                                 {tlb_2m_rd_pbase, tlb_req_va[20:0]},
                                    tlb_4k_hit ? tlb_4k_rd_perm : tlb_2m_rd_perm,
                                    tlb_req_return);
           end else begin
              start_ptw(tlb_req_va, tlb_req_access, tlb_req_prv, tlb_req_return);
           end
        end

        `S_PTW_PROCESS: begin
           try_launch_queued_decode_preserve_state();
           // Sv39 page table walk: process the latched PTE.
           // PTE is pte_latch[63:0]; use aligned as a local alias for readability.
           aligned = pte_latch;
           // PTE fields: V=[0] R=[1] W=[2] X=[3] U=[4] G=[5] A=[6] D=[7] PPN=[53:10]
           ptw_fault_cause = ptw_access == 0 ? `TRAP_INSTRUCTIONPAGE_FAULT :
                             ptw_access == 1 ? `TRAP_LOAD_PAGE_FAULT :
                                               `TRAP_STORE_PAGE_FAULT;

           if (ptw_va[63:39] != {25{ptw_va[38]}}) begin
              // Non-canonical Sv39 virtual address
              cause = ptw_fault_cause;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end else if (!aligned[0] || !aligned[1] && aligned[2]) begin
              // Invalid PTE: V=0 or (R=0 && W=1)
              cause = ptw_fault_cause;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end else if (aligned[1] || aligned[3]) begin
              // Leaf PTE (R=1 or X=1)

              if (aligned[63] && (ptw_level != 0 || aligned[13:10] != 4'b1000)) begin
                 // NAPOT: N bit reserved on superpages; at level 0 only 64 KiB supported
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if ((ptw_level == 2 && aligned[27:10] != 0) ||
                  (ptw_level == 1 && aligned[18:10] != 0)) begin
                 // Misaligned superpage
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (ptw_access == 0 && !aligned[3]) begin
                 // Fetch requires X
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if ((ptw_access == 1 || ptw_access == 3) &&
                           !aligned[1] && !(ptw_mxr && aligned[3])) begin
                 // Load/AMO requires R (or X when MXR)
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if ((ptw_access == 2 || ptw_access == 3) && !aligned[2]) begin
                 // Store/AMO requires W
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (ptw_prv == 0 && !aligned[4]) begin
                 // U-mode but page not marked U
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (ptw_prv == 1 && aligned[4] && (ptw_access == 0 || !ptw_sum)) begin
                 // S-mode accessing U page: forbidden for fetch, or load/store without SUM
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (!aligned[6] || (ptw_access >= 2 && !aligned[7])) begin
                 // Software A/D management (RVA22): fault if A=0 or D=0 for store/AMO
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else begin
                 // Translation successful - compute physical address.
                 // Svpbmt PBMT (pte[62:61]) is honored leniently regardless of
                 // menvcfg.PBMTE and without faulting on the reserved (11)
                 // encoding -- matching the simmerv cosim model and tolerating
                 // firmware (e.g. OpenSBI 0.9) that does not set PBMTE. NC/IO
                 // become uncacheable via ptw_pte_uncacheable; PMA stays cached.
                 case (ptw_level)
                   2: mem_addr = {8'd0, aligned[53:28], ptw_va[29:0]}; // 1 GiB superpage
                   1: mem_addr = {8'd0, aligned[53:19], ptw_va[20:0]}; // 2 MiB superpage
                   0: mem_addr = aligned[63]
                        ? {8'd0, aligned[53:14], ptw_va[15:0]}  // 64 KiB NAPOT
                        : {8'd0, aligned[53:10], ptw_va[11:0]}; // 4 KiB page
                   default: mem_addr = 0;
                 endcase

`ifdef SIMULATE
                  case (ptw_level)
                    2: ptw_stat_leaf_1g <= ptw_stat_leaf_1g + 1;
                    1: ptw_stat_leaf_2m <= ptw_stat_leaf_2m + 1;
                    default: begin
                       if (aligned[63])
                          ptw_stat_leaf_64k_napot <= ptw_stat_leaf_64k_napot + 1;
                       else
                          ptw_stat_leaf_4k <= ptw_stat_leaf_4k + 1;
                    end
                  endcase
`endif

                  case (ptw_level)
                    2: hpm_ptw_leaf_1g_pulse <= 1;
                    1: hpm_ptw_leaf_2m_pulse <= 1;
                    default: begin
                       if (aligned[63])
                          hpm_ptw_leaf_napot_pulse <= 1;
                       else
                          hpm_ptw_leaf_4k_pulse <= 1;
                    end
                  endcase

                  if (ptw_level == 0 && aligned[63]) begin
                     hpm_tlb_uncached_napot_pulse <= 1;
                     route_translated_addr(mem_addr, {ptw_pte_uncacheable, 1'b0, aligned[4:1]}, ptw_return);
                  end else if (ptw_level == 2) begin
                     hpm_tlb_uncached_1g_pulse <= 1;
                     route_translated_addr(mem_addr, {ptw_pte_uncacheable, 1'b0, aligned[4:1]}, ptw_return);
                  end else begin
                     stage_tlb_insert(ptw_va, mem_addr, ptw_level, ptw_access, ptw_prv,
                                      {ptw_pte_uncacheable, 1'b0, aligned[4:1]},
                                      ptw_satp, ptw_sum, ptw_mxr);
                     ptw_route_pa <= mem_addr;
                     ptw_route_perm <= {ptw_pte_uncacheable, 1'b0, aligned[4:1]};
                     ptw_route_return <= ptw_return;
                     state <= `S_TLB_INSERT;
                  end
              end
           end else if (ptw_level == 0) begin
              // Non-leaf at level 0: invalid
              cause = ptw_fault_cause;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end else begin
              // Non-leaf PTE: descend to next level
              ptw_level <= ptw_level - 1;
              case (ptw_level)
                2: ptw_pte_addr <= {8'd0, aligned[53:10], 12'd0} + {52'd0, ptw_va[29:21], 3'd0};
                1: ptw_pte_addr <= {8'd0, aligned[53:10], 12'd0} + {52'd0, ptw_va[20:12], 3'd0};
                default: ptw_pte_addr <= 0;
              endcase
              state <= `S_PTW_LAUNCH;
           end
        end

        `S_TLB_INSERT: begin
           try_launch_queued_decode_preserve_state();
           commit_staged_tlb_insert;
           route_translated_addr(ptw_route_pa, ptw_route_perm, ptw_route_return);
        end

        `S_PTW_LAUNCH: begin
           try_launch_queued_decode_preserve_state();
           if (phys_region(ptw_pte_addr) == `REGION_BRAM ||
               phys_region(ptw_pte_addr) == `REGION_DRAM) begin
              ptw_direct_addr  <= ptw_pte_addr[30:3];
              ptw_direct_read  <= 1;
              cache_issue_dw_addr <= ptw_pte_addr[30:3];
              state               <= `S_PTW_DIRECT_WAIT;
           end else begin
              cause = ptw_access == 0 ? `TRAP_INSTRUCTIONPAGE_FAULT :
                      ptw_access == 1 ? `TRAP_LOAD_PAGE_FAULT :
                                        `TRAP_STORE_PAGE_FAULT;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end
        end

        `S_FETCH2_HALF: begin
           // Second half of cross-page or cross-doubleword instruction fetch.
           // Reassemble the 32-bit fetch word and then decode based on its actual
           // low 2 bits, so both compressed and uncompressed cases work.
           translated <= 0;
           if (fetch_from_ifetch_rsp)
              // pc+2 is at byte 0 of the fetched 8B chunk.
              aligned = {64'bx, ifetch_latched_half_data};
           else
              aligned = 128'd0;
           insn = {aligned[15:0], insn_half};
           stage_rf_decode_current(pc, frontend_fallthrough_pc(pc, insn),
                                   frontend_fallthrough_pc(pc, insn),
                                   insn, prv, fetch_epoch,
                                   fetch_from_ifetch_rsp);
        end

        `S_IFETCH_WAIT: if (ifetch_rsp_valid) begin
           ifetch_latched_window                <= icache_rsp_window;
           ifetch_latched_next_valid            <= icache_rsp_next_valid;
           ifetch_latched_insn_valid            <= icache_rsp_insn_valid;
           ifetch_latched_insn                  <= icache_rsp_insn;
           ifetch_latched_next_pc               <= icache_rsp_next_pc;
           state                                <= `S_IFETCH_RESP;
        end

        `S_IFETCH_HALF_WAIT: if (ifetch_rsp_valid) begin
           // Second-half fetches consume the raw 64-bit chunk at pc+2.  The
           // frontend instruction view is only valid for whole-instruction
           // fetch responses.
           ifetch_latched_half_data             <= icache_rsp_window[63:0];
           ifetch_latched_next_valid            <= icache_rsp_next_valid;
           ifetch_latched_insn_valid            <= 0;
           state                                <= `S_FETCH2_HALF;
        end

        `S_PTW_DIRECT_WAIT: begin
           if (ptw_direct_rsp_valid_r) begin
              pte_latch <= ptw_direct_rsp_data_r;
              state     <= `S_PTW_PROCESS;
           end else begin
              try_issue_queued_decode_current_wb(ptw_access == 2'd1 &&
                                                 ptw_return == `S_LOAD_ALIGN);
           end
        end

        `S_DMEM_LOAD_WAIT: begin
           if (dmem_rsp_valid) begin
              if ({1'b0, mem_addr[2:0]} + (1 << (load_size_lg2 & 3)) > 8) begin
                 if (dmem_rsp_next_valid) begin : dmem_load_cross_cached
                    reg [127:0] combo;
                    combo = {dmem_rsp_next_data, dmem_rsp_data} >> (mem_addr[2:0] * 8);
                    write_back_value = align_dmem_load_value(combo, load_size_lg2);
                    finish_load_writeback();
                    if (do_atomic)
                       state <= `S_AMO;
                    else begin
                       retire_current_wb_linear_fetch();
                    end
                 end else begin
                    // Access crosses a cache-line boundary and the second line
                    // missed during the parallel lookup; request it only now.
                    load_latched_data    <= dmem_rsp_data;
                    issue_dmem_cache_read(mem_addr[30:3] + 1,
                                          {mem_va[63:3], 3'b000} + 64'd8,
                                          mem_asid, mem_perm, mem_ctx);
                    state           <= `S_DMEM_LOAD2_WAIT;
                 end
              end else begin
                 aligned = dmem_rsp_data >> (mem_addr[2:0] * 8);
                 write_back_value = align_dmem_load_value(aligned, load_size_lg2);
                 finish_load_writeback();
                 if (do_atomic)
                    state <= `S_AMO;
                 else begin
                    retire_current_wb_linear_fetch();
                 end
              end
           end else begin
              try_launch_queued_decode_preserve_state();
           end
        end

        `S_DMEM_LOAD2_WAIT: begin
           if (dmem_rsp_valid) begin
              begin : dmem_load2
                 reg [127:0] combo;
                 combo = {dmem_rsp_data, load_latched_data} >> (mem_addr[2:0] * 8);
                 write_back_value = align_dmem_load_value(combo, load_size_lg2);
              end
              finish_load_writeback();
              if (do_atomic)
                 state <= `S_AMO;
              else begin
                 retire_current_wb_linear_fetch();
              end
           end else begin
              try_launch_queued_decode_preserve_state();
           end
        end

        `S_DMEM_STORE_WAIT: begin
           if (dmem_write_ready) begin
              // Issue the first write now that the master is idle
              dmem_write <= 1;
              state      <= `S_DMEM_STORE_RESP_ARM;
           end else begin
              try_issue_queued_decode_current_wb(1'b1);
           end
        end

        `S_DMEM_STORE2: begin
           if (dmem_write_ready) begin
              cache_issue_dw_addr <= dmem_store2_dw_addr;
              cache_issue_va      <= dmem_store2_va;
              cache_issue_asid    <= dmem_store2_asid;
              cache_issue_perm    <= dmem_store2_perm;
              cache_issue_ctx     <= dmem_store2_ctx;
              dmem_write_data    <= dmem_store2_data;
              dmem_write_strb    <= dmem_store2_strb;
              dmem_write         <= 1;
              dmem_store_split   <= 0;
              state              <= `S_DMEM_STORE_RESP_ARM;
           end else begin
              try_issue_queued_decode_current_wb(1'b1);
           end
        end

        `S_DMEM_STORE_RESP_ARM: begin
           try_issue_queued_decode_current_wb(1'b1);
           state <= `S_DMEM_STORE_RESP_WAIT;
        end

        `S_DMEM_STORE_RESP_WAIT: begin
           if (dmem_write_done) begin
              if (dmem_store_split) begin
                 state <= `S_DMEM_STORE2;
              end else begin
                 retire_no_wb_linear_fetch();
              end
           end else begin
              try_issue_queued_decode_current_wb(1'b1);
           end
        end

      endcase
`ifdef PC_TRACE
      end
`endif

      // The early pending-drain path above runs before the backend can pop an
      // rf_decode entry.  Re-check after case(state) so a full queue can drain
      // a pending frontend hit into the slot freed by this same cycle's pop.
      if (!core_reset_now && frontend_decode_pending_valid &&
          !frontend_decode_pending_drain &&
          !rf_decode_enqueue_this_cycle &&
          (rf_decode_pop_this_cycle || !rf_decode_full)) begin
         enqueue_frontend_decode_pending();
      end

      // RF BRAM data is available one cycle after ID drives rs1/rs2.  This
      // readiness advances even while the backend FSM remains in a long
      // execute/wait state after a background decode launch.
      if (!core_reset_now && id_valid && !id_rf_ready)
         id_rf_ready <= 1;

      // Pre-arm BRAM rs1/rs2/rs3 reads for the next queued decode once the
      // current state's operands have already been latched. Keep EX/RF states
      // excluded because they still own the read address ports.
      if (!core_reset_now && !id_valid && rf_decode_valid &&
          !rf_decode_prearmed && !rf_decode_prearm_block &&
          rf_prearm_safe_state(state))
         prearm_rf_decode_head();

      if (!core_reset_now && !frontend_flush_this_cycle &&
          frontend_spec_fetch_state(state))
         try_frontend_speculative_fetch_buf_enqueue();

      if (!core_reset_now && !frontend_flush_this_cycle &&
          frontend_spec_miss_state(state))
         try_frontend_speculative_miss_start();

      if (!core_reset_now && ifetch_refill_retry_valid) begin
         issue_ifetch_cache_read(cache_addr[30:3], cache_req_va,
                                 cache_req_asid, cache_req_perm,
                                 cache_req_ctx);
      end

      if (!core_reset_now && !frontend_flush_this_cycle &&
          frontend_miss_valid && ifetch_rsp_valid) begin
         frontend_miss_valid      <= 0;
         frontend_miss_done       <= 1;
         frontend_miss_window     <= icache_rsp_window;
         frontend_miss_next_valid <= icache_rsp_next_valid;
         frontend_miss_insn_valid <= icache_rsp_insn_valid;
         frontend_miss_insn <= icache_rsp_insn;
      end

      // Bus timeout: fault if an external bus access doesn't respond
      begin : bus_timeout_logic
         reg bus_waiting;
         bus_waiting = state == `S_IFETCH_WAIT || state == `S_IFETCH_HALF_WAIT ||
                       state == `S_DMEM_LOAD_WAIT  || state == `S_DMEM_LOAD2_WAIT ||
                       state == `S_PTW_DIRECT_WAIT   ||
                       state == `S_DMEM_STORE_WAIT || state == `S_DMEM_STORE2 ||
                       state == `S_DMEM_STORE_RESP_WAIT || state == `S_DMEM_STORE_RESP_ARM ||
                       state == `S_MMIO_ALIGN ||
                       (state == `S_FRONTEND_MISS_WAIT && frontend_miss_valid);
         bus_timeout_expired <= bus_waiting && &bus_timeout_ctr;
         if (bus_waiting) begin
            bus_timeout_ctr <= bus_timeout_ctr + 1;
            case (state)
              `S_IFETCH_WAIT, `S_IFETCH_HALF_WAIT, `S_FRONTEND_MISS_WAIT: begin
                 bus_timeout_cause <= `TRAP_INSTRUCTION_ACCESS_FAULT;
                 bus_timeout_tval  <= state == `S_FRONTEND_MISS_WAIT ? frontend_miss_pc : cache_issue_va;
              end
              `S_DMEM_STORE_WAIT, `S_DMEM_STORE2, `S_DMEM_STORE_RESP_WAIT, `S_DMEM_STORE_RESP_ARM: begin
                 bus_timeout_cause <= `TRAP_STORE_ACCESS_FAULT;
                 bus_timeout_tval  <= cache_issue_va;
              end
              `S_PTW_DIRECT_WAIT: begin
                 bus_timeout_cause <= ptw_access == 0 ? `TRAP_INSTRUCTION_ACCESS_FAULT :
                                      ptw_access == 2 || ptw_access == 3 ? `TRAP_STORE_ACCESS_FAULT :
                                      `TRAP_LOAD_ACCESS_FAULT;
                 bus_timeout_tval <= ptw_va;
              end
              default: begin // S_DMEM_LOAD_WAIT, S_DMEM_LOAD2_WAIT, S_MMIO_ALIGN
                 bus_timeout_cause <= `TRAP_LOAD_ACCESS_FAULT;
                 bus_timeout_tval  <= state == `S_MMIO_ALIGN ? mmio_timeout_tval : cache_issue_va;
              end
            endcase
            if (bus_timeout_expired) begin
               bus_timeout_ctr     <= 0;
               bus_timeout_expired <= 0;
               csr_mig_timeouts <= csr_mig_timeouts + 1;
               // Late R beats from an abandoned read are silently swallowed by
               // the AXI master (it gates dmem_rsp_valid on bus_waiting), so
               // no separate "abandon" handshake is needed.
               // Latch context of the FIRST timeout in this measurement
               // window (don't overwrite on aftershock faults in the trap
               // handler).  Cleared when csr_mig_timeouts is cleared.
               if (csr_mig_timeouts == 0) begin
                  csr_mig_to_pc    <= pc;
                  csr_mig_to_tval  <= bus_timeout_tval;
                  csr_mig_to_state <= {59'd0, state};
                  csr_mig_to_cause <= {52'd0, bus_timeout_cause};
                  csr_mig_to_addr  <= {33'd0, cache_issue_dw_addr, 3'd0};
               end
`ifdef SIMULATE
`ifdef VERBOSE
               $display("%05d  ** Bus timeout in state %0d, cause %0d, tval %x", $time, state, cause, tval);
`endif
`endif
               frontend_miss_valid <= 0;
               frontend_miss_done <= 0;
               cause_intr = 0;
               cause = bus_timeout_cause;
               tval = bus_timeout_tval;
               write_back_register = 0;
               state <= `S_EXCEPTION;
            end
         end else
            bus_timeout_ctr <= 0;

         // Memory latency stats: measure cycles in any long-latency wait state.
         // Counter runs while bus_waiting; on falling edge, fold into stats.
         mig_prev_waiting <= bus_waiting;
         if (bus_waiting) begin
            mig_latency_ctr <= mig_latency_ctr + 1;
         end else begin
            mig_latency_ctr <= 0;
            if (mig_prev_waiting) begin
               csr_mig_count <= csr_mig_count + 1;
               csr_mig_total <= csr_mig_total + {32'd0, mig_latency_ctr};
               if (mig_latency_ctr < csr_mig_min) csr_mig_min <= mig_latency_ctr;
               if (mig_latency_ctr > csr_mig_max) csr_mig_max <= mig_latency_ctr;
            end
         end
      end

      if (core_reset_now) begin
         core_reset_pending <= 0;
         state <= `S_FETCH1;
         csr_minstret <= 0;
         csr_mcycle <= 0;
         clint_mtime <= 0;
         write_back_register <= 0;
         int_commit_result <= 0;
         int_commit_fflags <= 0;
         int_commit_prepared_fetch <= 0;
         int_commit_redirect_fetch <= 0;
         npc <= `RESET_PC;
         clear_frontend_cmd();
         frontend_cmd_epoch <= 0;
         fetch_epoch <= 0;
         frontend_cmd_pc <= `RESET_PC;
         frontend_cmd_prv <= 3;
         frontend_cmd_asid <= 0;
         frontend_redirect_valid <= 0;
         frontend_redirect_pc <= `RESET_PC;
         frontend_redirect_prv <= 3;
         frontend_redirect_epoch <= 0;
         f_latched_hit <= 0;
         f_latched_insn <= 0;
         f_latched_decode_next_pc <= `RESET_PC;
         f_latched_next_pc <= `RESET_PC;
         f_latched_cmd_pc <= `RESET_PC;
         f_latched_cmd_prv <= 3;
         f_latched_cmd_asid <= 0;
         f_latched_cmd_epoch <= 0;
         f_state <= `F_IDLE;
         ex_state <= `EX_IDLE;
         retire_now_q <= 0;
         frontend_miss_valid <= 0;
         frontend_miss_done <= 0;
         frontend_miss_pc <= `RESET_PC;
         frontend_miss_prv <= 3;
         frontend_miss_asid <= 0;
         frontend_miss_epoch <= 0;
         frontend_miss_window <= 0;
         frontend_miss_next_valid <= 0;
         frontend_miss_insn_valid <= 0;
         frontend_miss_insn <= 0;
         frontend_miss_wait_action <= FRONTEND_MISS_WAIT_CONSUME;
         rf_decode_head <= 0;
         rf_decode_tail <= 0;
         rf_decode_count <= 0;
         rf_decode_prearmed <= 0;
         rf_decode_prearmed_head <= 0;
         rf_decode_prearm_block = 1;
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 0;
         id_valid <= 0;
         id_rf_ready <= 0;
         id_pc <= `RESET_PC;
         id_next_pc <= `RESET_PC;
         id_predicted_pc <= `RESET_PC;
         id_prv <= 0;
         id_epoch <= 0;
         id_insn <= 0;
         id_rd <= 0;
         id_rs1 <= 0;
         id_rs2 <= 0;
         id_shamt <= 0;
         execute_req_valid <= 0;
         execute_req_pc <= `RESET_PC;
         execute_req_next_pc <= `RESET_PC;
         execute_req_predicted_pc <= `RESET_PC;
         execute_req_prv <= 0;
         execute_req_epoch <= 0;
         execute_req_insn <= 0;
         execute_req_rd <= 0;
         execute_req_rs1 <= 0;
         execute_req_rs2 <= 0;
         execute_req_shamt <= 0;
         execute_res_valid <= 0;
         pre_npc <= `RESET_PC;
        pre_jalr_target <= `RESET_PC;
        pre_branch_target <= `RESET_PC;
        pre_branch_taken <= 0;
         bus_timeout_ctr <= 0;
         bus_timeout_expired <= 0;
         bus_timeout_tval <= 0;
         bus_timeout_cause <= 0;
         // Note: mig_* stats CSRs intentionally NOT reset here, so they
         // survive a soft reset (e.g. key[1] on the FPGA board).  Initial
         // values come from the reg declarations (FPGA config-time init).
         // Clear them with a CSR write to CSR_MIG_COUNT (see CSR write block).

         // Architectural state needed to cleanly resume execution from
         // RESET_PC in M-mode with paging off (e.g. after a soft reset
         // that returns to the monitor from a running Linux workload).
         prv              <= 3;
         csr_satp         <= 0;
         csr_mie          <= 0;
         csr_mideleg      <= 0;
         csr_medeleg      <= 0;
         csr_mcounteren   <= 0;
         csr_menvcfg      <= 0;
         csr_stimecmp     <= ~0;
         csr_scounteren   <= 0;
         csr_senvcfg      <= 0;
         csr_mcountinhibit <= 0;
         csr_mcyclecfg    <= 0;
         csr_minstretcfg  <= 0;
         lcofip           <= 0;
         csr_mtvec        <= 0;
         csr_stvec        <= 0;
         mie              <= 0;
         sie              <= 0;
         uie              <= 0;
         mpie             <= 0;
         spie             <= 0;
         upie             <= 0;
         mpp              <= 0;
         spp              <= 0;
         mprv             <= 0;
         pre_intr_pending <= 0;
         pre_fp_rnd_mode  <= 0;
         pre_fp_rmode_ok  <= 1'b1;
         just_trapped     <= 0;
         just_xret        <= 0;
         frontend_buf_flush <= 1'b1;
         f_latched_hit <= 0;
         f_latched_insn <= 0;
         f_state <= `F_IDLE;
         ex_state <= `EX_IDLE;
         muldiv_start_op <= `MULDIV_MUL;
         uart_tx_head     <= 0;
         uart_tx_tail     <= 0;
         uart_rx_head     <= 0;
         uart_rx_tail     <= 0;
         uart_rx_front_valid <= 0;
         uart_rx_refill_pending <= 0;
         uart_thre_pending <= 0;
         plic_pending     <= 0;
         plic_in_service  <= 0;
         plic_enabled     <= 0;
         plic_threshold   <= 0;
         fetch_from_ifetch_rsp <= 0;
         ifetch_latched_window <= 0;
         ifetch_latched_half_data <= 0;
         ifetch_latched_next_valid <= 0;
         ifetch_latched_insn_valid <= 0;
         ifetch_latched_insn <= 0;
         ifetch_latched_next_pc <= 0;
         translated       <= 0;
         ifetch_read      <= 0;
         dmem_read        <= 0;
         dmem_write       <= 0;
         cache_issue_va   <= 0;
         cache_issue_asid <= 0;
         cache_issue_perm <= CACHE_PERM_PHYS;
         cache_issue_ctx  <= 0;
         mem_va           <= 0;
         mem_asid         <= 0;
         mem_perm         <= CACHE_PERM_PHYS;
         mem_ctx          <= 0;
         mmio_timeout_tval <= 0;
         dmem_store2_va   <= 0;
         dmem_store2_asid <= 0;
         dmem_store2_perm <= CACHE_PERM_PHYS;
         dmem_store2_ctx  <= 0;
         dmem_store_split <= 0;
         ptw_direct_read <= 0;
         ptw_direct_addr <= 0;
         cache_flush_req  <= 0;
         vhpr_epoch_bump_req <= 0;
         mig_latency_ctr  <= 0;
         mig_prev_waiting <= 0;
         hpm_counter_wr_en <= 0;
         hpm_event_wr_en   <= 0;
         tlb_4k_rd_idx     <= 0;
         tlb_2m_rd_idx     <= 0;
         tlb_4k_wr_idx     <= 0;
         tlb_2m_wr_idx     <= 0;
         tlb_4k_wr_en      <= 0;
         tlb_2m_wr_en      <= 0;
         tlb_4k_wr_data    <= 0;
         tlb_2m_wr_data    <= 0;
         tlb_insert_level  <= 0;
         tlb_insert_4k_idx <= 0;
         tlb_insert_2m_idx <= 0;
         tlb_insert_4k_data <= 0;
         tlb_insert_2m_data <= 0;
         ptw_route_pa      <= 0;
         ptw_route_return  <= 0;
         hpm_tlb_insert_4k_pulse <= 0;
         hpm_tlb_insert_2m_pulse <= 0;
         hpm_tlb_evict_4k_pulse <= 0;
         hpm_tlb_evict_2m_pulse <= 0;
         hpm_tlb_uncached_1g_pulse <= 0;
         hpm_tlb_uncached_napot_pulse <= 0;
         hpm_ptw_leaf_4k_pulse <= 0;
         hpm_ptw_leaf_2m_pulse <= 0;
         hpm_ptw_leaf_1g_pulse <= 0;
         hpm_ptw_leaf_napot_pulse <= 0;
         flush_tlb;
`ifdef SIMULATE
         fetch_buf_stat_hits <= 0;
         fetch_buf_stat_misses <= 0;
         ptw_stat_leaf_4k <= 0;
         ptw_stat_leaf_64k_napot <= 0;
         ptw_stat_leaf_2m <= 0;
         ptw_stat_leaf_1g <= 0;
         tlb_stat_lookups <= 0;
         tlb_stat_hits <= 0;
         tlb_stat_misses <= 0;
         tlb_stat_hits_4k <= 0;
         tlb_stat_hits_2m <= 0;
         tlb_stat_inserts_4k <= 0;
         tlb_stat_inserts_2m <= 0;
         tlb_stat_evicts_4k <= 0;
         tlb_stat_evicts_2m <= 0;
         tlb_stat_uncached_1g <= 0;
         tlb_stat_uncached_napot <= 0;
         tlb_stat_entries_4k = 0;
         tlb_stat_entries_2m = 0;
         tlb_stat_entries_1g = 0;
`endif
      end
   end

   assign dmem_rsp_valid = dmem_rsp_valid_r;
   assign dmem_rsp_data      = dmem_rsp_data_r;
   assign dmem_rsp_next_data = dmem_rsp_next_data_r;
   assign dmem_rsp_next_valid = dmem_rsp_next_valid_r;
   assign dmem_write_ready   = cache_idle;
   assign dmem_write_done    = dmem_write_done_r;

   always @(posedge clock) begin
      dmem_rsp_valid_r <= 0;
      dmem_rsp_next_valid_r <= 0;
      ifetch_refill_retry_valid <= 0;
      ptw_direct_rsp_valid_r <= 0;
      dmem_write_done_r <= 0;
      cache_bram_write_done <= 0;
      cache_cbo_done_r <= 0;
      dcache_way0_tag_wr_en <= 0;
      dcache_way1_tag_wr_en <= 0;
      icache_invalidate_valid <= 0;
      hpm_vhpr_pulse <= 0;

      if (ptw_direct_read)
         ptw_direct_probe_pending <= 1;

      if (ptw_direct_wait_probe && cache_cbo_done_r) begin
         ptw_direct_wait_probe <= 0;
         ptw_direct_pending <= 1;
      end

      if (ptw_direct_wait_bram) begin
         ptw_direct_rsp_data_r <= ptw_direct_bram_word_bank
                                  ? mem1[ptw_direct_bram_word_idx]
                                  : mem0[ptw_direct_bram_word_idx];
         ptw_direct_rsp_valid_r <= 1;
         ptw_direct_wait_bram <= 0;
      end

      if (ptw_direct_wait_axi)
         l2_direct_read_rsp_ready <= 1;
      if (ptw_direct_wait_axi && l2_direct_read_rsp_valid && l2_direct_read_rsp_ready) begin
         ptw_direct_rsp_data_r <= l2_direct_read_rsp_data;
         ptw_direct_rsp_valid_r <= 1;
         l2_direct_read_rsp_ready <= 0;
         ptw_direct_wait_axi <= 0;
      end

      case (cache_state)
        CACHE_IDLE: begin
           if (cache_flush_all) begin
              if (vhpr_epoch_bump_pending) begin : vhpr_epoch_bump_start
                 reg [VHPR_EPOCH_BITS-1:0] next_epoch;
                 next_epoch = vhpr_epoch + {{VHPR_EPOCH_BITS-1{1'b0}}, 1'b1};
                 vhpr_next_epoch <= next_epoch;
                 vhpr_epoch_update_pending <= 1'b1;
                 vhpr_epoch_bump_ack <= vhpr_epoch_bump_req;
                 hpm_vhpr_pulse[`VHPRP_EPOCH_BUMPS] <= 1;
                 if (next_epoch == {VHPR_EPOCH_BITS{1'b0}})
                    hpm_vhpr_pulse[`VHPRP_EPOCH_ROLLOVERS] <= 1;
              end
              cache_flush_idx <= 0;
              cache_flush_way <= 0;
              cache_req_ifetch <= 0;
              cache_way0_rd_idx <= 0;
              cache_way1_rd_idx <= 0;
              cache_way0_bank0_rd_idx <= 0;
              cache_way1_bank0_rd_idx <= 0;
              cache_state <= CACHE_FLUSH_READ;
           end else if (vhpr_epoch_bump_pending) begin : vhpr_epoch_bump_only
              reg [VHPR_EPOCH_BITS-1:0] next_epoch;
              next_epoch = vhpr_epoch + {{VHPR_EPOCH_BITS-1{1'b0}}, 1'b1};
              vhpr_epoch_bump_ack <= vhpr_epoch_bump_req;
              hpm_vhpr_pulse[`VHPRP_EPOCH_BUMPS] <= 1;
              if (next_epoch == {VHPR_EPOCH_BITS{1'b0}}) begin
                 hpm_vhpr_pulse[`VHPRP_EPOCH_ROLLOVERS] <= 1;
                 vhpr_next_epoch <= next_epoch;
                 vhpr_epoch_update_pending <= 1'b1;
                 cache_flush_idx <= 0;
                 cache_flush_way <= 0;
                 cache_req_ifetch <= 0;
                 cache_way0_rd_idx <= 0;
                 cache_way1_rd_idx <= 0;
                 cache_way0_bank0_rd_idx <= 0;
                 cache_way1_bank0_rd_idx <= 0;
                 cache_state <= CACHE_FLUSH_READ;
              end else begin
                 vhpr_epoch <= next_epoch;
              end
	   end else if (cache_cbo_flush) begin
              hpm_vhpr_pulse[`VHPRP_CBO_PROBES] <= 1;
	      cache_addr              <= {33'd0, cache_cbo_line_addr, 6'd0};
	      cache_req_ptag          <= cache_cbo_ptag;
	      cache_req_cbo           <= 1;
	      cache_req_zero          <= 0;
	      cache_req_write         <= 0;
	      cache_req_ifetch         <= 0;
	      cache_req_line_addr     <= cache_cbo_line_addr;
	      cache_probe_color       <= 0;
	      cache_probe_found       <= 0;
	      cache_probe_found_way   <= 0;
	      cache_probe_found_idx   <= 0;
	      cache_probe_found_dirty <= 0;
              cache_probe_active      <= 1;
              cache_way0_rd_idx       <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_way1_rd_idx       <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_way0_bank0_rd_idx <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_way1_bank0_rd_idx <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_state             <= CACHE_PROBE_READ;
	   end else if (ptw_direct_probe_pending) begin
              hpm_vhpr_pulse[`VHPRP_PTW_PROBES] <= 1;
	      cache_addr              <= {33'd0, ptw_direct_addr[27:3], 6'd0};
	      cache_req_ptag          <= ptw_direct_addr[27:9];
	      cache_req_cbo           <= 1;
	      cache_req_zero          <= 0;
	      cache_req_write         <= 0;
	      cache_req_ifetch         <= 0;
	      cache_req_line_addr     <= ptw_direct_addr[27:3];
	      cache_probe_color       <= 0;
	      cache_probe_found       <= 0;
	      cache_probe_found_way   <= 0;
	      cache_probe_found_idx   <= 0;
	      cache_probe_found_dirty <= 0;
              cache_probe_active      <= 1;
              cache_way0_rd_idx       <= {3'd0, ptw_direct_addr[8:3]};
              cache_way1_rd_idx       <= {3'd0, ptw_direct_addr[8:3]};
              cache_way0_bank0_rd_idx <= {3'd0, ptw_direct_addr[8:3]};
              cache_way1_bank0_rd_idx <= {3'd0, ptw_direct_addr[8:3]};
              ptw_direct_probe_pending <= 0;
              ptw_direct_wait_probe    <= 1;
              cache_state             <= CACHE_PROBE_READ;
           end else if (ptw_direct_pending && !ptw_direct_wait_bram &&
                        !ptw_direct_wait_axi) begin
              if (ptw_direct_from_bram) begin
                 begin : ptw_direct_bram_req
                    reg [63:0] bram_addr;
                    bram_addr = ptw_direct_addr64 - cache_bram_base_addr;
                    ptw_direct_bram_word_idx  <= bram_addr[`MEM_SIZE_LG2-1:4];
                    ptw_direct_bram_word_bank <= bram_addr[3];
                 end
                 ptw_direct_wait_bram <= 1;
                 ptw_direct_pending <= 0;
              end else begin
                 if (!l2_direct_read_req_valid) begin
                    l2_direct_read_req_addr <= ptw_direct_addr;
                    l2_direct_read_req_valid <= 1;
                 end else if (l2_direct_read_req_ready) begin
                    l2_direct_read_req_valid <= 0;
                    ptw_direct_wait_axi <= 1;
                    ptw_direct_pending <= 0;
                 end
              end
           end else if (cache_read_req) begin
              hpm_vhpr_pulse[`VHPRP_READS] <= 1;
              cache_addr          <= cache_issue_addr;
              cache_req_va        <= cache_issue_va;
              cache_req_asid      <= cache_issue_asid;
              cache_req_perm      <= cache_issue_perm;
              cache_req_ctx       <= cache_issue_ctx;
              cache_req_write     <= 0;
              cache_req_ifetch     <= ifetch_read;
              cache_req_cbo       <= 0;
              cache_req_zero      <= 0;
              cache_req_vtag      <= cache_vtag(cache_issue_va);
              cache_req_next_vtag <= cache_vtag(cache_issue_next_va);
              cache_req_ptag      <= cache_issue_ptag;
              cache_req_bank      <= cache_issue_dw_addr[2:0];
              cache_req_next_bank <= cache_issue_dw_addr[2:0] + 3'd1;
              cache_req_same_line <= cache_issue_dw_addr[2:0] != 3'd7;
              cache_way0_rd_idx   <= cache_way0_index(cache_issue_va, cache_issue_asid);
              cache_way1_rd_idx   <= cache_way1_index(cache_issue_va, cache_issue_asid);
              cache_way0_bank0_rd_idx <= cache_issue_dw_addr[2:0] == 3'd7
                                      ? cache_way0_index(cache_issue_next_va, cache_issue_asid)
                                      : cache_way0_index(cache_issue_va, cache_issue_asid);
              cache_way1_bank0_rd_idx <= cache_issue_dw_addr[2:0] == 3'd7
                                      ? cache_way1_index(cache_issue_next_va, cache_issue_asid)
                                      : cache_way1_index(cache_issue_va, cache_issue_asid);
              cache_way0_next_rd_idx <= cache_way0_index(cache_issue_next_va, cache_issue_asid);
              cache_way1_next_rd_idx <= cache_way1_index(cache_issue_next_va, cache_issue_asid);
              cache_state         <= CACHE_TAG_READ;
           end else if (dmem_write) begin
              hpm_vhpr_pulse[`VHPRP_WRITES] <= 1;
              cache_addr          <= cache_issue_addr;
              cache_req_va        <= cache_issue_va;
              cache_req_asid      <= cache_issue_asid;
              cache_req_perm      <= cache_issue_perm;
              cache_req_ctx       <= cache_issue_ctx;
              cache_req_write     <= 1;
              cache_req_ifetch     <= 0;
              cache_req_cbo       <= 0;
              cache_req_zero      <= dmem_write_zero;
              cache_req_vtag      <= cache_vtag(cache_issue_va);
              cache_req_next_vtag <= cache_vtag(cache_issue_next_va);
              cache_req_ptag      <= cache_issue_ptag;
              cache_req_bank      <= cache_issue_dw_addr[2:0];
              cache_req_next_bank <= cache_issue_dw_addr[2:0] + 3'd1;
              cache_req_same_line <= 1;
              cache_store_data    <= dmem_write_data;
              cache_store_strb    <= dmem_write_strb;
              cache_way0_rd_idx   <= cache_way0_index(cache_issue_va, cache_issue_asid);
              cache_way1_rd_idx   <= cache_way1_index(cache_issue_va, cache_issue_asid);
              cache_way0_bank0_rd_idx <= cache_way0_index(cache_issue_va, cache_issue_asid);
              cache_way1_bank0_rd_idx <= cache_way1_index(cache_issue_va, cache_issue_asid);
              cache_way0_next_rd_idx <= cache_way0_index(cache_issue_next_va, cache_issue_asid);
              cache_way1_next_rd_idx <= cache_way1_index(cache_issue_next_va, cache_issue_asid);
              cache_state         <= CACHE_TAG_READ;
           end
        end

        CACHE_TAG_READ: begin
           cache_state <= CACHE_TAG_WAIT;
        end

        CACHE_TAG_WAIT: begin
           cache_state <= CACHE_TAG_CHECK;
        end

        CACHE_TAG_CHECK: begin
           dcache_rsp_hit <= dcache_lookup_hit;
           dcache_rsp_hit_way <= dcache_lookup_hit_way;
           dcache_rsp_next_hit <= dcache_lookup_next_hit;
           dcache_rsp_data <= dcache_lookup_data;
           dcache_rsp_next_data <= dcache_lookup_next_data;
           dcache_rsp_next_valid <= dcache_lookup_next_valid;
           dcache_rsp_dirty <= dcache_lookup_dirty;
           cache_target_way <= cache_req_ifetch
                              ? icache_target_way
                              : dcache_target_way;
           cache_target_idx <= cache_req_ifetch
                              ? icache_target_idx
                              : dcache_target_idx;
           cache_target_valid <= cache_req_ifetch
                                ? icache_target_valid
                                : dcache_target_valid;
           cache_target_dirty <= cache_req_ifetch
                                ? 1'b0
                                : dcache_target_dirty;
           cache_target_ptag <= cache_req_ifetch
                               ? {`CACHE_PHYS_TAG_BITS{1'b0}}
                               : dcache_target_ptag;
           cache_state <= CACHE_HIT_RESP;
        end

        CACHE_HIT_RESP: begin : cache_hit_resp
           if (cache_req_ifetch) begin
              if (icache_rsp_hit) begin
                 hpm_vhpr_pulse[`VHPRP_READ_HITS] <= 1;
                 cache_state <= CACHE_IDLE;
              end else begin
                 hpm_vhpr_pulse[`VHPRP_READ_MISSES] <= 1;
                 cache_start_fill_request();
              end
           end else if (dcache_rsp_hit) begin
              if (cache_req_write) begin
                 hpm_vhpr_pulse[`VHPRP_WRITE_HITS] <= 1;
                 cache_state <= CACHE_HIT_WRITE;
              end else begin
                 reg dcache_next_line_safe;

                 dcache_next_line_safe =
                    cache_req_same_line || cache_req_va[11:3] != 9'h1ff;
                 hpm_vhpr_pulse[`VHPRP_READ_HITS] <= 1;
                 emit_dmem_load_rsp(dcache_rsp_data,
                                    dcache_rsp_next_data,
                                    dcache_rsp_next_valid &&
                                    dcache_next_line_safe);
                 cache_state <= CACHE_IDLE;
              end
           end else begin
              if (cache_req_write)
                 hpm_vhpr_pulse[`VHPRP_WRITE_MISSES] <= 1;
              else
                 hpm_vhpr_pulse[`VHPRP_READ_MISSES] <= 1;
              cache_start_fill_request();
           end
        end

        CACHE_HIT_WRITE: begin
           dcache_way0_tag_wr_en <= !dcache_rsp_hit_way;
           dcache_way1_tag_wr_en <= dcache_rsp_hit_way;
           dcache_tag_wr_idx     <= dcache_rsp_hit_way ? cache_way1_rd_idx :
                                                          cache_way0_rd_idx;
           dcache_tag_wr_data    <= cache_make_meta(1'b1, 1'b1, cache_req_asid,
                                                    cache_req_perm,
                                                    cache_req_vtag, cache_req_ptag,
                                                    vhpr_epoch);
           dmem_write_done_r <= 1;
           cache_state <= CACHE_IDLE;
        end

        CACHE_FLUSH_READ: begin
           cache_state <= CACHE_FLUSH_WAIT;
        end

        CACHE_FLUSH_WAIT: begin
           cache_state <= CACHE_FLUSH_CHECK;
        end

        CACHE_FLUSH_CHECK: begin : cache_flush_check
           reg [`CACHE_META_BITS-1:0] flush_meta;

           flush_meta = cache_flush_way ? dcache_way1_tag_rd_data : dcache_way0_tag_rd_data;
           dcache_way0_tag_wr_en <= !cache_flush_way;
           dcache_way1_tag_wr_en <= cache_flush_way;
           dcache_tag_wr_idx <= cache_flush_idx;
           dcache_tag_wr_data <= 0;
           icache_invalidate_valid <= 1'b1;
           icache_invalidate_way <= cache_flush_way;
           icache_invalidate_idx <= cache_flush_idx;
           if (cache_meta_valid(flush_meta) && cache_meta_dirty(flush_meta)) begin
              hpm_vhpr_pulse[`VHPRP_FLUSH_EVICTS] <= 1;
              hpm_vhpr_pulse[`VHPRP_DIRTY_FLUSH_EVICTS] <= 1;
              cache_victim_way <= cache_flush_way;
              cache_victim_idx <= cache_flush_idx;
              cache_victim_ptag <= cache_meta_ptag(flush_meta);
              cache_wb_base <= {33'd0, cache_meta_ptag(flush_meta), cache_flush_idx[5:0], 6'd0};
              cache_wb_beat <= 0;
              cache_wb_after_cbo <= 1'b0;
              cache_wb_then_fill <= 1'b0;
              cache_wb_after_flush <= 1'b1;
              cache_way0_rd_idx <= cache_flush_idx;
              cache_way1_rd_idx <= cache_flush_idx;
              cache_way0_bank0_rd_idx <= cache_flush_idx;
              cache_way1_bank0_rd_idx <= cache_flush_idx;
              cache_state <= CACHE_WB_PREP;
           end else begin
              if (cache_meta_valid(flush_meta)) begin
                 hpm_vhpr_pulse[`VHPRP_FLUSH_EVICTS] <= 1;
              end
              cache_flush_next_line();
           end
        end

        CACHE_PROBE_READ: begin
           cache_state <= CACHE_PROBE_WAIT;
        end

        CACHE_PROBE_WAIT: begin
           cache_state <= CACHE_PROBE_CHECK;
        end

        CACHE_PROBE_CHECK: begin : cache_probe_check
           reg way0_phys_hit, way1_phys_hit;
           reg found;
           reg found_way;
           reg found_dirty;
           reg [`CACHE_INDEX_BITS-1:0] found_idx;
           reg [`CACHE_INDEX_BITS-1:0] next_probe_idx;
           reg same_as_target;

           way0_phys_hit = cache_meta_valid(dcache_way0_tag_rd_data) &&
                           cache_meta_ptag(dcache_way0_tag_rd_data) == cache_req_ptag;
           way1_phys_hit = cache_meta_valid(dcache_way1_tag_rd_data) &&
                           cache_meta_ptag(dcache_way1_tag_rd_data) == cache_req_ptag;

           found = cache_probe_found || way0_phys_hit || way1_phys_hit;
           found_way = cache_probe_found ? cache_probe_found_way : way1_phys_hit;
	   found_idx = cache_probe_found ? cache_probe_found_idx :
		       (cache_req_cbo ? cache_req_probe_line_index : cache_probe_line_index);
           found_dirty = cache_probe_found ? cache_probe_found_dirty :
                         (way1_phys_hit ? cache_meta_dirty(dcache_way1_tag_rd_data)
                                        : cache_meta_dirty(dcache_way0_tag_rd_data));

           if (!cache_probe_found && (way0_phys_hit || way1_phys_hit)) begin
              cache_probe_found <= 1;
              cache_probe_found_way <= way1_phys_hit;
	      cache_probe_found_idx <= cache_req_cbo ? cache_req_probe_line_index : cache_probe_line_index;
              cache_probe_found_dirty <= found_dirty;
           end

           if (cache_probe_color != 3'd7) begin
	      cache_probe_color <= cache_probe_color + 1;
	      next_probe_idx = {cache_probe_color + 1'b1,
				cache_req_cbo ? cache_req_line_addr[11:6] : cache_addr[11:6]};
              cache_way0_rd_idx <= next_probe_idx;
              cache_way1_rd_idx <= next_probe_idx;
              cache_way0_bank0_rd_idx <= next_probe_idx;
              cache_way1_bank0_rd_idx <= next_probe_idx;
              cache_state <= CACHE_PROBE_READ;
           end else begin
              cache_probe_active <= 0;
              cache_victim_way <= found_way;
              cache_victim_idx <= found_idx;
              cache_victim_ptag <= cache_req_ptag;
              same_as_target = found && found_way == cache_target_way && found_idx == cache_target_idx;
              if (found && !cache_req_cbo) begin
                 hpm_vhpr_pulse[`VHPRP_ALIAS_EVICTS] <= 1;
                 if (found_dirty)
                    hpm_vhpr_pulse[`VHPRP_DIRTY_ALIAS_EVICTS] <= 1;
              end
              cache_need_target_wb <= !cache_req_cbo && cache_target_dirty && !same_as_target;
              if (found && found_dirty) begin
                 cache_wb_base <= {33'd0, cache_req_ptag, found_idx[5:0], 6'd0};
                 cache_wb_beat <= 0;
                 cache_wb_after_cbo <= cache_req_cbo;
                 cache_wb_then_fill <= !cache_req_cbo;
                 cache_way0_rd_idx <= found_idx;
                 cache_way1_rd_idx <= found_idx;
                 cache_way0_bank0_rd_idx <= found_idx;
                 cache_way1_bank0_rd_idx <= found_idx;
                 cache_state <= CACHE_WB_PREP;
              end else if (found) begin
                 cache_state <= CACHE_INVALIDATE;
              end else if (cache_req_cbo) begin
                 cache_cbo_done_r <= 1;
                 cache_state <= CACHE_IDLE;
              end else if (cache_target_dirty) begin
                 hpm_vhpr_pulse[`VHPRP_VICTIM_EVICTS] <= 1;
                 hpm_vhpr_pulse[`VHPRP_DIRTY_VICTIM_EVICTS] <= 1;
                 cache_victim_way <= cache_target_way;
                 cache_victim_idx <= cache_target_idx;
                 cache_victim_ptag <= cache_target_ptag;
                 cache_wb_base <= {33'd0, cache_target_ptag, cache_target_idx[5:0], 6'd0};
                 cache_wb_beat <= 0;
                 cache_wb_after_cbo <= 1'b0;
                 cache_wb_then_fill <= 1'b0;
                 cache_way0_rd_idx <= cache_target_idx;
                 cache_way1_rd_idx <= cache_target_idx;
                 cache_way0_bank0_rd_idx <= cache_target_idx;
                 cache_way1_bank0_rd_idx <= cache_target_idx;
                 cache_state <= CACHE_WB_PREP;
              end else begin
                 cache_state <= CACHE_FILL_REQ;
              end
           end
        end

        CACHE_INVALIDATE: begin
           dcache_way0_tag_wr_en <= !cache_victim_way;
           dcache_way1_tag_wr_en <= cache_victim_way;
           dcache_tag_wr_idx <= cache_victim_idx;
           dcache_tag_wr_data <= 0;
           if (cache_req_cbo) begin
              cache_cbo_done_r <= 1;
              cache_state <= CACHE_IDLE;
           end else if (cache_need_target_wb) begin
              cache_need_target_wb <= 0;
              hpm_vhpr_pulse[`VHPRP_VICTIM_EVICTS] <= 1;
              hpm_vhpr_pulse[`VHPRP_DIRTY_VICTIM_EVICTS] <= 1;
              cache_victim_way <= cache_target_way;
              cache_victim_idx <= cache_target_idx;
              cache_victim_ptag <= cache_target_ptag;
              cache_wb_base <= {33'd0, cache_target_ptag, cache_target_idx[5:0], 6'd0};
              cache_wb_beat <= 0;
              cache_wb_after_cbo <= 1'b0;
              cache_wb_then_fill <= 1'b0;
              cache_way0_rd_idx <= cache_target_idx;
              cache_way1_rd_idx <= cache_target_idx;
              cache_way0_bank0_rd_idx <= cache_target_idx;
              cache_way1_bank0_rd_idx <= cache_target_idx;
              cache_state <= CACHE_WB_PREP;
           end else begin
              cache_state <= CACHE_FILL_REQ;
           end
        end

        CACHE_WB_PREP: begin
           cache_state <= CACHE_WB_READ_WAIT;
        end

        CACHE_WB_READ_WAIT: begin
           cache_state <= CACHE_WB_REQ;
        end

        CACHE_WB_REQ: begin
           if (cache_wb_to_bram) begin
              begin : cache_bram_wb_req
                 reg [63:0] bram_beat_addr;
                 bram_beat_addr = cache_wb_base + {58'd0, cache_wb_beat, 3'd0} -
                                  cache_bram_base_addr;
                 cache_bram_word_idx  <= bram_beat_addr[`MEM_SIZE_LG2-1:4];
                 cache_bram_word_bank <= bram_beat_addr[3];
                 cache_bram_wb_data   <= dcache_selected_bank_data(cache_victim_way, cache_wb_beat);
              end
              cache_state <= CACHE_BRAM_WB_WRITE;
           end else begin
`ifdef SIMULATE
              if (cache_trace_enabled && !l2_wb_req_valid) begin
                 $display("%05d CACHE WBREQ line addr=%016h data0=%016h",
                          $time,
                          cache_wb_base,
                          dcache_selected_bank_data(cache_victim_way, 3'd0));
              end
`endif
              if (!l2_wb_req_valid) begin
                 l2_wb_req_line_addr <= cache_wb_base[30:6];
                 l2_wb_req_line_data <= dcache_selected_line_data(cache_victim_way);
                 l2_wb_req_valid     <= 1;
              end else if (l2_wb_req_ready) begin
                 l2_wb_req_valid <= 0;
                 cache_state <= CACHE_WB_WAIT;
              end
           end
        end

        CACHE_BRAM_WB_WRITE: begin
           if (cache_bram_word_bank)
              mem1[cache_bram_word_idx] <= cache_bram_wb_data;
           else
              mem0[cache_bram_word_idx] <= cache_bram_wb_data;
           cache_bram_write_done <= 1;
           cache_state <= CACHE_WB_WAIT;
        end

        CACHE_WB_WAIT: begin
           if (cache_wb_to_bram && cache_bram_write_done) begin
              if (cache_wb_beat == 3'd7) begin
                 cache_finish_writeback_line();
              end else begin
                 cache_wb_beat <= cache_wb_beat + 1;
                 cache_state <= CACHE_WB_REQ;
              end
           end else if (!cache_wb_to_bram) begin
              l2_wb_rsp_ready <= 1;
              if (l2_wb_rsp_valid && l2_wb_rsp_ready) begin
                 l2_wb_rsp_ready <= 0;
                 cache_finish_writeback_line();
              end
           end
        end

        CACHE_FILL_REQ: begin
`ifdef SIMULATE
           if (cache_trace_enabled && !cache_fill_from_bram && !l2_fill_req_valid) begin
              $display("%05d %s FILLREQ line addr=%016h",
                       $time,
                       cache_req_ifetch ? "ICACHE" : "DCACHE",
                       cache_fill_base);
           end
`endif
           if (cache_fill_from_bram) begin
              begin : cache_bram_fill_req
                 reg [63:0] bram_beat_addr;
                 bram_beat_addr = cache_fill_base + {58'd0, cache_fill_beat, 3'd0} -
                                  cache_bram_base_addr;
                 cache_bram_word_idx  <= bram_beat_addr[`MEM_SIZE_LG2-1:4];
                 cache_bram_word_bank <= bram_beat_addr[3];
              end
              cache_state <= CACHE_BRAM_FILL_READ;
           end else begin
              if (!l2_fill_req_valid) begin
                 l2_fill_req_line_addr <= cache_fill_base[30:6];
                 l2_fill_req_valid <= 1;
              end else if (l2_fill_req_ready) begin
                 l2_fill_req_valid <= 0;
                 cache_state <= CACHE_FILL_LINE_WAIT;
              end
           end
        end

        CACHE_FILL_LINE_WAIT: begin
           l2_fill_rsp_ready <= 1;
           if (l2_fill_rsp_valid && l2_fill_rsp_ready) begin
              cache_fill_line_data <= l2_fill_rsp_data;
              l2_fill_rsp_ready <= 0;
              cache_fill_beat <= 0;
              cache_state <= CACHE_FILL_LINE_INSTALL;
           end
        end

        CACHE_BRAM_FILL_READ: begin
           cache_bram_read_data_stage <= cache_bram_word_bank ? mem1[cache_bram_word_idx]
                                                              : mem0[cache_bram_word_idx];
           cache_state <= CACHE_BRAM_FILL_CAPTURE;
        end

        CACHE_BRAM_FILL_CAPTURE: begin
           cache_bram_fill_data <= cache_bram_read_data_stage;
           cache_state <= CACHE_BRAM_FILL_COMMIT;
        end

        CACHE_BRAM_FILL_COMMIT,
        CACHE_FILL_LINE_INSTALL: begin
           if (cache_fill_data_valid) begin
              if (cache_fill_beat == cache_req_bank) begin
                 if (cache_req_write)
                    cache_fill_return_data <= merge_store_bytes(cache_fill_data, cache_store_data, cache_store_strb);
                 else
                    cache_fill_return_data <= cache_fill_data;
              end
              if (cache_fill_beat == cache_req_next_bank)
                 cache_fill_next_data <= cache_fill_data;
              if (cache_fill_beat == 3'd7) begin
                 hpm_vhpr_pulse[`VHPRP_FILLS] <= 1;
                 if (cache_target_valid && !cache_target_dirty)
                    hpm_vhpr_pulse[`VHPRP_VICTIM_EVICTS] <= 1;
                 if (cache_req_perm[5] && cache_req_write && !cache_req_ifetch) begin
                    // Svpbmt NC/IO store (flush-around): do NOT install. The
                    // filled+merged line sits in the cache_target banks; write it
                    // back to memory and invalidate the slot so the store reaches
                    // DRAM uncached. (No-install would lose the dirty store.)
                    cache_victim_way        <= cache_target_way;
                    cache_victim_idx        <= cache_target_idx;
                    cache_victim_ptag       <= cache_req_ptag;
                    cache_wb_base           <= {33'd0, cache_req_ptag, cache_target_idx[5:0], 6'd0};
                    cache_wb_beat           <= 0;
                    cache_wb_after_ncstore  <= 1'b1;
                    cache_way0_rd_idx       <= cache_target_idx;
                    cache_way1_rd_idx       <= cache_target_idx;
                    cache_way0_bank0_rd_idx <= cache_target_idx;
                    cache_way1_bank0_rd_idx <= cache_target_idx;
                    cache_state             <= CACHE_WB_PREP;
                 end else begin
                    if (!cache_req_ifetch) begin
                       dcache_way0_tag_wr_en <= !cache_target_way;
                       dcache_way1_tag_wr_en <= cache_target_way;
                       dcache_tag_wr_idx  <= cache_target_idx;
                       // Svpbmt NC/IO load: invalidate the slot (tag=0) instead of
                       // installing, so the uncacheable line is not retained.
                       if (cache_req_perm[5])
                          dcache_tag_wr_data <= 0;
                       else
                          dcache_tag_wr_data <= cache_make_meta(cache_req_write, 1'b1,
                                                                cache_req_asid,
                                                                cache_req_perm,
                                                                cache_req_vtag,
                                                                cache_req_ptag,
                                                                vhpr_epoch);
                    end
                    if (cache_req_write) begin
                       dmem_write_done_r <= 1;
                    end else if (cache_req_ifetch) begin
                       ifetch_refill_retry_valid <= 1;
                    end else begin
                       emit_dmem_load_rsp(
                          cache_req_bank == 3'd7 ? cache_fill_data : cache_fill_return_data,
                          cache_req_same_line
                          ? (cache_req_next_bank == 3'd7 ? cache_fill_data : cache_fill_next_data)
                          : dcache_rsp_next_data,
                          cache_req_same_line ||
                             (dcache_rsp_next_hit && cache_req_va[11:3] != 9'h1ff));
                    end
                    cache_state      <= CACHE_IDLE;
                 end
              end else begin
                 cache_fill_beat <= cache_fill_beat + 1;
                 cache_state     <= cache_bram_fill_commit ? CACHE_FILL_REQ :
                                                            CACHE_FILL_LINE_INSTALL;
              end
           end
        end

        default: cache_state <= CACHE_IDLE;
      endcase

      if (core_reset_now) begin
         cache_state <= CACHE_IDLE;
         dmem_rsp_valid_r <= 0;
         dmem_rsp_next_valid_r <= 0;
         dmem_write_done_r <= 0;
         cache_bram_write_done <= 0;
         l2_fill_req_valid <= 0;
         l2_fill_rsp_ready <= 0;
         l2_wb_req_valid <= 0;
         l2_wb_rsp_ready <= 0;
         l2_direct_read_req_valid <= 0;
         l2_direct_read_rsp_ready <= 0;
         dcache_way0_tag_wr_en <= 0;
         dcache_way1_tag_wr_en <= 0;
         icache_invalidate_valid <= 0;
	 cache_cbo_done_r <= 0;
	 cache_req_ifetch <= 0;
	 cache_req_perm <= CACHE_PERM_PHYS;
	 cache_req_ctx <= 0;
	 cache_req_line_addr <= 0;
	 cache_flush_ack <= cache_flush_req;
         cache_wb_after_cbo <= 0;
         cache_wb_then_fill <= 0;
         cache_wb_after_flush <= 0;
         cache_need_target_wb <= 0;
         ptw_direct_probe_pending <= 0;
         ptw_direct_wait_probe <= 0;
         ptw_direct_pending <= 0;
         ptw_direct_wait_bram <= 0;
         ptw_direct_wait_axi <= 0;
         ptw_direct_rsp_valid_r <= 0;
         ptw_direct_rsp_data_r <= 0;
         vhpr_epoch <= 0;
         vhpr_next_epoch <= 0;
         vhpr_epoch_update_pending <= 0;
         vhpr_epoch_bump_ack <= vhpr_epoch_bump_req;
      end
   end

`ifdef SIMULATE
   always @(posedge clock) begin
      if (state_summary_enabled && !reset) begin
         state_stat_total_cycles <= state_stat_total_cycles + 1;
         if (hpm_instret_pulse)
            state_stat_instret <= state_stat_instret + 1;
         if (state <= `S_LAST_STATE)
            state_stat_cycles[state] <= state_stat_cycles[state] + 1;
         cache_state_stat_cycles[cache_state] <= cache_state_stat_cycles[cache_state] + 1;
         if (state_summary_interval != 0 &&
             state_stat_total_cycles + 1 >= state_summary_next) begin
            dump_state_summary;
            state_summary_next <= state_summary_next + state_summary_interval;
         end
      end

      if (cache_summary_enabled) begin
         if (hpm_icache_read_pulse)
            icache_stat_reads <= icache_stat_reads + 1;
         if (hpm_icache_hit_pulse)
            icache_stat_hits <= icache_stat_hits + 1;
         if (hpm_icache_miss_pulse)
            icache_stat_misses <= icache_stat_misses + 1;
         if (hpm_icache_fill_line_pulse)
            icache_stat_fill_lines <= icache_stat_fill_lines + 1;

         if (hpm_dcache_read_pulse)
            dcache_stat_reads <= dcache_stat_reads + 1;
         if (hpm_dcache_write_pulse)
            dcache_stat_writes <= dcache_stat_writes + 1;
         if (hpm_dcache_hit_pulse)
            dcache_stat_hits <= dcache_stat_hits + 1;
         if (hpm_dcache_miss_pulse) begin
            dcache_stat_misses <= dcache_stat_misses + 1;
            if (dcache_rsp_dirty)
               dcache_stat_dirty_misses <= dcache_stat_dirty_misses + 1;
         end
         if (hpm_dcache_fill_line_pulse)
            dcache_stat_fill_lines <= dcache_stat_fill_lines + 1;
         if (hpm_dcache_wb_line_pulse)
            dcache_stat_wb_lines <= dcache_stat_wb_lines + 1;

         if ((hpm_icache_miss_pulse && icache_stat_misses[12:0] == 13'h1fff) ||
             (hpm_dcache_miss_pulse && dcache_stat_misses[12:0] == 13'h1fff)) begin
            $display("%05d L1 SUMMARY icache_reads=%0d icache_hits=%0d icache_misses=%0d icache_fill_lines=%0d dcache_reads=%0d dcache_writes=%0d dcache_hits=%0d dcache_misses=%0d dcache_dirty_misses=%0d dcache_fill_lines=%0d dcache_wb_lines=%0d",
                        $time,
                        icache_stat_reads + (hpm_icache_read_pulse ? 64'd1 : 64'd0),
                        icache_stat_hits + (hpm_icache_hit_pulse ? 64'd1 : 64'd0),
                        icache_stat_misses + (hpm_icache_miss_pulse ? 64'd1 : 64'd0),
                        icache_stat_fill_lines + (hpm_icache_fill_line_pulse ? 64'd1 : 64'd0),
                        dcache_stat_reads + (hpm_dcache_read_pulse ? 64'd1 : 64'd0),
                        dcache_stat_writes + (hpm_dcache_write_pulse ? 64'd1 : 64'd0),
                        dcache_stat_hits + (hpm_dcache_hit_pulse ? 64'd1 : 64'd0),
                        dcache_stat_misses + (hpm_dcache_miss_pulse ? 64'd1 : 64'd0),
                        dcache_stat_dirty_misses +
                           (hpm_dcache_miss_pulse && dcache_rsp_dirty ? 64'd1 : 64'd0),
                        dcache_stat_fill_lines + (hpm_dcache_fill_line_pulse ? 64'd1 : 64'd0),
                        dcache_stat_wb_lines + (hpm_dcache_wb_line_pulse ? 64'd1 : 64'd0));
         end
      end

      if (hpm_tlb_lookup_pulse)
         tlb_stat_lookups <= tlb_stat_lookups + 1;
      if (hpm_tlb_hit_pulse)
         tlb_stat_hits <= tlb_stat_hits + 1;
      if (hpm_tlb_miss_pulse)
         tlb_stat_misses <= tlb_stat_misses + 1;
      if (hpm_tlb_hit_4k_pulse)
         tlb_stat_hits_4k <= tlb_stat_hits_4k + 1;
      if (hpm_tlb_hit_2m_pulse)
         tlb_stat_hits_2m <= tlb_stat_hits_2m + 1;
      if (hpm_tlb_insert_4k_pulse)
         tlb_stat_inserts_4k <= tlb_stat_inserts_4k + 1;
      if (hpm_tlb_insert_2m_pulse)
         tlb_stat_inserts_2m <= tlb_stat_inserts_2m + 1;
      if (hpm_tlb_evict_4k_pulse)
         tlb_stat_evicts_4k <= tlb_stat_evicts_4k + 1;
      if (hpm_tlb_evict_2m_pulse)
         tlb_stat_evicts_2m <= tlb_stat_evicts_2m + 1;
      if (hpm_tlb_uncached_1g_pulse)
         tlb_stat_uncached_1g <= tlb_stat_uncached_1g + 1;
      if (hpm_tlb_uncached_napot_pulse)
         tlb_stat_uncached_napot <= tlb_stat_uncached_napot + 1;

      if (cache_trace_enabled) begin
         if (hpm_icache_miss_pulse || hpm_dcache_miss_pulse) begin
            $display("%05d %s MISS  op=%0d addr=%016h bank=%0d next_bank=%0d same_line=%0d dirty=%0d victim=%016h",
                     $time,
                     cache_req_ifetch ? "ICACHE" : "DCACHE",
                     cache_req_write,
                     cache_addr,
                     cache_req_bank,
                     cache_req_next_bank,
                     cache_req_same_line,
                     cache_req_ifetch ? 1'b0 : dcache_rsp_dirty,
                     {33'd0, cache_victim_ptag, cache_victim_idx[5:0], 6'd0});
         end
         if (hpm_icache_fill_beat_pulse || hpm_dcache_fill_beat_pulse) begin
            $display("%05d %s FILLD beat=%0d addr=%016h data=%016h",
                     $time,
                     cache_req_ifetch ? "ICACHE" : "DCACHE",
                     cache_fill_beat,
                     cache_fill_base + (64'd8 * cache_fill_beat),
                     cache_fill_data);
         end
         if (hpm_icache_fill_line_pulse || hpm_dcache_fill_line_pulse) begin
            $display("%05d %s FILLDONE addr=%016h write=%0d",
                     $time,
                     cache_req_ifetch ? "ICACHE" : "DCACHE",
                     cache_fill_base,
                     cache_req_write);
         end
      end
   end
`endif

   // ----- Lower-memory boundary -----
   // L1 tag/data hits stay in the core clock domain.  Only slow-path
   // non-BRAM line fills, dirty writebacks, and direct PTW reads cross to the
   // memory clock domain.  I$ and D$ line-fill ports are separate here so the
   // frontend can eventually issue misses independently of the D$ FSM.
   smolrv64_l2_boundary l2_boundary_inst (
      .core_clock          (clock),
      .mem_clock           (mem_clock),
      .reset               (core_reset_now),
      .idle                (mem_engine_idle),

      .icache_fill_req_valid     (icache_l2_fill_req_valid),
      .icache_fill_req_ready     (icache_l2_fill_req_ready),
      .icache_fill_req_line_addr (l2_fill_req_line_addr),
      .icache_fill_rsp_valid     (icache_l2_fill_rsp_valid),
      .icache_fill_rsp_ready     (icache_l2_fill_rsp_ready),
      .icache_fill_rsp_data      (icache_l2_fill_rsp_data),

      .dcache_fill_req_valid     (dcache_l2_fill_req_valid),
      .dcache_fill_req_ready     (dcache_l2_fill_req_ready),
      .dcache_fill_req_line_addr (l2_fill_req_line_addr),
      .dcache_fill_rsp_valid     (dcache_l2_fill_rsp_valid),
      .dcache_fill_rsp_ready     (dcache_l2_fill_rsp_ready),
      .dcache_fill_rsp_data      (dcache_l2_fill_rsp_data),

      .wb_req_valid        (l2_wb_req_valid),
      .wb_req_ready        (l2_wb_req_ready),
      .wb_req_line_addr    (l2_wb_req_line_addr),
      .wb_req_line_data    (l2_wb_req_line_data),
      .wb_rsp_valid        (l2_wb_rsp_valid),
      .wb_rsp_ready        (l2_wb_rsp_ready),

      .read_req_valid      (l2_direct_read_req_valid),
      .read_req_ready      (l2_direct_read_req_ready),
      .read_req_addr       (l2_direct_read_req_addr),
      .read_rsp_valid      (l2_direct_read_rsp_valid),
      .read_rsp_ready      (l2_direct_read_rsp_ready),
      .read_rsp_data       (l2_direct_read_rsp_data),

      .m_axi_awid          (m_axi_awid),
      .m_axi_awaddr        (m_axi_awaddr),
      .m_axi_awlen         (m_axi_awlen),
      .m_axi_awsize        (m_axi_awsize),
      .m_axi_awburst       (m_axi_awburst),
      .m_axi_awlock        (m_axi_awlock),
      .m_axi_awcache       (m_axi_awcache),
      .m_axi_awprot        (m_axi_awprot),
      .m_axi_awqos         (m_axi_awqos),
      .m_axi_awvalid       (m_axi_awvalid),
      .m_axi_awready       (m_axi_awready),
      .m_axi_wdata         (m_axi_wdata),
      .m_axi_wstrb         (m_axi_wstrb),
      .m_axi_wlast         (m_axi_wlast),
      .m_axi_wvalid        (m_axi_wvalid),
      .m_axi_wready        (m_axi_wready),
      .m_axi_bid           (m_axi_bid),
      .m_axi_bresp         (m_axi_bresp),
      .m_axi_bvalid        (m_axi_bvalid),
      .m_axi_bready        (m_axi_bready),
      .m_axi_arid          (m_axi_arid),
      .m_axi_araddr        (m_axi_araddr),
      .m_axi_arlen         (m_axi_arlen),
      .m_axi_arsize        (m_axi_arsize),
      .m_axi_arburst       (m_axi_arburst),
      .m_axi_arlock        (m_axi_arlock),
      .m_axi_arcache       (m_axi_arcache),
      .m_axi_arprot        (m_axi_arprot),
      .m_axi_arqos         (m_axi_arqos),
      .m_axi_arvalid       (m_axi_arvalid),
      .m_axi_arready       (m_axi_arready),
      .m_axi_rid           (m_axi_rid),
      .m_axi_rdata         (m_axi_rdata),
      .m_axi_rresp         (m_axi_rresp),
      .m_axi_rlast         (m_axi_rlast),
      .m_axi_rvalid        (m_axi_rvalid),
      .m_axi_rready        (m_axi_rready)
   );

endmodule
