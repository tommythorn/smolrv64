`ifndef SMOLRV64_STATES_VH
`define SMOLRV64_STATES_VH

// Core microarchitectural FSM state codes (S_*), fetch FSM (F_*), execute
// substate (EX_*), and the iterative M-extension op codes (MULDIV_*).

`define S_FETCH1         0
`define S_FETCH2         1  // fetch translation return token; not a live FSM state
`define S_EXECUTE        3

`define S_EXCEPTION      4

`define S_LOAD_ALIGN     5
`define S_MMIO_ALIGN     7
`define S_AMO            8

`define S_STORE          9

`define S_HANDLE_CSR    10

`define S_MUL_RUNNING   11
`define S_DIV_RUNNING   12

`define S_PTW_LAUNCH    14  // launch PTW PTE fetch after ptw_* request fields are registered
`define S_FETCH2_HALF   15

`define S_IFETCH_WAIT           16  // wait for I-cache instruction response
`define S_DMEM_LOAD_WAIT       17  // wait for D-cache load response
`define S_PTW_DIRECT_WAIT      18  // wait for direct page-table-walk PTE read
`define S_DMEM_STORE_WAIT      19  // backpressure wait before first store beat
`define S_IFETCH_HALF_WAIT      20  // wait for 2nd I-cache response of cross-line fetch
`define S_DMEM_LOAD2_WAIT      21  // wait for 2nd D-cache response of cross-line load
`define S_DMEM_STORE2          22  // issue 2nd beat of cross-line store
`define S_EXECUTE2             24  // complete write_back_value from pre-computed exe_add
`define S_PTW_PROCESS          25  // process PTE latched from the PTW response
`define S_RF                   26  // register BRAM output (s1_bram/s2_bram) into execute_req_rs1_value/execute_req_rs2_value flip-flops
`define S_CBO_EXEC             29  // execute translated cache-block operation
`define S_CBO_WAIT             30  // wait for cache-block operation completion
`define S_STORE_COMMIT         31  // commit a store after translation/routing decision
`define S_CVFPU_ISSUE          33  // present a CVFPU operation until accepted
`define S_CVFPU_WAIT           34  // wait for a CVFPU result
// (35, 36 retired: the FMA rs3 detour folded into the normal S_RF read once
//  rs3 got its own FP read port)
`define S_TLB_LOOKUP           37  // wait for direct-mapped TLB RAM outputs
`define S_TLB_CHECK            38  // compare direct-mapped TLB entries
`define S_DMEM_STORE_RESP_WAIT 43  // wait for an issued D-cache store to complete
`define S_DMEM_STORE_RESP_ARM  44  // absorb one cycle so AXI busy flags see a new write
`define S_IFETCH_RESP          45  // latch instruction from I-fetch response without fetch-source mux
`define S_FETCH_BUF_CHECK      46  // fallback register for fetch-buffer hit decision
`define S_FETCH_BUF_USE        47  // fallback consume for registered fetch-buffer hit
`define S_MULDIV_START         48  // initialize iterative M-extension datapath
`define S_FETCH_REQ            50  // issue registered PC/context fetch request
`define S_FRONTEND_MISS_WAIT   51  // wait for speculative frontend cache miss after backend retire
`define S_INT_COMMIT           52  // retire staged integer result after side effects
`define S_LOCAL_LOAD           54  // commit local UART/CLINT/PLIC load data after address dispatch
`define S_TLB_INSERT           55  // commit staged PTW result into the TLB, then route translated PA
`define S_LAST_STATE           55  // update state register width accordingly

// f_state: the free-running frontend FSM. Drives the cache-hit fetch path
// (FETCH_REQ -> FETCH_BUF_CHECK -> FETCH_BUF_USE -> enqueue to rf_decode_*)
// independently of the backend `state` register, so frontend work overlaps
// with backend long-latency states (CVFPU, MULDIV, AMO, DRAM, etc).
//
// I-cache miss / TLB miss / cross-doubleword fetch still escalate
// to the backend FSM — those resources are shared with the
// load/store path and require arbitration the frontend can't do alone.
`define F_IDLE                  0  // no fetch in flight
`define F_FETCH_BUF_CHECK       1  // latch frontend_rsp_* into f_latched_*
`define F_FETCH_BUF_USE         2  // on hit, enqueue rf_decode; on miss, hand to backend

// ex_state: scaffolding for the back-half pipeline split. Eventually owns
// S_EXECUTE / S_EXECUTE2 (and the various memory/EX states) so an instruction
// can be in EX while the next is in RF.
`define EX_IDLE              1'b0  // EX stage empty; nothing in flight
`define EX_EXECUTE2          1'b1  // compute write_back_value from exe_add / exe_sext32

`define MULDIV_MUL             4'd0
`define MULDIV_MULH            4'd1
`define MULDIV_MULHSU          4'd2
`define MULDIV_MULHU           4'd3
`define MULDIV_DIV             4'd4
`define MULDIV_DIVU            4'd5
`define MULDIV_REM             4'd6
`define MULDIV_REMU            4'd7
`define MULDIV_MULW            4'd8
`define MULDIV_DIVW            4'd9
`define MULDIV_DIVUW           4'd10
`define MULDIV_REMW            4'd11
`define MULDIV_REMUW           4'd12

`endif // SMOLRV64_STATES_VH
