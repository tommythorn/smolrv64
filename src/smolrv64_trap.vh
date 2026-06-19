`ifndef SMOLRV64_TRAP_VH
`define SMOLRV64_TRAP_VH

// RISC-V trap (exception) cause codes and interrupt cause codes.

`define TRAP_INSTRUCTION_ADDRESS_MISALIGNED      0
`define TRAP_INSTRUCTION_ACCESS_FAULT            1
`define TRAP_ILLEGAL_INSTRUCTION                 2
`define TRAP_BREAKPOINT                          3
`define TRAP_LOAD_ADDRESS_MISALIGNED             4
`define TRAP_LOAD_ACCESS_FAULT                   5
`define TRAP_STORE_ADDRESS_MISALIGNED            6
`define TRAP_STORE_ACCESS_FAULT                  7
`define TRAP_ENVIRONMENT_CALL_FROM_U_MODE        8
`define TRAP_ENVIRONMENT_CALL_FROM_S_MODE        9
// 10 is reserved
`define TRAP_ENVIRONMENT_CALL_FROM_M_MODE       11
`define TRAP_INSTRUCTIONPAGE_FAULT              12
`define TRAP_LOAD_PAGE_FAULT                    13
// 14 is reserved
`define TRAP_STORE_PAGE_FAULT                   15

`define USER_SOFTWARE_INTERRUPT                  0
`define SUPERVISOR_SOFTWARE_INTERRUPT            1
`define MACHINE_SOFTWARE_INTERRUPT               3

`define USER_TIMER_INTERRUPT                     4
`define SUPERVISOR_TIMER_INTERRUPT               5
`define MACHINE_TIMER_INTERRUPT                  7

`define USER_EXTERNAL_INTERRUPT                  8
`define SUPERVISOR_EXTERNAL_INTERRUPT            9
`define MACHINE_EXTERNAL_INTERRUPT              11
`define LOCAL_COUNTER_OVERFLOW_INTERRUPT        13

`endif // SMOLRV64_TRAP_VH
