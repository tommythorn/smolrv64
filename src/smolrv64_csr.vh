`ifndef SMOLRV64_CSR_VH
`define SMOLRV64_CSR_VH

// RISC-V CSR address map (plus the custom MRO latency-stat CSRs) and the
// internal CSR read-modify-write op codes, used by the core's CSR handling.

`define CSR_FFLAGS     12'h001
`define CSR_FRM        12'h002
`define CSR_FCSR       12'h003

`define CSR_SSTATUS    12'h100
`define CSR_SIE        12'h104
`define CSR_STVEC      12'h105
`define CSR_SCOUNTEREN 12'h106
`define CSR_SENVCFG    12'h10a
`define CSR_SSCRATCH   12'h140
`define CSR_SEPC       12'h141
`define CSR_SCAUSE     12'h142
`define CSR_STVAL      12'h143
`define CSR_SIP        12'h144
`define CSR_STIMECMP   12'h14d
`define CSR_SCOUNTOVF  12'hda0

`define CSR_SATP       12'h180

`define CSR_MSTATUS    12'h300
`define CSR_MISA       12'h301
`define CSR_MEDELEG    12'h302
`define CSR_MIDELEG    12'h303
`define CSR_MIE        12'h304
`define CSR_MTVEC      12'h305
`define CSR_MCOUNTEREN 12'h306
`define CSR_MENVCFG    12'h30a
`define CSR_MCOUNTINHIBIT 12'h320
`define CSR_MCYCLECFG  12'h321
`define CSR_MINSTRETCFG 12'h322
`define CSR_MHPMEVENT3 12'h323

`define CSR_MSCRATCH   12'h340
`define CSR_MEPC       12'h341
`define CSR_MCAUSE     12'h342
`define CSR_MTVAL      12'h343
`define CSR_MIP        12'h344

`define CSR_PMPCFG0    12'h3a0
`define CSR_PMPCFG1    12'h3a1
`define CSR_PMPCFG2    12'h3a2
`define CSR_PMPCFG3    12'h3a3
`define CSR_PMPCFG4    12'h3a4
`define CSR_PMPCFG5    12'h3a5
`define CSR_PMPCFG6    12'h3a6
`define CSR_PMPCFG7    12'h3a7
`define CSR_PMPCFG8    12'h3a8
`define CSR_PMPCFG9    12'h3a9
`define CSR_PMPCFG10   12'h3aa
`define CSR_PMPCFG11   12'h3ab
`define CSR_PMPCFG12   12'h3ac
`define CSR_PMPCFG13   12'h3ad
`define CSR_PMPCFG14   12'h3ae
`define CSR_PMPCFG15   12'h3af
`define CSR_PMPADDR0   12'h3b0
`define CSR_PMPADDR1   12'h3b1
`define CSR_PMPADDR2   12'h3b2
`define CSR_PMPADDR3   12'h3b3
`define CSR_PMPADDR4   12'h3b4
`define CSR_PMPADDR5   12'h3b5
`define CSR_PMPADDR6   12'h3b6
`define CSR_PMPADDR7   12'h3b7
`define CSR_PMPADDR8   12'h3b8
`define CSR_PMPADDR9   12'h3b9
`define CSR_PMPADDR10  12'h3ba
`define CSR_PMPADDR11  12'h3bb
`define CSR_PMPADDR12  12'h3bc
`define CSR_PMPADDR13  12'h3bd
`define CSR_PMPADDR14  12'h3be
`define CSR_PMPADDR15  12'h3bf

// https://www.five-embeddev.com/riscv-debug-spec/v0.13-release/hwbp_registers.html
`define CSR_TSELECT    12'h7a0 // which trigger is accessible through the other trigger registers
`define CSR_TDATA1     12'h7a1 // type:4 dmode:1 data:59
`define CSR_TDATA2     12'h7a2
`define CSR_TDATA3     12'h7a3
`define CSR_TINFO      12'h7a4 // RO
`define CSR_TCONTROL   12'h7a5 // This is optional

`define CSR_DCSR       12'h7b0
`define CSR_DSCRATCH   12'h7b2
`define CSR_MNSTATUS   12'h744 // mnstatus: resumable-NMI status (Smrnmi)
`define CSR_MCYCLE     12'hb00
`define CSR_MTIME      12'hb01
`define CSR_MINSTRET   12'hb02
`define CSR_MHPMCOUNTER3 12'hb03
`define CSR_CYCLE      12'hc00
`define CSR_TIME       12'hc01
`define CSR_INSTRET    12'hc02
`define CSR_HPMCOUNTER3 12'hc03
`define CSR_MHARTID    12'hf14
`define CSR_MVENDORID  12'hf11
`define CSR_MARCHID    12'hf12
`define CSR_MIMPID     12'hf13

// Custom MRO CSRs: memory transaction latency stats (cycles spent in
// I-fetch/D-memory/PTW wait states).
`define CSR_MIG_MIN      12'hfc0
`define CSR_MIG_MAX      12'hfc1
`define CSR_MIG_TOTAL    12'hfc2
`define CSR_MIG_COUNT    12'hfc3
`define CSR_MIG_TIMEOUTS 12'hfc4
`define CSR_MIG_TO_PC    12'hfc5
`define CSR_MIG_TO_TVAL  12'hfc6
`define CSR_MIG_TO_STATE 12'hfc7
`define CSR_MIG_TO_CAUSE 12'hfc8
`define CSR_MIG_TO_ADDR  12'hfc9
`define CSR_VHPR_EPOCH 12'hfdb
`define CSR_BUILD_STAMP 12'hfde

`define CSR_OP_COPY 0
`define CSR_OP_OR   1
`define CSR_OP_ANDN 2

`endif // SMOLRV64_CSR_VH
