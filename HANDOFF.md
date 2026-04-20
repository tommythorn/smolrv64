# Handoff: smolrv64 Linux boot — hang after sched_clock init

## Goal
smolrv64 (RV64IMAFDC, Verilog) boots OpenSBI v0.8 + Linux 5.4 from the
consolidated inputs under `workloads/linux/`. Drive boot further and match
`workloads/linux/golden-output.txt`.

## Current state
- Verilog simulation (`make -C workloads/linux run`) enters Linux and
  matches `golden-output.txt` up to:
    `[    0.047329] sched_clock: 64 bits at 1000kHz, resolution 1000ns, wraps every 2199023255500ns`
  then appears to hang.
- Next expected line per golden output:
    `[    0.224486] printk: console [ttyS0] disabled`
- Tried adjusting `mtime` to clock/32 — no change in where it stops.
- Cosim has **not** been tried on the current hang; `workloads/linux`
  lacks a cosim Makefile target. Adding one is a prerequisite.

## Key facts still valid
- Consolidating all inputs (`mem.even`, `mem.odd`, `rf.hex`, DTB) under
  `workloads/linux/` resolved the earlier "no kernel UART output" bug.
  Prior divergence was caused by Verilog sim / cosim / HW / simmerv
  loading *different* memory images — one had a stale/bogus DTB.
- Cosim now matches Verilog sim given the same consolidated inputs.
- OpenSBI→kernel handoff: `fw_payload.bin @ 0x80000000`, `dts.dtb @ 0x81000000`.
- UART is NS16550A at `0x10000000`; MMIO dispatch in `smolrv64.v` keyed
  on `mem_addr[63:4] == 60'h100_0000`; `uart_tx_valid` asserts only when
  offset==0 and `uart_lcr[7]==0` (DLAB=0).
- `src/uart5.v` was obsolete and has been deleted.
- `sim_main.cpp` uses `setvbuf(stdout, nullptr, _IOLBF, 0)` for reliable
  line-buffered output on SIGTERM.

## Cosim caveat (fundamental)
Differential cosim only checks that smolrv64 and simmerv agree given the
*same* inputs — it cannot catch input corruption (e.g. bogus DTB), since
both engines would agree on the wrong answer. simmerv also should not be
trusted as a gold reference for FP (uses host native + fenv). Golden
reference for boot remains `golden-output.txt` from a blessed run.

## Leading hypothesis
NS16550A emulation bug. This exact symptom (Linux boot stalls shortly
after sched_clock while still inside early console bring-up) has hit
before and was traced to the UART model. The next expected output is a
`printk: console [ttyS0] disabled` line, implying the kernel is about to
re-init/hand off the console — a natural place for a UART-side bug to
wedge forward progress.

## Next steps
1. Add a `cosim` target to `workloads/linux/Makefile` so this hang can
   be reproduced under cosim and the exact divergence retire found.
2. Check NS16550A register semantics against Linux 8250 driver
   expectations post-early-console: LSR THRE/TEMT, IER bits, FCR/LCR
   programming sequence at re-init.
3. If cosim shows arch-state agreement through the hang, the bug is in
   the device model (UART timer/IRQ), not the core.

## How to run
```
make -C workloads/linux run                  # Verilog sim vs golden
make -C workloads/linux are-we-there-yet     # diff against golden
```
Always use `timeout` when invoking smolrv64 binaries directly — they can
hang indefinitely.

## Conventions
- Do NOT regenerate DTS or `mem.even`/`mem.odd`; use the consolidated
  files under `workloads/linux/`.
- No Co-Authored-By lines in commits; terse commit messages.
- Verify fixes against a fresh rebuild before claiming they work.
