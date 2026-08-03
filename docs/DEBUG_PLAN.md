## RISC-V Debug Support

### Goal
Replace the current key[1] soft-reset escape hatch with real debug-mode entry,
resume, and external debugger control.

### Direction
- Implement the RISC-V external debug spec rather than adding another local
  monitor-only mechanism.
- Treat key[1] as a debugger interrupt request. It should enter debug mode
  without destroying architectural state, cache contents, or the debug CSRs
  used for postmortem counters.
- Preserve the ability to resume after monitor/debugger inspection. A halted
  Linux boot should be inspectable and then resumable unless the debugger
  explicitly resets the core.
- Keep soft reset as a separate operation for cases where restart is actually
  wanted.

### Work Items
- Add debug-mode architectural state: debug PC, cause, privilege tracking,
  single-step hooks if practical, and `dret`.
- Define the transport. The fastest useful version can be monitor-mediated,
  but the long-term target should match the spec closely enough for standard
  debugger tooling.
- Audit trap, interrupt, CSR, cache-flush, and pipeline-drain paths so debug
  entry is precise and resume is not a special case.
- Decide how debug-mode memory/register access interacts with the VHPR cache,
  TLBs, and in-flight cache maintenance.

### Non-Goals For The Current UART/LED Patch
- No debug-mode implementation.
- No change to key[1] behavior yet.
- No dependency on this plan for the PPP buffering workaround.
