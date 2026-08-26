# Handoff — the board hang, and what it cost to find, 2026-08-20

Written to be enough on its own. Everything is measured unless it says otherwise, and the
places I was wrong are recorded deliberately, because two of them cost hours.

## The result

**The board hang is FIXED and verified on hardware. The clock did not move.**

Shippable bitstream: `/var/tmp/inorder_111MHz_FIXED_102b2fd7.bit` — `PROBE_CLK_DIV8=72`
(111.11 MHz), `OOO2_CORE=1 OOO2_HW=4`, **closed natively at WNS +0.134** and boots to
`/sbin/init` over NFS with the UNMODIFIED interrupt-enabled `ubuntu-nfs.dtb`.

    cd platforms/rk-xcku5p-f-v1.2 && make program BIT=/var/tmp/inorder_111MHz_FIXED_102b2fd7.bit
    cd workloads/ubuntu && DTB=ubuntu-nfs.dtb ./ubuntu-boot.sh

Against the build it replaces, at the same clock:

| | `inorder_111MHz_1ba5da1b.bit` | this one |
|---|---|---|
| closure | +0.008, needed a `make physopt` rescue | **+0.134, closed natively** |
| boots?  | hangs at the console handover | **reaches /sbin/init** |
| IPC     | baseline | **+4.95%** (the F/X queue work is finally usable) |

A second verified bitstream exists at 66.67 MHz:
`/var/tmp/inorder_66MHz_FIXED_cb24e5ae.bit`.

## The defect

**`49eecf48` ("decouple the frontend from memory completion with an F/X queue") is the
first bad commit.** Found by hardware bisect: `4567b019` GOOD, `9c298425` GOOD (its
parent), `49eecf48` BAD — 0 console lines past `bootconsole [sbi0] disabled`, which is the
exact reported symptom.

It changed fetch's handshake and did not change the other end of it:

    ooo2_frontend.v:  .ready(accept)  ->  .ready(~q_full)
    ooo2_core.v:      inject_inflight <= irq_inject & accept;      (UNCHANGED)

The interrupt pseudo-op is consumed by fetch on the queue PUSH (`fire`), but the interlock
that guarantees one injection per interrupt stayed armed by the queue POP (`accept`).
Those were the same edge before the queue. After it, whenever the backend stalled — accept
low, queue not full — fetch re-emitted the SAME interrupt every cycle, because the only
thing that would have stopped it was waiting on an event that had stopped coinciding.

That is why the machine died the instant Linux enabled its first PLIC source, and why a DTB
with the uart's `interrupts`/`interrupt-parent` removed walks straight past it.

Fix (`cb24e5ae`): `ooo2_frontend` exports `irq_taken = irq_inject & fire`, and both the hold
and the interlock key off it. `rtl-rules.md` already has the rule — an interlock must NAME
the event it interlocks, not a proxy — and this is the second commit to pay for it, so it
is now asserted: fetch consuming two pseudo-ops for one interrupt is a `$fatal`.

## Why no gate caught it, and why no gate could

**Nothing in this project has ever delivered an external interrupt in simulation.** `seip=0`
and `uart_ier=0` in every cosim run ever recorded — verified over 499M cycles at HEAD and at
`e529c411`, with the DTB correctly wiring the uart to the PLIC and the kernel reporting
`ttyS0 ... (irq = 1)`. The reason is `serial8250_console_write` is POLLED by design: printk
never uses interrupts, and the tiny128 initrd never reaches a userspace tty writer.

And cosim locksteps RETIRED INSTRUCTION RESULTS. A duplicated trap is not a wrong value —
the instruction stream stays coherent — so this survived "BIT-IDENTICAL in cosim" on five
separate commits. **Simulation cannot reproduce this bug**: both controls boot cleanly past
the handover (14 vs 13 console lines). Bisecting it needed real hardware.

If you fix one thing in the gates, make it this.

## Where I was wrong (both cost hours)

1. **I spent the night on the device/MMIO path.** The symptom looked like device I/O, and
   five commits in range touched it. I read all five closely and concluded each was
   correctly guarded — `df96e431`'s `dev_addr` mux has an explicit `dmem_ren & dmem_wen`
   assertion and the PTWs use separate ports; `01e1f265`'s claim timing lands on the right
   cycle because `mem_raddr` is a held register. That reading was CORRECT and I kept
   doubting it. The bisect exonerated the whole group in one build.
2. **I flagged the actual defect early and talked myself out of it.** I noticed the
   `accept`-vs-`fire` mismatch, called it "benign because the redirect flushes the queue",
   and moved on. Three builds of bisect found what my reading of the code kept getting
   wrong.
3. **The baud detour.** I inferred 115200 from the Makefile's `connect` target, "fixed"
   `TX_DRAIN` from 222 to 5787, and built a whole theory on it. The UART is hardwired to
   3 Mbps; 222 was right. Reverted. See the memory note.

## Frequency: the structural work does not convert

Four cycle-neutral commits took the 6 ns build from **WNS -2.436 to -0.717**
(118.54 -> 148.9 MHz "as built"), all bit-identical in cosim:

| commit | what was on the critical path that could not be |
|---|---|
| `09f79e16` | `res_v`'s `m_done`/`~m_trap`, implied by its own (branch\|jump) term |
| `fc4a2195` | `irq_inject` as a wire — the last combinational M->fetch-PC link |
| `de5a71ee` | `pc_q+2` recomputed every cycle though `strad` implies pc_q frozen |
| `bdc6c234` | `csr_rdata` in the bypass though CSR ops are always serializing |

Plus `102b2fd7` (register `hpm_ev`), which retired 823 of 3113 failing endpoints.

**It does not convert into shippable clock.** Measured, all with identical directives:

| ask | achieved | |
|---|---|---|
| 6.000 | 6.717 | miss |
| 7.000 | 7.682 | miss |
| 8.000 | 9.080 | miss |
| 8.500 | 9.698 | miss |
| 8.750 | 10.999 | miss — WORSE than both neighbours |
| 9.000 | 8.866 | **met, +0.134** |

Achieved delay tracks the ask, and 8.750 being worse than 8.500 and 9.000 shows this is
placer NOISE, not a wall. Two causes, both real: the design is **67% routing-bound** on its
worst path (nets at fanout 129/142/160 cost ~1.4 ns between them), and these commits ADD
FLOPS, so the RTL is faster when pushed hard and heavier when relaxed — exactly what
`HANDOFF-2026-08-19-timing.md` predicted.

**166 MHz is a placement and routing problem now, not a logic-depth one.**

## Open, in the order I would do them

1. **Gate the interrupt path.** It is untested by construction. Cheapest real coverage is a
   directed test that raises a PLIC source, claims and completes it — the claim register is
   side-effecting and nothing has ever exercised it.
2. **Fanout/congestion on the fetch cone.** 67% routing, three nets over fanout 129. This is
   where the remaining nanosecond is.
3. **VA-tag the fetch buffer.** `fb_al` derives from `imem_addr` (the PA), so the whole iMMU
   sits in front of the buffer hit, the I$ mux, the aligner and the predictor read. Tagging
   with the VA lifts the iMMU out at zero IPC cost — needs an invalidation argument
   (satp, sfence.vma, privilege) written down FIRST.
4. **The CDC `set_max_delay -datapath_only 6.000`** does not scale with the clock and was the
   limiter at 66.67 MHz (+0.159). It crosses a 4-phase full handshake, so it is
   over-constrained by design intent — real slack held in reserve.
5. **The SD card is gone** (`SPI timeout during idle clocks`). `ubuntu-boot.sh` already
   XMODEMs the payload; `hybrid-boot.sh` is stale and still tries `SL2800`.

## Verification recipe

    src/lint.sh                                    # lint: clean
    ooo2/run-ooo2-vl.sh rv64ui-p rv64um-p rv64ua-p rv64uc-p    # pass=85 fail=0
    VDEFS=-DINO_HW=4 BUILD=1 CYC=60000000 ooo2/run-ooo2-cosim-linux.sh   # 11631167
    src/run-vl-tests.sh                            # only if src/ changed; failures: 0

    # and the only one that would have caught this bug:
    cd platforms/rk-xcku5p-f-v1.2 && make program BIT=<bit>
    cd workloads/ubuntu && DTB=ubuntu-nfs.dtb ./ubuntu-boot.sh   # must reach /sbin/init

Hardware bisect harness (built this session, reusable):
`/var/tmp/bisect/hw_step.sh <commit>` and `/var/tmp/bisect/verify_worktree.sh <label> <DIV8> <dtb>`.
The monitor banner prints the RTL commit baked into the bitstream — use it.
DTB timebase MUST match the build clock (`tools/check-dts-timebase.py`); `SCALE_DIV` is a
TRUNCATING divide, which is the trap that made it wrong for months.
