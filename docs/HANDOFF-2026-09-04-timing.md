# Handoff — 2026-09-04: timing at 166.67 MHz, second pass

Supersedes §4 of `HANDOFF-2026-09-03-timing.md`. Written while the confirmation builds were
still running; the numbers marked *pending* are filled in below as they land.

---

## 1. The 2026-09-03 thesis was wrong, and the data that refuted it was already on disk

The previous handoff said: every core module clears 166 MHz by 2x out of context, the cache
does not, so the failing core paths are "where the slack runs out, not where it is spent".
Three things were wrong with that.

1. **The OOC number was for a cache nothing instantiates.** `ooc.tcl` passed no `-generic`,
   so `rv_cache` was measured at its DEFAULTS: PAW=34, 128 KB, no prefetch -- a 2048-set array
   with 18-bit tags. The shipped D$ (PAW=64, 64 KB, 49-bit tags) measured 181 MHz. Fixed:
   `make ooc ... GENERICS="P=V"`, and the script says loudly when it measures defaults.
2. **Per-module OOC cannot see a path that crosses modules.** The worst core paths crossed
   FIVE (`ooo2_sq -> ooo2_lq -> ooo2_lsu/mmu -> ooo2_core -> ooo2_iq -> psmem`).
3. **The routed checkpoint BEFORE post-route phys_opt was at -0.169 with 709 failing endpoints,
   all core.** phys_opt equalised the worst paths at -0.03 and only then did a cache family
   sit level with them. Three families at the same slack is the signature of phys_opt, not of
   a shared budget.

The instrument that settled it is now `make census`: every endpoint under +0.35 ns grouped
into families, plus histograms by startpoint and endpoint module. On the morning's checkpoint:

    startpoint            endpoints < +0.35   mechanism
    u_sq/v_reg                   1708         ONE chain, §2
    u_dcache/cur_line             815         `hit` on enables and a dependent array read
    ps_out_reg                    463         issued tags -> PRF -> ALU -> store-queue snoop
    m_addr_reg                    145         dTLB compare -> M's done -> issue select
    fe/u_fetch                    129         the fetch loop
    slack histogram: 84 < 0, 827 < +0.10, 1831 < +0.20, 2819 < +0.30, 4004 < +0.40

WNS is one path. A change is judged by whether it EMPTIES a family, on both directives.

## 2. The mechanisms, and what landed

| commit | change | family it removes | cosim |
|---|---|---|---|
| `f1c500bb` | OOC takes generics; `make physopt` starts from the physopted checkpoint | (instruments) | -- |
| `1271c96d` | `rv_cache`: the compare decides, it does not enable. cbo.zero masks the install data; CBO dirty test in S_FIN; MSHR/window/response captures on state; shift-then-select; request registers captured every idle cycle | `u_dcache/cur_line` (815). D$ alone 181 -> 206 MHz, I$ 206 MHz | bit-identical |
| `60481808` | `unit_busy` after the select; in-order issue index is the head pointer; psmem read per class, pick selects the result; store-queue snoop from a registered writeback copy with a landing bypass | `ps_out -> u_sq/data` (392), `-> psmem` address | bit-identical |
| `ede8159e` | the translate-only pass reaches the MMU regardless of the FSM and the port grant; data faults complete M from the latched copy; writeback valids from `lsu_done_acc`; redirect from the non-memory done | `u_sq/v_reg` (1708) | bit-identical (count unchanged) |
| `f2000a2d` | `make census` | (instrument) | -- |
| `88984b53` | tag = 34 significant PA bits (49 -> 19), always-on check at the door | tag array -60%, compare 19 bits | bit-identical |
| `05a1fcba` | store queue clears a pending landing on flush (review finding) | -- | bit-identical |
| `b37e8bdf` | the port grant leaves `fault` (FSM-starting requests ask the MMU unconditionally) and the CSR unit's trap input takes the non-memory done | `u_lq/sqt_reg` (1632 in build A) | bit-identical |
| `7f9bcc97` | fetch: the F/X queue stores the prediction's CHOICE and target, decode rebuilds pred_npc from its own length; the arrival bypass is VA-tested like the hit | `fe/u_fetch -> q_dat` (342), the fetch loop | bit-identical |
| `e0983292` | the Sstc timer compare is registered, like the CLINT's mtip | `u_csr/stimecmp` (559 in build B) | bit-identical |
| `4ebec992` | M's fault tests (`xpage`, `amo_mis`) use M's own width, not the port-selected one | `u_sq/head_reg` (5667 in build C) | bit-identical |

**A defect found by timing.** `b37e8bdf` let the translate-only pass be judged in the same
cycle the port may be granted, and `xpage`/`amo_mis` took their width from `nb`, which
follows the grant: for that cycle M's page-crossing test used the queued access's size. A
doubleword load at the last byte of a page, granted alongside a byte-wide queued access,
would have been queued instead of trapping. No gate reached it (two accesses, one cycle,
different sizes, a page end); the census did, because the wrong width was also the door
5667 endpoints walked through. `4ebec992`.

**The one chain behind 1708 + 1632 endpoints.** `ooo2_lsu` granted its FSM to the
pre-translated port with priority and implemented it by gating M's request to the MMU on
`~pt_start` -- for the translate-only pass too, which needs the MMU and nothing else. So the
store queue's live bits (through the alias matrix and the load queue's candidate) sat in
series with M's completion for every plain load and store, and M's completion is the wakeup
broadcast (`we_ld`, fanout 194), the redirect (the fetch adder, the F/X queue) and the hpm
events. After `ede8159e` the same door was still open through the FSM-starting arm of
`fault` and the CSR unit's trap input; `b37e8bdf` closed those. Rule I9.

## 3. Builds

| build | tree | directive | probe_clk routed | after phys_opt | failing | census: worst families |
|---|---|---|---|---|---|---|
| morning | `fb82d188` | AltSpread | -0.169 / 709 | -0.035 | 82 | u_sq/v_reg 1708, u_dcache 815, ps_out 463 |
| A | `ede8159e` | AltSpread | -0.090 / 318 | -0.060 | 213 | u_lq/sqt_reg 1632 (closed by `b37e8bdf`), fe 673, ps_out 247; 213 < 0, 754 < +0.10, 2218 < +0.30 |
| B | `b37e8bdf` | AltSpread | **+0.014 / 0** | +0.014 | **0** | u_csr/stimecmp 559 (the Sstc compare, registered next), fe 481, ps_out 176; 0 < 0, 129 < +0.10, 537 < +0.20, 1194 < +0.30. **Bitstream written**, banked as `/var/tmp/ooo2_166MHz_b37e8bdf_altspread.bit` |
| C | `7f9bcc97` (fetch pair) | AltSpread | **-0.879 / 3199** | -0.670 | 2661 | u_sq/head_reg 5667: the port grant reaching `xo_ok` through `sel_pt -> nb -> xpage`, a door B had placed at +0.038 with 49 endpoints. Closed by `4ebec992`, which was also a functional defect (below) |
| D | `4ebec992` (+ Sstc register, width fix) | AltSpread | -0.575 (est.) | **-0.406** | many | u_fpu/u_fpnew internal 132 (an fpnew retiming outcome, not in B or C), m_addr_reg_replica -> fe_red_tgt_q 62 / ps_out 45 (the CSR write-valid chain, cut by `3caf36e8`) |
| E | `4ebec992` | Explore | -0.073 | **0.000 / 0** | **0** | u_fpu/u_fpnew 124 at 0.000, m_addr_reg_replica -> u_fpu/req_ops_q 234, ps_out; 0 < 0, 215 < +0.10, 1198 < +0.30 (B: 129 / 537). Bitstream banked: `/var/tmp/ooo2_166MHz_4ebec992_explore.bit` |
| I | `4a769238` (H + the store-seqno wrap bit) | AltSpread | **+0.082 / 0** | +0.082 | **0** | bitstream banked: `/var/tmp/ooo2_166MHz_4a769238_altspread.bit`; board gate in `scratchpad/board-I` -- see 4 (1) |
| H | `1d80d59c` (F + the load queue's uncached bit) | AltSpread | **+0.059 / 0** | +0.059 | **0** | bitstream banked: `/var/tmp/ooo2_166MHz_1d80d59c_altspread.bit`; board gate in `scratchpad/board-H` -- see 4 (1) |
| F | `c7e9051a` (main HEAD: + `3caf36e8`, `c7e9051a`) | AltSpread | **+0.084 / 0** | +0.084 | **0** | m_csr_func_reg_replica -> ps_out 19 at +0.152 (the CSR function decode on the operand select); **0 < +0.10, 14 < +0.20, 56 < +0.30** (B: 129 / -- / 537). 63,809 LUTs, 46,406 regs (the 09-03 tree: 66,754 / 46,223 -- nothing pruned). Bitstream banked: `/var/tmp/ooo2_166MHz_c7e9051a_altspread.bit`; board gate in `scratchpad/board-F` |

Build A is one placement sample (rule I2) and its worst family was the chain `b37e8bdf`
closes. Build B, with it closed, is the first build in this tree's history to close at the
ROUTED stage, before post-route phys_opt (the morning's tree was -0.169 there): closure no
longer depends on phys_opt's yield. The population under +0.30 went 2819 -> 2218 -> 1194.

**F is the verdict on the post-B commits, and it is the best build in the tree's history:**
+0.084 routed on the shipping directive, 0 endpoints under +0.10 and 56 under +0.30 against
B's 129 and 537. D (-0.406) and F differ in RTL by `3caf36e8` alone -- the CSR unit's write
valid taken from the non-memory done, which removed `m_addr -> upd_valid` from under every
family D listed first (`m_addr_reg_replica -> fe_red_tgt_q`, `-> ps_out`) -- and D's other
head, the fpnew-internal retiming, is absent from F. So D was a real chain plus a lottery,
E (Explore, 0.000) was the lottery alone, and the chain is gone. The 0.4 ns between D and
F on near-identical RTL is a reminder that one AltSpread sample judges nothing under 400 ps;
F's margin over B is larger than that, and its census is 10x thinner.

## 4. What is left, in order

1. **F and B BOTH FAIL THE BOARD** -- and not on timing. Both boot to
   `virtio_net virtio1: ...:id 0 is not a head!` at ~3 s guest time, then TX timeouts forever
   (`scratchpad/board-F`, `board-B`; the old gate script waited for `login:` and did not
   flag it -- it does now). Root cause found by reading, not bisecting: `ooo2_lq` never
   carried the Svpbmt uncached bit and `ooo2_core` fed the LSU `pt_unc(pt_store ? sq_c_unc :
   1'b0)`, so a queued NC load was cached and the virtio used ring -- device-written memory
   behind the cache -- read stale. Six days old (`cb028682`), invisible to every simulation
   (the cosim's guest maps nothing NC), surfaced because the day's LSU changes shifted which
   loads get queued. Fix: the bit travels with the entry (rule B7). Build H = main + fix
   (`1d80d59c`, +0.059 routed, 0 failing): virtio-net WORKS, NFS root mounts, Ubuntu
   reaches `login:` -- with ONE userspace fault, twice in two boots, at the same place:
   `ds-identify[59/60]: unhandled signal 11 ... in libc.so.6[88ea2]`, epc/ra ending
   `ea2`/`e64`: the first `ld a1,0(a0)` of glibc's `strchrnul`, called from `strchr` with an
   already-corrupted string pointer (bad address `0xffffff893783fea0` on boot 1, `0x0` on
   boot 2). A data corruption upstream -- a register or a memory word -- at a deterministic
   code point with varying garbage. No earlier boot in the cumulative console ever had a
   userspace fault. Instruments running when this was written: (a) the Geekbench image
   (a full glibc userspace) under the ooo2 cosim on `1d80d59c` with a fresh model,
   lockstep clean past 220 M cycles and still in the kernel; (b) bisect build "B+unc"
   (`b37e8bdf` + only the LQ fix, worktree `/home/tommy/smolrv64-bisect`, banked as
   `/var/tmp/ooo2_166MHz_5ce62d89_Bunc_altspread.bit`) to split the day's commits at B on
   the board. **ROOT CAUSE FOUND by the Geekbench cosim**, not by the bisect: at retire
   123,081,278 (cycle ~447 M) the DUT's `ld a3,8(s7)` returned the word from BEFORE the
   `sd a2,8(s7)` eight instructions earlier -- a load that overtook an older store to its
   own address. `ooo2_sq` handed a dispatching load its tail INDEX as the store-seqno;
   with the queue full (tail == head) the age arithmetic read zero older stores where
   there were eight, the load took the early start, and read stale memory. Eight
   back-to-back stores then a load is ordinary compiled code, so dash read back the
   pointers it corrupted. Six days old (`cb028682`), never caught: tiny128's 60 M cycles
   never line it up (bit-identical with and without the fix). Fix: the seqno is the tail
   COUNTER with a wrap bit (`SQ_TB`), the distance is exact, and the queue asserts no load
   claims more older stores than are live (rule B8). `tb_ooo2_lqsq` case 9 is the
   full-queue and the wrapped-pointer shape: 6 of its checks fail on the old arithmetic,
   66/66 pass on the fix. The "B+unc" bisect build failed timing by 0.5 ns (placement
   lottery; B's own RTL closed at +0.014) and was not used. **Build I = main + this fix
   (`4a769238`, +0.082 routed, 0 failing) PASSES THE BOARD: NFS boot to `login:`, zero
   faults (`scratchpad/board-I`). It is the bitstream to ship:
   `/var/tmp/ooo2_166MHz_4a769238_altspread.bit`.**
2. **The fetch loop** (`fe/u_fetch -> u_fetch`, `-> u_bp`): `7f9bcc97` took the adder off the
   queue's data and the iTLB off `imem_avail`; the adder stays in the PC update and in the
   RAS push data (`ras[..] <= ft_npc`). Next: the RAS push from a registered copy with a
   pop-time bypass, the same shape as the store queue's landing.
3. **`m_csr_func_reg_replica -> ps_out`** is F's only family under +0.20 (19 paths at
   +0.152, 20 levels): the CSR function decode selecting the operand. Not needed for
   166.67 now; it is where the next 150 ps is. E's `m_addr_reg_replica` fanout family
   (-> `u_fpu/req_ops_q` 234) does not appear in F at all.
4. **The ALU read-execute-write loop** (`ps_out -> u_prf/mem_ie`, 12-20 levels, +0.005 in E):
   the design's floor. Route-bound (4.1-4.5 ns of 6); if it does not improve as the families
   around it empty, the shifter's structure is the next thing to look at.
5. `ooo2_pending` is still a shadow; the wakeup broadcast is still `we_*` compares in every
   entry. Not needed for 166.67 by the numbers above; it is the scalable form (rule I2).

## 4b. IPC: multiple outstanding loads (branch `ipc/multi-loads`, on top of `c7e9051a`)

Now at `3351a3d3`: G's RTL plus both fixes above cherry-picked (the load queue's
uncached bit, the store-seqno wrap bit), lint clean, `tb_ooo2_lqsq` 66/66, 240/240,
tiny128 cosim (see the gate log). Build J (this branch, AltSpread) and its board gate
were launched as this file was written. Before the fixes: build G (`d8108049`, AltSpread) routes at -0.031 with 14 failing
and closes at **+0.026 / 0 failing** after post-route phys_opt; census 534 endpoints under
+0.35 (F: 78), worst family `m_csr_func_reg -> ps_out` at +0.026 and fpnew-internal at
+0.045. Thinner than F, as the new terms on the port grant predicted; it closes. Bitstream
banked: `/var/tmp/ooo2_166MHz_d8108049_ipc_altspread.bit`. It carries the uncached-bit
defect above (fixed on main after it was built) and needs the fix cherry-picked before a
board gate. A queued cacheable load is a tagged, non-blocking D$ read: the LSU's FSM stays idle, the
load-queue index is the read's tag, the response is claimed by tag and landed on the entry
it names. Spec section 8 and P0 have the design and the measurements:

    ldbench            latency 5.00 -> 5.00, throughput 4.00 -> 2.25 cyc/load, overlap 1.24x -> 2.22x
    tiny128 60 M cyc   DDR_LAT=4   16,025,548 -> 16,725,431  (+4.4%; first reported as +14.1%
                                   against a stale main model, section 4c)
                       DDR_LAT=80  10,189,800 -> 10,253,856  (+0.63%)

The slow-memory point barely moves: one MSHR serialises misses (a second miss holds the
lookup pipeline and the port), and a 16-entry ROB fills behind a missing head in ~20 of 80
cycles. **The next lever on the board's operating point is a second MSHR**, not the cache's
door or the `S_CHECK` self-loop, which are hit-throughput work and pay only at `DDR_LAT`=4.

Three defects found by the cosim on the way, each now an always-on check (commit message
`da28895b` has them): ring-slot reuse under an in-flight load, `c_rd_want` set by a device
read, and the read tag flipping under its own accept wait. Timing risk: the LSU's
`port_free` and `o_v[tag]` are new terms in `pt_start`/`xl_early`, on the pre-translated
port's grant -- rule I9 territory; build it on AltSpread before judging.

## 4c. The day's "bit-identical" cosim verdicts were STALE BINARIES

Every latency-4 cosim on main between `f1c500bb` and `c7e9051a` reported 14,657,366
retires, and none of those logs contains `building obj_dir_ooo2_clinux`: the script reused
the existing model whenever `BUILD` was unset and its MEM_LG2/VDEFS stamp matched, so the
lockstep that passed was the OLD core's. Rebuilt from scratch at `c7e9051a` the count is
**16,025,548** (+9.3%), lockstep clean; the latency-80 and jittered runs were built fresh
(their VDEFS forced the wipe) and are valid, so HEAD is verified -- but per-commit
bit-identity was never actually checked, and which commits moved the count is unknown
(the translate-only pass and the scheduler changes are the candidates). The script's
stamp now hashes the sources. The IPC branch's numbers were built fresh (its testbench
ports changed), so 16,725,431 is real, and its gain over main is **+4.4%**, not +14.1%.

## 5. Gates run on every commit above

    src/lint.sh                     lint: clean
    ooo2/run-ooo2-cache-tb.sh       PASS both shapes, LAT=4/20/100/200 (cache commits)
    ooo2/run-ooo2-iq-tb.sh          IQ-TB PASS (scheduler commit)
    ooo2/run-ooo2-vl.sh             pass=240 fail=0
    ooo2/run-ooo2-cosim-linux.sh    CYC=60000000: 14,657,366 retires "every commit" -- STALE, see 4c
    make ooc MODULE=rv_cache GENERICS=<D$>   +0.478 -> +1.134 ns; <I$> +1.144 ns
