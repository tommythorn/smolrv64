# Timing-probe harness (Fmax benchmarking on the XCKU5P)

How to measure a circuit's cycle-time limit (Fmax) in isolation, plus the results
that led to the current ALU. The tooling lives in `probe/`; this doc is the prose
+ results record (the numbers below otherwise live only in chat history).

## What it is

`probe/` runs an **out-of-context** synth + place & route of a circuit wrapped in
flip-flops, over-constrains the clock, and backs Fmax out of the worst setup
slack. It answers "how fast can *this datapath* clock on this part," independent
of the rest of the core.

Files (all tracked):
- `probe/flopwrap.v` — generic flop-in/flop-out harness. An LFSR drives
  `din_q -> probe_dut -> dout_q -> probe_out`, so every path that matters is a
  clean register-to-register path *through* your DUT, and synthesis can't fold the
  logic away (LFSR keeps inputs non-constant; single `probe_out` keeps the chain).
- `probe/probe.tcl` — the OOC flow: `synth_design -mode out_of_context`, one
  `create_clock`, pack into a single clock region (`pblock` =
  `CLOCKREGION_X0Y0`), then `opt/place/phys_opt/route`, then report.
- `probe/Makefile` — driver (self-documenting header).
- `probe/rf_alu.v` — worked example: a register file feeding a simple ALU and
  writing back (the `probe_dut` interface pattern to copy).
- `probe/alu_probe.v`, `probe/alu_rva22_probe.v` — DUT wrappers for the ALUs.

## Method

Constrain `clk` **tighter than achievable** so the tools keep optimizing, run a
real route, then:

```
achieved_period = constraint − WNS        (WNS < 0 ⇒ achieved > constraint)
Fmax (MHz)      = 1000 / achieved_period
```

Over-constrain `PERIOD` until WNS goes negative, then read the reported Fmax. The
single-clock-region `pblock` matters: without it the placer scatters the isolated
harness across the die and inflates routing far beyond what a compact datapath
sees. LUT count from the report is essentially the DUT's (the harness has ~no
combinational logic); FF count includes the harness flops.

## Usage

From `probe/`:

```sh
make probe                         # rf_alu example at 2.0 ns
make probe PERIOD=1.5              # tighter constraint to push Fmax
make probe TOP=alu_rva22_probe PERIOD=1.7 \
     SRCS="flopwrap.v alu_rva22_probe.v ../src/alu.v ../src/alu_ops.vh"
```

Your circuit must be presented as `probe_dut(clk, din[IN_W-1:0], dout[OUT_W-1:0])`
so it drops into `flopwrap` (see `rf_alu.v`). Read `PROBE RESULT` / `PROBE AREA`
from stdout; the worst path lands in `probe/probe_timing.rpt`.

> Note: `alu_rva22_probe.v`'s comment still points at `../alu_rva22/alu.v`; the
> ALU now lives at **`src/alu.v`** (`+ src/alu_ops.vh`). Pass that in `SRCS`.

## Results

Measured with this harness on `xcku5p-ffvb676-2-i`, single clock region. Figures
are the values observed during development — reproducible by re-running the probe,
not a guaranteed spec (placement varies run to run, especially near the limit).

| Design | Fmax | Area | Notes |
|---|---|---|---|
| yarvi RVA20 ALU (`~/github/yarvi`) | ~390 MHz | — | baseline reference brought into the harness |
| Optimized RVA20-class ALU (funnel shifter) | ~568 MHz | — | shared right-funnel + bit-reverse; no bitmanip |
| **`src/alu.v` — RVA22 (Zba/Zbb/Zbs) + Zicond** | **~409 MHz** | **~1676 LUTs** | the ALU now in the core; single-cycle |

Adding full RVA22 + Zicond (count/extend/permute, min/max, rotates, single-bit,
`czero`) to the fast RVA20 core is what moved ~568 → ~409 MHz; ~409 MHz
single-cycle was accepted as the target (a 2-cycle proposal was rejected). The
ALU's layered result mux keeps the adder and shifter on the shallowest paths.

## Functional verification

`src/alu.v` is checked standalone by `alu_rva22/tb_alu.v` (Verilator,
self-checking against base-ISA equivalents): **443,360 tests / 0 errors** at last
run. In-core, the same ops are checked by `tests/bitmanip/test.S` (Test Passed)
and the full riscv-tests gate (240/240). `cpopw` confirmed working on hardware.

## Relationship to the real core

The probe measures a datapath *in isolation*; the core has ~zero timing margin
(WNS typically ~0.01 ns, limited by `npc→rf_decode` and the FPU CDC, not the ALU).
So a standalone Fmax is a *ceiling*, not a promise — a fast block can still be
on, or shift, the core's critical path once placed in context. Use the probe to
compare design alternatives, then confirm in-core with `make timing`.
