VIVADO   := /home/tommy/Xilinx/2025.1/Vivado/bin/vivado
VFLAGS   := -mode batch -nojournal -nolog
PROJECT  := rk_xcku5p
XPR      := $(PROJECT).xpr
RUNS_DIR := $(PROJECT).runs
BIT      := $(RUNS_DIR)/impl_1/$(PROJECT).bit
TTY      := /dev/ttyUSB1
WORKLOAD ?= ../../workloads/monitor
export LD_LIBRARY_PATH=/home/tommy/Xilinx/2025.1/Vivado/lib/lnx64.o/Ubuntu/24

# Optional: PC_TRACE=1 enables the synth-time debug PC tracer in smolrv64.v.
# When set, the bitstream includes the tracer: SW sees a dummy UART after
# OpenSBI's mret into Linux, and the real TX emits one line per retire.
ifneq ($(strip $(PC_TRACE)),)
export PC_TRACE
endif

# The sharded-OoO probe core is the DEFAULT build (soc_top + DDR line/AXI bridge,
# via the `PROBE_CORE ifdef in rk_xcku5p.v).  Set PROBE_CORE=0 to build the legacy
# scalar smolrv64 instead.  Exported here (incl. =0) so build.tcl sees it.
ifneq ($(strip $(PROBE_CORE)),)
export PROBE_CORE
endif
# Isolation experiment: NO_VIRTIO_WIRE=1 deactivates the virtio wrapper wiring.
ifneq ($(strip $(NO_VIRTIO_WIRE)),)
export NO_VIRTIO_WIRE
endif
# Diagnostic: PROBE_DIAG=1 decouples probe_clk from ui_rst + drives LED cal/clock heartbeats.
ifneq ($(strip $(PROBE_DIAG)),)
export PROBE_DIAG
endif

# OOO2_CORE=1 builds the IN-ORDER core (ooo2/rv_soc_top) instead of the OoO
# soc_top.  e.g. `make bit OOO2_CORE=1`.
ifneq ($(strip $(OOO2_CORE)),)
export OOO2_CORE
endif

# In-order fetch window halfwords: OOO2_HW=4 gives the chunk-aligned fetch buffer 8-byte
# chunks (RTL default 2). Only 2 or 4; 8 trips the sdpram geometry guard.
ifneq ($(strip $(OOO2_HW)),)
export OOO2_HW
endif

# Width: PROBE_IW=1 (or 4, ...) overrides the RTL default (2). e.g. `make bit PROBE_IW=1`.
ifneq ($(strip $(PROBE_IW)),)
export PROBE_IW
endif

# Fmax sweep: probe_clk = 1000MHz*8/PROBE_CLK_DIV8, from an MMCM, so the frequency is
# CONTINUOUS (the old BUFGCE_DIV could only reach 66.7/83.3/111.1/166.7/333.3, which made
# any gain smaller than a whole rung worth nothing). RTL default 120 = 66.67 MHz;
# 72 = 111.11 MHz is the GB5 milestone build. build.tcl hard-errors on the retired
# PROBE_CLK_DIV rather than let it silently mean a different clock.
ifneq ($(strip $(PROBE_CLK_DIV8)),)
export PROBE_CLK_DIV8
endif
ifneq ($(strip $(PROBE_CLK_DIV)),)
export PROBE_CLK_DIV
endif

# Debug: ILA_VIRTIO=1 inserts an ILA on the MMIO clock-bridge ui-side (virtio-mmio access +
# device read data), on ui_clk. Capture the driver's DeviceFeaturesSel/DeviceFeatures sequence
# to see whether the Sel write reaches virtio before the Features read (probe VERSION_1 -22).
ifneq ($(strip $(ILA_VIRTIO)),)
export ILA_VIRTIO
endif

# Debug: ILA_IRQ=1 inserts an ILA on the probe-clk interrupt path (plic src-11 lifecycle from
# soc_top's irq_dbg bus) -- to see whether the virtio_blk completion IRQ is raised/serviced or lost.
ifneq ($(strip $(ILA_IRQ)),)
export ILA_IRQ
endif

# Debug: NO_SSTC=1 hides Sstc (stimecmp traps) so OpenSBI uses the CLINT timer path.
ifneq ($(strip $(NO_SSTC)),)
export NO_SSTC
endif
# Debug: ILA_TIMER=1 inserts an ILA on the probe-clk csr_file timer/interrupt path (the
# ubuntu post-generator freeze: STIP level, stimecmp/mscratch write strobes, trap/xret).
ifneq ($(strip $(ILA_TIMER)),)
export ILA_TIMER
endif
# Debug: ILA_PARITY=1 builds with -DCACHE_PARITY (parity bit per cache-bank word,
# checked on every read) and inserts an ILA triggered by the error pulse -- catching a
# bad cache-array read AT THE MOMENT it happens, instead of the kernel Oops millions of
# cycles later. If it never fires while the board still corrupts, the cache data path is
# exonerated on real BRAM.
ifneq ($(strip $(ILA_PARITY)),)
export ILA_PARITY
endif
# Debug: ILA_DEV=1 inserts an ILA on the ui-clk virtio_blk backend (FSM/SD/DMA state + AXI DMA
# handshakes + SD SPI pins) -- to see which block request / SPI-DMA step wedges during the root mount.
ifneq ($(strip $(ILA_DEV)),)
export ILA_DEV
endif

# Debug: ILA_CORE=1 inserts an ILA on the probe-clk core/DDR-line steady state (commit pulse +
# line req/we/ack + stuck line addr + irq bus) -- for a -trigger_now capture of a FROZEN wedge
# (post-root-mount, e.g. systemd "Hostname set"): DDR-path deadlock vs irq livelock vs SW spin.
ifneq ($(strip $(ILA_CORE)),)
export ILA_CORE
endif

.PHONY: all synth impl bit program load clean timing ila-capture ila-strand fmax-report

all: bit

synth: $(XPR)
	$(VIVADO) $(VFLAGS) -source build.tcl -tclargs synth

impl: $(XPR)
	$(VIVADO) $(VFLAGS) -source build.tcl -tclargs impl

bit: $(XPR)
	$(VIVADO) $(VFLAGS) -source build.tcl -tclargs bit

program:
	@test -f $(BIT) || { echo "ERROR: $(BIT) not found — run 'make bit' first"; exit 1; }
	$(VIVADO) $(VFLAGS) -source program.tcl -tclargs $(BIT)

# Open the impl_1 checkpoint and dump worst setup paths.  Use this to diagnose
# timing violations without rebuilding.
# Reset the runs after a KILLED build. A run terminated mid-flight leaves its directory
# half-written and the next build inherits it, failing ~30 minutes in with an error that
# points at the wrong thing. There was no target for this and it has bitten more than once.
.PHONY: reset
reset:
	$(VIVADO) $(VFLAGS) -source reset_runs.tcl

timing:
	$(VIVADO) $(VFLAGS) -source report_timing.tcl

# Last-mile timing closure: targeted post-route phys_opt on the routed checkpoint
# (no re-place/route). Writes the .bit iff it reaches WNS >= 0. See physopt_close.tcl.
physopt:
	$(VIVADO) $(VFLAGS) -source physopt_close.tcl

cdc:
	$(VIVADO) $(VFLAGS) -source report_cdc.tcl

# Headless ILA capture of the CURRENT (frozen) hardware state over JTAG. Requires a bit built
# with an ILA_* flag already programmed and the board wedged. Writes a CSV next to the script.
# CSV=<path> overrides the output. No GUI. See ila_capture.tcl.
ila-capture:
	$(VIVADO) $(VFLAGS) -source ila_capture.tcl $(if $(CSV),-tclargs $(CSV),)

# ARMED capture: waits for the mtvec-stranding event (mtvec parked at OpenSBI's probe
# handler) and dumps the window that caused it, with the system-op probes. Run it while
# the board is booting; it blocks until the trigger fires. See ila_strand.tcl.
# Per-clock achievable Fmax from the routed checkpoint. Global WNS is often pinned by the
# virtio/MIG ui_clk domain, so this is the number that tells us what the CORE can clock.
fmax-report:
	$(VIVADO) $(VFLAGS) -source fmax_report.tcl

# ARMED capture on a cache data-array parity failure (needs a bit built ILA_PARITY=1).
# Blocks until an array returns a word that is not what was stored -- the moment of
# corruption. Run it while the board boots. See ila_parity.tcl.
ila-parity:
	$(VIVADO) $(VFLAGS) -source ila_parity.tcl -tclargs $(if $(CSV),$(CSV),ila_parity.csv) $(if $(HOURS),$(HOURS),0)

ila-strand:
	$(VIVADO) $(VFLAGS) -source ila_strand.tcl $(if $(CSV),-tclargs $(CSV),)

load:
	$(MAKE) -C $(WORKLOAD) load MEM_BASE=0x70000000
	$(VIVADO) $(VFLAGS) -source build.tcl -tclargs bit force
	$(MAKE) program

$(BIT): bit

# Wipe all generated outputs (keeps sources and .xpr).  This must leave exactly what
# `git clean -fdx` leaves -- the .gitignore beside it lists the same paths -- so that
# the two ways of getting to a clean tree agree and `make` can rebuild from either.
clean:
	rm -rf $(PROJECT).cache $(PROJECT).gen $(PROJECT).hw $(PROJECT).sim $(RUNS_DIR) \
	       ddr4_0_ex.gen ddr4_0_ex.srcs \
	       $(PROJECT).srcs/utils_1 $(PROJECT).srcs/sources_1/ip/ila_* \
	       .Xil *.jou *.log

connect:
	@screen $(TTY) 3000000

help:
	@echo "Targets:"
	@echo "  make          — full build (synth + impl + bitstream)"
	@echo "  make synth    — synthesis only"
	@echo "  make impl     — synthesis + implementation"
	@echo "  make bit      — synthesis + implementation + bitstream"
	@echo "  make program  — program device over JTAG"
	@echo "  make load     — build workload, rebuild bitstream, program (WORKLOAD=path)"
	@echo "  make clean    — remove all generated files"
	@echo "  make connect  — connect to the serial port"
