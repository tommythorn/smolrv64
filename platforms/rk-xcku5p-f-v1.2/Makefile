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

.PHONY: all synth impl bit program load clean timing

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
timing:
	$(VIVADO) $(VFLAGS) -source report_timing.tcl

load:
	$(MAKE) -C $(WORKLOAD) load MEM_BASE=0x70000000
	$(VIVADO) $(VFLAGS) -source build.tcl -tclargs bit force
	$(MAKE) program

$(BIT): bit

# Wipe all generated outputs (keeps sources and .xpr)
clean:
	rm -rf $(PROJECT).cache $(PROJECT).hw $(RUNS_DIR) \
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
