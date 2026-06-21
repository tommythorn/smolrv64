# Timing-probe flow: find a circuit's cycle-time limit (Fmax) on the XCKU5P.
#
#   make probe                       # probe the rf_alu example at 2.0 ns
#   make probe PERIOD=1.5            # tighter constraint (over-constrain to push)
#   make probe TOP=flopwrap SRCS="flopwrap.v my_dut.v"   # probe your own circuit
#
# Your circuit must be presented as a module `probe_dut(clk, din[IN_W-1:0],
# dout[OUT_W-1:0])` so it drops into flopwrap (see rf_alu.v for the pattern).
# Over-constrain PERIOD until WNS goes negative, then read the reported Fmax.

VIVADO := /home/tommy/Xilinx/2025.1/Vivado/bin/vivado
VFLAGS := -mode batch -nojournal -nolog
PART   := xcku5p-ffvb676-2-i
export LD_LIBRARY_PATH=/home/tommy/Xilinx/2025.1/Vivado/lib/lnx64.o/Ubuntu/24

TOP    ?= flopwrap
PERIOD ?= 2.0
SRCS   ?= flopwrap.v rf_alu.v
REGION ?=                       # pblock region override, e.g. CLOCKREGION_X0Y0:CLOCKREGION_X1Y1

.PHONY: probe clean
probe:
	PROBE_REGION="$(REGION)" $(VIVADO) $(VFLAGS) -source probe.tcl -tclargs $(TOP) $(PERIOD) $(PART) $(SRCS)

clean:
	rm -rf probe_timing.rpt clockInfo.txt .Xil cong iter_*.txt vivado*.jou vivado*.log *.pb *.rpx
