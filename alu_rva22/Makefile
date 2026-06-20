# RVA22 ALU: self-checking functional test (Verilator).
#   make test     # build + run the randomized/directed self-check
# Timing/area: use ../probe  (make -C ../probe probe TOP=alu_rva22_probe
#   SRCS="flopwrap.v alu_rva22_probe.v ../alu_rva22/alu.v" PERIOD=2.5)

VL := verilator
# -I../src so alu.v's `include "alu_ops.vh" resolves (alu.v lives in ../src).
VLFLAGS := --binary --timing -j 0 -I../src \
	-Wno-WIDTH -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED -Wno-CASEINCOMPLETE -Wno-fatal

.PHONY: test clean
test:
	$(VL) $(VLFLAGS) -o tb_alu tb_alu.v
	./obj_dir/tb_alu

clean:
	rm -rf obj_dir
