# Top-level convenience targets. The real flows are the scripts they call.
#
#   make lint      src/lint.sh             -- the static gate, `lint: clean`
#   make test      ooo2/run-ooo2-vl.sh     -- riscv-tests on the verilated core, `pass=240 fail=0`
#   make tb        ooo2/run-ooo2-*-tb.sh and src/run-tb.sh -- the unit benches
#   make cosim     ooo2/run-ooo2-cosim-linux.sh -- the Linux boot in lockstep with simmerv
#   make bit       the FPGA bitstream (platforms/rk-xcku5p-f-v1.2)
#   make gate      tools/gate.sh: build, program, boot Ubuntu, judge
.PHONY: all lint test tb cosim bit gate
all: lint test
lint:  ; src/lint.sh
test:  ; ooo2/run-ooo2-vl.sh
tb:    ; for t in ooo2/run-ooo2-*-tb.sh; do $$t || exit 1; done; src/run-tb.sh
cosim: ; CYC=300000000 ooo2/run-ooo2-cosim-linux.sh
bit:   ; $(MAKE) -C platforms/rk-xcku5p-f-v1.2
gate:  ; tools/gate.sh
