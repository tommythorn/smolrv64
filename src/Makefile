P=mem
OPTS=
SIM_MEM_BASE="64'h70000000"
SIM_RESET_PC="64'h80000000"
# Stamp every build's mimpid with the leading bits of the HEAD commit (matches
# build.tcl's SMOLRV64_GIT_COMMIT for bitstreams). Double-quoted so the literal
# apostrophe survives make -> sh; falls back to 00000000 outside a git tree.
GIT_COMMIT := $(shell git -C .. rev-parse --short=8 HEAD 2>/dev/null || echo 00000000)
GIT_VDEF := "-DSMOLRV64_GIT_COMMIT=32'h$(GIT_COMMIT)"
SIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=27 -DAXI_MEM_SIZE_LG2=27 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) $(GIT_VDEF)
TESTER_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=22 -DAXI_MEM_SIZE_LG2=22 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) $(GIT_VDEF)
TINY128_SIM_MEM_BASE="64'h80000000"
# 2 GiB (LG2=31): the gb5/Geekbench5 workload needs ~2 GiB of guest RAM.
# Drives the SRAM model size (MEM_SIZE_LG2) and simmerv's MEM_BYTES
# (COSIM_MEM_SIZE_LG2). axi_mem stays 512 MiB (LG2=29) below — it is loaded
# but unused by tiny128/gb5, and the ~511 MiB image fits.
TINY128_SIM_MEM_SIZE_LG2=31
TINY128_SIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=$(TINY128_SIM_MEM_SIZE_LG2) -DAXI_MEM_SIZE_LG2=27 -DMEM_BASEADDR=$(TINY128_SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) $(GIT_VDEF)
# Cosim's sim_main mandates +even/+odd, which the RTL also $readmemh's into
# axi_mem; tiny128 runs from the SRAM model and doesn't use axi_mem, but the
# array must be large enough (512 MiB) to not abort the load on the 512 MiB image.
TINY128_COSIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=$(TINY128_SIM_MEM_SIZE_LG2) -DAXI_MEM_SIZE_LG2=29 -DMEM_BASEADDR=$(TINY128_SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) $(GIT_VDEF)

# For cache stress testing
# SIM_CACHE_INDEX_BITS?=1
# SIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=27 -DAXI_MEM_SIZE_LG2=27 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) -DCACHE_INDEX_BITS=$(SIM_CACHE_INDEX_BITS)
# TESTER_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=22 -DAXI_MEM_SIZE_LG2=22 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) -DCACHE_INDEX_BITS=$(SIM_CACHE_INDEX_BITS)

IVOPTS=$(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb $(SIM_VDEFS)
platform=$(shell uname -o)

# Shared Verilator speed flags: -O3 model optimization + fast X-handling,
# and host-native codegen for the generated C++ model.
VL_OPT=-O3 --x-assign fast --x-initial fast
VL_CFLAGS=-O3 -march=native -mtune=native

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRCS=smolrv64.v
CVFPU_SRCS=smolrv64_cvfpu.sv
CVFPU_DEPS=$(CVFPU_SRCS) cvfpu_sources.f
CVFPU_VL_FLAGS=-sv \
	-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
	-Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE
CVFPU_VL_SRCS=-f cvfpu_sources.f $(CVFPU_SRCS)
CVFPU_SMOKE_TB=cvfpu_smoke_tb.sv

testall:
	@(cd ../tests;./run-riscv-tests.sh passes)

run: $(P).even $(P).odd smolrv64-run
	./smolrv64-run +even=$(P).even +odd=$(P).odd

verbose: $(P).even $(P).odd smolrv64-verbose
	./smolrv64-verbose +even=$(P).even +odd=$(P).odd

smolrv64-verbose: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DVERBOSE -DNO_TIMEOUT \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_run_verbose -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_run_verbose/$@ $@

smolrv64-run: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DNO_TIMEOUT \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_run -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_run/$@ $@

smolrv64-sim: smolrv64-run
	cp smolrv64-run $@

smolrv64-linux-sim: smolrv64-linux-vl
	cp smolrv64-linux-vl $@

smolrv64-linux-verbose: smolrv64-linux-verbose-vl
	cp smolrv64-linux-verbose-vl $@

# Verilator build of the same config — much faster simulation.
# Clock is driven by sim_main.cpp (under `ifdef VERILATOR), so no --timing
# is needed.
smolrv64-linux-verbose-vl: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DVERBOSE -DNO_TIMEOUT \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_verbose -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_verbose/$@ $@

# Verilator build of the same config — much faster simulation.
# Clock is driven by sim_main.cpp (under `ifdef VERILATOR), so no --timing
# is needed.
smolrv64-linux-vl: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DNO_TIMEOUT \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_verbose -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_verbose/$@ $@

smolrv64-tiny128-verbose: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(TINY128_SIM_VDEFS) -DVERBOSE -DNO_TIMEOUT \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_tiny128_verbose -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_tiny128_verbose/$@ $@

smolrv64-tiny128-vl: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(TINY128_SIM_VDEFS) -DNO_TIMEOUT \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_tiny128 -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_tiny128/$@ $@

# Cosim build: smolrv64 under Verilator + simmerv (via C ABI) in lockstep.
# Every retired instruction is sent to simmerv via a DPI callback and the
# two architectural states are compared.  Fails loudly on divergence.
SIMMERV_DIR     ?= $(HOME)/simmerv
SIMMERV_LIB_DIR := $(SIMMERV_DIR)/target/release
SIMMERV_INCLUDE := $(SIMMERV_DIR)/cosim

$(SIMMERV_LIB_DIR)/libsimmerv_cosim.a: \
		$(wildcard $(SIMMERV_DIR)/src/*.rs) \
		$(wildcard $(SIMMERV_DIR)/src/**/*.rs) \
		$(wildcard $(SIMMERV_DIR)/cosim/src/*.rs) \
		$(SIMMERV_DIR)/Cargo.toml \
		$(SIMMERV_DIR)/cosim/Cargo.toml
	cd $(SIMMERV_DIR) && cargo build --release -p simmerv-cosim

smolrv64-linux-cosim: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile \
		$(SIMMERV_LIB_DIR)/libsimmerv_cosim.a
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DNO_TIMEOUT -DVERILATOR_COSIM \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS) -DVERILATOR_COSIM -I$(SIMMERV_INCLUDE)" \
		-LDFLAGS "-L$(SIMMERV_LIB_DIR) -l:libsimmerv_cosim.a -lpthread -ldl -lm" \
		-Mdir obj_dir_cosim -o smolrv64-linux-cosim \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_cosim/smolrv64-linux-cosim $@

smolrv64-tiny128-cosim: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile \
		$(SIMMERV_LIB_DIR)/libsimmerv_cosim.a
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(TINY128_COSIM_VDEFS) -DNO_TIMEOUT -DVERILATOR_COSIM \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS) -DVERILATOR_COSIM -DCOSIM_MEM_SIZE_LG2=$(TINY128_SIM_MEM_SIZE_LG2) -I$(SIMMERV_INCLUDE)" \
		-LDFLAGS "-L$(SIMMERV_LIB_DIR) -l:libsimmerv_cosim.a -lpthread -ldl -lm" \
		-Mdir obj_dir_tiny128_cosim -o smolrv64-tiny128-cosim \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_tiny128_cosim/smolrv64-tiny128-cosim $@

smolrv64-trace: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS -DTRACE \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_trace -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_trace/$@ $@

smolrv64-tester: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_tester -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_tester/$@ $@

smolrv64-cvfpu-tester: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS)" \
		-Mdir obj_dir_cvfpu_tester -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_cvfpu_tester/$@ $@

# Cosim build with CVFPU enabled, in riscv-tests mode. Same DPI lockstep
# as smolrv64-linux-cosim but with FP support so rv64uf/rv64ud tests can
# be driven through cosim for divergence analysis.
smolrv64-cvfpu-cosim: $(SRCS) $(CVFPU_DEPS) sim_main.cpp Makefile \
		$(SIMMERV_LIB_DIR)/libsimmerv_cosim.a
	verilator $(CVFPU_VL_FLAGS) --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS -DVERILATOR_COSIM \
		-Wno-fatal \
		$(VL_OPT) -CFLAGS "$(VL_CFLAGS) -DVERILATOR_COSIM -I$(SIMMERV_INCLUDE)" \
		-LDFLAGS "-L$(SIMMERV_LIB_DIR) -l:libsimmerv_cosim.a -lpthread -ldl -lm" \
		-Mdir obj_dir_cvfpu_cosim -o $@ \
		$(CVFPU_VL_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_cvfpu_cosim/$@ $@

lint:
	verilator $(CVFPU_VL_FLAGS) -Wall --lint-only --timing --top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS \
		-Wno-DECLFILENAME -Wno-BLKSEQ -Wno-PROCASSINIT \
		-Wno-UNUSEDSIGNAL \
		$(CVFPU_VL_SRCS) $(SRCS)

cvfpu-lint: $(CVFPU_DEPS) Makefile
	verilator -sv --lint-only -Wno-fatal \
		-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
		--top-module smolrv64_cvfpu \
		-f cvfpu_sources.f $(CVFPU_SRCS)

smolrv64-cvfpu-lint: $(SRCS) $(CVFPU_DEPS) Makefile
	verilator $(CVFPU_VL_FLAGS) --lint-only -Wno-fatal \
		--top-module smolrv64 \
		$(CVFPU_VL_SRCS) $(SRCS)

cvfpu-smoke: $(CVFPU_DEPS) $(CVFPU_SMOKE_TB) Makefile
	verilator -sv --binary --timing -Wno-fatal \
		-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
		--top-module cvfpu_smoke_tb \
		-Mdir obj_dir_cvfpu_smoke \
		-f cvfpu_sources.f $(CVFPU_SRCS) $(CVFPU_SMOKE_TB)
	./obj_dir_cvfpu_smoke/Vcvfpu_smoke_tb

%.even: %.bin
	./evenodd.py $^ 0 > $@

%.odd: %.bin
	./evenodd.py $^ 1 > $@

%.o: %.c
	$(RV)gcc -march=rv64gc -c $^ -o $@

%.o: %.s
	$(RV)as -march=rv64gc $^ -o $@

%.elf: %.o
	$(RV)ld -Ttext=0x80000000 $^ -o $@

%.bin: %.elf
	$(RV)objcopy -O binary $^ $@

%.dis: %
	$(RV)objdump -Mmax,no-aliases,numeric -d $^ > $@

tty_vpi.vpi:
	@echo "Icarus/VPI simulation is intentionally disabled; use the Verilator targets." >&2
	@exit 1
