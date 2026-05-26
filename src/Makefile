P=mem
OPTS=
SIM_MEM_BASE="64'h70000000"
SIM_RESET_PC="64'h80000000"
SIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=27 -DAXI_MEM_SIZE_LG2=27 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC)
TESTER_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=22 -DAXI_MEM_SIZE_LG2=22 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC)

# For cache stress testing
# SIM_CACHE_INDEX_BITS?=1
# SIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=27 -DAXI_MEM_SIZE_LG2=27 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) -DCACHE_INDEX_BITS=$(SIM_CACHE_INDEX_BITS)
# TESTER_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=22 -DAXI_MEM_SIZE_LG2=22 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC) -DCACHE_INDEX_BITS=$(SIM_CACHE_INDEX_BITS)

IVOPTS=$(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb $(SIM_VDEFS)
platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRCS=smolrv64.v
CVFPU_SRCS=smolrv64_cvfpu.sv
CVFPU_SMOKE_TB=cvfpu_smoke_tb.sv

testall:
	@(cd ../tests;./run-riscv-tests.sh passes)

run: $(P).even $(P).odd smolrv64-run
	./smolrv64-run +even=$(P).even +odd=$(P).odd

verbose: $(P).even $(P).odd smolrv64-verbose
	./smolrv64-verbose +even=$(P).even +odd=$(P).odd

smolrv64-verbose: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DVERBOSE -DNO_TIMEOUT \
		-Wno-fatal \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_run_verbose -o $@ \
		$(SRCS) sim_main.cpp
	cp obj_dir_run_verbose/$@ $@

smolrv64-run: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DNO_TIMEOUT \
		-Wno-fatal \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_run -o $@ \
		$(SRCS) sim_main.cpp
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
smolrv64-linux-verbose-vl: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DVERBOSE -DNO_TIMEOUT \
		-Wno-fatal \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_verbose -o $@ \
		$(SRCS) sim_main.cpp
	cp obj_dir_verbose/$@ $@

# Verilator build of the same config — much faster simulation.
# Clock is driven by sim_main.cpp (under `ifdef VERILATOR), so no --timing
# is needed.
smolrv64-linux-vl: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DNO_TIMEOUT \
		-Wno-fatal \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_verbose -o $@ \
		$(SRCS) sim_main.cpp
	cp obj_dir_verbose/$@ $@

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

smolrv64-linux-cosim: $(SRCS) sim_main.cpp Makefile \
		$(SIMMERV_LIB_DIR)/libsimmerv_cosim.a
	verilator --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DNO_TIMEOUT -DVERILATOR_COSIM \
		-Wno-fatal \
		-O3 -CFLAGS "-O3 -DVERILATOR_COSIM -I$(SIMMERV_INCLUDE)" \
		-LDFLAGS "-L$(SIMMERV_LIB_DIR) -l:libsimmerv_cosim.a -lpthread -ldl -lm" \
		-Mdir obj_dir_cosim -o smolrv64-linux-cosim \
		$(SRCS) sim_main.cpp
	cp obj_dir_cosim/smolrv64-linux-cosim $@

smolrv64-trace: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS -DTRACE \
		-Wno-fatal \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_trace -o $@ \
		$(SRCS) sim_main.cpp
	cp obj_dir_trace/$@ $@

smolrv64-tester: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS \
		-Wno-fatal \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_tester -o $@ \
		$(SRCS) sim_main.cpp
	cp obj_dir_tester/$@ $@

smolrv64-cvfpu-tester: $(SRCS) $(CVFPU_SRCS) cvfpu_sources.f sim_main.cpp Makefile
	verilator -sv --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		-DUSE_CVFPU $(TESTER_VDEFS) -DRISCV_TESTS \
		-Wno-fatal \
		-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
		-Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
		-O3 -CFLAGS "-O3" \
		-Mdir obj_dir_cvfpu_tester -o $@ \
		-f cvfpu_sources.f $(CVFPU_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_cvfpu_tester/$@ $@

# Cosim build with CVFPU enabled, in riscv-tests mode. Same DPI lockstep
# as smolrv64-linux-cosim but with FP support so rv64uf/rv64ud tests can
# be driven through cosim for divergence analysis.
smolrv64-cvfpu-cosim: $(SRCS) $(CVFPU_SRCS) cvfpu_sources.f sim_main.cpp Makefile \
		$(SIMMERV_LIB_DIR)/libsimmerv_cosim.a
	verilator -sv --cc --exe --build -j 0 --timing \
		--top-module smolrv64_tb \
		-DUSE_CVFPU $(TESTER_VDEFS) -DRISCV_TESTS -DVERILATOR_COSIM \
		-Wno-fatal \
		-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
		-Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
		-O3 -CFLAGS "-O3 -DVERILATOR_COSIM -I$(SIMMERV_INCLUDE)" \
		-LDFLAGS "-L$(SIMMERV_LIB_DIR) -l:libsimmerv_cosim.a -lpthread -ldl -lm" \
		-Mdir obj_dir_cvfpu_cosim -o $@ \
		-f cvfpu_sources.f $(CVFPU_SRCS) $(SRCS) sim_main.cpp
	cp obj_dir_cvfpu_cosim/$@ $@

lint:
	verilator -Wall --lint-only --timing --top-module smolrv64_tb \
		$(TESTER_VDEFS) -DRISCV_TESTS \
		-Wno-DECLFILENAME -Wno-BLKSEQ -Wno-PROCASSINIT \
		-Wno-UNUSEDSIGNAL \
		$(SRCS)

cvfpu-lint: $(CVFPU_SRCS) cvfpu_sources.f Makefile
	verilator -sv --lint-only -Wno-fatal \
		-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
		--top-module smolrv64_cvfpu \
		-f cvfpu_sources.f $(CVFPU_SRCS)

smolrv64-cvfpu-lint: $(SRCS) $(CVFPU_SRCS) cvfpu_sources.f Makefile
	verilator -sv --lint-only -Wno-fatal -DUSE_CVFPU \
		-Wno-ASCRANGE -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED \
		-Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
		--top-module smolrv64 \
		-f cvfpu_sources.f $(CVFPU_SRCS) $(SRCS)

cvfpu-smoke: $(CVFPU_SRCS) $(CVFPU_SMOKE_TB) cvfpu_sources.f Makefile
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
