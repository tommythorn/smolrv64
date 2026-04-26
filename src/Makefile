P=mem
OPTS=
SIM_MEM_BASE="64'h70000000"
SIM_RESET_PC="64'h80000000"
SIM_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=27 -DAXI_MEM_SIZE_LG2=27 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC)
TESTER_VDEFS=-DSIMULATE -DMEM_SIZE_LG2=22 -DAXI_MEM_SIZE_LG2=22 -DMEM_BASEADDR=$(SIM_MEM_BASE) -DRESET_PC=$(SIM_RESET_PC)
IVOPTS=$(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb $(SIM_VDEFS)
platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

# Icarus VPI support
CFLAGS = -fPIC -shared -O2 -Wall
VPI_CFLAGS = $(shell iverilog-vpi --cflags)
VPI_LDFLAGS = $(shell iverilog-vpi --ldflags)

SRCS=smolrv64.v

testall:
	@(cd ../tests;./run-riscv-tests.sh passes)

run: $(P).even $(P).odd smolrv64-run
	./smolrv64-run +even=$(P).even +odd=$(P).odd

verbose: $(P).even $(P).odd smolrv64-verbose
	./smolrv64-verbose +even=$(P).even +odd=$(P).odd

smolrv64-verbose: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(IVOPTS) -o $@ -DDISASS -DVERBOSE $(SRCS)

smolrv64-run: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(IVOPTS) -o $@ -DNO_TIMEOUT -DVPI $(SRCS)

smolrv64-sim: $(SRCS) Makefile
	iverilog $(IVOPTS) -o $@ $(SRCS)

smolrv64-linux-sim: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb $(SIM_VDEFS) -DVPI -DNO_TIMEOUT -o $@ $(SRCS)

smolrv64-linux-verbose: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb $(SIM_VDEFS) -DDISASS -DVERBOSE -DNO_TIMEOUT -o $@ $(SRCS)

# Verilator build of the same config — much faster simulation.
# Clock is driven by sim_main.cpp (under `ifdef VERILATOR), so no --timing
# is needed.
smolrv64-linux-verbose-vl: $(SRCS) sim_main.cpp Makefile
	verilator --cc --exe --build -j 0 \
		--top-module smolrv64_tb \
		$(SIM_VDEFS) -DDISASS -DVERBOSE -DNO_TIMEOUT \
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
SIMMERV_DIR     ?= $(HOME)/github/simmerv
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

smolrv64-trace: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(IVOPTS) -o $@ -DTRACE -DRISCV_TESTS $(SRCS)

smolrv64-tester: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb $(TESTER_VDEFS) -o $@ -DRISCV_TESTS $(SRCS)

lint:
	verilator  -Wall --cc --top-module smolrv64 $(SRCS)

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

tty_vpi.vpi: tty_vpi.c
	iverilog-vpi $^
