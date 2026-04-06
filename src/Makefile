P=mem
OPTS=
IVOPTS=$(OPTS) -Wall -m ./tty_vpi.vpi -s smolrv64_tb -DSIMULATE -DMEM_SIZE_LG2=20
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

SRCS=smolrv64.v uart5.v

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

smolrv64-trace: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(IVOPTS) -o $@ -DTRACE -DRISCV_TESTS $(SRCS)

smolrv64-tester: $(SRCS) tty_vpi.vpi Makefile
	iverilog $(IVOPTS) -o $@ -DRISCV_TESTS -DMEM_SIZE_LG2=22 $(SRCS)

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
