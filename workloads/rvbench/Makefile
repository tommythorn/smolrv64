# rvbench: the riscv-tests single-core benchmarks, built for EVERY extension the
# core implements, run bare-metal in the Linux testbench at the shipping configuration, in
# lockstep with Simmerv, ending at the program's own tohost exit (tb +tohost=).
#
#   make                          build them all (<name>.elf/.bin/.dump)
#   make run B=dhrystone          one, IW=3 lockstep; prints the benchmark's own mcycle/minstret
#   make runall                   all of them, serially (one Verilator job at a time)
#
# Sources stay upstream (RVT); only syscall() is swapped for the UART (uart-syscalls.py).
# Left out: mt-* (multi-hart), pmp (no PMP), vec-* (no V).
RVT    ?= $(HOME)/projects/riscv-tests/benchmarks
RV     ?= riscv64-linux-gnu-
# encoding.h lives in riscv-tests' `env` submodule; when that is not checked out, the copy in
# riscv-arch-test (Simmerv's riscof tree) carries the same definitions.
ENVINC ?= $(dir $(firstword $(wildcard $(RVT)/../env/encoding.h \
            $(HOME)/simmerv/riscof/riscv-arch-test/riscv-test-suite/env/encoding.h)))
# The ISA the core implements (workloads/ubuntu/ubuntu-nfs.dts riscv,isa): rv64imafdc + Zicsr,
# Zifencei, Zba, Zbb, Zbs, Zicond, Zicbom, Zicbop, Zicboz. Tommy: build with all of them.
MARCH  ?= rv64gc_zicsr_zifencei_zba_zbb_zbs_zicond_zicbom_zicbop_zicboz
BMARKS ?= median qsort rsort towers vvadd memcpy multiply mm dhrystone spmv
OPT    ?= -O2
CFLAGS  = -U_FORTIFY_SOURCE -DPREALLOCATE=1 -mcmodel=medany -static -std=gnu99 $(OPT) -ffast-math \
          -fno-common -fno-builtin-printf -fno-tree-loop-distribute-patterns \
          -Wno-implicit-int -Wno-implicit-function-declaration -march=$(MARCH) -mabi=lp64d \
          -I$(ENVINC) -I$(RVT)/common
LDFLAGS = -static -nostdlib -nostartfiles -lm -lgcc -T $(RVT)/common/test.ld
VDEFS  ?= -DOOO2_IW=3
CYC    ?= 400000000
B      ?= dhrystone
PLUSARGS ?=
LOG    ?= $(B).run.log

all: $(addsuffix .bin,$(BMARKS))

syscalls-uart.c: $(RVT)/common/syscalls.c uart-syscalls.py
	./uart-syscalls.py $< > $@

%.elf: syscalls-uart.c $(RVT)/common/crt.S
	$(RV)gcc $(CFLAGS) -I$(RVT)/$* -o $@ $(wildcard $(RVT)/$*/*.c) $(wildcard $(RVT)/$*/*.S) \
	   syscalls-uart.c $(RVT)/common/crt.S $(LDFLAGS)
	$(RV)objdump -d $@ > $*.dump

# Dhrystone's run count is an unguarded #define in dhrystone.h, so the three sources are
# copied with it replaced (DHRY_RUNS; upstream's 500 runs is mostly cold caches).
DHRY_RUNS ?= 5000
dhry-src/dhrystone.h: $(RVT)/dhrystone/dhrystone.h Makefile
	mkdir -p dhry-src && cp $(RVT)/dhrystone/*.c dhry-src/
	sed 's/^#define NUMBER_OF_RUNS.*/#define NUMBER_OF_RUNS $(DHRY_RUNS)/' $< > $@
	grep -q '^#define NUMBER_OF_RUNS $(DHRY_RUNS)$$' $@

dhrystone.elf: syscalls-uart.c $(RVT)/common/crt.S dhry-src/dhrystone.h
	$(RV)gcc $(CFLAGS) -Idhry-src -o $@ dhry-src/*.c syscalls-uart.c $(RVT)/common/crt.S $(LDFLAGS)
	$(RV)objdump -d $@ > dhrystone.dump

%.bin: %.elf
	$(RV)objcopy -O binary $< $@

# The lockstep run: the verdict is the lockstep plus the benchmark's own "passed" (exit 0).
run: $(B).bin
	@th=$$($(RV)nm $(B).elf | awk '$$3=="tohost"{print $$1}'); out=$(CURDIR)/$(LOG); \
	 FW=$(CURDIR)/$(B).bin INITRD= CYC=$(CYC) VDEFS="$(VDEFS)" PLUSARGS="+tohost=$$th $(PLUSARGS)" \
	   ../../ooo2/run-ooo2-cosim-linux.sh > $$out 2>&1; \
	 grep -a -E 'building|Diverging|STORE DATA|fatal|TOHOST exit|TIMEOUT after|mcycle|minstret|Dhrystones|Microseconds' $$out; \
	 if grep -aq 'TOHOST exit=0 ' $$out && ! grep -aqE 'Diverging retire|STORE DATA MISMATCH' $$out; then echo "RVBENCH $(B): PASS"; \
	 else echo "RVBENCH $(B): FAIL (log: $$out)"; exit 1; fi

runall:
	@for b in $(BMARKS); do $(MAKE) --no-print-directory run B=$$b || exit 1; done

clean:
	rm -rf *.elf *.bin *.dump *.run.log syscalls-uart.c dhry-src
.PHONY: all run runall clean
.PRECIOUS: %.elf
