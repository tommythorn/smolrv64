# rvbench: the riscv-tests single-core benchmarks, built for EVERY extension the
# core implements, run bare-metal in the Linux testbench at the shipping configuration, in
# lockstep with Simmerv, ending at the program's own tohost exit (tb +tohost=).
#
#   make                          build them all (<name>.elf/.bin/.dump)
#   make run B=dhrystone          one, IW=3 lockstep; prints the benchmark's own mcycle/minstret
#   make runall                   all of them, serially (one Verilator job at a time)
#   make run B=<name>             also a program of your own, local/<name>.c: main(0, 0) is called
#                                 and its return value is the tohost exit
#   make run B=coremark           CoreMark (github.com/eembc/coremark, checked out at COREMARK),
#                                 the performance run's seeds, COREMARK_ITERS iterations; its
#                                 port is coremark/core_portme.[ch], ticks = mcycle
#   make ideal B=dhrystone        the ideal-frontend study: the run, its retired path recorded
#                                 (<name>.trace), then replayed by the oracle frontend
#                                 (-DSMOLRV64_ORACLE_FE, core/smolrv64_oracle_fe.v); prints both
#                                 counts (mcycle, or CoreMark's ticks) and the real one's share of
#                                 the ideal. IDEAL_CUT=1 ends the oracle's bundles at a taken branch.
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
VDEFS  ?=
CYC    ?= 400000000
B      ?= dhrystone
COREMARK ?= $(HOME)/projects/coremark
# CoreMark's own rule wants ten seconds of run; in lockstep that is out of reach, so the run is
# short and its CRCs (the standard 2K performance run's) are the check. Ticks are mcycle and
# EE_TICKS_PER_SEC is 10^6, so its Iterations/Sec reads as CoreMark/MHz (1e6 / cycles an iteration).
COREMARK_ITERS ?= 30
PLUSARGS ?=
LOG    ?= $(B).run.log

all: $(addsuffix .bin,$(BMARKS))

syscalls-uart.c: $(RVT)/common/syscalls.c uart-syscalls.py
	./uart-syscalls.py $< > $@

%.elf: syscalls-uart.c $(RVT)/common/crt.S
	$(RV)gcc $(CFLAGS) -I$(RVT)/$* -o $@ $(wildcard $(RVT)/$*/*.c) $(wildcard $(RVT)/$*/*.S) \
	   $(wildcard local/$*.c) syscalls-uart.c $(RVT)/common/crt.S $(LDFLAGS)
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

coremark.elf: syscalls-uart.c $(RVT)/common/crt.S coremark/core_portme.c coremark/core_portme.h
	$(RV)gcc $(CFLAGS) -Icoremark -I$(COREMARK) -DPERFORMANCE_RUN=1 -DITERATIONS=$(COREMARK_ITERS) \
	   '-DFLAGS_STR="$(OPT) -march=$(MARCH)"' -o $@ $(addprefix $(COREMARK)/,core_list_join.c \
	   core_main.c core_matrix.c core_state.c core_util.c) coremark/core_portme.c syscalls-uart.c \
	   $(RVT)/common/crt.S $(LDFLAGS)
	$(RV)objdump -d $@ > coremark.dump

%.bin: %.elf
	$(RV)objcopy -O binary $< $@

# The lockstep run: the verdict is the lockstep plus the benchmark's own "passed" (exit 0).
run: $(B).bin
	@th=$$($(RV)nm $(B).elf | awk '$$3=="tohost"{print $$1}'); out=$(CURDIR)/$(LOG); \
	 FW=$(CURDIR)/$(B).bin INITRD= CYC=$(CYC) VDEFS="$(VDEFS)" PLUSARGS="+tohost=$$th $(PLUSARGS)" \
	   ../../core/run-cosim-linux.sh > $$out 2>&1; \
	 grep -a -E 'building|Diverging|STORE DATA|fatal|TOHOST exit|TIMEOUT after|mcycle|minstret|Dhrystones|Microseconds|Total ticks|Iterations  |crc[a-z]+ +:|ERROR! [a-z]+ crc' $$out; \
	 if grep -aq 'TOHOST exit=0 ' $$out && ! grep -aqE 'Diverging retire|STORE DATA MISMATCH|ERROR! [a-z]+ crc' $$out; then echo "RVBENCH $(B): PASS"; \
	 else echo "RVBENCH $(B): FAIL (log: $$out)"; exit 1; fi

ideal: $(B).bin
	@$(MAKE) --no-print-directory run B=$(B) LOG=$(B).run.log PLUSARGS="+trace_out=$(CURDIR)/$(B).trace $(PLUSARGS)"
	@$(MAKE) --no-print-directory run B=$(B) LOG=$(B).ideal.run.log VDEFS="-DSMOLRV64_ORACLE_FE $(VDEFS)" \
	   PLUSARGS="+oracle=$(CURDIR)/$(B).trace $(if $(IDEAL_CUT),+oracle_cut) $(PLUSARGS)"
	@grep -a 'smolrv64_oracle_fe: [0-9]* redirects' $(B).ideal.run.log
	@r=$$(grep -a -E '^mcycle =|^Total ticks' $(B).run.log | grep -o '[0-9]*$$' | head -1); \
	 i=$$(grep -a -E '^mcycle =|^Total ticks' $(B).ideal.run.log | grep -o '[0-9]*$$' | head -1); \
	 awk -v r=$$r -v i=$$i -v b=$(B) 'BEGIN { printf "IDEAL %s: %d cycles, %d with an ideal frontend: the frontend runs at %.1f%% of ideal\n", b, r, i, 100 * i / r }'

runall:
	@for b in $(BMARKS); do $(MAKE) --no-print-directory run B=$$b || exit 1; done

clean:
	rm -rf *.elf *.bin *.dump *.run.log *.trace syscalls-uart.c dhry-src
.PHONY: all run ideal runall clean
.PRECIOUS: %.elf
