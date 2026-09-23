# memrand (B4, 2026-09-17): a generated random memory-ordering program under the lockstep cosim.
#   make cosim SEED=1 [OPS=50000] [CYC=6000000]        one seed, the verdict = the lockstep + MEMRAND-DONE
#   (50 k ops is ~4 M cycles: the stream is memory-bound by design, ~16 cycles per instruction)
#   make sweep SEEDS="1 2 3 4"                        several, serially (one obj_dir)
#   make sweep-iw3 ...                                the same at IW=3
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif
SEED ?= 1
OPS  ?= 50000
CYC  ?= 6000000
GENFLAGS ?=          # e.g. --misaligned-megapage (with TAG=-mm): both models must fault at the same retire
# The trap the variant expects (a load or store page fault, cause 13 or 15): with it set, that
# MEMRAND-TRAP line is the PASS instead of a failure.
EXPECT_TRAP ?= $(if $(findstring misaligned-megapage,$(GENFLAGS)),MEMRAND-TRAP cause=00000000000000(0d|0f),)
TAG  ?=
ARCH = rv64gc_zicsr_zifencei_zicbom_zicboz
CFLAGS = -mabi=lp64d -mcmodel=medany -march=$(ARCH) -fno-builtin -nostdlib -O2 -Wall
P = memrand-$(SEED)$(TAG)
all: $(P).bin
ops-$(SEED)$(TAG).S: gen.py
	./gen.py --seed $(SEED) --ops $(OPS) $(GENFLAGS) > $@
$(P).elf: crt0.S memrand.c ops-$(SEED)$(TAG).S memrand.ld
	$(RV)gcc $(CFLAGS) -Wl,-Tmemrand.ld crt0.S memrand.c ops-$(SEED)$(TAG).S -o $@
%.bin: %.elf
	$(RV)objcopy -O binary $< $@
cosim: $(P).bin
	@out=$$(mktemp); FW=$(CURDIR)/$(P).bin INITRD= CYC=$(CYC) PLUSARGS="+dma_rand=$(SEED) $(PLUSARGS)" \
	  ../../ooo2/run-ooo2-cosim-linux.sh 2>&1 | tee $$out | grep -a -E 'building|MEMRAND|STORE DATA|Diverging|TIMEOUT after|cosim-perf|dma rd'; \
	if { [ -n "$(EXPECT_TRAP)" ] && sed 's/\[c=[^]]*\]//g' $$out | tr -d '\n' | grep -q -E '$(EXPECT_TRAP)' || \
	     { sed 's/\[c=[^]]*\]//g' $$out | tr -d '\n' | grep -q 'MEMRAND-DONE' && \
	       ! sed 's/\[c=[^]]*\]//g' $$out | tr -d '\n' | grep -q 'MEMRAND-TRAP'; }; } && \
	   ! sed 's/\[c=[^]]*\]//g' $$out | tr -d '\n' | grep -q -E 'Diverging retire|STORE DATA MISMATCH'; \
	then echo "MEMRAND seed=$(SEED): PASS"; rm -f $$out; else echo "MEMRAND seed=$(SEED): FAIL (log: $$out)"; exit 1; fi
sweep:
	@for s in $(SEEDS); do $(MAKE) --no-print-directory cosim SEED=$$s || exit 1; done
sweep-iw3:
	@for s in $(SEEDS); do VDEFS=-DOOO2_IW=3 $(MAKE) --no-print-directory cosim SEED=$$s || exit 1; done
clean:
	rm -f ops-*.S memrand-*.elf memrand-*.bin
.PHONY: all cosim sweep sweep-iw3 clean
