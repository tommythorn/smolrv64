platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src
MEM_BASE ?= 0x70000000
BUILD_STAMP := $(shell date +%Y%m%d%H%M%S)

monitor.bin: monitor.elf
	$(RV)objcopy -O binary monitor.elf monitor.bin

SRCS = crt0.s monitor.c disk.c video.c blake3.c
# The core's whole ISA (the device trees' riscv,isa), soft-float ABI: the monitor has no FP state.
CFLAGS = -mabi=lp64 -mcmodel=medany \
	-march=rv64imafdc_zicsr_zifencei_zba_zbb_zbs_zicond_zicbom_zicbop_zicboz \
	-fno-builtin -fno-pic -fno-pie -fno-asynchronous-unwind-tables \
	-nostdlib -static -no-pie -Wall -O \
	-Wl,-Tmonitor.ld -Wl,--defsym,MEM_BASE=$(MEM_BASE)

monitor.elf: $(SRCS) mon.h font.h blake3.h monitor.ld
	$(RV)gcc $(CFLAGS) -DMONITOR_BUILD_STAMP=0x$(BUILD_STAMP)ULL $(SRCS) -o $@

# The simulation's monitor (workloads/sdboot): no countdown, it boots from the disk image's
# /smoltest at once, its DMA scratch and framebuffer are inside the simulation's 512 MiB DDR, and a request reads
# at most 32 sectors, so a cluster run takes several.
monitor-sim.elf: $(SRCS) mon.h font.h blake3.h monitor.ld
	$(RV)gcc $(CFLAGS) -DMONITOR_BUILD_STAMP=0 -DAUTOBOOT_S=0 -DDMA_BASE=0x9E000000ul -DFB_BASE=0x9F000000ul \
		-DBOOT_DIR='"/smoltest"' -DRD_MAX=32 $(SRCS) -o $@

monitor-sim.bin: monitor-sim.elf
	$(RV)objcopy -O binary $< $@

# The splash and console as the screen shows them, rendered on the host.
splash.ppm: preview.c video.c font.h
	cc -O2 -o preview preview.c && ./preview && rm -f preview

preview: splash.ppm
.PHONY: preview
