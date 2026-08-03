SRC_DIR  = ../../src
SIMMERV = $(HOME)/simmerv
FW=fw_payload.bin

ifeq ($(shell uname -s),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

# Byte offsets are relative to 0x80000000.  The DTB sits just above the
# advertised RAM window, with the initrd packed immediately below it.
mem.even mem.odd: gb5.dtb $(FW) img.Geekbench5.cpio
	$(SRC_DIR)/mkhex.py mem 0:$(FW) 16270e00:img.Geekbench5.cpio 1ff00000:gb5.dtb



.PHONY: sim





# Probe (sharded-OoO) core cosim vs simmerv -- the analogue of `make cosim`, driving
# probe/tb_cosim_linux (flat fw/dtb/initrd load) with gb5's memory layout (2 GiB; DTB
# @0x9ff00000, initrd @0x96270e00). First run builds; BUILD=1 or CYC=N to override.
# MEM_LG2=31 (2 GiB) MUST match the gb5.dtb memory node (reg <.. 0x7ff00000>, ~2 GiB)
# and the `ref` target's simmerv -m 2048; a smaller cosim wraps OOB accesses where
# simmerv access-faults -> spurious divergence once the workload exceeds the size.
PROBE ?= ../../src
CYC   ?= 0          # 0 = unbounded: run until a cosim divergence (or Ctrl-C)
pcosim: gb5.dtb $(FW) img.Geekbench5.cpio
	NAME=gb5 MEM_LG2=31 A1=9ff00000 OFF_DTB=1ff00000 OFF_INITRD=16270e00 \
	FW=$(CURDIR)/$(FW) DTB=$(CURDIR)/gb5.dtb INITRD=$(CURDIR)/img.Geekbench5.cpio \
	CYC=$(CYC) BUILD=$(BUILD) $(PROBE)/run-cosim-linux.sh

.PHONY: cosim pcosim

%: %.xz
	xz -d < $^ > $@

fw_payload.elf-6.15.0: fw_payload.elf-6.15.0.xz

gb5.dtb: gb5.dts
	dtc -I dts -O dtb < $^ > $@

fw_payload.elf-6.15.0.bin: fw_payload.elf-6.15.0
	$(RV)objcopy -O binary $^ $@

ref: gb5.dtb $(FW) img.Geekbench5.cpio
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -m 2048 -d gb5.dtb,0x9ff00000 $(FW),0x80000000 img.Geekbench5.cpio,0x96270e00
