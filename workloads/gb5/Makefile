SIMMERV = $(HOME)/simmerv
FW=fw_payload.bin

ifeq ($(shell uname -s),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

.PHONY: sim cosim

# The Geekbench image under lockstep with simmerv (the long-guest gate); CYC=N bounds it.
cosim: gb5.dtb $(FW) img.Geekbench5.cpio
	CYC=$(CYC) ../../ooo2/run-ooo2-cosim-gb5.sh

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
