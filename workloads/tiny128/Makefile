SIMMERV = $(HOME)/simmerv
FW=fw_payload.bin

ifeq ($(shell uname -o),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

.PHONY: sim cosim

# The tiny128 Linux boot in lockstep with simmerv (300 M cycles is the required length).
cosim: tiny128.dtb $(FW) tiny128.cpio
	CYC=300000000 ../../ooo2/run-ooo2-cosim-linux.sh


%: %.xz
	xz -d < $^ > $@

tiny128.cpio: tiny128.cpio.xz

tiny128.dtb: tiny128.dts
	dtc -I dts -O dtb < $^ > $@

ref: tiny128.dtb $(FW) tiny128.cpio
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -m 512 -d tiny128-simmerv.dtb,0x9ff00000 $(FW),0x80000000 tiny128.cpio,0x9f52c000
