SIMMERV = $(HOME)/simmerv
FW=fw_payload-7.1.0-rc6.bin
IMG=new-img.gb6.cpio

ifeq ($(shell uname -s),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

.PHONY: sim

# No cosim runner for this image yet: ooo2/run-ooo2-cosim-gb5.sh is the shape to copy.

%: %.xz
	xz -d < $^ > $@

fw_payload.elf-6.15.0: fw_payload.elf-6.15.0.xz

gb6.dtb: gb6.dts
	dtc -I dts -O dtb < $^ > $@

fw_payload.elf-6.15.0.bin: fw_payload.elf-6.15.0
	$(RV)objcopy -O binary $^ $@

ref: gb6.dtb $(FW) $(IMG)
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -m 2048 -d gb6.dtb,0xfff00000 $(FW),0x80000000 $(IMG),0xe5f8ca00
