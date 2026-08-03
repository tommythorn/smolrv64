SRC_DIR  = ../../src
SIMMERV = $(HOME)/simmerv
FW=fw_payload.bin

ifeq ($(shell uname -o),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

# Byte offsets are relative to 0x80000000.  The DTB sits just above the
# advertised RAM window, with the initrd packed immediately below it.
mem.even mem.odd: tiny128.dtb $(FW) tiny128.cpio
	$(SRC_DIR)/mkhex.py mem 0:$(FW) 1f52c000:tiny128.cpio 1ff00000:tiny128.dtb



.PHONY: sim





.PHONY: cosim

%: %.xz
	xz -d < $^ > $@

tiny128.cpio: tiny128.cpio.xz

tiny128.dtb: tiny128.dts
	dtc -I dts -O dtb < $^ > $@

ref: tiny128.dtb $(FW) tiny128.cpio
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -m 512 -d tiny128-simmerv.dtb,0x9ff00000 $(FW),0x80000000 tiny128.cpio,0x9f52c000
