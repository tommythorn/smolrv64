SRC_DIR  = ../../src
SIM_ARGS ?=
FW=fw_payload.bin
#FW=fw_payload-7.1.0-rc6.bin

all: linux.dtb

mem.even mem.odd: linux.dtb $(FW) tiny128.cpio
	$(SRC_DIR)/mkhex.py mem 0:$(FW) 2000000:linux.dtb 762b000:tiny128.cpio

linux.dtb: linux.dts
	dtc -I dts -O dtb < $^ > $@



.PHONY: sim





.PHONY: cosim

tiny128.cpio: ../tiny128/tiny128.cpio.xz
	xz -d < $< > $@

%: %.xz
	xz -d < $^ > $@
