SRC_DIR  = ../../src
SIMMERV = $(HOME)/simmerv
FW=fw_payload-7.1.0-rc6.bin
IMG=new-img.gb6.cpio

ifeq ($(shell uname -s),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

# Byte offsets are relative to 0x80000000.  The DTB sits just above the
# advertised RAM window, with the initrd packed immediately below it.
mem.even mem.odd: gb6.dtb $(FW) $(IMG)
	$(SRC_DIR)/mkhex.py mem 0:$(FW) 65f8ca00:$(IMG) 7ff00000:gb6.dtb

$(SRC_DIR)/smolrv64-tiny128-verbose: $(SRC_DIR)/smolrv64.v
	$(MAKE) -C $(SRC_DIR) smolrv64-tiny128-verbose

sim: mem.even mem.odd $(SRC_DIR)/smolrv64-tiny128-verbose
	cd $(SRC_DIR) && ./smolrv64-tiny128-verbose \
		+sram_even=$(CURDIR)/mem.even \
		+sram_odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: sim

$(SRC_DIR)/smolrv64-tiny128-vl:
	$(MAKE) -C $(SRC_DIR) smolrv64-tiny128-vl

run: mem.even mem.odd $(SRC_DIR)/smolrv64-tiny128-vl
	@cd $(SRC_DIR) && ./smolrv64-tiny128-vl \
		+sram_even=$(CURDIR)/mem.even \
		+sram_odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

$(SRC_DIR)/smolrv64-tiny128-cosim:
	$(MAKE) -C $(SRC_DIR) smolrv64-tiny128-cosim

cosim: mem.even mem.odd $(SRC_DIR)/smolrv64-tiny128-cosim
	@cd $(SRC_DIR) && ./smolrv64-tiny128-cosim \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+sram_even=$(CURDIR)/mem.even \
		+sram_odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: cosim
.PHONY: $(SRC_DIR)/smolrv64-tiny128-verbose $(SRC_DIR)/smolrv64-tiny128-vl $(SRC_DIR)/smolrv64-tiny128-cosim

%: %.xz
	xz -d < $^ > $@

fw_payload.elf-6.15.0: fw_payload.elf-6.15.0.xz

gb6.dtb: gb6.dts
	dtc -I dts -O dtb < $^ > $@

fw_payload.elf-6.15.0.bin: fw_payload.elf-6.15.0
	$(RV)objcopy -O binary $^ $@

ref:
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -n -m 2048 -d gb6.dtb,0xfff00000 $(FW),0x80000000 $(IMG),0xe5f8ca00
