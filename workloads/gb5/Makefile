SRC_DIR  = ../../src
SIMMERV = $(HOME)/simmerv
FW=fw_payload-7.1.0-rc6.bin

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

gb5.dtb: gb5.dts
	dtc -I dts -O dtb < $^ > $@

fw_payload.elf-6.15.0.bin: fw_payload.elf-6.15.0
	$(RV)objcopy -O binary $^ $@

ref:
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -n -m 512 -d gb5.dtb,0x9ff00000 $(FW),0x80000000 img.Geekbench5.cpio,0x96270e00
