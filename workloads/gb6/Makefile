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

# Probe (sharded-OoO) core cosim vs simmerv -- the analogue of `make cosim`, driving
# probe/tb_cosim_linux (flat fw/dtb/initrd load) with gb6's memory layout (2 GiB; DTB
# @0xfff00000, initrd @0xe5f8ca00). First run builds; BUILD=1 or CYC=N to override.
PROBE ?= ../../probe
CYC   ?= 200000000
pcosim: gb6.dtb $(FW) $(IMG)
	NAME=gb6 MEM_LG2=31 A1=fff00000 OFF_DTB=7ff00000 OFF_INITRD=65f8ca00 \
	FW=$(CURDIR)/$(FW) DTB=$(CURDIR)/gb6.dtb INITRD=$(CURDIR)/$(IMG) \
	CYC=$(CYC) BUILD=$(BUILD) $(PROBE)/run-cosim-linux.sh

.PHONY: cosim pcosim
.PHONY: $(SRC_DIR)/smolrv64-tiny128-verbose $(SRC_DIR)/smolrv64-tiny128-vl $(SRC_DIR)/smolrv64-tiny128-cosim

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
