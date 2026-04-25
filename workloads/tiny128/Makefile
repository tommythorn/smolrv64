SRC_DIR  = ../../src
SIMMERV = $(HOME)/github/simmerv

# XXX The offsets should be computed automatically from the starting address
# It's printf "%x" $((addr-base)/16), eg. (0x87f00000-0x80000000)/16 = 0x7f0000
# (0x8752c000-0x80000000)/16 = 0x752c00
mem.even mem.odd: tiny128.dtb fw_payload.elf-6.15.0.bin tiny128.cpio
	$(SRC_DIR)/mkhex.py mem 0:fw_payload.elf-6.15.0.bin 752c000:tiny128.cpio 7f00000:tiny128.dtb

$(SRC_DIR)/smolrv64-linux-verbose: $(SRC_DIR)/smolrv64.v
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-verbose

sim: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-verbose
	cd $(SRC_DIR) && ./smolrv64-linux-verbose \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: sim

$(SRC_DIR)/smolrv64-linux-vl:
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-vl

run: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-vl
	@cd $(SRC_DIR) && ./smolrv64-linux-vl \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

$(SRC_DIR)/smolrv64-linux-cosim:
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-cosim

cosim: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-cosim
	@cd $(SRC_DIR) && ./smolrv64-linux-cosim \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: cosim

$(SIMMERV)/target/release/simmerv_cli:
	cd $(SIMMERV); cargo b -r

golden-output.txt: $(SIMMERV)/target/release/simmerv_cli 
	timeout 10 $(SIMMERV)/target/release/simmerv_cli -n -m 128 -d tiny128.dtb,0x87f00000 fw_payload.elf-6.15.0 tiny128.cpio,0x8752c000 > $@

are-we-there-yet:
	@(timeout 40 $(MAKE) -s run) | tee /tmp/sim.run.txt
	@if diff /tmp/sim.run.txt golden-output.txt; then echo YES; else echo NO; fi

%: %.xz
	xz -d < $^ > $@

fw_payload.elf-6.15.0: fw_payload.elf-6.15.0.xz

tiny128.cpio: tiny128.cpio.xz

tiny128.dtb: tiny128.dts
	dtc -I dts -O dtb < $^ > $@

fw_payload.elf-6.15.0.bin: fw_payload.elf-6.15.0
	riscv64-linux-gnu-objcopy -O binary $^ $@
