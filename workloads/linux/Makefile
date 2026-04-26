SRC_DIR  = ../../src
SIMMERV = $(HOME)/github/simmerv

mem.even mem.odd: dts.dtb fw_payload.bin
	$(SRC_DIR)/mkhex.py mem 0:fw_payload.bin 1000000:dts.dtb

$(SRC_DIR)/smolrv64-linux-verbose:
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
.PHONY: $(SRC_DIR)/smolrv64-linux-verbose $(SRC_DIR)/smolrv64-linux-vl $(SRC_DIR)/smolrv64-linux-cosim

$(SIMMERV)/target/release/simmerv_cli:
	cd $(SIMMERV); cargo b -r

golden-output.txt: $(SIMMERV)/target/release/simmerv_cli 
	timeout 1 $(SIMMERV)/target/release/simmerv_cli -n -d dts.dtb,0x81000000 fw_payload.bin,0x80000000 > $@

are-we-there-yet:
	@(timeout 40 $(MAKE) -s run) | tee /tmp/sim.run.txt
	@if diff /tmp/sim.run.txt golden-output.txt; then echo YES; else echo NO; fi
