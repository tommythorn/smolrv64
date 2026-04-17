SRC_DIR  = ../../src

mem.even mem.odd: dts.dtb fw_payload.bin
	$(SRC_DIR)/mkhex.py mem 0:fw_payload.bin 1000000:dts.dtb

$(SRC_DIR)/smolrv64-linux-verbose: $(SRC_DIR)/smolrv64.v $(SRC_DIR)/uart5.v
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

golden-output.txt:
	(cd ~/github/simmerv; cargo b -rq;\
         timeout 1 target/release/simmerv_cli -n -d $(CURDIR)/dts.dtb,0x81000000 $(CURDIR)/fw_payload.bin,0x80000000) > $@

are-we-there-yet:
	@(timeout 10 $(MAKE) -s run) | tee /tmp/sim.run.txt
	@if cmp -s /tmp/sim.run.txt golden-output.txt; then echo YES; else echo NO; fi
