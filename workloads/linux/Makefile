SRC_DIR  = ../../src
LINUX    = ../../linux

mem.even mem.odd: $(LINUX)/dts.dtb $(LINUX)/fw_payload.bin
	$(SRC_DIR)/mkhex.py mem \
		0:$(LINUX)/fw_payload.bin \
		1000000:$(LINUX)/dts.dtb

$(SRC_DIR)/smolrv64-linux-verbose: $(SRC_DIR)/smolrv64.v $(SRC_DIR)/uart5.v
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-verbose

sim: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-verbose
	cd $(SRC_DIR) && ./smolrv64-linux-verbose \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: sim
