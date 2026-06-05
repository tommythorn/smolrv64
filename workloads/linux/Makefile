SRC_DIR  = ../../src
SIM_ARGS ?=
FW=fw_payload.bin
#FW=fw_payload-7.1.0-rc6.bin

all: run

mem.even mem.odd: linux.dtb $(FW) tiny128.cpio
	$(SRC_DIR)/mkhex.py mem 0:$(FW) 2000000:linux.dtb 762b000:tiny128.cpio

linux.dtb: linux.dts
	dtc -I dts -O dtb < $^ > $@

$(SRC_DIR)/smolrv64-linux-verbose:
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-verbose

sim: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-verbose
	cd $(SRC_DIR) && ./smolrv64-linux-verbose \
		$(SIM_ARGS) \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: sim

$(SRC_DIR)/smolrv64-linux-vl:
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-vl

run: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-vl
	@cd $(SRC_DIR) && ./smolrv64-linux-vl \
		$(SIM_ARGS) \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

$(SRC_DIR)/smolrv64-linux-cosim:
	$(MAKE) -C $(SRC_DIR) smolrv64-linux-cosim

cosim: mem.even mem.odd $(SRC_DIR)/smolrv64-linux-cosim
	@cd $(SRC_DIR) && ./smolrv64-linux-cosim \
		$(SIM_ARGS) \
		+even=$(CURDIR)/mem.even \
		+odd=$(CURDIR)/mem.odd \
		+rf=$(CURDIR)/rf.hex

.PHONY: cosim
.PHONY: $(SRC_DIR)/smolrv64-linux-verbose $(SRC_DIR)/smolrv64-linux-vl $(SRC_DIR)/smolrv64-linux-cosim

tiny128.cpio: ../tiny128/tiny128.cpio.xz
	xz -d < $< > $@

%: %.xz
	xz -d < $^ > $@
