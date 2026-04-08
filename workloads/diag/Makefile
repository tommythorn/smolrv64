platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src

diag.bin: diag.elf
	$(RV)objcopy -O binary diag.elf diag.bin

diag.elf: diag.c crt0.s
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=rv64imac_zicsr -fno-builtin -nostdlib \
		-Wl,-Tdiag.ld -O \
		crt0.s diag.c -o diag.elf

diag.even: diag.bin
	$(SRC_DIR)/evenodd.py $< 0 > $@

diag.odd: diag.bin
	$(SRC_DIR)/evenodd.py $< 1 > $@

$(SRC_DIR)/smolrv64-run: $(SRC_DIR)/smolrv64.v $(SRC_DIR)/uart5.v
	$(MAKE) -C $(SRC_DIR) smolrv64-run

run: diag.even diag.odd $(SRC_DIR)/smolrv64-run
	cd $(SRC_DIR) && ./smolrv64-run \
		+even=$(CURDIR)/diag.even \
		+odd=$(CURDIR)/diag.odd

load: diag.bin
	$(SRC_DIR)/evenodd.py diag.bin 0 > $(SRC_DIR)/mem.even
	$(SRC_DIR)/evenodd.py diag.bin 1 > $(SRC_DIR)/mem.odd

.PHONY: run load
