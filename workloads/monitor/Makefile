platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src
MEM_BASE ?= 0x80000000

monitor.bin: monitor.elf
	$(RV)objcopy -O binary monitor.elf monitor.bin

monitor.elf: monitor.c blake3.c blake3.h crt0.s
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=rv64imac_zicsr -fno-builtin -nostdlib \
		-Wl,-Tmonitor.ld -Wl,--defsym,MEM_BASE=$(MEM_BASE) -O \
		crt0.s monitor.c blake3.c -o monitor.elf

monitor.even: monitor.bin
	$(SRC_DIR)/evenodd.py $< 0 > $@

monitor.odd: monitor.bin
	$(SRC_DIR)/evenodd.py $< 1 > $@

$(SRC_DIR)/smolrv64-run: $(SRC_DIR)/smolrv64.v
	$(MAKE) -C $(SRC_DIR) smolrv64-run

sim: monitor.even monitor.odd $(SRC_DIR)/smolrv64-sim
	cd $(SRC_DIR) && ./smolrv64-sim \
		+even=$(CURDIR)/monitor.even \
		+odd=$(CURDIR)/monitor.odd

run: monitor.even monitor.odd $(SRC_DIR)/smolrv64-run
	cd $(SRC_DIR) && ./smolrv64-run \
		+even=$(CURDIR)/monitor.even \
		+odd=$(CURDIR)/monitor.odd

load: monitor.bin
	$(SRC_DIR)/evenodd.py monitor.bin 0 > $(SRC_DIR)/mem.even
	$(SRC_DIR)/evenodd.py monitor.bin 1 > $(SRC_DIR)/mem.odd

.PHONY: sim run load
