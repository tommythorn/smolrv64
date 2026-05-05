platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src

trivial.bin: trivial.elf
	$(RV)objcopy -O binary trivial.elf trivial.bin

trivial.elf: trivial.c crt0.s
	$(RV)gcc -mabi=lp64 -O -mcmodel=medany -march=rv64imac_zicsr -fno-builtin -nostdlib -Wl,-Ttrivial.ld -O \
		crt0.s trivial.c -o trivial.elf

trivial.even: trivial.bin
	$(SRC_DIR)/evenodd.py $< 0 > $@

trivial.odd: trivial.bin
	$(SRC_DIR)/evenodd.py $< 1 > $@

$(SRC_DIR)/smolrv64-sim: $(SRC_DIR)/smolrv64.v
	$(MAKE) -C $(SRC_DIR) smolrv64-sim

sim: trivial.even trivial.odd $(SRC_DIR)/smolrv64-sim
	cd $(SRC_DIR) && ./smolrv64-sim \
		+even=$(CURDIR)/trivial.even \
		+odd=$(CURDIR)/trivial.odd

load: trivial.bin
	$(SRC_DIR)/evenodd.py trivial.bin 0 > $(SRC_DIR)/mem.even
	$(SRC_DIR)/evenodd.py trivial.bin 1 > $(SRC_DIR)/mem.odd

.PHONY: sim load
