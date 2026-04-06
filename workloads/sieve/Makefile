platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src

sieve.bin: sieve.elf
	$(RV)objcopy -O binary sieve.elf sieve.bin

sieve.elf: sieve.c crt0.s
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=rv64imac_zicsr -fno-builtin -nostdlib -Wl,-Tsieve.ld -O \
		crt0.s sieve.c -o sieve.elf

sieve.even: sieve.bin
	$(SRC_DIR)/evenodd.py $< 0 > $@

sieve.odd: sieve.bin
	$(SRC_DIR)/evenodd.py $< 1 > $@

$(SRC_DIR)/smolrv64-sim: $(SRC_DIR)/smolrv64.v $(SRC_DIR)/uart5.v
	$(MAKE) -C $(SRC_DIR) smolrv64-sim

sim: sieve.even sieve.odd $(SRC_DIR)/smolrv64-sim
	cd $(SRC_DIR) && ./smolrv64-sim \
		+even=$(CURDIR)/sieve.even \
		+odd=$(CURDIR)/sieve.odd

.PHONY: sim
