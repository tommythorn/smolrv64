# Self-checking RVA22 (Zba/Zbb/Zbs) + Zicond test for smolrv64.
#   make run   # build image + run on smolrv64-tester (expects "Test Passed")
RV ?= riscv64-linux-gnu-
TESTER ?= ../../src/smolrv64-tester

test.elf: test.S link.ld
	$(RV)gcc -nostdlib -nostartfiles -march=rv64gc_zba_zbb_zbs_zicond -mabi=lp64d \
		-T link.ld -o $@ test.S

test.bin: test.elf
	$(RV)objcopy -O binary $< $@

test.even test.odd: test.bin
	python3 ../../src/evenodd.py test.bin 0 > test.even
	python3 ../../src/evenodd.py test.bin 1 > test.odd

run: test.even test.odd
	@th=$$($(RV)nm test.elf | awk '/ tohost$$/{print $$1}'); \
	 $(MAKE) -C ../../src smolrv64-tester >/dev/null; \
	 $(TESTER) +even=test.even +odd=test.odd +tohost=$$th 2>&1 | grep -iE 'Test Passed|Test Failed'

clean:
	rm -f test.elf test.bin test.even test.odd min.*
