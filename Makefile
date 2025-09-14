P=hw
OPTS=
#RV=riscv64-elf-
RV=riscv64-linux-gnu-

testall:
	@./run-riscv-tests.sh passes fails

fails:
	@./run-riscv-tests.sh fails

run: $(P).even  $(P).odd smolrv64-run
	./smolrv64-run +even=$(P).even +odd=$(P).odd

verbose: $(P).even  $(P).odd smolrv64-verbose
	./smolrv64-verbose +even=$(P).even +odd=$(P).odd

smolrv64-verbose: smolrv64.v rs232tx.v Makefile
	iverilog -o $@ -s smolrv64_tb -DSIMULATE -DDISASS $(OPTS) smolrv64.v rs232tx.v

smolrv64-run: smolrv64.v rs232tx.v Makefile
	iverilog -o $@ -s smolrv64_tb -DSIMULATE -DNO_TIMEOUT $(OPTS) smolrv64.v rs232tx.v

%.even: %.bin
	./evenodd.py $^ 0 > $@

%.odd: %.bin
	./evenodd.py $^ 1 > $@

%.o: %.c
	$(RV)gcc -march=rv64gc -c $^ -o $@

%.o: %.s
	$(RV)as -march=rv64gc $^ -o $@

%.elf: %.o
	$(RV)ld -Ttext=0x80000000 $^ -o $@

%.bin: %.elf
	$(RV)objcopy -O binary $^ $@

%.dis: %
	$(RV)objdump -Mmax,no-aliases,numeric -d $^ > $@
