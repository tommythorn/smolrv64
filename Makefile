P=hw
OPTS=

testall:
	@./run-riscv-tests.sh passes fails

fails:
	@./run-riscv-tests.sh fails

run: $(P).bin
	./evenodd.py $^ 0 > mem0.hex
	./evenodd.py $^ 1 > mem1.hex
	iverilog -s smolrv64_tb -DSIMULATE -DNO_TIMEOUT $(OPTS) smolrv64.v rs232tx.v
	./a.out

verbose: $(P).bin
	./evenodd.py $^ 0 > mem0.hex
	./evenodd.py $^ 1 > mem1.hex
	iverilog -s smolrv64_tb -DSIMULATE -DDISASS $(OPTS) smolrv64.v rs232tx.v
	./a.out

mem.hex: $(P).bin

%.o: %.s
	riscv64-elf-as -march=rv64gc $^ -o $@

%.elf: %.o
	riscv64-elf-ld -Ttext=0x80000000 $^ -o $@

%.bin: %.elf
	riscv64-elf-objcopy -O binary $^ $@

%.dis: %
	riscv64-elf-objdump -Mmax,no-aliases,numeric -d $^ > $@

%0.hex: %.bin
	./evenodd.py $^ 0 > $@

%1.hex: %.hex
	./evenodd.py $^ 1 > $@
