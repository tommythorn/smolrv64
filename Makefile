P=hw

run: $(P).bin
	hexdump -ve '1/8 "%016x\n"' $^ > mem.hex
	cut -c9-16 < mem.hex > mem0.hex
	cut -c1-8 < mem.hex > mem1.hex
	iverilog -s smolrv64_tb -DSIMULATE smolrv64.v rs232tx.v
	./a.out

verbose: $(P).bin
	hexdump -ve '1/8 "%016x\n"' $^ > mem.hex
	cut -c9-16 < mem.hex > mem0.hex
	cut -c1-8 < mem.hex > mem1.hex
	iverilog -s smolrv64_tb -DSIMULATE -DDISASS smolrv64.v rs232tx.v
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

%0.hex: %.hex
	cut -c9-16 < $^ > $@

%1.hex: %.hex
	cut -c1-8 < $^ > $@
