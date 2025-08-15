run: mem.hex
	iverilog -s smolrv64_tb -DSIMULATE smolrv64.v rs232tx.v
	./a.out

verbose: mem.hex
	iverilog -s smolrv64_tb -DSIMULATE -DDISASS smolrv64.v rs232tx.v
	./a.out

mem.hex: hw.s
	riscv64-elf-as hw.s -o hw.o
	riscv64-elf-ld -Ttext=0 hw.o -o hw
	riscv64-elf-objdump -d hw > hw.dis
	riscv64-elf-objcopy -O binary hw hw.bin
	hexdump -ve '1/8 "%016x\n"' hw.bin > mem.hex
