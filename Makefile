run: mem0.hex mem1.hex
	iverilog -s smolrv64_tb -DSIMULATE smolrv64.v rs232tx.v
	./a.out

verbose: mem0.hex mem1.hex
	iverilog -s smolrv64_tb -DSIMULATE -DDISASS smolrv64.v rs232tx.v
	./a.out

mem.hex: hw.s
	riscv64-elf-as -march=rv64gc hw.s -o hw.o
	riscv64-elf-ld -Ttext=0 hw.o -o hw
	riscv64-elf-objdump -Mmax,no-aliases,numeric -d hw > hw.dis
	riscv64-elf-objcopy -O binary hw hw.bin
	hexdump -ve '1/8 "%016x\n"' hw.bin > mem.hex

mem0.hex: mem.hex
	cut -c9-16 < mem.hex > mem0.hex

mem1.hex: mem.hex
	cut -c1-8 < mem.hex > mem1.hex
