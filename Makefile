run:
	riscv64-elf-as hw.s -o hw.o
	riscv64-elf-ld -Ttext=0 hw.o -o hw
	riscv64-elf-objdump -d hw > hw.dis
	riscv64-elf-objcopy -O binary hw hw.bin
	hexdump -ve '1/4 "%08x\n"' hw.bin > mem.hex
	iverilog smolrv64.v
	./a.out
.tools:;rustup target add riscv64imac-unknown-none-elf && touch .tools
