# fphammer: bare-metal FP->INT chain hammer (see fphammer.c). Boots at 0x8000_0000
# via the monitor (FW=fphammer.bin) or tb_virtio (+fw=fphammer.bin).
platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

fphammer.bin: fphammer.elf
	$(RV)objcopy -O binary fphammer.elf fphammer.bin

fphammer.elf: fphammer.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s fphammer.c -o fphammer.elf

clean:
	rm -f fphammer.elf fphammer.bin

misalign.bin: misalign.elf
	$(RV)objcopy -O binary misalign.elf misalign.bin

misalign.elf: misalign.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s misalign.c -o misalign.elf

fpseq.bin: fpseq.elf
	$(RV)objcopy -O binary fpseq.elf fpseq.bin

fpseq.elf: fpseq.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s fpseq.c -o fpseq.elf

csrtest.bin: csrtest.elf
	$(RV)objcopy -O binary csrtest.elf csrtest.bin

csrtest.elf: csrtest.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s csrtest.c -o csrtest.elf

probetrap.bin: probetrap.elf
	$(RV)objcopy -O binary probetrap.elf probetrap.bin

probetrap.elf: probetrap.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s probetrap.c -o probetrap.elf

csrmprv.bin: csrmprv.elf
	$(RV)objcopy -O binary csrmprv.elf csrmprv.bin

csrmprv.elf: csrmprv.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s csrmprv.c -o csrmprv.elf

csrreplay.bin: csrreplay.elf
	$(RV)objcopy -O binary csrreplay.elf csrreplay.bin

csrreplay.elf: csrreplay.c crt0.s fphammer.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfphammer.ld -O2 crt0.s csrreplay.c -o csrreplay.elf
