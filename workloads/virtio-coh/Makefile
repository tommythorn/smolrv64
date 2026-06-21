platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

ARCH = rv64imac_zicsr_zicbom

all: load.mon load_nc.mon

%.elf: crt0.s %.c virtio_coh.ld
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=$(ARCH) -fno-builtin -nostdlib -Os \
		-Wl,-Tvirtio_coh.ld crt0.s $*.c -o $@

%.bin: %.elf
	$(RV)objcopy -O binary $< $@

# Monitor command scripts: paste into the monitor prompt to load + run over UART.
load.mon: virtio_coh.bin gen_w.py
	./gen_w.py virtio_coh.bin 0x80000000 > $@

load_nc.mon: virtio_coh_nc.bin gen_w.py
	./gen_w.py virtio_coh_nc.bin 0x80000000 > $@

.PHONY: all clean
clean:
	rm -f virtio_coh.elf virtio_coh.bin load.mon \
	      virtio_coh_nc.elf virtio_coh_nc.bin load_nc.mon
