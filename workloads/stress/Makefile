# sandbox-stress: nolibc fork/COW/namespace stressor (systemd generator shape).
# MUST stay nolibc: the host cross-glibc is next-Ubuntu-baseline (vector insns
# in memset) and would SIGILL on the rv64gc core.
CROSS ?= riscv64-linux-gnu-
all: sandbox-stress tiny128-stress.cpio
sandbox-stress: sandbox-stress.c nolibc.h
	$(CROSS)gcc -static -nostdlib -nostartfiles -ffreestanding -O2 \
	  -march=rv64gc -mabi=lp64d -o $@ sandbox-stress.c
	@test $$($(CROSS)objdump -d $@ | grep -cE 'vset|vmv|vle|vse') = 0
# overlay: /bin/sandbox-stress + S99 hook auto-runs it (cosim has no typist).
# NOTE: S99 also reruns at halt (no stop-arg check) -- deliberately kept, the
# shutdown-time rerun was the first trigger of the wrong-address-store oops.
tiny128-stress.cpio: sandbox-stress
	rm -rf ov && mkdir -p ov/bin ov/etc/init.d
	cp sandbox-stress ov/bin/
	printf '#!/bin/sh\necho "=== S99stress ==="\ncp /bin/sandbox-stress /tmp/ss && X=/tmp/ss || X=/bin/sandbox-stress\n$$X\necho "=== S99stress: exit $$? ==="\n' > ov/etc/init.d/S99stress
	chmod +x ov/etc/init.d/S99stress ov/bin/sandbox-stress
	cd ov && find . | cpio -o -H newc > ../overlay.cpio 2>/dev/null
	cat ../tiny128/tiny128.cpio overlay.cpio > ../tiny128/tiny128-stress.cpio
	# Stamp the EXACT initrd-end into the cosim-stress dts + regen the dtb, so the
	# size can never skew from the archive again (the manual-update treadmill bit twice).
	@sz=$$(wc -c < ../tiny128/tiny128-stress.cpio); end=$$(printf '0x%x' $$((0x9f400000 + sz))); \
	 sed -i "s|linux,initrd-end   = <0 0x[0-9a-f]*>;.*|linux,initrd-end   = <0 $$end>; /* exact: tiny128-stress.cpio ($$sz B, auto-stamped) */|" \
	     ../tiny128/tiny128-cosim-stress.dts; \
	 dtc -I dts -O dtb -o ../tiny128/tiny128-cosim-stress.dtb ../tiny128/tiny128-cosim-stress.dts 2>/dev/null; \
	 echo "initrd-end auto-stamped: 0x9f400000 + $$sz = $$end"
