# glibc userspace under the cosim: the Ubuntu root's dash, ld.so, libc and a few libc-only
# coreutils, overlaid on the tiny128 initramfs, with an init hook that runs glibc-test.sh.
# The binaries are the ones the BOARD runs (not the host cross-toolchain's, which emits
# vector instructions this core lacks -- see workloads/stress/Makefile).
#
#   make            # tiny128-glibc.cpio + ../tiny128/tiny128-cosim-glibc.dtb
#   ./run-cosim.sh  # boot it under lockstep; expect "GLIBC-TEST PASS ... checksum=..."
R    ?= /srv/ubuntu-root
LIBD  = usr/lib/riscv64-linux-gnu
BINS  = dash wc sort head cat tr
LIBS  = libc.so.6 ld-linux-riscv64-lp64d.so.1
START = 0x9f000000

# initrd-start; tiny128's own sits at 0x9f52c000, the DTB at 0x9ff00000
all: tiny128-glibc.cpio ../tiny128/tiny128-cosim-glibc.dtb

tiny128-glibc.cpio: S98glibc glibc-test.sh Makefile $(addprefix $(R)/usr/bin/,$(BINS)) $(addprefix $(R)/$(LIBD)/,$(LIBS))
	rm -rf ov && mkdir -p ov/usr/bin ov/$(LIBD) ov/lib ov/etc/init.d
	for b in $(BINS); do cp $(R)/usr/bin/$$b ov/usr/bin/; done
	for l in $(LIBS); do cp $(R)/$(LIBD)/$$l ov/$(LIBD)/; done
	# the ELF interpreter path and glibc's default library directory, both under /lib
	ln -s ../$(LIBD)/ld-linux-riscv64-lp64d.so.1 ov/lib/ld-linux-riscv64-lp64d.so.1
	ln -s ../$(LIBD) ov/lib/riscv64-linux-gnu
	cp glibc-test.sh ov/usr/bin/glibc-test.sh; cp S98glibc ov/etc/init.d/S98glibc
	chmod +x ov/usr/bin/* ov/etc/init.d/S98glibc
	cd ov && find . | cpio -o -H newc > ../overlay.cpio 2>/dev/null
	# The kernel's initramfs loop recognises a cpio header only at a 4-byte-aligned offset
	# (init/initramfs.c: `*buf == '0' && !(this_header & 3)`) and skips zero bytes one at a
	# time, so the compressed first archive is padded to 4 before the overlay is appended.
	# Unpadded (tiny128.cpio is 3,301,923 B) the kernel said "Initramfs unpacking failed:
	# invalid magic at start of compressed archive" and the overlay never existed: no run
	# of this harness before 2026-09-07 executed its test.
	cat ../tiny128/tiny128.cpio > $@; truncate -s $$(( ($$(wc -c < $@) + 3) / 4 * 4 )) $@; cat overlay.cpio >> $@

# The device tree is tiny128's cosim one with the initrd window moved to START and its end
# stamped from the archive's exact size, so the size can never skew from the archive.
../tiny128/tiny128-cosim-glibc.dtb: tiny128-glibc.cpio ../tiny128/tiny128-cosim.dts
	@sz=$$(wc -c < tiny128-glibc.cpio); end=$$(printf '0x%x' $$(($(START) + sz))); \
	 sed -e "s|linux,initrd-start = <0 0x[0-9a-f]*>;|linux,initrd-start = <0 $(START)>;|" \
	     -e "s|linux,initrd-end   = <0 0x[0-9a-f]*>;.*|linux,initrd-end   = <0 $$end>; /* exact: tiny128-glibc.cpio ($$sz B, auto-stamped) */|" \
	     ../tiny128/tiny128-cosim.dts > ../tiny128/tiny128-cosim-glibc.dts; \
	 dtc -I dts -O dtb -o $@ ../tiny128/tiny128-cosim-glibc.dts 2>/dev/null; \
	 echo "initrd: $(START) .. $$end ($$sz B)"

clean:
	rm -rf ov overlay.cpio tiny128-glibc.cpio ../tiny128/tiny128-cosim-glibc.dts ../tiny128/tiny128-cosim-glibc.dtb
