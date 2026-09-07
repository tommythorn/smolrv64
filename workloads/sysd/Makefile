# The board's own init through the board's own loader, under the cosim: /usr/lib/systemd/systemd
# and its 20-library closure from the NFS root, overlaid on the tiny128 initramfs, with an init
# hook that runs `systemd --version` -- ld.so loads and relocates every library, which is where
# the two-wide core with the run-ahead fetch buffer (10e) dies on the board at 2.4 s (W1, W5:
# epc == ra, a data address in ld.so's writable segment). No such code runs in the tiny128 or
# Geekbench cosims. Same shape as workloads/glibc.
#
#   make            # tiny128-sysd.cpio + ../tiny128/tiny128-cosim-sysd.dtb
#   ./run-cosim.sh  # boot it under lockstep; expect "SYSD-TEST PASS"
R     ?= /srv/ubuntu-root
LIBD   = usr/lib/riscv64-linux-gnu
SYSD   = usr/lib/systemd/systemd
LIBS   = ld-linux-riscv64-lp64d.so.1 libacl.so.1 libapparmor.so.1 libaudit.so.1 libblkid.so.1 libc.so.6 \
         libcap-ng.so.0 libcap.so.2 libcrypt.so.1 libcrypto.so.3 libm.so.6 libmount.so.1 libpam.so.0 \
         libpcre2-8.so.0 libseccomp.so.2 libselinux.so.1 libz.so.1 libzstd.so.1
PLIBS  = systemd/libsystemd-core-257.so systemd/libsystemd-shared-257.so
START  = 0x9e000000

all: tiny128-sysd.cpio ../tiny128/tiny128-cosim-sysd.dtb

tiny128-sysd.cpio: S98sysd Makefile $(R)/$(SYSD) $(addprefix $(R)/$(LIBD)/,$(LIBS) $(PLIBS))
	rm -rf ov && mkdir -p ov/usr/lib/systemd ov/$(LIBD)/systemd ov/lib ov/etc/init.d
	cp $(R)/$(SYSD) ov/usr/lib/systemd/
	for l in $(LIBS) $(PLIBS); do cp $(R)/$(LIBD)/$$l ov/$(LIBD)/$$l; done
	ln -s ../$(LIBD)/ld-linux-riscv64-lp64d.so.1 ov/lib/ld-linux-riscv64-lp64d.so.1
	ln -s ../$(LIBD) ov/lib/riscv64-linux-gnu
	cp S98sysd ov/etc/init.d/S98sysd; chmod +x ov/etc/init.d/S98sysd
	cd ov && find . | cpio -o -H newc > ../overlay.cpio 2>/dev/null
	cat ../tiny128/tiny128.cpio overlay.cpio > $@

../tiny128/tiny128-cosim-sysd.dtb: tiny128-sysd.cpio ../tiny128/tiny128-cosim.dts
	@sz=$$(wc -c < tiny128-sysd.cpio); end=$$(printf '0x%x' $$(($(START) + sz))); \
	 sed -e "s|linux,initrd-start = <0 0x[0-9a-f]*>;|linux,initrd-start = <0 $(START)>;|" \
	     -e "s|linux,initrd-end   = <0 0x[0-9a-f]*>;.*|linux,initrd-end   = <0 $$end>; /* exact: tiny128-sysd.cpio ($$sz B, auto-stamped) */|" \
	     ../tiny128/tiny128-cosim.dts > ../tiny128/tiny128-cosim-sysd.dts; \
	 dtc -I dts -O dtb -o $@ ../tiny128/tiny128-cosim-sysd.dts 2>/dev/null; \
	 echo "initrd: $(START) .. $$end ($$sz B)"

clean:
	rm -rf ov overlay.cpio tiny128-sysd.cpio ../tiny128/tiny128-cosim-sysd.dts ../tiny128/tiny128-cosim-sysd.dtb
