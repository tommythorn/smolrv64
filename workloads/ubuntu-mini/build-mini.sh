#!/bin/bash
# build-mini.sh -- prune the rdump'd Ubuntu rootfs (rootfs/, extracted via debugfs from the
# repro image) into ubuntu-mini.cpio: an initramfs that boots the SAME systemd/GLib/generator
# stack to login: with the rootfs entirely in RAM -- one identical artifact for HW and sim,
# and the virtio/non-coherent-DMA-vs-pure-core discriminator for the generator wedge.
#
# Keeps: systemd + generators, GLib/netplan, cloud-init (+python), ssh, pam/login, dbus.
# Drops: dev toolchain (gcc/LLVM/binutils/yosys/gdb/emacs/vim/perl/git...), firmware, kernel
# modules (we run the fw_payload kernel: built-ins only), docs/man/locales, apt/dpkg state,
# logs, /home payloads, and the /lib /bin /sbin trees rdump duplicated (restored as usrmerge
# symlinks). Kernel-side needs /dev/console before systemd mounts devtmpfs -> a tiny crafted
# cpio segment with the device nodes is prepended (non-root can't mknod).
set -eu
cd "$(dirname "$0")"
R=rootfs

# ---- usrmerge: rdump materialized the /lib,/bin,/sbin symlinks as full copies ----
for d in lib lib64 bin sbin; do
   if [ -d "$R/$d" ] && [ ! -L "$R/$d" ]; then rm -rf "$R/$d"; ln -s "usr/$d" "$R/$d"; fi
done

# ---- big non-boot subtrees ----
rm -rf "$R"/usr/lib/firmware "$R"/usr/lib/modules "$R"/usr/src "$R"/usr/include \
       "$R"/boot/* "$R"/var/lib/apt "$R"/var/lib/dpkg "$R"/var/cache/* \
       "$R"/var/backups/* "$R"/var/tmp/* "$R"/usr/libexec/gcc "$R"/usr/libexec/emacs \
       "$R"/usr/lib/gcc "$R"/usr/lib/riscv64-linux-gnu/guile \
       "$R"/usr/lib/riscv64-linux-gnu/perl "$R"/usr/lib/riscv64-linux-gnu/perl-base \
       "$R"/snap "$R"/var/lib/snapd
for s in emacs vim doc man info locale python-babel-localedata perl perl5 icons i18n \
         ieee-data fonts lintian bug gdb git gitweb yosys aclocal autoconf automake* \
         build-essential; do rm -rf "$R/usr/share/$s"; done
find "$R/var/log" -type f -delete

# ---- fat toolchain/editor binaries (measured; login needs none of these) ----
( cd "$R/usr/bin" && rm -f riscv64-linux-gnu-* yosys* gdb* emacs* vim* rvim rview \
     snap rg perf bpftrace perl* git git-* cc gcc* cpp* c++* g++* ld ld.* as objdump \
     objcopy ar nm ranlib readelf strip addr2line gprof make cmake* ctest cpack ninja )
# fat non-boot libraries (LLVM/clang/sanitizers/static archives/editor+gui deps)
( cd "$R/usr/lib/riscv64-linux-gnu" && rm -rf libLLVM* libclang* lib*san.so* *.a \
     libicu* libgtk-3* libgdk* libmysqlclient* libgccjit* )

# ---- /home payloads (keep the login user, empty home) ----
rm -rf "$R"/home/tommy/* "$R"/home/tommy/.[!.]* 2>/dev/null || true

# ---- fstab: initramfs root ignores root=, but fstab-generator would emit device-wait
#      units for vda/boot partitions that never appear -> comment every entry out ----
sed -i 's/^[^#]/#&/' "$R/etc/fstab"

# ---- kernel entrypoint: initramfs boots /init ----
ln -sf sbin/init "$R/init"

# ---- pack: main tree (root-owned) + prepended segment with /dev/console + /dev/null ----
python3 - <<'EOF'
import struct, os
def member(name, mode, rdev_maj=0, rdev_min=0, body=b""):
    ino = member.ino; member.ino += 1
    h = b"070701" + b"".join(f"{v:08X}".encode() for v in
        [ino, mode, 0, 0, 1, 0, len(body), 0, 0, rdev_maj, rdev_min, len(name)+1, 0])
    d = h + name.encode() + b"\0"
    d += b"\0" * ((4 - len(d) % 4) % 4)
    d += body + b"\0" * ((4 - len(body) % 4) % 4)
    return d
member.ino = 721
out  = member("dev",         0o040755)
out += member("dev/console", 0o020600, 5, 1)
out += member("dev/null",    0o020666, 1, 3)
out += member("TRAILER!!!",  0)
open("devnodes.cpio", "wb").write(out)
EOF
( cd "$R" && find . | LC_ALL=C sort | cpio -o -H newc -R +0:+0 --quiet ) > main.cpio
cat devnodes.cpio main.cpio > ubuntu-mini.cpio
rm -f devnodes.cpio main.cpio
gzip -1 -kf ubuntu-mini.cpio
ls -la ubuntu-mini.cpio ubuntu-mini.cpio.gz

# ---- stamp the exact initrd range into ubuntu-ram.dts. UNCOMPRESSED at guest 0x9000_0000:
# the fw kernel's gzip initramfs path returns 'decompressor failed' on valid streams
# (SOFTWARE behavior -- oracle-verified architecture-clean on the DUT), so no compression.
sz=$(wc -c < ubuntu-mini.cpio); end=$(printf '0x%x' $((0x90000000 + sz)))
sed -i "s|linux,initrd-end   = <0 0x[0-9a-f]*>;.*|linux,initrd-end   = <0 $end>; /* exact: ubuntu-mini.cpio ($sz B, auto-stamped) */|" ubuntu-ram.dts
dtc -I dts -O dtb -o ubuntu-ram.dtb ubuntu-ram.dts 2>/dev/null
echo "initrd range stamped: 0x90000000 + $sz = $end"
