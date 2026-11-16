#!/bin/sh
# Make a SmolRV64 NFS root from Ubuntu's preinstalled riscv64 server image. Run as root on the NFS
# server:
#
#   sudo ./nfs-root.sh ubuntu-25.04-preinstalled-server-riscv64.img.xz /srv/smolrv64
#
# It copies the image's root partition (partition 1) into the directory and comments out the
# card's own mounts in its /etc/fstab (the root and the EFI System Partition, which an NFS root
# does not have). The root keeps the image's first-boot setup: cloud-init creates the user ubuntu,
# password ubuntu, to be changed at the first login. Then export the directory, e.g. with
#   /srv/smolrv64 192.168.1.0/24(rw,sync,no_root_squash,no_subtree_check)
# in /etc/exports and `exportfs -ra`.
set -eu
[ $# -eq 2 ] || { sed -n '2,12p' "$0"; exit 2; }
img=$1 dir=$2
[ "$(id -u)" -eq 0 ] || { echo "run it as root: it mounts the image and keeps its owners" >&2; exit 1; }
[ ! -e "$dir" ] || [ -z "$(ls -A "$dir")" ] || { echo "$dir is not empty" >&2; exit 1; }
mkdir -p "$dir"
# The scratch space is beside the root, not in /tmp: the image is 10 GB uncompressed.
tmp=$(mktemp -d -p "$(dirname "$(realpath "$dir")")")
loop=
cleanup() {
    umount "$tmp/p1" 2>/dev/null || true
    [ -z "$loop" ] || losetup -d "$loop"
    rm -rf "$tmp"
}
trap cleanup EXIT
case $img in
    *.xz)  echo "decompressing $img"; xz -dc "$img" > "$tmp/img"; raw=$tmp/img ;;
    *.zst) echo "decompressing $img"; zstd -dc "$img" > "$tmp/img"; raw=$tmp/img ;;
    *)     raw=$img ;;
esac
loop=$(losetup -P -r -f --show "$raw")
mkdir "$tmp/p1"
mount -o ro "${loop}p1" "$tmp/p1"
echo "copying the root into $dir"
# Owners, modes, links and times, not ACLs or extended attributes: NFS carries neither.
tar -C "$tmp/p1" -cf - . | tar -C "$dir" -xpf - --numeric-owner
sed -i -E 's/^(LABEL=(cloudimg-rootfs|UEFI)[[:space:]])/# on the SD card, not on an NFS root: \1/' "$dir/etc/fstab"
echo "done: $dir"
grep -v '^#' "$dir/etc/fstab" || true
