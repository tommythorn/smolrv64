#!/bin/bash
# Install the SD autoboot's files on the card's EFI System Partition. Runs ON THE BOARD:
#
#   sudo [DISK=/dev/sdX] ./install-esp.sh <fw_payload.bin> <device tree .dtb>
#
# It mounts the card's ESP (the GPT partition of type C12A7328-F81F-11D2-BA4B-00A0C93EC93B),
# copies the payload and the device tree into /smolrv64 there and writes /smolrv64/boot.txt,
# which the ROM monitor runs line by line ten seconds after reset. The device tree decides the
# root file system: workloads/ubuntu/ubuntu-nfs.dtb mounts NFS, ubuntu-sd.dtb the card's first
# partition (make -C workloads/ubuntu dtbs). It runs wherever the card is: on the board, or on a
# host with the card in a reader (DISK=/dev/sdX).
set -eu
FW=$1; DTB=$2
DISK=${DISK:-/dev/vda}
ESP=$(lsblk -nrpo NAME,PARTTYPE "$DISK" | awk 'tolower($2) == "c12a7328-f81f-11d2-ba4b-00a0c93ec93b" { print $1; exit }')
[ -n "$ESP" ] || { echo "no EFI System Partition on $DISK" >&2; exit 1; }
MNT=$(mktemp -d)
mount "$ESP" "$MNT"
trap 'umount "$MNT"; rmdir "$MNT"' EXIT
mkdir -p "$MNT/smolrv64"
cp "$FW"  "$MNT/smolrv64/fw_payload.bin"
cp "$DTB" "$MNT/smolrv64/smolrv64.dtb"
{
   echo "# The smolrv64 ROM monitor runs these lines at autoboot; '#' starts a comment line."
   echo "L80000000 /smolrv64/fw_payload.bin"
   echo "Lffdff000 /smolrv64/smolrv64.dtb"
   echo "X80000000 0 ffdff000"
} > "$MNT/smolrv64/boot.txt"
sync
echo "installed on $ESP:"; ls -la "$MNT/smolrv64"; cat "$MNT/smolrv64/boot.txt"
