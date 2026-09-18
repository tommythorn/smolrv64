#!/bin/sh
# B6 (2026-09-17): the disk-backed lockstep's own verdict. Installed as /etc/init.d/S99blkcheck
# in tiny128-blk.cpio (the Makefile appends it to tiny128.cpio), so busybox's rcS runs it after
# the initrd's own init. It mounts the virtio-blk disk, checks every byte of its payload, writes
# a copy back, drops the page cache and re-reads both files from the device. One console line
# decides: BLKCHECK-OK or BLKCHECK-FAIL (ooo2/run-ooo2-cosim-linux.sh greps for it when DISK is set).
mkdir -p /mnt/blk
ok=0
if mount -t ext4 /dev/vda /mnt/blk; then
    cd /mnt/blk
    if sha256sum -c SHA256SUMS; then
        dd if=data.bin of=copy.bin bs=4096 2>/dev/null && sync
        echo 3 > /proc/sys/vm/drop_caches
        a=$(sha256sum < data.bin); b=$(sha256sum < copy.bin)
        [ "$a" = "$b" ] && ok=1
    fi
    cd /; umount /mnt/blk
fi
[ $ok = 1 ] && echo BLKCHECK-OK || echo BLKCHECK-FAIL
