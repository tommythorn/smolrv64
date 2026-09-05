#!/usr/bin/env bash
# The board's software stack under simmerv: the same fw_payload, the same NFS root from
# /srv/ubuntu-root, over a host-only TAP. Faster than the board, and the oracle for "does the
# software itself do this" (e.g. Geekbench's Structure from Motion) against the core.
#
# One-time host setup (root; coffee is the NFS server, so no LAN bridge is needed):
#   sudo ip tuntap add dev tap0 mode tap user $USER
#   sudo ip addr add 192.168.10.1/24 dev tap0
#   sudo ip link set tap0 up
#   echo '/srv/ubuntu-root 192.168.10.0/24(rw,sync,no_root_squash,no_subtree_check)' | sudo tee -a /etc/exports
#   sudo exportfs -ra
# Then:  ./simmerv-nfs.sh            (Ctrl-C ends it; -n = no popup terminal, console on stdout)
set -u
cd "$(dirname "$0")"
SV=${SIMMERV:-$HOME/simmerv}/target/release/simmerv_cli
[ -x "$SV" ] || { echo "no $SV -- build simmerv first"; exit 1; }
ip -br link show tap0 >/dev/null 2>&1 || { echo "no tap0: run the setup in the header"; exit 1; }
dtc -q -I dts -O dtb -o ubuntu-nfs-simmerv.dtb ubuntu-nfs-simmerv.dts || exit 1
exec "$SV" -n -c -m 2048 -T tap0 -d ubuntu-nfs-simmerv.dtb,0x9ff00000 fw_payload.bin,0x80000000 "$@"
