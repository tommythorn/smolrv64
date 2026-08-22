#!/usr/bin/env bash
# Make perf usable WITHOUT sudo on the board, persistently.
#
# perf_event_paranoid defaults to 2 on Ubuntu, which blocks the raw PMU events the CPI
# stack needs, so every measurement needed `sudo perf stat`.  Lowering it is the whole fix;
# there is nothing about the RTL or the DTB that requires root.  -1 = no restrictions,
# which is the right setting for a development board and the wrong one for a shared host.
set -eu
echo 'kernel.perf_event_paranoid = -1' | sudo tee /etc/sysctl.d/99-smolrv64-perf.conf
sudo sysctl -p /etc/sysctl.d/99-smolrv64-perf.conf
echo "perf_event_paranoid now $(cat /proc/sys/kernel/perf_event_paranoid) -- no sudo needed"
