#!/usr/bin/dash
# A glibc userspace under the cosim: the shell the board runs (dash), doing the string and
# path work cloud-init's ds-identify does at boot -- the process that segfaulted on the
# board on 2026-09-04 on a pointer the core had corrupted (a store-seqno wrap in
# ooo2_sq), a bug tiny128's static busybox never lined up. Parameter expansion, case,
# read loops over /proc, pipelines through glibc-linked coreutils, arithmetic. Prints one
# line per iteration the cosim log can be grepped for, with a checksum that must be the same
# every run. Each iteration forks ~8 glibc processes; under the cosim that is ~50 M cycles, so
# the default is 4 iterations (argument 1 overrides) -- 40 ran past 1.8 G cycles unseen.
n=0; acc=0; iters=${1:-4}
paths="/usr/bin/dash /etc/cloud/cloud.cfg /run/cloud-init/ds-identify.cfg /sys/class/dmi/id /proc/1/environ"
while [ $n -lt $iters ]; do
   for p in $paths; do
      d=${p%/*}; b=${p##*/}; e=${b#*.}; s=${b%.*}
      case "$p" in
         */cloud*) k=cloud;;
         /proc/*)  k=proc;;
         *.cfg)    k=cfg;;
         *)        k=other;;
      esac
      l=$(printf '%s' "$p" | wc -c)
      acc=$(( (acc * 31 + l + ${#d} + ${#b} + ${#e} + ${#s}) % 1000003 ))
      [ "$k" = "other" ] && acc=$((acc + 7))
   done
   while read -r line; do
      case "$line" in
         isa*|mmu*|hart*) w=$(printf '%s' "$line" | tr -s ' ' | wc -w); acc=$(( (acc + w * 13) % 1000003 ));;
      esac
   done < /proc/cpuinfo
   x=$(printf '%s\n' $paths | sort | head -n 2 | wc -l)
   acc=$(( (acc + x) % 1000003 ))
   n=$((n + 1))
   echo "GLIBC-TEST iteration=$n checksum=$acc"
done
echo "GLIBC-TEST PASS iterations=$n checksum=$acc"
