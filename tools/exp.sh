#!/usr/bin/env bash
# One command for a performance experiment: the standard measurements of a tree, spread over the
# simulation hosts, compared with the baseline's.
#
#   tools/exp.sh <tag> [<tree>]     # measure <tree> (default: this checkout) as <tag>
#   tools/exp.sh main               # the baseline: run it once per main commit, on main's tree
#   BASE=<tag> tools/exp.sh ...     # compare against another tag (default: main-<main's hash>)
#   tools/exp.sh -t <tag> [<base>]  # print a finished experiment's table again
#   SET=sp tools/exp.sh <tag> ...   # GB5's single-core section as SimPoints instead (fastmac, M5, mercury)
#   SP_HOSTS=mercury|fastmac|m5|coffee  # ...all of them on one host (the others busy)
#   RUNS='pdf boot60m' tools/exp.sh <tag> ...   # only the named runs of the set
#
# The set: the 60 M Linux lockstep (retires; the lockstep is the correctness check), Dhrystone and
# CoreMark in lockstep (workloads/rvbench, cycles), the four pi checkpoint slices and the five GB5
# checkpoint slices (20 M retires warm, 50 M measured; cycles). M5 runs the locksteps and pi,
# fastmac the GB5 slices. SET=sp runs workloads/gb5/simpoints.txt's 77 by the hosts' measured
# throughput (2026-10-21: a point takes 674 s on fastmac 12 at a time, 764 s on M5 9 at a time,
# 1046 s on mercury 12 at a time): of every seven three on fastmac, two on M5, two on mercury --
# or, when no Vivado build runs on coffee (Tommy, 2026-10-20: sims on coffee only between builds),
# of every nine two more on coffee (mercury: 192.168.1.166, Linux, its checkpoints and trees on
# its local disk, /scratch/tommy; M5 and coffee: ~/gb5ck/sp),
# points instead (the same 20 + 50 M each), and the table totals them into each subtest's cycles.
# All runs start together, at most J at a time per kind, the kinds side by side: the runners build
# a stale model once under their build lock and then run from it together. The tree is copied
# with rsync (tracked and untracked-not-ignored files, no platforms/), so an uncommitted edit is
# measured as it stands. Results land in ~/exp-results/<tag>/ on coffee: a
# log per run, and the table in table.txt; the hosts' copies go once the logs are in.
set -u
here=${EXP_HERE:-$(cd "$(dirname "$0")/.." && pwd)}
# It runs from a private copy: bash reads a script as it executes, and an edit to this file would
# reach the experiments still waiting in it. The copy deletes itself (the open file stays readable).
if [ -z "${EXP_HERE:-}" ]; then c=$(mktemp); cp "$0" "$c"; EXP_HERE=$here exec bash "$c" "$@"; fi
case $0 in /tmp/*|"${TMPDIR:-/tmp}"/*) rm -f "$0" ;; esac
R=$HOME/exp-results
O="-o ControlMaster=no -o BatchMode=yes -o ConnectTimeout=20"
M5_OPT="$O -o HostKeyAlias=192.168.1.110"; M5_IP=$(getent hosts M5.local | awk '{print $1; exit}')
FM_OPT="$O"; FM_IP=192.168.1.135
MC_OPT="$O"; MC_IP=192.168.1.166
M5_ROOT=exp; FM_ROOT=/Volumes/tommy/exp; MC_ROOT=/scratch/tommy/exp          # M5's under its $HOME
M5_CK='$HOME/gb5ck'; FM_CK=/Volumes/tommy/gb5ck; MC_CK=/scratch/tommy/gb5ck   # <ckpt>.ckpt, pi/ck/<n>.ckpt, sp/<n>.ckpt
CF_ROOT=$HOME/exp-runs; CF_CK=$HOME/gb5ck      # coffee itself, local, between builds
cf_build() { ps -eo comm | grep -qx vivado || ps -eo args | grep -qE '[t]ools/gate\.sh|[b]oard-gate\.sh'; }   # a gate in any phase is a build
cf_ok=0; [ "${SET:-}" = sp ] && ! cf_build && [ -d "$CF_CK/sp" ] && cf_ok=1

table() { python3 "$here/tools/exp-table.py" "$R/$1" ${2:+"$R/$2"}; }
if [ "${1:-}" = "-t" ]; then table "${2:?tag}" "${3:-}"; exit; fi
tag=${1:?usage: tools/exp.sh <tag> [<tree>]}
tree=$(cd "${2:-$here}" && pwd)
if [ "$tag" = main ]; then tag=main-$(git -C "$tree" rev-parse --short=8 HEAD); fi
base=${BASE:-main-$(git -C "$here" rev-parse --short=8 origin/main)}
mkdir -p "$R/$tag"
echo "exp $tag: $tree at $(git -C "$tree" rev-parse --short=8 HEAD)$( [ -n "$(git -C "$tree" status --porcelain -- core src | grep -v '^??')" ] && echo ' with edits')"

# the rvbench binaries are built here (the hosts have no RISC-V toolchain)
make -s -C "$tree/workloads/rvbench" dhrystone.bin coremark.bin > /dev/null || exit 1
tohost() { riscv64-linux-gnu-nm "$tree/workloads/rvbench/$1.elf" | awk '$3 == "tohost" {print $1}'; }
m5_runs="boot60m:boot:60000000 dhry:rvb:dhrystone,$(tohost dhrystone) coremark:rvb:coremark,$(tohost coremark) pi450:pi:450000000 pi700:pi:700000000 pi950:pi:950000000 pi1150:pi:1150000000"
fm_runs="nav:gb5:140000000000 html5:gb5:154500000000 sqlite:gb5:161000000000 pdf:gb5:171000000000 camera:gb5:202000000000"
mc_runs="";  cf_runs=""
if [ "${SET:-}" = sp ]; then   # a point's checkpoint is sp/<instret>.ckpt; the points alternate between the two
   # the points host $1 takes: of every seven fastmac three, M5 two, mercury two, or with coffee
   # free of builds, of every nine two more on coffee
   pts() { awk -F'\t' -v h="$1" -v cf=$cf_ok '$1 == "point" { k = n++ % (cf ? 9 : 7)
              w = (k < 3) ? "fm" : (k < 5) ? "m5" : (cf && k < 7) ? "cf" : "mc"
              if (h == "all" || h == w) printf " sp%s:gb5:sp/%s", $5, $5 }' "$here/workloads/gb5/simpoints.txt"; }
   m5_runs="";  fm_runs=""
   case ${SP_HOSTS:-} in
      mercury) mc_runs=$(pts all) ;;
      fastmac) fm_runs=$(pts all) ;;
      m5)      m5_runs=$(pts all) ;;
      coffee)  [ $cf_ok = 1 ] || { echo "exp $tag: a build runs on coffee (or no checkpoints there): no sims"; exit 1; }
               cf_runs=$(pts all) ;;
      *)       fm_runs=$(pts fm);  m5_runs=$(pts m5);  mc_runs=$(pts mc);  cf_runs=$(pts cf) ;;
   esac
fi
if [ -n "${RUNS:-}" ]; then   # only the runs named
   keep() { local r o=""; for r in $1; do [[ " $RUNS " == *" ${r%%:*} "* ]] && o="$o $r"; done; echo "$o"; }
   m5_runs=$(keep "$m5_runs");  fm_runs=$(keep "$fm_runs");  mc_runs=$(keep "$mc_runs");  cf_runs=$(keep "$cf_runs")
fi

# the hosts that have runs: a host with none gets no tree, no runner and no wait, so an unreachable
# one does not stop an experiment it has no part in (SP_HOSTS=m5 with fastmac down)
declare -A H_OPT=([M5]="$M5_OPT" [fastmac]="$FM_OPT" [mercury]="$MC_OPT") H_IP=([M5]="$M5_IP" [fastmac]="$FM_IP" [mercury]="$MC_IP")
declare -A H_ROOT=([M5]="$M5_ROOT" [fastmac]="$FM_ROOT" [mercury]="$MC_ROOT") H_CK=([M5]="$M5_CK" [fastmac]="$FM_CK" [mercury]="$MC_CK")
declare -A H_J=([M5]=9 [fastmac]=12 [mercury]=12) H_RUNS=([M5]="$m5_runs" [fastmac]="$fm_runs" [mercury]="$mc_runs")
hosts=""; for h in M5 fastmac mercury; do [ -n "${H_RUNS[$h]// /}" ] && hosts="$hosts $h"; done
h_root() { case ${H_ROOT[$1]} in /*) echo "${H_ROOT[$1]}" ;; *) echo "\$HOME/${H_ROOT[$1]}" ;; esac; }   # M5's is under its $HOME

# the tree and the rvbench binaries, to each host that has runs
list=$(mktemp)
(cd "$tree" && git ls-files -co --exclude-standard | grep -v '^platforms/') > "$list"
echo workloads/rvbench/dhrystone.bin >> "$list";  echo workloads/rvbench/coremark.bin >> "$list"
for h in $hosts; do rsync -a --files-from="$list" -e "ssh ${H_OPT[$h]}" "$tree/" "${H_IP[$h]}:${H_ROOT[$h]}/$tag/" || exit 1; done
[ -n "$cf_runs" ] && { mkdir -p "$CF_ROOT/$tag" && rsync -a --files-from="$list" "$tree/" "$CF_ROOT/$tag/" || exit 1; }
rm -f "$list"

runner() {   # <tree> <out> <ckpt dir> <parallel runs> <runs: name:kind:arg ...>
   local t=$1 o=$2 ck=$3 j=$4 k r n kk a rs; shift 4
   cat <<'HEAD'
#!/bin/bash
export PATH=/opt/homebrew/bin:$HOME/.cargo.arm64-darwin/bin:$HOME/install/x86_64-linux/bin:$HOME/.cargo.x86_64-linux/bin:$PATH
run() {   # <name> <kind> <arg>
   case $2 in
      boot) CYC=$3 ./run-cosim-linux.sh ;;
      rvb)  FW=$T/workloads/rvbench/${3%,*}.bin INITRD= CYC=400000000 \
               PLUSARGS="+tohost=${3#*,}" ./run-cosim-linux.sh ;;
      pi)   CKPT=$CK/pi/ck/$3.ckpt CYC=0 ./run-linux.sh +measure_from=20000000 +max_retires=70000000 ;;
      gb5)  CKPT=$CK/$3.ckpt MEM_LG2=31 CYC=0 ./run-linux.sh +measure_from=20000000 +max_retires=70000000 ;;
   esac > $O/$1.log 2>&1; echo "rc=$?" >> $O/$1.log
}
kind() {  # <kind> <name:arg ...>: the runs in parallel, J at a time; each runner books its memory
          # before it starts (tools/simmem.sh), so several experiments can share a host
   local k=$1 n a r; shift
   for r in "$@"; do
      IFS=: read n a <<< "$r"
      while [ $(jobs -pr | wc -l) -ge $J ]; do sleep 10; done
      run $n $k $a &
   done
   wait
}
HEAD
   echo "T=$t; O=$o; CK=$ck; J=$j; mkdir -p \$O; cd \$T/core || exit 1"
   for k in boot rvb pi gb5; do
      rs=""
      for r in "$@"; do IFS=: read n kk a <<< "$r"; [ "$kk" = "$k" ] && rs="$rs $n:$a"; done
      [ -n "$rs" ] && echo "kind $k$rs &"
   done
   echo 'wait'
   echo 'echo EXP-DONE > $O/done'
}
for h in $hosts; do
   r=${H_ROOT[$h]}
   runner "$(h_root $h)/$tag" "$(h_root $h)/$tag.out" "${H_CK[$h]}" ${H_J[$h]} ${H_RUNS[$h]} |
      ssh ${H_OPT[$h]} ${H_IP[$h]} "rm -rf $r/$tag.out; cat > $r/$tag.sh; chmod +x $r/$tag.sh; (nohup $r/$tag.sh > /dev/null 2>&1 < /dev/null &)" || exit 1
done
if [ -n "$cf_runs" ]; then
   runner "$CF_ROOT/$tag" "$CF_ROOT/$tag.out" "$CF_CK" 10 $cf_runs > "$CF_ROOT/$tag.sh" || exit 1
   rm -rf "$CF_ROOT/$tag.out"; chmod +x "$CF_ROOT/$tag.sh"; (nohup "$CF_ROOT/$tag.sh" > /dev/null 2>&1 < /dev/null &)
fi
echo "exp $tag: running on${hosts}$([ -n "$cf_runs" ] && echo ' coffee') ($(date +%H:%M))"

# wait for all, never silently (tools/watch-remote.sh), then fetch the logs and print the table. A
# host gives up after 3 hours in which none of its runs finished: the probes print the count of
# finished runs, and several experiments sharing the hosts take far longer than one alone.
done_or_count() { echo "test -f $1/done && echo DONE || grep -l '^rc=' $1/*.log 2>/dev/null | wc -l"; }
for h in $hosts; do
   "$here/tools/watch-remote.sh" $h "$(done_or_count ${H_ROOT[$h]}/$tag.out)" 10800 60 > /dev/null ||
      { echo "exp $tag: $h did not finish"; exit 1; }
done
if [ -n "$cf_runs" ]; then   # coffee's own, by its done file, on the same terms
   end=$(( $(date +%s) + 10800 )); last=""
   until [ -f "$CF_ROOT/$tag.out/done" ]; do
      n=$(grep -l '^rc=' "$CF_ROOT/$tag.out"/*.log 2>/dev/null | wc -l)
      [ "$n" != "$last" ] && { last=$n; end=$(( $(date +%s) + 10800 )); }
      [ $(date +%s) -gt $end ] && { echo "exp $tag: coffee did not finish"; exit 1; }
      sleep 60
   done
fi
got=1
for h in $hosts; do rsync -a -e "ssh ${H_OPT[$h]}" "${H_IP[$h]}:${H_ROOT[$h]}/$tag.out/" "$R/$tag/" || got=0; done
if [ -n "$cf_runs" ]; then rsync -a "$CF_ROOT/$tag.out/" "$R/$tag/" || got=0; fi
[ -d "$R/$base" ] && [ "$base" != "$tag" ] || base=""
table "$tag" "$base" | tee "$R/$tag/table.txt"
# The hosts keep nothing of an experiment whose logs are all here: its tree (with the model built
# in it), its logs and its runner go. One that failed or did not finish keeps them, to look at.
if [ $got = 1 ]; then
   for h in $hosts; do r=${H_ROOT[$h]}; ssh ${H_OPT[$h]} ${H_IP[$h]} "rm -rf $r/$tag $r/$tag.out $r/$tag.sh"; done
   rm -rf "$CF_ROOT/$tag" "$CF_ROOT/$tag.out" "$CF_ROOT/$tag.sh"
fi
