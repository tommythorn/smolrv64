#!/usr/bin/env bash
# One line per boot in the serial console log: where it starts, what RTL the monitor announced,
# and the first NIC symptom, the first NFS stall and the login prompt with their kernel times.
# The console is cumulative across boots and the previous kernel keeps printing while the board
# is reprogrammed, so a verdict is only ever read from a boot's OWN kernel marker onward -- this
# is the table that exposed the 2026-09-17 misreadings (every bitstream of the branch sick, the
# old bitstream clean, the gate's PASS/FAIL lines wrong). tools/board-gate.sh REPLAY=<byte>
# re-judges one boot; this lists them all.
#   tools/console-boots.sh                 # the main checkout's workloads/ubuntu/screenlog.0
#   tools/console-boots.sh <screenlog> [N] # another log, the last N boots (default 20)
set -u
LOG=${1:-$(cd "$(dirname "$0")/.." && git worktree list | head -1 | awk '{print $1}')/workloads/ubuntu/screenlog.0}
N=${2:-20}
awk -v n="$N" '
  /smolrv64 monitor/ { m = $0; sub(/.*rtl=/, "", m); sub(/ .*/, "", m); rtl = m }
  /riscv: base ISA extensions/ {
     if (b) flush(); b = NR; brtl = rtl; net = ""; nfs = ""; login = ""; last = ""; bad = 0 }
  b && /^\[ *[0-9]+\.[0-9]+\]/ { t = $0; sub(/^\[ */, "", t); sub(/\].*/, "", t); last = t }
  b && net == ""   && /NETDEV WATCHDOG/          { net = last }
  b && nfs == ""   && /nfs: server .* not responding/ { nfs = last }
  b && login == "" && /login:/                   { login = last ? last : "y" }
  b && /Kernel panic|Oops \[#|segfault|SIGSEGV|core dumped|is not a head/ { bad++ }
  function flush() { line[++k] = sprintf("%-9d %-10s %-10s %-10s %-10s %-4d %s", b, brtl, net ? net : "-", nfs ? nfs : "-", login ? login : "-", bad, (net == "" && nfs == "" && bad == 0) ? (login != "" ? "CLEAN" : "no verdict (cut short?)") : (login != "" ? "sick (login reached)" : "sick")) }
  END { if (b) flush(); printf("%-9s %-10s %-10s %-10s %-10s %-4s %s\n", "line", "rtl", "1st NETDEV", "1st NFS", "login", "bad", "verdict"); for (i = (k > n ? k - n + 1 : 1); i <= k; i++) print line[i] }
' "$LOG"
