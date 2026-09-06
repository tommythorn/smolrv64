#!/usr/bin/env bash
# The post-synthesis netlist boots the ROM monitor (docs/rtl-rules.md F5): rv_soc_top synthesized
# out of context with the shipping options, its funcsim netlist under xsim with
# ooo2/tb_ooo2_netlist.v. PASS = the banner's characters come out. ~6 minutes.
#   tools/netlist-boot.sh                # this tree; output under gate-results/netlist-<sha>/
#   MAXCYC=8000 MINCHARS=40 tools/netlist-boot.sh
set -u
REPO=$(cd "$(dirname "$0")/.." && pwd)
XIL=${XIL:-/home/tommy/Xilinx/2025.1/Vivado}; export LD_LIBRARY_PATH=$XIL/lib/lnx64.o/Ubuntu/24
SHA=$(git -C "$REPO" rev-parse --short HEAD); O=$REPO/gate-results/netlist-$SHA; mkdir -p "$O"; cd "$O"
[ -f "$REPO/src/mem.linehex" ] || python3 "$REPO/src/binline.py" "$REPO/workloads/monitor/monitor.bin" > "$REPO/src/mem.linehex"
echo "netlist-boot $SHA: synthesizing rv_soc_top out of context ($(date +%H:%M))"
"$XIL/bin/vivado" -mode batch -nojournal -log "$O/synth.vlog" -source "$REPO/tools/ooc-netlist.tcl" -tclargs "$REPO" "$SHA" "$O" > "$O/synth.out" 2>&1
grep -q "^OOC-DONE" "$O/synth.out" || { echo "NETLIST-BOOT: FAIL (synthesis) $(grep -E '^ERROR' "$O/synth.out" | head -1 | cut -c1-160)"; exit 1; }
source "$XIL/settings64.sh" >/dev/null 2>&1
xvlog --relax -log "$O/xvlog.log" "$O/ooc-$SHA-funcsim.v" "$REPO/ooo2/tb_ooo2_netlist.v" "$XIL/data/verilog/src/glbl.v" > "$O/xvlog.out" 2>&1 || { echo "NETLIST-BOOT: FAIL (xvlog) $(grep ERROR "$O/xvlog.out" | head -1 | cut -c1-160)"; exit 1; }
xelab -log "$O/xelab.log" -L unisims_ver -L secureip -timescale 1ns/1ps --snapshot boot-$SHA tb glbl > "$O/xelab.out" 2>&1 || { echo "NETLIST-BOOT: FAIL (xelab) $(grep ERROR "$O/xelab.out" | head -1 | cut -c1-160)"; exit 1; }
xsim boot-$SHA -R -log "$O/xsim.log" -testplusarg maxcyc=${MAXCYC:-8000} -testplusarg maxchars=${MINCHARS:-40} > "$O/trace.out" 2>&1
SUM=$(grep -a "^SUMMARY" "$O/trace.out" | tail -1); CH=$(echo "$SUM" | grep -oE "chars=[0-9]+" | cut -d= -f2)
TXT=$(grep -a "UART" "$O/trace.out" | sed -n 's/.*UART \([0-9a-f]*\) .*/\1/p' | tr -d '\n' | xxd -r -p | tr -d '\r\n' | cut -c1-60)
if [ "${CH:-0}" -ge "${MINCHARS:-40}" ]; then echo "NETLIST-BOOT: PASS $SUM  \"$TXT\""; exit 0; fi
echo "NETLIST-BOOT: FAIL $SUM  \"$TXT\"  [evidence: gate-results/netlist-$SHA/]"; exit 1
