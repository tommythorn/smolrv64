# report_util.tcl -- open the newest impl_1 checkpoint and dump device utilization.
# Usage: make util          (never call vivado directly; the Makefile sets LD_LIBRARY_PATH)
#
# Exists because "is the device full?" is the first question to ask before doing any more
# critical-path surgery: at 65-83% route on every failing path (rule I7), utilization and
# placement pressure decide the answer, not logic depth. `make` produces no utilization
# report of its own, so this was being answered by guesswork.
set rundir rk_xcku5p.runs/impl_1
set dcp ""
foreach c {rk_xcku5p_postroute_physopted.dcp rk_xcku5p_postroute_physopt.dcp rk_xcku5p_routed.dcp rk_xcku5p_placed.dcp} {
    if {[file exists $rundir/$c]} { set dcp $rundir/$c; break }
}
if {$dcp eq ""} { error "no impl_1 checkpoint found -- run `make` first" }
puts "UTIL: reading $dcp"
open_checkpoint $dcp
report_utilization -file $rundir/rk_xcku5p_utilization.rpt
puts "UTIL: wrote $rundir/rk_xcku5p_utilization.rpt"
foreach l [split [report_utilization -return_string] "\n"] {
    if {[regexp {^\| (CLB LUTs|CLB Registers|CLB  |LUT as Logic|LUT as Memory|Block RAM Tile|DSPs|CARRY8)} $l]} { puts "UTIL| $l" }
}
