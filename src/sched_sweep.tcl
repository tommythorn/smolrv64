# Sweep the scheduler RS depth (backend_top SCHED_N) and report, per N, the worst
# reg-to-reg path's LOGIC delay + endpoint + LUT count. The select/eligibility logic
# scales with N and lands on the registered issue path (sched_bundle -> q_iss in
# backend_top = reg-to-reg), so the full backend captures it; the shared scoreboard
# write (wake -> ready) is N-independent. Logic delay only (OOC route is fake).
#   vivado -mode batch -source sched_sweep.tcl -tclargs <N>
set n    [lindex $argv 0]
set part xcku5p-ffvb676-2-i

set files {}
foreach f [glob *.v] {
   if {[string match "tb_*" $f]}        continue
   if {[string match "*_probe.v" $f]}   continue
   if {$f eq "flopwrap.v"}              continue
   if {$f eq "rf_alu.v"}                continue
   if {$f eq "exec_shard_bp.v"}         continue
   if {$f eq "renamer.v"}               continue
   lappend files $f
}
lappend files ./alu.v
foreach f $files { read_verilog $f }
synth_design -top backend_top -part $part -mode out_of_context \
   -include_dirs {.} -flatten_hierarchy rebuilt -generic SCHED_N=$n
create_clock -name clk -period 3.0 [get_ports clk]

set u [report_utilization -return_string]
set luts "?"
regexp {CLB LUTs\*?\s+\|\s+(\d+)} $u -> luts

set p [lindex [get_timing_paths -setup -max_paths 1 -nworst 1] 0]
puts [format ">>> N=%-2s LUTs=%s logic=%.3f route=%.3f levels=%s slack=%.3f" \
  $n $luts [get_property DATAPATH_LOGIC_DELAY $p] [get_property DATAPATH_NET_DELAY $p] \
  [get_property LOGIC_LEVELS $p] [get_property SLACK $p]]
puts ">>> N=$n SRC [get_property STARTPOINT_PIN $p]"
puts ">>> N=$n DST [get_property ENDPOINT_PIN $p]"
