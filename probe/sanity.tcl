# Area/timing sanity check for the sharded-OoO backend (out-of-context synth).
# Reports utilization (LUT/FF/DSP/BRAM/CARRY) and post-synth timing at the target
# clock + the worst path. Post-synth timing is optimistic (no routing) but a clear
# negative or a surprising critical path is the signal we want before going further.
#   vivado -mode batch -source sanity.tcl -tclargs <period_ns>
set period [expr {[llength $argv] >= 1 ? [lindex $argv 0] : 3.0}]
set part   xcku5p-ffvb676-2-i

# all backend RTL (exclude testbenches, probe harnesses, BP variant)
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
lappend files ../src/alu.v
puts "=== sanity: synth backend_top @ ${period}ns ==="
puts "    files: $files"

foreach f $files { read_verilog $f }
synth_design -top backend_top -part $part -mode out_of_context \
   -include_dirs {. ../src} -flatten_hierarchy rebuilt

create_clock -name clk -period $period [get_ports clk]

report_utilization -file sanity_util.rpt
report_timing_summary -delay_type max -max_paths 1 -report_unconstrained -file sanity_tsum.rpt
report_timing -setup -max_paths 3 -file sanity_timing.rpt
set sp [get_timing_paths -setup -max_paths 1 -nworst 1]
if {[llength $sp] > 0} {
   set wns [get_property SLACK $sp]
   puts [format ">>> post-synth WNS @ %sns = %.3f ns  (achieved ~%.3f ns, ~%.1f MHz)" \
            $period $wns [expr {$period - $wns}] [expr {1000.0/($period - $wns)}]]
}
