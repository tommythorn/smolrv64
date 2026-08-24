# opcond_sweep.tcl -- how much of our negative slack is Vivado's operating-condition
# pessimism rather than the design?
#
# The part is a -2 INDUSTRIAL device, so by default STA covers junction -40..+100 C and
# worst-case VCCINT. This board is actively cooled: SYSMON read 32.9 C on the die under
# sustained md5sum load, with VCCINT at 0.853 V. Neither default corner is reachable here.
# On UltraScale+ this matters at BOTH ends -- temperature inversion means the cold corner
# can be the slow one, and -40 C is pure fiction for a lab board with a fansink.
#
# Re-timing a routed checkpoint under different operating conditions costs seconds: no
# re-place, no re-route, the delays are simply re-derated. Usage:
#   vivado -mode batch -source opcond_sweep.tcl [-tclargs <checkpoint.dcp>]
set here [file dirname [info script]]
set dcp [file join $here rk_xcku5p.runs impl_1 rk_xcku5p_postroute_physopt.dcp]
if {[llength $argv] > 0} { set dcp [lindex $argv 0] }
if {![file exists $dcp]} { set dcp [file join $here rk_xcku5p.runs impl_1 rk_xcku5p_routed.dcp] }
puts "=== checkpoint: $dcp ==="
open_checkpoint $dcp

proc pwns {} {
   set c [get_clocks -quiet probe_clk]
   if {[llength $c] == 0} { return "n/a" }
   set p [get_timing_paths -quiet -setup -max_paths 1 -nworst 1 -group $c]
   if {[llength $p] == 0} { return "n/a" }
   return [format "%7.3f" [get_property SLACK [lindex $p 0]]]
}
proc gwns {} {
   set p [get_timing_paths -quiet -setup -max_paths 1 -nworst 1]
   if {[llength $p] == 0} { return "n/a" }
   return [format "%7.3f" [get_property SLACK [lindex $p 0]]]
}

puts "\n=== default operating conditions (what every build so far assumed) ==="
report_operating_conditions
puts [format "\n%-46s %9s %9s" "operating condition" "probe_clk" "global"]
puts [string repeat - 68]
puts [format "%-46s %9s %9s" "DEFAULT (-2I: -40..100 C, worst-case VCCINT)" [pwns] [gwns]]

# Temperature only. Measured 32.9 C under load; sweep a band of defensible ceilings.
foreach t {85 70 55 45 35} {
   set_operating_conditions -junction_temp $t
   puts [format "%-46s %9s %9s" "junction_temp = $t C" [pwns] [gwns]]
}

# ...and then voltage on top of the most defensible temperature.
set_operating_conditions -junction_temp 55
foreach v {0.840 0.845 0.850} {
   set_operating_conditions -voltage [list VCCINT $v VCCBRAM $v]
   puts [format "%-46s %9s %9s" "junction_temp = 55 C, VCCINT = $v V" [pwns] [gwns]]
}
puts "\nNOTE: every row below the first is a PROMISE about the operating envelope, not a"
puts "free win. Temperature is the defensible one (measured, and enforceable with a SYSMON"
puts "over-temp alarm). Voltage is the risky one -- SYSMON reads the rail after on-die"
puts "decoupling and will not show a transient droop under a load step."
close_project
