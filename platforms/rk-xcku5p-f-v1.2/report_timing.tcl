# report_timing.tcl — open impl_1 checkpoint and dump the worst failing paths.
# Usage: vivado -mode batch -source report_timing.tcl

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
open_run impl_1

set wns [get_property STATS.WNS [get_runs impl_1]]
set tns [get_property STATS.TNS [get_runs impl_1]]
set whs [get_property STATS.WHS [get_runs impl_1]]
set ths [get_property STATS.THS [get_runs impl_1]]
puts "\n=== Timing summary ==="
puts "WNS = ${wns} ns (setup)"
puts "TNS = ${tns} ns (setup)"
puts "WHS = ${whs} ns (hold)"
puts "THS = ${ths} ns (hold)"

puts "\n=== Worst 5 setup paths (all) ==="
report_timing -max_paths 5 -delay_type max -sort_by slack

puts "\n=== Worst 5 failing setup paths (slack < 0) ==="
report_timing -max_paths 5 -slack_lesser_than 0 -delay_type max

puts "\n=== Worst 5 hold paths (all) ==="
report_timing -max_paths 5 -delay_type min -sort_by slack

puts "\n=== Worst 5 failing hold paths (slack < 0) ==="
report_timing -max_paths 5 -slack_lesser_than 0 -delay_type min

close_project
