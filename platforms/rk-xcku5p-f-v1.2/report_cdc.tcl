# report_cdc.tcl — open impl_1 checkpoint and audit clock-domain crossings.
# Usage: vivado -mode batch -source report_cdc.tcl

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
open_run impl_1

puts "\n=== Clocks ==="
report_clocks

puts "\n=== Clock interaction (domain pairs / common-path) ==="
report_clock_interaction -delay_type min_max -significant_digits 3

puts "\n=== CDC report (summary) ==="
report_cdc -details

close_project
