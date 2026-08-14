set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
open_run impl_1
puts "=== IDDRE1 -> fanout (setup) ==="
report_timing -from [get_cells -hier -filter {REF_NAME == IDDRE1}] -max_paths 4 -delay_type max -sort_by slack
puts "=== IDDRE1 -> fanout (hold) ==="
report_timing -from [get_cells -hier -filter {REF_NAME == IDDRE1}] -max_paths 4 -delay_type min -sort_by slack
puts "=== eth_rxc clock networks ==="
report_clock_utilization -clock_roots
puts "=== unconstrained eth inputs ==="
report_timing -from [get_ports {eth_rxd[*] eth_rx_ctl}] -max_paths 2 -delay_type max
