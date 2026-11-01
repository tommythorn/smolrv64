# timing-census.tcl <dcp> <out>: every failing setup endpoint's worst path of a routed checkpoint --
# slack, logic levels, logic and net delay, start and end pin -- one line each after a WNS line
# (tools/timing-census.sh runs it and groups the lines into families).
lassign $argv dcp out
open_checkpoint $dcp
set f [open $out w]
puts $f "WNS [get_property SLACK [lindex [get_timing_paths -max_paths 1 -delay_type max] 0]]"
foreach p [get_timing_paths -max_paths 4000 -nworst 1 -slack_lesser_than 0 -delay_type max -unique_pins] {
   puts $f [format "%.3f L%d logic=%.3f net=%.3f %s -> %s" [get_property SLACK $p] [get_property LOGIC_LEVELS $p] \
      [get_property DATAPATH_LOGIC_DELAY $p] [get_property DATAPATH_NET_DELAY $p] \
      [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]]
}
close $f
close_design
