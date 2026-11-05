# report_worst.tcl -- the worst setup paths of a checkpoint over EVERY clock, with their clocks.
# Usage: make worst DCP=<checkpoint> [N=10]
# The census (report_census.tcl) counts probe_clk only; the build's and physopt_close's WNS is the
# worst path of any clock, so a miss the census does not show is in another domain.
lassign $argv dcp n
if {$n eq ""} { set n 10 }
open_checkpoint $dcp
foreach c [get_clocks] {
   set p [get_timing_paths -max_paths 1 -nworst 1 -setup -group $c]
   if {[llength $p]} { puts [format "WORST %-28s %+.3f" [get_property NAME $c] [get_property SLACK $p]] }
}
report_timing -max_paths $n -nworst 1 -delay_type max -sort_by slack -input_pins
