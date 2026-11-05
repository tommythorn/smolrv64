# report_path.tcl -- the worst routed path between two groups of cells, pin by pin.
# Usage: make path FROM=<cell glob> TO=<cell glob> [N=1] [DCP=<checkpoint>]
#        (globs match hierarchical cell names; the run's routed checkpoint unless DCP names one)
#
# The census names families (start register -> end register); this prints one member whole, so
# the logic between them can be read without opening the GUI.
set here [pwd]
lassign $argv from to n dcp
if {$n eq ""} { set n 1 }
if {$dcp eq ""} {
   foreach c [lmap s {postroute_physopt routed} {string cat $::env(PROJECT) _ $s .dcp}] {
      set f [file join $here $::env(PROJECT).runs impl_1 $c]
      if {[file exists $f]} { set dcp $f; break }
   }
}
if {$dcp eq ""} { error "no routed checkpoint in impl_1" }
open_checkpoint $dcp
set fc [get_cells -hier -filter "NAME =~ $from"]
set tc [get_cells -hier -filter "NAME =~ $to"]
puts "PATH: [llength $fc] start cells match $from, [llength $tc] end cells match $to"
report_timing -from $fc -to $tc -max_paths $n -delay_type max -input_pins -sort_by slack
