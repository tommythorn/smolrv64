# report_path.tcl -- the worst routed path between two groups of cells, pin by pin.
# Usage: make path FROM=<cell glob> TO=<cell glob> [N=1]   (globs match hierarchical cell names)
#
# The census names families (start register -> end register); this prints one member whole, so
# the logic between them can be read without opening the GUI.
set here [file dirname [info script]]
set dcp ""
foreach c {rk_xcku5p_postroute_physopt.dcp rk_xcku5p_routed.dcp} {
   set f [file join $here rk_xcku5p.runs impl_1 $c]
   if {[file exists $f]} { set dcp $f; break }
}
if {$dcp eq ""} { error "no routed checkpoint in impl_1" }
lassign $argv from to n
if {$n eq ""} { set n 1 }
open_checkpoint $dcp
set fc [get_cells -hier -filter "NAME =~ $from"]
set tc [get_cells -hier -filter "NAME =~ $to"]
puts "PATH: [llength $fc] start cells match $from, [llength $tc] end cells match $to"
report_timing -from $fc -to $tc -max_paths $n -delay_type max -input_pins -sort_by slack
