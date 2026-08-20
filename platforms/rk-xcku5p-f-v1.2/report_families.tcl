# report_families.tcl — bucket ALL failing setup paths by {start module -> end module}.
#
# `report_timing -max_paths 5` answers "what is the single worst path", which is the
# wrong question once closure needs nanoseconds rather than picoseconds: fixing the
# worst path just promotes the next member of the same family. This reports where the
# MASS of negative slack lives, so the next RTL change can be chosen by how many
# picosecond-endpoints it retires rather than by which path happened to sort first.
#
# Usage: vivado -mode batch -nojournal -nolog -source report_families.tcl
#        (add -tclargs <N> to change the path budget, default 5000)

# Open the design. Prefer a checkpoint on disk over `open_run impl_1`: a run whose
# bookkeeping was disturbed (an external reset_run, a killed parent) reports PROGRESS 0%
# and refuses to open, even though the checkpoint it produced is perfectly good.
set here [file dirname [info script]]
set dcps [list \
   [file join $here rk_xcku5p.runs impl_1 rk_xcku5p_postroute_physopt.dcp] \
   [file join $here rk_xcku5p.runs impl_1 rk_xcku5p_routed.dcp] \
   [file join $here rk_xcku5p.runs impl_1 rk_xcku5p_physopt.dcp] \
   [file join $here rk_xcku5p.runs impl_1 rk_xcku5p_placed.dcp]]
set opened ""
foreach d $dcps {
   if {[file exists $d]} { open_checkpoint $d ; set opened $d ; break }
}
if {$opened eq ""} {
   open_project [file normalize [file join $here rk_xcku5p.xpr]]
   open_run impl_1
   set opened "impl_1 (run)"
}
puts "=== design: $opened ==="

set budget 5000
if {[llength $argv] > 0} { set budget [lindex $argv 0] }

# Hierarchy depth used as the family key. 3 keeps u_ino/u_core/u_lsu distinct from
# u_ino/u_core/u_csr without splitting on individual registers.
set depth 3

proc famkey {pinname depth} {
   set cell [file dirname $pinname]           ;# drop the pin, keep the cell
   set parts [split $cell "/"]
   if {[llength $parts] <= $depth} { return $cell }
   return [join [lrange $parts 0 [expr {$depth-1}]] "/"]
}

set paths [get_timing_paths -delay_type max -max_paths $budget -nworst 1 \
                            -slack_lesser_than 0 -sort_by slack]

puts "\n=== failing setup endpoints: [llength $paths] (budget $budget) ==="

array set cnt {} ; array set worst {} ; array set sumlvl {} ; array set sumdly {}
array set example {}
foreach p $paths {
   set slack [get_property SLACK $p]
   set lvl   [get_property LOGIC_LEVELS $p]
   set dly   [get_property DATAPATH_DELAY $p]
   set s     [get_property NAME [get_property STARTPOINT_PIN $p]]
   set d     [get_property NAME [get_property ENDPOINT_PIN   $p]]
   set k     "[famkey $s $depth]  ->  [famkey $d $depth]"
   if {![info exists cnt($k)]} {
      set cnt($k) 0 ; set worst($k) 0.0 ; set sumlvl($k) 0 ; set sumdly($k) 0.0
      set example($k) "$s -> $d"
   }
   incr cnt($k)
   if {$slack < $worst($k)} { set worst($k) $slack ; set example($k) "$s -> $d" }
   set sumlvl($k) [expr {$sumlvl($k) + $lvl}]
   set sumdly($k) [expr {$sumdly($k) + $dly}]
}

# rank by total negative slack owned, not by the single worst path
set rows {}
foreach k [array names cnt] {
   lappend rows [list $k $cnt($k) $worst($k) \
                      [expr {double($sumlvl($k))/$cnt($k)}] \
                      [expr {$sumdly($k)/$cnt($k)}] $example($k)]
}
set rows [lsort -real -index 2 $rows]

puts [format "\n%6s %9s %7s %8s  %s" "paths" "worst" "avg_lvl" "avg_dly" "family"]
puts [string repeat - 110]
foreach r $rows {
   puts [format "%6d %9.3f %7.1f %8.3f  %s" \
         [lindex $r 1] [lindex $r 2] [lindex $r 3] [lindex $r 4] [lindex $r 0]]
   puts [format "%35s worst path: %s" "" [lindex $r 5]]
}

