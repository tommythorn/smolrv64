# report_paths.tcl -- the worst routed path of each family in a list, pin by pin, in one session.
# Usage: make paths PAIRS=<file> [DCP=<checkpoint>] [OUT=paths.txt]
#        each line of PAIRS is "<from cell glob> <to cell glob>"; # starts a comment
#
# make path's batch form: the census names thirty families, and opening the checkpoint is most of
# a path report's time.
set here [pwd]
lassign $argv pf dcp out
if {$out eq ""} { set out paths.txt }
if {$dcp eq ""} {
   foreach c [lmap s {postroute_physopt routed} {string cat $::env(PROJECT) _ $s .dcp}] {
      set f [file join $here $::env(PROJECT).runs impl_1 $c]
      if {[file exists $f]} { set dcp $f; break }
   }
}
if {$dcp eq ""} { error "no routed checkpoint in impl_1" }
set fh [open $pf r]; set pairs [split [read $fh] "\n"]; close $fh
open_checkpoint $dcp
set o [open $out w]
puts $o "paths of $dcp"
foreach l $pairs {
   if {[string trim $l] eq "" || [string index [string trim $l] 0] eq "#"} continue
   lassign $l from to
   set fc [get_cells -hier -quiet -filter "NAME =~ $from"]
   set tc [get_cells -hier -quiet -filter "NAME =~ $to"]
   puts $o "\n######## $from -> $to ([llength $fc] -> [llength $tc] cells)"
   if {[llength $fc] == 0 || [llength $tc] == 0} { puts $o "no cells"; continue }
   set p [get_timing_paths -from $fc -to $tc -max_paths 1 -delay_type max -quiet]
   if {$p eq ""} { puts $o "no path"; continue }
   puts $o [report_timing -of_objects $p -input_pins -return_string]
}
close $o
puts "PATHS: [file normalize $out]"
