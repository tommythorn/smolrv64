# The 166 MHz work list, from a design ACTUALLY CONSTRAINED at 6 ns.
#
# The previous "12,082 paths over 6 ns" number was read off a 9 ns build, where paths with
# 3 ns of slack were never pushed -- Vivado only optimizes to the constraint it is given, so
# that figure counts paths that were merely unoptimized, not paths that cannot make it.
# This runs against a build that was told 6 ns and tried, so what still fails is real.
#
# Paths are collapsed to hierarchy prefixes so one structural problem shows up as one family
# rather than a thousand near-identical bit slices.

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
open_run impl_1

set pclk [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *probe_clk_buf/O}]]
set period [get_property PERIOD $pclk]
puts [format "\nprobe_clk '%s': period %.3f ns = %.2f MHz" \
         [get_property NAME $pclk] $period [expr {1000.0/$period}]]

set wns [get_property SLACK [lindex [get_timing_paths -delay_type max -max_paths 1 -quiet] 0]]
puts [format "WNS %.3f ns  =>  longest path is %.3f ns  =>  Fmax as built is %.2f MHz" \
         $wns [expr {$period - $wns}] [expr {1000.0/($period - $wns)}]]

puts "\n=== failing setup paths, grouped by family ==="
set paths [get_timing_paths -delay_type max -max_paths 8000 -slack_lesser_than 0.0 -quiet]
puts "total failing endpoints sampled: [llength $paths]"

array set fam {}
foreach p $paths {
    set sp [get_property STARTPOINT_PIN $p]
    set ep [get_property ENDPOINT_PIN   $p]
    if {$sp eq "" || $ep eq ""} { continue }
    set k [format "%-44s -> %s" \
              [join [lrange [split $sp /] 0 3] /] [join [lrange [split $ep /] 0 3] /]]
    if {![info exists fam($k)]} { set fam($k) [list 0 99.0 0 0.0 0.0] }
    lassign $fam($k) n worst maxll sumlogic sumroute
    incr n
    set s [get_property SLACK $p]
    if {$s < $worst} { set worst $s }
    set ll [get_property LOGIC_LEVELS $p]
    if {$ll > $maxll} { set maxll $ll }
    set fam($k) [list $n $worst $maxll \
                     [expr {$sumlogic + [get_property DATAPATH_LOGIC_DELAY $p]}] \
                     [expr {$sumroute + [get_property DATAPATH_NET_DELAY   $p]}]]
}

set rows {}
foreach k [array names fam] {
    lassign $fam($k) n worst maxll sl sr
    lappend rows [list $worst $n $maxll [expr {$sl/$n}] [expr {$sr/$n}] $k]
}
puts [format "\n%9s %7s %7s %8s %8s  %s" \
         "worst(ns)" "paths" "levels" "logic" "route" "family (startpoint -> endpoint)"]
foreach r [lrange [lsort -real -index 0 $rows] 0 39] {
    lassign $r worst n maxll al ar k
    puts [format "%9.3f %7d %7d %8.3f %8.3f  %s" $worst $n $maxll $al $ar $k]
}

puts "\n=== how deep is the hole? slack histogram ==="
foreach lim {-3.0 -2.5 -2.0 -1.5 -1.0 -0.5 0.0} {
    set ps [get_timing_paths -delay_type max -max_paths 40000 -slack_lesser_than $lim -quiet]
    puts [format "  slack < %5.1f ns : %6d paths   (path delay > %5.2f ns)" \
             $lim [llength $ps] [expr {$period - $lim}]]
}

puts "\n=== worst path in full ==="
report_timing -delay_type max -max_paths 1 -nworst 1 -input_pins

close_project
