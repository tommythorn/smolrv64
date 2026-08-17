# fmax166.tcl -- what stands between the core and PROBE_CLK_DIV=2 (166.67 MHz)?
#
# BUFGCE_DIV divides ui_clk (333.333 MHz) by an integer, so the frequency ladder is
# 66.7 / 83.3 / 111.1 / 166.7 / 333.3 -- there is no step between the current 111.1 MHz
# (div 3, 9.0 ns) and 166.7 MHz (div 2, 6.0 ns). Every path must therefore fit in 6.0 ns.
#
# At the current 9.0 ns period, a path whose delay already exceeds 6.0 ns shows a slack
# below 3.0 ns. So "slack < 3.0" enumerates exactly the work list for 166 MHz. Paths are
# collapsed to their hierarchy prefixes so distinct problems group into families rather
# than appearing as thousands of near-identical bit-slices.

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
open_run impl_1

set PERIOD    9.000
set TARGET    6.000
set THRESH    [expr {$PERIOD - $TARGET}]

puts "\n=== paths that do NOT fit in ${TARGET} ns (slack < ${THRESH} ns at ${PERIOD} ns) ==="
set paths [get_timing_paths -delay_type max -group probe_clk \
                            -max_paths 4000 -slack_lesser_than $THRESH -quiet]
puts "probe_clk paths over budget: [llength $paths]"

array set fam {}
foreach p $paths {
    set sp [get_property STARTPOINT_PIN $p]
    set ep [get_property ENDPOINT_PIN   $p]
    set sl [get_property SLACK          $p]
    set ll [get_property LOGIC_LEVELS   $p]
    if {$sp eq "" || $ep eq ""} { continue }
    set s [join [lrange [split $sp /] 0 3] /]
    set e [join [lrange [split $ep /] 0 3] /]
    set k [format "%-46s -> %s" $s $e]
    if {![info exists fam($k)]} { set fam($k) [list 0 99.0 0] }
    lassign $fam($k) n worst maxll
    incr n
    if {$sl   < $worst} { set worst $sl }
    if {$ll   > $maxll} { set maxll $ll }
    set fam($k) [list $n $worst $maxll]
}

set rows {}
foreach k [array names fam] {
    lassign $fam($k) n worst maxll
    lappend rows [list $worst $n $maxll $k]
}
puts [format "\n%9s %7s %7s  %s" "worst(ns)" "paths" "levels" "family (startpoint -> endpoint)"]
foreach r [lsort -real -index 0 $rows] {
    lassign $r worst n maxll k
    puts [format "%9.3f %7d %7d  %s" $worst $n $maxll $k]
}

# Slack histogram: how much of the design is already close to the 6 ns budget?
puts "\n=== probe_clk slack distribution (intra-clock setup) ==="
foreach lim {0.5 1.0 1.5 2.0 2.5 3.0 4.0 5.0} {
    set ps [get_timing_paths -delay_type max -group probe_clk \
                             -max_paths 20000 -slack_lesser_than $lim -quiet]
    puts [format "  slack < %4.1f ns : %6d paths   (delay > %4.1f ns)" \
                 $lim [llength $ps] [expr {$PERIOD - $lim}]]
}

close_project
