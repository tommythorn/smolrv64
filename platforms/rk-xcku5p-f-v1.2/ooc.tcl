# ooc.tcl -- synthesise ONE module out of context and report what limits it.
#
#   make ooc MODULE=ooo2_iq            # default period 6.000 ns == 166.67 MHz
#   make ooc MODULE=ooo2_iq PERIOD=4
#
# WHY. In the full design every critical path is 65-83% ROUTE (rule I7) and the placer's
# choices swamp differences smaller than ~200 ps (rule I2), so the flat design cannot tell you
# whether a STRUCTURE is good -- only whether today's placement was lucky. Out of context the
# module is alone on the die: the logic depth and the intrinsic Fmax are its own. Use it to
# compare a proposed replacement against the incumbent at the same interface, then confirm in
# the full build, never the other way round.
#
# It reports Fmax for the module ALONE. That number will be far above the system clock and is
# NOT a promise about the integrated design -- it is a comparison instrument.
set part xcku5p-ffvb676-2-i
set mod  $::env(OOC_MODULE)
set per  [expr {[info exists ::env(OOC_PERIOD)] && $::env(OOC_PERIOD) ne "" ? $::env(OOC_PERIOD) : 6.000}]
set root [file normalize [file join [file dirname [info script]] ../..]]

set files {}
foreach d [list [file join $root ooo2] [file join $root src]] {
    foreach f [glob -nocomplain [file join $d *.v]] {
        if {[string match *tb_* [file tail $f]]} continue
        lappend files $f
    }
}
set incdirs [list [file join $root ooo2] [file join $root src] [file join $root src generated]]
read_verilog -quiet $files
synth_design -top $mod -part $part -mode out_of_context -flatten_hierarchy rebuilt \
    -include_dirs $incdirs
create_clock -name clk -period $per [get_ports -quiet clk]
opt_design -quiet
place_design -quiet
route_design -quiet
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1]]
puts "-----------------------------------------------------------------"
puts [format "OOC %-16s period %.3f ns   WNS %+0.3f ns   Fmax %.1f MHz" \
        $mod $per $wns [expr {1000.0/($per - $wns)}]]
puts "-----------------------------------------------------------------"
report_timing -max_paths 3 -nworst 3 -path_type full
report_utilization -quiet
puts "OOC-DONE"
