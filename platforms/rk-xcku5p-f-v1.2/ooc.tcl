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

# THE SHIPPED GEOMETRY, NOT THE MODULE DEFAULTS. Without -generic, synth_design elaborates
# the parameter defaults: for rv_cache that is PAW=34, SIZE_KB=128, PREFETCH=0 -- a 2048-set
# array with 18-bit tags that nothing instantiates. The 178 MHz figure recorded on 2026-09-03
# was measured on that shape; the SoC's D$ is PAW=64/64 KB (49-bit tags) and its I$ adds
# PREFETCH=1. Pass the instance's parameters, and pass BOTH roles for a two-role module:
#   make ooc MODULE=rv_cache GENERICS="PAW=64 SIZE_KB=64 WRITABLE=1 WRTHRU=0 PERF_ID=1"   # D$
#   make ooc MODULE=rv_cache GENERICS="PAW=64 SIZE_KB=64 RDW=64 WRITABLE=0 PREFETCH=1"    # I$
set generics {}
if {[info exists ::env(OOC_GENERICS)] && $::env(OOC_GENERICS) ne ""} {
    foreach g $::env(OOC_GENERICS) { lappend generics -generic $g }
    puts "OOC generics: $::env(OOC_GENERICS)"
} else {
    puts "OOC generics: NONE -- module defaults (for rv_cache that is NOT the shipped shape)"
}
set report [expr {[info exists ::env(OOC_REPORT)] ? $::env(OOC_REPORT) : ""}]

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
    -include_dirs $incdirs {*}$generics
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
if {$report ne ""} {
    # the module's worst 40 endpoints, unique, one line each -- enough to see FAMILIES rather
    # than three copies of one path -- then the full detail of the worst three.
    set fh [open $report w]
    puts $fh [format "OOC %s period %.3f generics {%s} WNS %+0.3f" $mod $per \
        [expr {[info exists ::env(OOC_GENERICS)] ? $::env(OOC_GENERICS) : ""}] $wns]
    foreach p [get_timing_paths -max_paths 40 -nworst 1 -unique_pins -quiet] {
        puts $fh [format "%7.3f lv=%2d logic=%.3f route=%.3f  %s -> %s" [get_property SLACK $p] \
            [get_property LOGIC_LEVELS $p] [get_property DATAPATH_LOGIC_DELAY $p] \
            [get_property DATAPATH_NET_DELAY $p] [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]]
    }
    puts $fh [report_timing -max_paths 3 -nworst 1 -unique_pins -path_type full -return_string]
    puts $fh [report_utilization -return_string]
    close $fh
    puts "OOC report: $report"
}
report_utilization -quiet
puts "OOC-DONE"
