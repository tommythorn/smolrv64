# Timing probe: out-of-context synth + place & route of a flop-wrapped circuit,
# constrained with an aggressive clock, to read its Fmax on this FPGA.
#
# Usage (via the Makefile):
#   vivado -mode batch -source probe.tcl -tclargs <top> <period_ns> <part> <src.v> [<src.v> ...]
#
# Method: constrain `clk` tighter than achievable so the tools keep optimizing,
# run a real route, then back Fmax out of the worst setup slack (WNS):
#   achieved_period = constraint - WNS      (WNS<0 => achieved>constraint)
#   Fmax            = 1000 / achieved_period   (MHz, period in ns)

if {[llength $argv] < 4} {
    puts "ERROR: usage: probe.tcl <top> <period_ns> <part> <src.v> \[<src.v> ...\]"
    exit 1
}
set top    [lindex $argv 0]
set period [lindex $argv 1]
set part   [lindex $argv 2]
set srcs   [lrange $argv 3 end]

puts "=== PROBE: top=$top  period=${period}ns  part=$part ==="
puts "    sources: $srcs"

foreach f $srcs { read_verilog $f }

# Out-of-context: no I/O buffers, no pinout needed — just the clock matters.
synth_design -top $top -part $part -mode out_of_context -flatten_hierarchy rebuilt

create_clock -name clk -period $period [get_ports clk]

opt_design
place_design
phys_opt_design
route_design

# Worst setup path / slack.
set spaths [get_timing_paths -setup -max_paths 1 -nworst 1]
if {[llength $spaths] == 0} {
    puts "PROBE RESULT: no constrained setup paths found (check clk port / design)."
    exit 1
}
set wns      [get_property SLACK $spaths]
set achieved [expr {$period - $wns}]
set fmax     [expr {1000.0 / $achieved}]

report_timing -setup -max_paths 5 -file probe_timing.rpt
puts "----------------------------------------------------------------"
puts [format "PROBE RESULT: top=%s  constraint=%sns  WNS=%.3fns  achieved_period=%.3fns  Fmax=%.1f MHz" \
        $top $period $wns $achieved $fmax]
puts "  (worst-path details in probe_timing.rpt)"
puts "----------------------------------------------------------------"
