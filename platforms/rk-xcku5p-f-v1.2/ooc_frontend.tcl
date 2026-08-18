# Out-of-context synthesis of the sharded-OoO frontend against a 6 ns budget.
#
# Question: frontend.v is byte-identical to the version that demonstrably worked
# (4af2ebc0), and it is item 4's frontend, not the in-order core's. Tommy's caveat is
# that it "might not have met 166 MHz timing". Its own header says fetch+decode is
# "one combinational cloud from the PC register through imem read, the aligner, and
# decode_stage", which at 6 ns is the entire budget. This measures that.
#
# OOC is only a logic-depth signal -- no placement, no routing. Read it as a LOWER bound
# on the path: the real design added 5.9 ns of route to 2.97 ns of logic on the current
# core's worst path. So an OOC result that already fails 6 ns is fatal; one that passes
# is necessary but nowhere near sufficient.

set part xcku5p-ffvb676-2-i
set srcdir [lindex $argv 0]
set iw     [lindex $argv 1]
set period 6.000

# Read every non-testbench source in src/. Unused modules are simply not elaborated
# under -top frontend, so an over-broad list is safe, whereas a hand-curated one just
# fails on the first dependency you forgot (rvc_expand, the first time round).
foreach f [lsort [glob -nocomplain $srcdir/*.v]] {
    set b [file tail $f]
    if {[string match "tb_*" $b]} { continue }
    if {[catch {read_verilog -sv $f} e]} { puts "skip $b: $e" }
}

synth_design -mode out_of_context -top frontend -part $part \
             -generic IW=$iw -include_dirs $srcdir

create_clock -name clk -period $period [get_ports clk]

puts "\n################ IW=$iw at ${period} ns ################"
report_timing_summary -delay_type max -no_header -max_paths 1

set p [lindex [get_timing_paths -delay_type max -max_paths 1 -quiet] 0]
if {$p ne ""} {
    set slack [get_property SLACK $p]
    puts [format "\nIW=%s  WNS %.3f ns  =>  logic-only path %.3f ns  =>  OOC Fmax %.1f MHz" \
             $iw $slack [expr {$period - $slack}] [expr {1000.0/($period - $slack)}]]
    puts [format "IW=%s  logic levels %d   start %s   end %s" $iw \
             [get_property LOGIC_LEVELS $p] \
             [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]]
}
report_utilization -hierarchical -hierarchical_depth 2
