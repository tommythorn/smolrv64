# fmax_report.tcl -- per-clock achievable Fmax from the routed checkpoint.
#
# Global WNS is NOT the core's speed limit: on this board it has repeatedly been pinned by the
# virtio/MIG ui_clk domain at 333 MHz while probe_clk was absent from the worst-5 setup paths.
# Judging the core by global WNS therefore under-reports it (and a failing global WNS may say
# nothing about how fast the core could run).
#
# For each clock, report the worst INTRA-clock setup path (launch and capture both on that
# clock) and turn it into an achievable period: period - slack. That is the number that tells
# us which PROBE_CLK_DIV8 a given core width can support, from ONE build instead of a divisor
# binary search.
#
#   Usage: vivado -mode batch -source fmax_report.tcl   (or: make fmax-report)

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
open_run impl_1

puts "\n=== Per-clock achievable Fmax (intra-clock setup) ==="
puts [format "%-22s %10s %10s %10s %12s" "clock" "period(ns)" "MHz" "slack(ns)" "Fmax(MHz)"]
foreach c [get_clocks] {
    set name   [get_property NAME $c]
    set period [get_property PERIOD $c]
    if {$period <= 0} { continue }
    # path groups are named after their capture clock; -group selects them (there is no
    # -from_clock/-to_clock on get_timing_paths).
    set paths [get_timing_paths -delay_type max -max_paths 1 -group $c -quiet]
    if {[llength $paths] == 0} {
        puts [format "%-22s %10.3f %10.1f %10s %12s" $name $period [expr {1000.0/$period}] "-" "(no intra path)"]
        continue
    }
    set slack [get_property SLACK [lindex $paths 0]]
    set achiev [expr {$period - $slack}]
    if {$achiev <= 0} { set fmax "inf" } else { set fmax [format "%.1f" [expr {1000.0/$achiev}]] }
    puts [format "%-22s %10.3f %10.1f %10.3f %12s" $name $period [expr {1000.0/$period}] $slack $fmax]
}

puts "\n=== Worst 5 setup paths overall (which domain actually limits the design) ==="
report_timing -max_paths 8 -delay_type max -sort_by slack

close_project
