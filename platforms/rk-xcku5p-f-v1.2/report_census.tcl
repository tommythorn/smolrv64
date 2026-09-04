# report_census.tcl -- the NEAR-CRITICAL CENSUS of a routed checkpoint.
#
#   make census                       # the run's post-route phys_opt checkpoint
#   make census DCP=path/to/x.dcp OUT=census.txt [SLACK=0.35]
#
# WHY. WNS is one path. This design closes on placement luck because THOUSANDS of endpoints
# sit within the 81-400 ps placement spread of the edge (rule I2), and a fix is only a fix
# if it empties a FAMILY, not if it moves today's worst path. Groups every endpoint under
# SLACK ns by (startpoint, endpoint) with bit indices stripped, then by startpoint module
# and endpoint module, and prints the slack histogram -- the three numbers a timing change
# is judged by (rules I2, I8, I9). Read-only: opens the checkpoint, writes a text file.
set dcp   [lindex $argv 0]
set out   [lindex $argv 1]
set lim   [expr {[llength $argv] > 2 ? [lindex $argv 2] : 0.35}]
open_checkpoint $dcp
set fh [open $out w]
proc P {fh s} { puts $fh $s; puts $s }
set pclk [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *probe_clk_buf/O}]]
P $fh [format "census of %s: probe_clk period %.3f, endpoints with slack < %+.2f" $dcp [get_property PERIOD $pclk] $lim]
set worst [get_timing_paths -group $pclk -delay_type max -max_paths 1 -nworst 1 -quiet]
P $fh [format "probe_clk WNS %+.3f" [get_property SLACK $worst]]
proc base {pin d} { set p [join [lrange [split $pin /] 0 $d] /]; regsub -all {\[[0-9]+\]} $p "" p; regsub {_reg_[0-9_]+$} $p "_reg" p; regsub {_rep(_[0-9]+)?$} $p "" p; regsub {_replica(_[0-9]+)?$} $p "" p; return $p }
proc fam {fh paths title d} {
    P $fh "\n=== $title: [llength $paths] paths, keys at depth $d, indices stripped ==="
    array unset f
    foreach p $paths {
        set sp [get_property STARTPOINT_PIN $p]; set ep [get_property ENDPOINT_PIN $p]
        if {$sp eq "" || $ep eq ""} continue
        set k [format "%-44s -> %s" [base $sp $d] [base $ep $d]]
        if {![info exists f($k)]} { set f($k) [list 0 99.0 0 0.0 0.0] }
        lassign $f($k) n w ml sl sr
        incr n; set s [get_property SLACK $p]; if {$s < $w} {set w $s}
        set ll [get_property LOGIC_LEVELS $p]; if {$ll > $ml} {set ml $ll}
        set f($k) [list $n $w $ml [expr {$sl + [get_property DATAPATH_LOGIC_DELAY $p]}] [expr {$sr + [get_property DATAPATH_NET_DELAY $p]}]]
    }
    set rows {}
    foreach k [array names f] { lassign $f($k) n w ml sl sr; lappend rows [list $w $n $ml [expr {$sl/$n}] [expr {$sr/$n}] $k] }
    P $fh "-- sorted by worst slack --"
    P $fh [format "%8s %6s %6s %6s %6s  %s" worst paths maxlv logic route family]
    foreach r [lrange [lsort -real -index 0 $rows] 0 49] { lassign $r w n ml al ar k; P $fh [format "%8.3f %6d %6d %6.3f %6.3f  %s" $w $n $ml $al $ar $k] }
    P $fh "-- sorted by path count --"
    foreach r [lrange [lsort -integer -decreasing -index 1 $rows] 0 29] { lassign $r w n ml al ar k; P $fh [format "%8.3f %6d %6d %6.3f %6.3f  %s" $w $n $ml $al $ar $k] }
}
set near [get_timing_paths -group $pclk -delay_type max -max_paths 40000 -slack_lesser_than $lim -unique_pins -quiet]
fam $fh $near "unique endpoints under +$lim" 3
proc hist {fh paths title which} {
    P $fh "\n=== $title ==="
    array unset m
    foreach p $paths { set e [base [get_property $which $p] 2]; if {![info exists m($e)]} {set m($e) 0}; incr m($e) }
    foreach k [lsort -command {apply {{a b} {global m; expr {$m($b) - $m($a)}}}} [array names m]] { P $fh [format "%6d  %s" $m($k) $k] }
}
hist $fh $near "by STARTPOINT module (unique endpoints under +$lim)" STARTPOINT_PIN
hist $fh $near "by ENDPOINT module (unique endpoints under +$lim)" ENDPOINT_PIN
P $fh "\n=== slack histogram (probe_clk, unique endpoints) ==="
foreach l {0.0 0.1 0.2 0.3 0.4 0.5 0.75 1.0} {
    P $fh [format "slack < %+.2f : %d endpoints" $l [llength [get_timing_paths -group $pclk -delay_type max -max_paths 60000 -slack_lesser_than $l -unique_pins -quiet]]]
}
P $fh "\n=== the worst path, in full ==="
P $fh [report_timing -of_objects $worst -return_string]
close $fh
puts "CENSUS-DONE: $out"
