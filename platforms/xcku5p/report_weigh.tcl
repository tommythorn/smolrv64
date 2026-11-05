# report_weigh.tcl -- what an instance's LUTs and flops ARE, by the signal each one drives.
# Usage: make weigh INST="<hier path> ..." [DCP=<checkpoint>] [OUT=<file>] [TOP=40]
#        (INST=. weighs the checkpoint's top: `make ooc ... DCP=<file>` keeps one)
#        (one checkpoint open, every instance listed weighed in turn, all into OUT)
#
# A per-instance utilization row says how much, not what: the core's own logic is one row of
# 17K LUTs, and a flattened submodule's logic lands in whichever row its consumer sits. Every
# LUT is named after the net it drives and every flop after its register, so the cell name with
# the index and the replication suffixes stripped (x_reg[3]_i_2__0 -> x) names the signal, and
# grouping by it is a census of the instance's structures: a queue's read muxes, a comparator
# bank, a register file's write enables. A LUT6_2 counts once (one site), a LUTRAM primitive once.
set rundir $::env(PROJECT).runs/impl_1
set insts [lindex $argv 0]
set dcp  [lindex $argv 1]
set out  [lindex $argv 2]
set top  [expr {[lindex $argv 3] ne "" ? [lindex $argv 3] : 40}]
if {$insts eq ""} { error "usage: make weigh INST=<hier path> [DCP=...] [OUT=...] [TOP=40]" }
if {$dcp eq ""} {
    foreach c [lmap s {postroute_physopted postroute_physopt routed placed} {string cat $::env(PROJECT) _ $s .dcp}] {
        if {[file exists $rundir/$c]} { set dcp $rundir/$c; break }
    }
}
if {$dcp eq ""} { error "no impl_1 checkpoint found -- pass DCP=" }
if {$out eq ""} { set out weigh.txt }
puts "WEIGH: reading $dcp"
open_checkpoint $dcp

proc key_of {n pfx} {
    set n [string range $n [string length $pfx] end]
    regsub {_i_[0-9]+(__[0-9]+)?$} $n "" n          ;# a LUT's ordinal under its net
    regsub {(_rep|_replica)(__[0-9]+)?$} $n "" n    ;# replication
    regsub {__[0-9]+$} $n "" n
    regsub -all {\[[0-9]+\]} $n "" n                ;# bit and element indices
    regsub {_reg(_[0-9]+)*$} $n "" n                ;# a register
    regsub {_(carry|CARRY)[0-9_]*$} $n "" n
    return $n
}
set fh [open $out w]
foreach inst $insts {
    set pfx [expr {$inst eq "." ? "" : "$inst/"}]    ;# "." is the top: an out-of-context module
    set lut [dict create]; set ff [dict create]; set ram [dict create]
    set nl 0; set nf 0; set nr 0
    foreach c [get_cells -hierarchical -quiet -filter "NAME =~ $pfx* && IS_PRIMITIVE"] {
        set r [get_property REF_NAME $c]
        if {[string match LUT* $r]} {
            dict incr lut [key_of [get_property NAME $c] $pfx]; incr nl
        } elseif {[regexp {^FD[CPRS]E$} $r]} {
            dict incr ff [key_of [get_property NAME $c] $pfx]; incr nf
        } elseif {[string match RAMD* $r] || [string match RAMS* $r]} {
            set n [get_property NAME $c]
            regsub {/(RAM[A-H]|RAM[A-H]_D1|SP|DP|[A-H]PORT)$} $n "" n
            dict incr ram [key_of $n $pfx]; incr nr
        }
    }
    puts $fh "\nWEIGH $inst  ($dcp)"
    puts $fh [format "  %d LUTs (logic), %d LUTRAM primitives, %d flops" $nl $nr $nf]
    foreach {title d} [list "LUTs by the net they drive" $lut "LUTRAM by array" $ram "flops by register" $ff] {
        puts $fh "\n== $title"
        set rows [lsort -integer -decreasing -stride 2 -index 1 [dict get $d]]
        set i 0; set rest 0
        foreach {k v} $rows {
            if {$i < $top} { puts $fh [format "%7d  %s" $v $k] } else { incr rest $v }
            incr i
        }
        if {$rest} { puts $fh [format "%7d  (%d more)" $rest [expr {$i - $top}]] }
    }
}
close $fh
puts "WEIGH: wrote $out"
