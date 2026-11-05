# report_util.tcl -- open a routed checkpoint and dump device utilization and the memory census.
# Usage: make util [DCP=<checkpoint>] [OUT=<dir>]   (never call vivado directly; the Makefile sets
#        LD_LIBRARY_PATH). Without DCP, the newest impl_1 checkpoint; without OUT, impl_1's directory.
#
# Exists because "is the device full?" is the first question to ask before doing any more
# critical-path surgery: at 65-83% route on every failing path (rule I7), utilization and
# placement pressure decide the answer, not logic depth. `make` produces no utilization
# report of its own, so this was being answered by guesswork.
set rundir $::env(PROJECT).runs/impl_1
set dcp [lindex $argv 0]
if {$dcp eq ""} {
    foreach c [lmap s {postroute_physopted postroute_physopt routed placed} {string cat $::env(PROJECT) _ $s .dcp}] {
        if {[file exists $rundir/$c]} { set dcp $rundir/$c; break }
    }
}
if {$dcp eq ""} { error "no impl_1 checkpoint found -- run `make` first" }
if {[lindex $argv 1] ne ""} { set rundir [lindex $argv 1]; file mkdir $rundir }
puts "UTIL: reading $dcp"
open_checkpoint $dcp
report_utilization -file $rundir/$::env(PROJECT)_utilization.rpt
puts "UTIL: wrote $rundir/$::env(PROJECT)_utilization.rpt"
# Per instance, four levels down: what each structure costs (the PRF shards, the schedulers, the ROB).
report_utilization -hierarchical -hierarchical_depth 4 -file $rundir/$::env(PROJECT)_utilization_hier.rpt
puts "UTIL: wrote $rundir/$::env(PROJECT)_utilization_hier.rpt"
foreach l [split [report_utilization -return_string] "\n"] {
    if {[regexp {^\| (CLB LUTs|CLB Registers|CLB  |LUT as Logic|LUT as Memory|Block RAM Tile|DSPs|CARRY8)} $l]} { puts "UTIL| $l" }
}
# The memory census: every inferred array by what it costs. A distributed RAM's cost is its LUTs
# (one per RAMD/RAMS primitive, so a RAM64M8 counts 8); a block RAM's its RAMB36 equivalents. The
# array is the cell name with Vivado's bit-slice suffix stripped, so the 8 RAM64M8 of a 64 x 56
# array are one row. LUTs in distributed RAM are the cheapest congestion relief there is: each one
# moved to a block RAM leaves a CLB to the logic around it.
proc array_of {c} {
    set n [get_property NAME $c]
    regsub {/(RAM[A-H]|RAM[A-H]_D1|SP|DP|[A-H]PORT)$} $n "" n
    regsub {_reg_[0-9]+_[0-9]+(_[0-9]+_[0-9]+)?$} $n "" n
    regsub {_reg_(r[0-9]+_)?[0-9]+_[0-9]+.*$} $n "" n
    regsub {_reg(_[0-9]+)*$} $n "" n
    return $n
}
array unset lr; array unset br; array unset dp
foreach c [get_cells -hier -quiet -filter {IS_PRIMITIVE && (REF_NAME =~ RAMD* || REF_NAME =~ RAMS*)}] {
    set k [array_of $c]; incr lr($k)
    # the depth from Vivado's slice name, mem_reg_<first>_<last>_<bits>: the deepest slice wins
    if {[regexp {_reg_([0-9]+)_([0-9]+)_[0-9]+_[0-9]+} [get_property NAME $c] -> lo hi]} {
        set d [expr {$hi + 1}]
        if {![info exists dp($k)] || $d > $dp($k)} { set dp($k) $d }
    }
}
foreach c [get_cells -hier -quiet -filter {IS_PRIMITIVE && (REF_NAME =~ RAMB36* || REF_NAME =~ RAMB18* || REF_NAME =~ URAM288*)}] {
    set r [get_property REF_NAME $c]
    set w [expr {[string match RAMB18* $r] ? 0.5 : 1.0}]
    if {[string match URAM* $r]} { set k "[array_of $c] (URAM)" } else { set k [array_of $c] }
    if {![info exists br($k)]} { set br($k) 0.0 }
    set br($k) [expr {$br($k) + $w}]
}
set fh [open $rundir/$::env(PROJECT)_memories.rpt w]
set tot 0; foreach k [array names lr] { incr tot $lr($k) }
puts $fh "distributed RAM: $tot LUTs in [llength [array names lr]] arrays (largest first)"
puts $fh "   LUTs  depth  array (depth: entries, from the slice names; ? when Vivado's name carries none)"
foreach k [lsort -command {apply {{a b} {upvar lr lr; expr {$lr($b) - $lr($a)}}}} [array names lr]] {
    puts $fh [format "%7d  %5s  %s" $lr($k) [expr {[info exists dp($k)] ? $dp($k) : "?"}] $k]
}
set btot 0.0; foreach k [array names br] { set btot [expr {$btot + $br($k)}] }
puts $fh "\nblock RAM / UltraRAM: $btot RAMB36 equivalents (URAMs counted as tiles) in [llength [array names br]] arrays"
foreach k [lsort -command {apply {{a b} {upvar br br; expr {$br($b) > $br($a) ? 1 : ($br($b) < $br($a) ? -1 : 0)}}}} [array names br]] { puts $fh [format "%7.1f  %s" $br($k) $k] }
close $fh
report_ram_utilization -file $rundir/$::env(PROJECT)_ram_utilization.rpt
puts "UTIL: wrote $rundir/$::env(PROJECT)_memories.rpt ($tot distributed-RAM LUTs) and $::env(PROJECT)_ram_utilization.rpt"
