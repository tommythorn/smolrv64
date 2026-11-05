# report_floorplan.tcl -- every placed cell's site and BEL, and the device's site columns, for
# tools/floorplan.py to draw coloured by block.  Usage: make floorplan [DCP=<checkpoint>] [OUT=<dir>]
#
#   <OUT>/cells.tsv   name  ref  site  bel          one line per placed leaf cell
#   <OUT>/sites.tsv   site  clock-region            every SLICE, RAMB36, DSP48E2 and URAM288 site
#   <OUT>/cols.tsv    kind  x  grid-column          where each site column sits in the device grid
set rundir $::env(PROJECT).runs/impl_1
set dcp [lindex $argv 0]
set out [lindex $argv 1]
if {$dcp eq ""} {
    foreach c [lmap s {postroute_physopted postroute_physopt routed placed} {string cat $::env(PROJECT) _ $s .dcp}] {
        if {[file exists $rundir/$c]} { set dcp $rundir/$c; break }
    }
}
if {$dcp eq ""} { error "no impl_1 checkpoint found -- pass DCP=" }
if {$out eq ""} { set out floorplan }
file mkdir $out
puts "FLOORPLAN: reading $dcp"
open_checkpoint $dcp

set fh [open $out/cells.tsv w]
set cells [get_cells -hier -filter {IS_PRIMITIVE && PRIMITIVE_LEVEL != MACRO}]
foreach n [get_property NAME $cells] r [get_property REF_NAME $cells] l [get_property LOC $cells] b [get_property BEL $cells] {
    if {$l ne ""} { puts $fh "$n\t$r\t$l\t$b" }
}
close $fh

set sites [get_sites -filter {NAME =~ SLICE_X* || NAME =~ RAMB36_X* || NAME =~ DSP48E2_X* || NAME =~ URAM288_X*}]
set fh [open $out/sites.tsv w]
set col [dict create]
foreach s $sites cr [get_property CLOCK_REGION $sites] {
    puts $fh "$s\t$cr"
    regexp {^(\w+)_X(\d+)Y} $s -> kind x
    if {![dict exists $col "$kind $x"]} { dict set col "$kind $x" $s }
}
close $fh
set fh [open $out/cols.tsv w]
dict for {k s} $col {
    puts $fh "[join $k \t]\t[get_property GRID_POINT_X [get_tiles -of_objects [get_sites $s]]]"
}
close $fh
puts "FLOORPLAN-DONE [llength $cells] cells, [llength $sites] sites -> $out"
