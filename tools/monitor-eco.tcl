# tools/monitor-eco.sh's Vivado half: load a new monitor image into a routed checkpoint's boot SRAM
# and write the checkpoint and the bitstream. Placement and routing are untouched: only INIT strings
# change, so the build's timing stands. Refuses unless the checkpoint holds exactly the old image
# under lmem-init.py's packing, and checks the new image the same way before writing.
#   -tclargs <routed.dcp> <old monitor.bin> <new monitor.bin> <out dir> <bitstream name>
lassign $argv dcp old new out bitname
set here [file dirname [file normalize [info script]]]
open_checkpoint $dcp
set cells [lsort -dictionary [get_cells -hier -filter {PRIMITIVE_GROUP == BLOCKRAM && NAME =~ *lmem_reg_bram_*}]]
if {[llength $cells] != 57} { error "monitor-eco: [llength $cells] lmem block RAMs, lmem-init.py packs 57" }
regsub {[0-9]+$} [lindex $cells 0] {} prefix
proc dump {cells f} {
    set fh [open $f w]
    foreach c $cells {
        puts $fh "cell [get_property NAME $c]"
        for {set i 0} {$i < 128} {incr i} { set k [format INIT_%02X $i];  puts $fh "  $k [get_property $k $c]" }
        for {set i 0} {$i < 16} {incr i}  { set k [format INITP_%02X $i]; puts $fh "  $k [get_property $k $c]" }
    }
    close $fh
}
proc check {f bin what} {
    if {[catch {exec python3 $::here/lmem-init.py check $f $bin} msg]} { error "monitor-eco: $what: $msg" }
    puts "monitor-eco: $what: $msg"
}
dump $cells $out/lmem-before.txt
check $out/lmem-before.txt $old "the checkpoint holds the old monitor"
set fh [open $out/lmem-new.tcl w]
puts $fh [exec python3 $here/lmem-init.py tcl $new $prefix]
close $fh
source -notrace $out/lmem-new.tcl
dump $cells $out/lmem-after.txt
check $out/lmem-after.txt $new "the checkpoint now holds the new monitor"
set wns [get_property SLACK [get_timing_paths -setup -max_paths 1]]
set whs [get_property SLACK [get_timing_paths -hold -max_paths 1]]
puts "monitor-eco: WNS $wns WHS $whs"
if {$wns < 0 || $whs < 0} { error "monitor-eco: timing not met (WNS $wns, WHS $whs)" }
write_checkpoint -force $out/routed.dcp
write_bitstream -force $out/$bitname
puts "monitor-eco: wrote $out/$bitname"
