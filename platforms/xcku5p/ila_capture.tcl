# ila_capture.tcl -- headless "read the frozen wedge" ILA capture over JTAG.
#
# Use for a post-mortem of a wedged board (no GUI): the design must already be programmed
# with a bit built ILA_*=1 (debug_nets.ltx present), and the board should be sitting in the
# wedged steady state. Does an immediate -trigger_now capture of EVERY ILA the bit carries
# (the levels are frozen, so no real trigger condition is needed) and writes each sample window
# to <out>_<instance>.csv (ila_capture_u_ila_core.csv, ..._u_ila_mem_p.csv, ...).
#
#   Usage: vivado -mode batch -source ila_capture.tcl [-tclargs <out.csv>]
#   (or:   make ila-capture [CSV=<out.csv>])    JTAG=<cable serial> picks the board

set here   [pwd]
set ltx    [file join $here $::env(PROJECT).runs/impl_1/debug_nets.ltx]
set outcsv [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] \
                                      : [file join $here ila_capture.csv]}]

if {![file exists $ltx]} {
    error "debug_nets.ltx not found: $ltx\nBuild+program a bit with an ILA_* flag first."
}

open_hw_manager
connect_hw_server -allow_non_jtag
source [file join $here hw_target.tcl]
open_board_target   ;# JTAG=<cable serial> picks the board (hw_target.tcl)
set dev [lindex [get_hw_devices xcku5p*] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev

# Immediate capture of the frozen state, one CSV per ILA, named by its instance. TRIGGER_POSITION
# mid-window so the CSV shows whatever little motion (if any) precedes/surrounds the sample point.
set base [file rootname $outcsv]
if {[llength [get_hw_ilas]] == 0} { error "no ILA on the device: is this an ILA_*=1 bit?" }
foreach ila [get_hw_ilas] {
    set cell [lindex [split [get_property CELL_NAME $ila] /] end]
    set csv ${base}_$cell.csv
    puts "ILA: $ila ($cell)  probes: [get_hw_probes -of $ila]"
    set_property CONTROL.TRIGGER_POSITION 2048 $ila
    run_hw_ila $ila -trigger_now
    wait_on_hw_ila -timeout 60 $ila
    write_hw_ila_data -csv_file -force $csv [upload_hw_ila_data $ila]
    puts "ILA-CAPTURE-DONE $cell -> $csv"
}

close_hw_target
disconnect_hw_server
close_hw_manager
