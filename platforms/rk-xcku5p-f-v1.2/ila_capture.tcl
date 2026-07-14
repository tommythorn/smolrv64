# ila_capture.tcl -- headless "read the frozen wedge" ILA capture over JTAG.
#
# Use for a post-mortem of a wedged board (no GUI): the design must already be programmed
# with a bit built ILA_*=1 (debug_nets.ltx present), and the board should be sitting in the
# wedged steady state. Does an immediate -trigger_now capture (the levels are frozen, so no
# real trigger condition is needed) and writes the sample window to CSV.
#
#   Usage: vivado -mode batch -source ila_capture.tcl [-tclargs <out.csv>]
#   (or:   make ila-capture [CSV=<out.csv>])

set here   [file dirname [file normalize [info script]]]
set ltx    [file join $here rk_xcku5p.runs/impl_1/debug_nets.ltx]
set outcsv [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] \
                                      : [file join $here ila_capture.csv]}]

if {![file exists $ltx]} {
    error "debug_nets.ltx not found: $ltx\nBuild+program a bit with an ILA_* flag first."
}

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xcku5p*] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev

set ila [lindex [get_hw_ilas] 0]
puts "ILA: $ila"
puts "probes: [get_hw_probes -of $ila]"

# Immediate capture of the frozen state. TRIGGER_POSITION mid-window so the CSV shows
# whatever little motion (if any) precedes/surrounds the sample point.
set_property CONTROL.TRIGGER_POSITION 2048 $ila
run_hw_ila $ila -trigger_now
wait_on_hw_ila -timeout 60 $ila
upload_hw_ila_data $ila
write_hw_ila_data -csv_file -force $outcsv [current_hw_ila_data]
puts "ILA-CAPTURE-DONE -> $outcsv"

close_hw_target
disconnect_hw_server
close_hw_manager
