# ila_probe_status.tcl -- connect, report the hw_ila's real property names, and take an
# IMMEDIATE (-trigger_now) capture of the CURRENT hardware state.
#
# Two jobs: (1) tell us which status property this Vivado actually exposes (CORE_STATUS
# does not exist here), so the armed-wait script can poll the right one; (2) freeze-frame
# a wedged board -- the sticky parity bit in probe1 answers "did any cache array ever
# return bad data" even if the error pulse itself was missed.
set here   [file dirname [file normalize [info script]]]
set ltx    [file join $here rk_xcku5p.runs/impl_1/debug_nets.ltx]
set outcsv [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] \
                                      : [file join $here ila_now.csv]}]

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xcku5p*] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev

set ila [lindex [get_hw_ilas] 0]
puts "=== hw_ila properties ==="
report_property $ila

puts "=== immediate capture ==="
run_hw_ila -trigger_now $ila
wait_on_hw_ila -timeout 1 $ila
upload_hw_ila_data $ila
write_hw_ila_data -csv_file -force $outcsv [current_hw_ila_data]
puts "NOW-CAPTURE-DONE -> $outcsv"
