# ila_strand.tcl -- arm the ILA on the mtvec-stranding event and dump the window.
#
# Trigger: the named 1-bit probe `mtvec_stranded` (rk_xcku5p.v), which rises once mtvec has
# sat at OpenSBI's __sbi_expected_trap for >2000 cycles -- i.e. an MPRV-accessor window whose
# restoring `csrw mtvec` never took effect. Almost the whole buffer is PRE-trigger history so
# the capture contains the window itself.
#
# The point of the capture is probe5/probe6 (probe_csrop / probe_csrop_v): every system op
# that reached the CSR unit, tagged with its PC in [63:32]. That says directly whether the
# install (csrrw at 0x8000e828) and the restore (csrw at 0x8000e838) executed -- as opposed
# to probe1, which is the FETCH PA and runs ahead of execution.
#
#   Usage: vivado -mode batch -source ila_strand.tcl [-tclargs <out.csv>]

set here   [file dirname [file normalize [info script]]]
set ltx    [file join $here rk_xcku5p.runs/impl_1/debug_nets.ltx]
set outcsv [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] \
                                      : "/tmp/ila_strand.csv"}]

if {![file exists $ltx]} { error "debug_nets.ltx not found: $ltx" }

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

# Dedicated 1-bit probe, so no concat-splitting/constant-trimming games (see rk_xcku5p.v).
set ps [get_hw_probes mtvec_stranded -of $ila]
set_property TRIGGER_COMPARE_VALUE eq1'b1 $ps
set_property CONTROL.TRIGGER_POSITION 8000 $ila

run_hw_ila $ila
wait_on_hw_ila -timeout 3600 $ila
upload_hw_ila_data $ila
write_hw_ila_data -csv_file -force $outcsv [current_hw_ila_data]
puts "STRAND-TRIG-DONE -> $outcsv"

close_hw_target
disconnect_hw_server
close_hw_manager
