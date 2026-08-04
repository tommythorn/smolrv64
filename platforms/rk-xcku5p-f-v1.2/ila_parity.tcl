# ila_parity.tcl -- ARMED capture on a cache data-array integrity failure.
#
# Requires a bit built with ILA_PARITY=1 (which also builds -DCACHE_PARITY). The design
# checks a parity bit per cache-bank word on every bank read; probe0 (cache_par_err,
# {I$,D$}) pulses in the cycle an array returns a word that is not what was stored.
#
# That pulse is the trigger. The board's corruption is otherwise SILENT -- the kernel
# Oops arrives millions of cycles later, far outside any pre-trigger window -- so this
# is the only way to capture the moment itself. Trigger position is late in the buffer,
# so the capture is almost entirely PRE-trigger history: what the caches, fetch PA and
# LSU were doing in the ~4000 cycles leading to the bad read.
#
#   Usage: vivado -mode batch -source ila_parity.tcl [-tclargs <out.csv>]
#   (or:   make ila-parity [CSV=<out.csv>])
#
# Blocks until the trigger fires (1 hour timeout). Run it while the board boots.
# NO TRIGGER = the cache arrays never lied: a decisive negative that exonerates the
# data path on real BRAM and moves the hunt to the core/LSU/MMU.

set here   [file dirname [file normalize [info script]]]
set ltx    [file join $here rk_xcku5p.runs/impl_1/debug_nets.ltx]
set outcsv [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] \
                                      : [file join $here ila_parity.csv]}]

if {![file exists $ltx]} {
    error "debug_nets.ltx not found: $ltx\nBuild+program a bit with ILA_PARITY=1 first."
}

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xcku5p*] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev

# The parity core is the one carrying a probe named for cache_par_err; pick it
# explicitly rather than assuming index 0 (other ILA_* cores may also be present).
set ila ""
foreach cand [get_hw_ilas] {
    if {[llength [get_hw_probes -quiet -of $cand *par_err*]] > 0} { set ila $cand; break }
}
if {$ila eq ""} { set ila [lindex [get_hw_ilas] 0] }
puts "ILA: $ila"
puts "probes: [get_hw_probes -of $ila]"

set ps [lindex [get_hw_probes -of $ila *par_err*] 0]
# any error bit set: {I$,D$} != 00
set_property TRIGGER_COMPARE_VALUE neq2'b00 $ps
set_property CONTROL.TRIGGER_POSITION 3800 $ila

run_hw_ila $ila
wait_on_hw_ila -timeout 3600 $ila
upload_hw_ila_data $ila
write_hw_ila_data -csv_file -force $outcsv [current_hw_ila_data]
puts "PARITY-TRIG-DONE -> $outcsv"
