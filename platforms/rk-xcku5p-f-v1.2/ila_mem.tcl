# ila_mem.tcl -- ARMED capture of a memory-path wedge (a bit built with ILA_MEM=1).
#
# Both memory-path ILAs trigger on the port watchdog in rk_xcku5p.v (ila_trig: something owed at
# the core's port and no handshake for 2047 probe_clk cycles), the top bit of each core's probe0.
# The trigger sits at the end of the window, so each capture is the ~4000 samples leading into
# the stop. Arm it after the board boots, then run the workload; it waits until the trigger
# fires (or max_hours, when given, elapses) and writes <out>_p.csv (probe_clk, the core's port)
# and <out>_m.csv (ui_clk, the CDC's far end and the AXI masters and controller).
#
#   Usage: vivado -mode batch -source ila_mem.tcl [-tclargs <out> [<max_hours>]]
#   (or:   make ila-mem [CSV=<out>] [HOURS=<n>])

set here [file dirname [file normalize [info script]]]
set ltx  [file join $here rk_xcku5p.runs/impl_1/debug_nets.ltx]
set out  [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] : [file join $here ila_mem]}]
set max_hours [expr {[llength $argv] > 1 ? [lindex $argv 1] : 0}]

if {![file exists $ltx]} { error "debug_nets.ltx not found: $ltx\nBuild+program a bit with ILA_MEM=1 first." }

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xcku5p*] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev

# the two cores by instance name, and in each the trigger probe by its width (probe0)
set ilas {}
foreach side {p m} width {10 8} {
    set ila ""
    foreach cand [get_hw_ilas] {
        if {[string match "*u_ila_mem_$side*" [get_property CELL_NAME $cand]]} { set ila $cand }
    }
    if {$ila eq ""} { error "no u_ila_mem_$side core on the device: is this an ILA_MEM=1 bit?" }
    set tp ""
    foreach p [get_hw_probes -of $ila] {
        if {[get_property WIDTH $p] == $width} { set tp $p; break }
    }
    if {$tp eq ""} { error "u_ila_mem_$side has no $width-bit trigger probe" }
    set_property TRIGGER_COMPARE_VALUE "eq${width}'b1[string repeat X [expr {$width - 1}]]" $tp
    set_property CONTROL.TRIGGER_POSITION 4000 $ila
    puts "ILA $side: $ila  trigger probe: $tp"
    lappend ilas $side $ila
}
foreach {side ila} $ilas { run_hw_ila $ila }
puts "MEM-ARMED: waiting for the port watchdog[expr {$max_hours > 0 ? " (max ${max_hours}h)" : " (no deadline)"}]"
flush stdout

set p_ila [dict get $ilas p]
set waited 0
while {1} {
    wait_on_hw_ila -timeout 5 $p_ila
    catch {refresh_hw_device -quiet -update_hw_probes false [current_hw_device]}
    if {[catch {set st [get_property CORE_STATUS $p_ila]}]} { set st "armed" }
    if {[string match -nocase "*full*" $st] || [string match -nocase "*trigger*" $st]} { break }
    incr waited 5
    if {$max_hours > 0 && $waited >= $max_hours * 3600} {
        puts "MEM-NO-TRIGGER after ${max_hours}h: the port never stopped with something owed."
        exit 0
    }
    if {$waited % 300 == 0} { puts "  ... armed [expr {$waited / 60}] min, status: $st"; flush stdout }
}

foreach {side ila} $ilas {
    catch {wait_on_hw_ila -timeout 1 $ila}
    upload_hw_ila_data $ila
    write_hw_ila_data -csv_file -force ${out}_$side.csv [current_hw_ila_data]
    puts "MEM-TRIG-DONE $side -> ${out}_$side.csv"
}
