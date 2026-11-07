# program.tcl — Program the FPGA via JTAG using Vivado hw_server
# Usage: vivado -mode batch -source program.tcl [-tclargs <bitfile>]

set default_bit [file join [pwd] $::env(PROJECT).runs/impl_1/$::env(PROJECT).bit]

set bitfile $default_bit
if {[llength $argv] > 0} { set bitfile [lindex $argv 0] }
set bitfile [file normalize $bitfile]

if {![file exists $bitfile]} {
    error "Bitfile not found: $bitfile\nRun 'make' first to build."
}
puts "Programming with: $bitfile"

open_hw_manager
connect_hw_server -allow_non_jtag
source [file join [file dirname [file normalize [info script]]] hw_target.tcl]
open_board_target   ;# JTAG=<cable serial> picks the board (hw_target.tcl)

set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
# No refresh before programming: it scans the running design's debug cores, and a design stopped
# by a readback capture answers that scan with a core hw_server rejects.
set_property PROGRAM.FILE $bitfile $dev
program_hw_devices $dev
refresh_hw_device $dev

puts "\nDevice programmed successfully."
close_hw_target
disconnect_hw_server
close_hw_manager
