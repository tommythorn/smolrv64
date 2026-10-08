# flash.tcl -- write a bitstream into the board's configuration flash, so it is the design the
# FPGA loads at power-on (mode pins M[2:0] = 001, Master SPI). The flash is a Macronix
# MX25U51245G (512 Mib, 1.8 V, bank 0 CFGBVS low), used x4 from address 0, as rk_xcku5p.xdc's
# SPI_BUSWIDTH 4 / CONFIG_MODE SPIx4 build it for. A compressed bitstream fits 24-bit addressing.
#
# Usage: vivado -mode batch -source flash.tcl [-tclargs <bitfile>]
# Erases, programs and verifies the flash through Vivado's indirect-programming core (this
# replaces the running design), then pulses PROGRAM_B so the FPGA boots from the flash.

set default_bit [file join [file dirname [info script]] rk_xcku5p.runs/impl_1/rk_xcku5p.bit]
set bitfile $default_bit
if {[llength $argv] > 0} { set bitfile [lindex $argv 0] }
set bitfile [file normalize $bitfile]
if {![file exists $bitfile]} { error "Bitfile not found: $bitfile" }
set mcs [file rootname $bitfile].mcs
puts "Flashing with: $bitfile"

write_cfgmem -force -format mcs -size 64 -interface SPIx4 -loadbit "up 0x0 $bitfile" -file $mcs

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev

set mem [create_hw_cfgmem -hw_device $dev [lindex [get_cfgmem_parts {mx25u51245gxxj-spi-x1_x2_x4}] 0]]
set_property PROGRAM.ADDRESS_RANGE  {use_file} $mem
set_property PROGRAM.FILES          [list $mcs] $mem
set_property PROGRAM.PRM_FILE       {} $mem
set_property PROGRAM.UNUSED_PIN_TERMINATION {pull-none} $mem
set_property PROGRAM.BLANK_CHECK    0 $mem
set_property PROGRAM.ERASE          1 $mem
set_property PROGRAM.CFG_PROGRAM    1 $mem
set_property PROGRAM.VERIFY         1 $mem
set_property PROGRAM.CHECKSUM       0 $mem
create_hw_bitstream -hw_device $dev [get_property PROGRAM.HW_CFGMEM_BITFILE $dev]
program_hw_devices $dev
refresh_hw_device -update_hw_probes false $dev
program_hw_cfgmem -hw_cfgmem $mem

boot_hw_device $dev
puts "\nFlash programmed and verified; the FPGA is booting from it."
close_hw_target
disconnect_hw_server
close_hw_manager
