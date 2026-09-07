# Timing hooks beyond the XDC: constraints that need Tcl control flow (Vivado's XDC parser
# rejects it). The hook runs after the implementation design is opened. The file keeps its
# historical name (the .xpr's utils_1 fileset records it); the CVFPU clock-domain bridge it
# was written for left with the scalar core -- the FPU now runs at probe_clk inside the core.

# --- Ethernet PHY RX clock (eth_rxc, 125 MHz) ---------------------------------
# eth_rxc (BUFG -> gmii_rx_clk, BUFIO for the IDDRE1s) is fully asynchronous to
# the DDR ui_clk / core_clk / fpu_clk tree.  The only crossing is eth_tx_engine,
# which uses 2-FF (async_reg) synchronizers for the send/done handshake and an
# async-read distributed-RAM frame buffer (written in ui_clk, read in
# gmii_rx_clk while stable).  Declaring eth_rxc async to all other clocks keeps
# the tool from timing those CDC paths as if related.  A single -group makes it
# asynchronous to every other clock in the design.
set eth_clk [get_clocks -quiet -include_generated_clocks eth_rxc]
if {[llength $eth_clk]} {
   puts "eth_rxc declared asynchronous to all other clocks"
   set_clock_groups -asynchronous -group $eth_clk
} else {
   puts "WARNING: eth_rxc clock not found; skipping async clock grouping"
}

# Xilinx DDR4 MIG bid-FIFO const-0 outputs get merged into the GND const-0 net by
# opt_design's constant propagation (awid is tied 0), tripping MDRV-1 "multiple
# drivers" on a net where every driver is 0 -- electrically benign, opt created it
# itself. Downgrade so opt_design's DRC precondition proceeds.
if {[llength [get_drc_checks -quiet MDRV-1]]} {
   set_property SEVERITY {Warning} [get_drc_checks MDRV-1]
   puts "MDRV-1 downgraded to Warning (benign DDR4 bid-FIFO const-0 merge)"
}

# --- Optional floorplan: keep the core in a compact block ------------------------------
# `make place-report` on T1F2 (2026-09-06): the core's 69k cells sit over 10 clock regions and
# every near-critical path is 70-80% route -- twelve levels needing 4.7 ns of wire. Set
# PBLOCK_CORE to a clock-region range (e.g. CLOCKREGION_X1Y0:CLOCKREGION_X3Y1) to pin
# probe_core/core there; unset, nothing changes. An experiment knob, like PLACE_DIRECTIVE.
if {[info exists env(PBLOCK_CORE)] && $env(PBLOCK_CORE) ne ""} {
   set core_cell [get_cells -quiet probe_core/core]
   if {[llength $core_cell]} {
      puts "PBLOCK_CORE: probe_core/core -> $env(PBLOCK_CORE)"
      set pb [create_pblock pb_core]
      add_cells_to_pblock $pb $core_cell
      resize_pblock $pb -add $env(PBLOCK_CORE)
   } else {
      puts "PBLOCK_CORE set but probe_core/core not found; skipping"
   }
}
