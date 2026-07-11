# Timing constraints for the CVFPU clock-domain bridge.
#
# This is a Tcl hook rather than XDC because Vivado's XDC parser rejects normal
# Tcl control flow. The hook runs after the implementation design is opened.

set core_clk_src_pins [get_pins -quiet core_clk_buf/I]
set core_clk_out_pins [get_pins -quiet core_clk_buf/O]
if {[llength $core_clk_src_pins] && [llength $core_clk_out_pins] &&
    ![llength [get_clocks -quiet -of_objects $core_clk_out_pins]]} {
   create_generated_clock -name core_clk_div2 -divide_by 2 \
      -source $core_clk_src_pins $core_clk_out_pins
}

set core_clocks [get_clocks -quiet -of_objects [get_nets -quiet core_clk]]
set fpu_clk_src_pins [get_pins -quiet fpu_clk_buf/I]
set fpu_clk_out_pins [get_pins -quiet fpu_clk_buf/O]
if {[llength $fpu_clk_src_pins] && [llength $fpu_clk_out_pins] &&
    ![llength [get_clocks -quiet -of_objects $fpu_clk_out_pins]]} {
   create_generated_clock -name fpu_clk_div4 -divide_by 4 \
      -source $fpu_clk_src_pins $fpu_clk_out_pins
}

set fpu_clocks  [get_clocks -quiet -of_objects [get_nets -quiet fpu_clk]]
# NOTE: core_clk and fpu_clk are NOT asynchronous.  Both are BUFGCE_DIV outputs
# of the same ui_clk (core = ui_clk/2 = 6 ns, fpu = ui_clk/4 = 12 ns) with a
# common CLR, so they are phase-aligned and integer 2:1 -- a synchronous
# multi-rate interface, not a CDC.  A `set_clock_groups -asynchronous` here was
# wrong: it left every core<->fpu path untimed (P&R could route the FPU result
# bus past its capture window -> stale FP data, non-deterministically, invisible
# to setup/hold STA) AND it outranks the set_max_delay below.  Time the
# interface normally and bound the data buses (below).  The req/resp toggles are
# short FF->FF paths that meet the synchronous requirement on their own.

set cvfpu_core_req_regs  [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/core_req_q_reg*]
set cvfpu_fpu_req_regs   [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/fpu_req_q_reg*]
set cvfpu_fpu_resp_regs  [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/fpu_resp_*_q_reg*]
set cvfpu_core_resp_regs [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/core_*_q_reg*]

# The req/resp buses are an MCP-style CDC handshake: a toggle is 2-FF
# synchronized across the domains and gates a clock-enable capture of the data.
# The data must therefore be BOUNDED (arrive within the handshake window), NOT
# false-pathed.  A plain set_false_path leaves these buses untimed, so P&R can
# route a result bit longer than the capture window and the core latches stale
# FP data -- non-deterministically, invisibly to setup/hold STA.  Bound the data
# to one core_clk period (6 ns); the paths are FF->FF (0 logic levels) so this is
# trivially routable.  -datapath_only excludes clock skew, the correct MCP form.
set cvfpu_data_bound 6.000
if {[llength $cvfpu_core_req_regs] && [llength $cvfpu_fpu_req_regs]} {
   set_max_delay -datapath_only $cvfpu_data_bound \
      -from $cvfpu_core_req_regs -to $cvfpu_fpu_req_regs
}
if {[llength $cvfpu_fpu_resp_regs] && [llength $cvfpu_core_resp_regs]} {
   set_max_delay -datapath_only $cvfpu_data_bound \
      -from $cvfpu_fpu_resp_regs -to $cvfpu_core_resp_regs
}

# ---------------------------------------------------------------------------
# core_clk hold margin.
#
# The D-cache LUTRAM (mem1, RAMD64E) write-address path
#   cache_bram_word_idx_reg[*]_rep__* -> mem1_*/WADR
# closes at only +0.010 ns hold under the aggressive (ExtraTimingOpt +
# AggressiveExplore) place/route that was needed to pull the frontend SETUP
# path positive. 10 ps is inside STA's clock-skew error band (this path sees
# 0.179 ns of core_clk distribution skew), so it goes negative on real silicon
# -> the cache write address is captured wrong -> deterministic, temperature-
# insensitive data corruption (wrong operands/pointers in userspace, mangled
# monitor XMODEM bytes). The router leaves it at +0.010 because STA reports it
# as "met" and never tries harder.
#
# Add hold pessimism on core_clk so the tool must route in real hold margin.
# Hold fixing inserts delay on the fast (short) paths and does not lengthen the
# setup-critical (long) paths, so this should not regress WNS -- but verify both
# WNS and WHS after the build (this is a zero-margin design). Tunable via env.
set core_clk_obj [get_clocks -quiet core_clk]
set hold_unc 0.100
if {[info exists env(HOLD_UNCERTAINTY)] && $env(HOLD_UNCERTAINTY) ne ""} {
   set hold_unc $env(HOLD_UNCERTAINTY)
}
if {[llength $core_clk_obj] && $hold_unc > 0} {
   puts "core_clk hold uncertainty (intra): $hold_unc ns"
   # MUST be the explicit -from/-to intra-clock form. The single-object form
   # `set_clock_uncertainty -hold X [get_clocks core_clk]` only sets INTER-clock
   # uncertainty and has zero effect on the core_clk->core_clk LUTRAM path
   # (verified: single-object form leaves the path at +0.010; -from/-to drops it
   # to -0.090, which forces route's hold-fixer to add real margin).
   set_clock_uncertainty -hold $hold_unc -from $core_clk_obj -to $core_clk_obj
}

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
