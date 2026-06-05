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
