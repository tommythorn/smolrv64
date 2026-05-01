# Timing constraints for the CVFPU clock-domain bridge.
#
# This is a Tcl hook rather than XDC because Vivado's XDC parser rejects normal
# Tcl control flow. The hook runs after the implementation design is opened.

set core_clocks [get_clocks -quiet -of_objects [get_nets -quiet ui_clk]]
set fpu_clocks  [get_clocks -quiet -of_objects [get_nets -quiet fpu_clk]]
if {[llength $core_clocks] && [llength $fpu_clocks]} {
   set_clock_groups -asynchronous -group $core_clocks -group $fpu_clocks
}

set cvfpu_core_req_regs  [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/core_req_q_reg*]
set cvfpu_fpu_req_regs   [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/fpu_req_q_reg*]
set cvfpu_fpu_resp_regs  [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/fpu_resp_*_q_reg*]
set cvfpu_core_resp_regs [get_cells -quiet -hierarchical *smolrv64_inst/cvfpu_inst/core_*_q_reg*]

if {[llength $cvfpu_core_req_regs] && [llength $cvfpu_fpu_req_regs]} {
   set_false_path -from $cvfpu_core_req_regs -to $cvfpu_fpu_req_regs
}
if {[llength $cvfpu_fpu_resp_regs] && [llength $cvfpu_core_resp_regs]} {
   set_false_path -from $cvfpu_fpu_resp_regs -to $cvfpu_core_resp_regs
}
