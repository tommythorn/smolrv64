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
