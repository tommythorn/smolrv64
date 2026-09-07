# Assert that probe_clk resolves, and record what it actually is.
#
# This is a Tcl hook rather than XDC for the same reason cvfpu_timing.tcl is:
# Vivado's XDC parser rejects normal Tcl control flow (an `if` in an .xdc is a
# CRITICAL WARNING and is then silently ignored, which is worse than useless for
# a guard).  The hook runs after the implementation design is opened.
#
# WHY THIS EXISTS.  rk_xcku5p.xdc carries two set_max_delay -datapath_only
# constraints on the probe_clk <-> ui_clk 512-bit line crossing.  Without them
# Vivado times that bus as a single 3 ns ui_clk hop -- zero logic levels, ~90%
# routing -- and that over-constraint, not core logic, was the sole thing keeping
# the in-order core off 111 MHz.
#
# The trap: those constraints locate the clock with get_clocks.  An auto-derived
# clock is named after the net at its source pin, so any change to how probe_clk
# is generated can rename it -- and a get_clocks that matches nothing does not
# fail.  It returns an empty list, the constraint binds to nothing, and the build
# quietly reverts to the over-constrained timing that cost us a whole frequency
# rung.  Nothing in the log says so.  This hook turns that silence into an error.

set probe_clk_pins [get_pins -quiet -hier -filter {NAME =~ *probe_clk_buf/O}]
if {![llength $probe_clk_pins]} {
   error "probe_clk_check: no probe_clk_buf/O pin in the design -- the clock buffer was renamed."
}

set probe_clks [get_clocks -quiet -of_objects $probe_clk_pins]
if {[llength $probe_clks] != 1} {
   error "probe_clk_check: expected exactly one clock on probe_clk_buf/O, found\
 [llength $probe_clks] ('$probe_clks').  The probe_clk <-> ui_clk CDC max_delay\
 constraints in rk_xcku5p.xdc would bind to nothing, restoring the over-constraint\
 that kept this core off 111 MHz.  Run report_clocks and fix the lookup."
}

# Record it.  The build log should always be able to answer "what clock did this
# bitstream actually run at" without anyone having to infer it from a knob.
set probe_period [get_property PERIOD $probe_clks]
puts [format "probe_clk_check: clock '%s' on probe_clk_buf/O, period %.3f ns = %.2f MHz." \
         [get_property NAME $probe_clks] $probe_period [expr {1000.0 / $probe_period}]]

# The MMCM must have been given a legal operating point.  Vivado does check this
# itself, but it reports it as one line among thousands; a sweep that walks the VCO
# out of spec should stop the build, not be found later in a log.
set probe_mmcm [get_cells -quiet -hier -filter {NAME =~ *probe_mmcm}]
if {[llength $probe_mmcm] == 1} {
   set vco [expr {(1000.0 / [get_property CLKIN1_PERIOD $probe_mmcm]) \
                  * [get_property CLKFBOUT_MULT_F $probe_mmcm] \
                  / [get_property DIVCLK_DIVIDE $probe_mmcm]}]
   puts [format "probe_clk_check: MMCM VCO %.1f MHz, CLKOUT0_DIVIDE_F %.3f." \
            $vco [get_property CLKOUT0_DIVIDE_F $probe_mmcm]]
   if {$vco < 600.0 || $vco > 1200.0} {
      error [format "probe_clk_check: MMCM VCO %.1f MHz is outside the 600-1200 MHz\
 window that holds for every speed grade of this part.  Fix CLKFBOUT_MULT_F /\
 DIVCLK_DIVIDE in rk_xcku5p.v." $vco]
   }
}
