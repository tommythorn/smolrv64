# The SoC's constraints (platforms/xcku5p/smolrv64_shell.v), every board's: its clock crossings,
# its clock root and its pblocks. Each board's own .xdc has its pins, its PHYs' timing and its
# configuration; build.tcl adds this file to every board's project.

# Timing waiver for DDR4 calibration IP internal signal
create_waiver -internal -user ddr4_v2_2_19 -scope -type METHODOLOGY -id {TIMING-17} -description "Ignore the TIMING-17 Critical Warning for sl_iport_i" -objects [get_pins -quiet -leaf -of [get_nets -quiet shell/u_ddr4_0/inst/u_ddr4_mem_intfc/u_ddr_cal_top/u_ddr_cal/U_XSDB_SLAVE/sl_iport_i*] -filter {DIRECTION==IN}]

# CVFPU generated-clock and CDC constraints live in cvfpu_timing.tcl.  They
# need normal Tcl conditionals, which Vivado's XDC parser rejects.

# MMIO clock-bridge FIFO reset is an async crossing (fifo_reset_q on ui_clk ->
# XPM async-FIFO reset synchronizers in the core_clk/ui_clk domains). Because
# core_clk/ui_clk are synchronous BUFGCE_DIV derivatives, Vivado otherwise times
# this reset as a single-cycle path; it was the chronic worst path (WNS=0.000)
# pinned by routing, not logic. The XPM reset block re-synchronizes assertion
# internally, so this path is a false path. Removing it recovers ~0.05 ns and
# lifts the design WNS off zero (next path is the core npc->frontend path).
set_false_path \
    -from [get_cells shell/mmio_clock_bridge_inst/fifo_reset_q_reg] \
    -through [get_pins -hier -filter {NAME =~ *mmio_clock_bridge_inst*xpm_fifo_rst_inst*/D}]


# ---- probe_clk clock root ------------------------------------------------------------
# MEASURED 2026-08-20, same RTL, DIV8=48 (166.67 MHz), only the clock source differing:
#
#   probe_mmcm (MMCM)   root X1Y1   clock-net routing 0.749 ns   WNS -0.328   1617 failing
#   BUFGCE_DIV(ui_clk)  root X2Y1   clock-net routing 1.466 ns   WNS -0.857   4121 failing
#
# Identical logic; the placer simply clustered probe_core around a worse root and the same
# fetch-cone family went from 87 paths to 614.  The MMCM was never buying frequency
# granularity (see the knob comment in rk_xcku5p.v) -- it was buying THIS, and a root is a
# constraint, not a primitive.  Pin it so a BUFGCE_DIV gets the same placement.
#
# Found by object, not by literal name, for the same reason as the CDC constraints below:
# an auto-derived name follows the net at the source pin and a lookup that matches nothing
# fails SILENTLY.
set_property USER_CLOCK_ROOT X1Y1 [get_nets -of_objects [get_pins -hier -filter {NAME =~ *probe_clk_buf/O}]]

# fbdiag self-reset: probe_clk -> ui_clk 2-FF synchronizer.  These are RELATED clocks, so
# without this exception Vivado times the crossing, and an off-integer period ratio collapses
# the requirement -- that is what limited the design to the N x 3.000 ns ladder before.
set_false_path -to [get_pins -hier -filter {NAME =~ *fbdiag_rst_sync_reg[0]/D}]

# ddr4_mem (smolrv64_shell u_mem), our DDR4 controller, sits where the IP's own controller, UI and AXI
# shim sat: the two clock regions of the DDR4 pins. The PHY-only IP is half the old IP's cells, and
# left free the placer spread the controller toward the devices and reflowed the core around it
# (f30ebc84/a47f5c1f: probe_clk -0.14 to -0.32 over most of the core's families).
create_pblock pb_ddr4_mem
add_cells_to_pblock [get_pblocks pb_ddr4_mem] [get_cells shell/u_mem]
resize_pblock [get_pblocks pb_ddr4_mem] -add {CLOCKREGION_X0Y0:CLOCKREGION_X0Y1}

# rv_l2 (rv_soc_top u_l2), the L2, sits along its data array: all 64 URAM288, the part's one URAM
# column (between SLICE_X79 and SLICE_X80, the die's full height through the core), with the slices
# either side and the two block-RAM columns beside it for its tags. Left free, its logic spread over
# the die between the column, the caches' arbiter and the DDR crossing (SLICE X0-X110).
create_pblock pb_l2
add_cells_to_pblock [get_pblocks pb_l2] [get_cells shell/probe_core/u_l2]
resize_pblock [get_pblocks pb_l2] -add {SLICE_X74Y0:SLICE_X85Y239 URAM288_X0Y0:URAM288_X0Y63 RAMB36_X6Y0:RAMB36_X7Y47 RAMB18_X6Y0:RAMB18_X7Y95}

# ddr_port_sync (rk_xcku5p.v probe_sync): the core's memory port into ui_clk as registers.
# probe_clk is BUFGCE_DIV(ui_clk) / 2, its edges ~2.7 ns after ui_clk's, so Vivado's default check
# of a probe_clk -> ui_clk path lands on the ui_clk edge 0.3 ns after the launch. The crossing
# captures at the edge after that (its CAP edge, found from probe_clk's own toggle sampled by
# ui_clk's falling edge), so its capture registers -- the request's capture register (c_*) and that
# falling-edge sample -- are multicycle 2. The paths back to probe_clk launch at the other ui_clk
# edges and need no exception.
set_multicycle_path -setup 2 -end -from [get_clocks probe_clk] \
    -to [get_cells -hier -filter {NAME =~ *probe_sync/c_*_reg* || NAME =~ *probe_sync/m_tf_reg}]
set_multicycle_path -hold 1 -end -from [get_clocks probe_clk] \
    -to [get_cells -hier -filter {NAME =~ *probe_sync/c_*_reg* || NAME =~ *probe_sync/m_tf_reg}]

# The pixel clock is an MMCM output fed from ui_clk, so Vivado would time ui_clk <-> pixel clock
# as RELATED clocks at an off-integer ratio (and DRP retunes it at run time anyway). Every
# crossing is designed as asynchronous (vga_scanout's header):
#   ui_clk -> pixel: CTRL.enable through a 2-FF synchronizer; the mode registers quasi-static.
#   pixel -> ui_clk: one toggle a line each -- a line shown (done_tog), a line scanned before it was
#     whole (late_tog) -- and the scan's running (en_p), each through a synchronizer.
#   LOCKED -> both domains: an asynchronous status.
# Objects, not literal clock names (see the CDC note above: a lookup that matches nothing
# fails silently).
set vga_ui_clk  [get_clocks -of_objects [get_nets ui_clk]]
set vga_pix_clk [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *vga_mmcm_inst/CLKOUT0}]]
set_false_path -from $vga_ui_clk -to $vga_pix_clk
set_false_path -to [get_pins -hier -filter {NAME =~ *vga_scanout_inst/done_s_reg[0]/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *vga_scanout_inst/late_s_reg[0]/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *vga_scanout_inst/en_a_reg[0]/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *vga_scanout_inst/locked_s_reg[0]/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *vga_pix_rst_s_reg[0]/D}]
