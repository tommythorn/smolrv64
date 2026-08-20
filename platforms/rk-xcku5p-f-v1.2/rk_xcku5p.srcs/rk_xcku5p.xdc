# this is the constraints

# PL_CLK, a 50 MHz clock (doesn't work on my board)
# create_clock -period 20.0        [get_ports "pl_clk50"]
# set_property IOSTANDARD LVCMOS33 [get_ports "pl_clk50"]
# set_property PACKAGE_PIN AC13    [get_ports "pl_clk50"]

# SYS CLK 200 MHz
set_property PACKAGE_PIN T24 [ get_ports "sys_clk_p" ]
set_property PACKAGE_PIN U24 [ get_ports "sys_clk_n" ]
set_property IOSTANDARD DIFF_SSTL12 [ get_ports "sys_clk_p" ]
set_property IOSTANDARD DIFF_SSTL12 [ get_ports "sys_clk_n" ]
# The DDR4 IP XDC creates the 200 MHz input clock on this port.

# QSFP28 CLK 156.25 MHz
# create_clock -period 6.05 [get_ports "gt_clk156p25_p"]
# TBD, NOT THIS set_property IOSTANDARD DIFF_SSTL12 [ get_ports "gt_clk156p25_p" ]
# TBD, NOT THIS set_property IOSTANDARD DIFF_SSTL12 [ get_ports "gt_clk156p25_n" ]
# set_property PACKAGE_PIN V6) [ get_ports "gt_clk156p25_n" ]
# set_property PACKAGE_PIN V7 [ get_ports "gt_clk156p25_p" ]

# SD card in SPI mode.  sd_cmd is MOSI, sd_d[0] is MISO, sd_d[3] is CS#,
# and sd_cd is active-low card detect.
set_property PACKAGE_PIN Y15  [get_ports "sd_clk"]
set_property PACKAGE_PIN AA15 [get_ports "sd_cmd"]
set_property PACKAGE_PIN AB14 [get_ports "sd_d[0]"]
set_property PACKAGE_PIN AA14 [get_ports "sd_d[1]"]
set_property PACKAGE_PIN AB16 [get_ports "sd_d[2]"]
set_property PACKAGE_PIN AB15 [get_ports "sd_d[3]"]
set_property PACKAGE_PIN Y16  [get_ports "sd_cd"]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_cmd sd_cd sd_d[*]}]
set_property DRIVE 8 [get_ports {sd_clk sd_cmd sd_d[*]}]
set_property PULLUP true [get_ports {sd_cmd sd_cd sd_d[*]}]

# From schematics
#set_property PACKAGE_PIN AB6 [ get_ports "pcie_clk_n" ]
#set_property PACKAGE_PIN AB7 [ get_ports "pcie_clk_p" ]
#set_property PACKAGE_PIN Y11 [ get_ports "qspi0_sclk" ]


# LED[1-4]
set_property PACKAGE_PIN H9 [ get_ports {led[0]} ]
set_property IOSTANDARD LVCMOS33 [ get_ports {led[0]} ]
set_property DRIVE 8 [ get_ports {led[0]} ]

set_property PACKAGE_PIN J9 [ get_ports {led[1]} ]
set_property IOSTANDARD LVCMOS33 [ get_ports {led[1]} ]
set_property DRIVE 8 [ get_ports {led[1]} ]

set_property PACKAGE_PIN G11 [ get_ports "led[2]" ]
set_property IOSTANDARD LVCMOS33 [ get_ports "led[2]" ]
set_property DRIVE 8 [ get_ports "led[2]" ]

set_property PACKAGE_PIN H11 [ get_ports {led[3]} ]
set_property IOSTANDARD LVCMOS33 [ get_ports {led[3]} ]
set_property DRIVE 8 [ get_ports {led[3]} ]

# KEY[1-4] (really, four push buttons)
set_property IOSTANDARD LVCMOS33 [get_ports "key[0]"]
set_property PACKAGE_PIN K9 [get_ports "key[0]"]
set_property IOSTANDARD LVCMOS33 [get_ports "key[1]"]
set_property PACKAGE_PIN K10 [get_ports "key[1]"]
set_property IOSTANDARD LVCMOS33 [get_ports "key[2]"]
set_property PACKAGE_PIN J10 [get_ports "key[2]"]
set_property IOSTANDARD LVCMOS33 [get_ports "key[3]"]
set_property PACKAGE_PIN J11 [get_ports "key[3]"]

# FPGA_UART_{RX,TX}
set_property IOSTANDARD LVCMOS33 [get_ports rxd]
set_property PACKAGE_PIN AD13 [get_ports rxd]
set_property IOSTANDARD LVCMOS33 [get_ports txd]
set_property PACKAGE_PIN AC14 [get_ports txd]

# DDR4 pin constraints (from ddr4_0_ex/imports/example_design.xdc)
# Note: sys_clk_p/n (T24/U24) are also used by the DDR4 IP (already constrained above)

set_property PACKAGE_PIN AE25 [ get_ports "c0_ddr4_dm_dbi_n[0]" ]
set_property PACKAGE_PIN AE22 [ get_ports "c0_ddr4_dm_dbi_n[1]" ]
set_property PACKAGE_PIN AD20 [ get_ports "c0_ddr4_dm_dbi_n[2]" ]
set_property PACKAGE_PIN Y20  [ get_ports "c0_ddr4_dm_dbi_n[3]" ]

set_property PACKAGE_PIN AC26 [ get_ports "c0_ddr4_dqs_t[0]" ]
set_property PACKAGE_PIN AD26 [ get_ports "c0_ddr4_dqs_c[0]" ]
set_property PACKAGE_PIN AA22 [ get_ports "c0_ddr4_dqs_t[1]" ]
set_property PACKAGE_PIN AB22 [ get_ports "c0_ddr4_dqs_c[1]" ]
set_property PACKAGE_PIN AC18 [ get_ports "c0_ddr4_dqs_t[2]" ]
set_property PACKAGE_PIN AD18 [ get_ports "c0_ddr4_dqs_c[2]" ]
set_property PACKAGE_PIN AB17 [ get_ports "c0_ddr4_dqs_t[3]" ]
set_property PACKAGE_PIN AC17 [ get_ports "c0_ddr4_dqs_c[3]" ]

set_property PACKAGE_PIN AF24 [ get_ports "c0_ddr4_dq[0]" ]
set_property PACKAGE_PIN AF25 [ get_ports "c0_ddr4_dq[1]" ]
set_property PACKAGE_PIN AD24 [ get_ports "c0_ddr4_dq[2]" ]
set_property PACKAGE_PIN AB26 [ get_ports "c0_ddr4_dq[3]" ]
set_property PACKAGE_PIN AC24 [ get_ports "c0_ddr4_dq[4]" ]
set_property PACKAGE_PIN AB25 [ get_ports "c0_ddr4_dq[5]" ]
set_property PACKAGE_PIN AD25 [ get_ports "c0_ddr4_dq[6]" ]
set_property PACKAGE_PIN AB24 [ get_ports "c0_ddr4_dq[7]" ]
set_property PACKAGE_PIN AC21 [ get_ports "c0_ddr4_dq[8]" ]
set_property PACKAGE_PIN AD23 [ get_ports "c0_ddr4_dq[9]" ]
set_property PACKAGE_PIN AD21 [ get_ports "c0_ddr4_dq[10]" ]
set_property PACKAGE_PIN AC22 [ get_ports "c0_ddr4_dq[11]" ]
set_property PACKAGE_PIN AB21 [ get_ports "c0_ddr4_dq[12]" ]
set_property PACKAGE_PIN AE23 [ get_ports "c0_ddr4_dq[13]" ]
set_property PACKAGE_PIN AE21 [ get_ports "c0_ddr4_dq[14]" ]
set_property PACKAGE_PIN AC23 [ get_ports "c0_ddr4_dq[15]" ]
set_property PACKAGE_PIN AE16 [ get_ports "c0_ddr4_dq[16]" ]
set_property PACKAGE_PIN AD19 [ get_ports "c0_ddr4_dq[17]" ]
set_property PACKAGE_PIN AD16 [ get_ports "c0_ddr4_dq[18]" ]
set_property PACKAGE_PIN AF17 [ get_ports "c0_ddr4_dq[19]" ]
set_property PACKAGE_PIN AC19 [ get_ports "c0_ddr4_dq[20]" ]
set_property PACKAGE_PIN AF19 [ get_ports "c0_ddr4_dq[21]" ]
set_property PACKAGE_PIN AF18 [ get_ports "c0_ddr4_dq[22]" ]
set_property PACKAGE_PIN AE17 [ get_ports "c0_ddr4_dq[23]" ]
set_property PACKAGE_PIN AA20 [ get_ports "c0_ddr4_dq[24]" ]
set_property PACKAGE_PIN AA18 [ get_ports "c0_ddr4_dq[25]" ]
set_property PACKAGE_PIN AA19 [ get_ports "c0_ddr4_dq[26]" ]
set_property PACKAGE_PIN Y18  [ get_ports "c0_ddr4_dq[27]" ]
set_property PACKAGE_PIN AB20 [ get_ports "c0_ddr4_dq[28]" ]
set_property PACKAGE_PIN Y17  [ get_ports "c0_ddr4_dq[29]" ]
set_property PACKAGE_PIN AB19 [ get_ports "c0_ddr4_dq[30]" ]
set_property PACKAGE_PIN AA17 [ get_ports "c0_ddr4_dq[31]" ]

set_property PACKAGE_PIN Y22  [ get_ports "c0_ddr4_adr[0]" ]
set_property PACKAGE_PIN Y25  [ get_ports "c0_ddr4_adr[1]" ]
set_property PACKAGE_PIN W23  [ get_ports "c0_ddr4_adr[2]" ]
set_property PACKAGE_PIN V26  [ get_ports "c0_ddr4_adr[3]" ]
set_property PACKAGE_PIN R26  [ get_ports "c0_ddr4_adr[4]" ]
set_property PACKAGE_PIN U26  [ get_ports "c0_ddr4_adr[5]" ]
set_property PACKAGE_PIN R21  [ get_ports "c0_ddr4_adr[6]" ]
set_property PACKAGE_PIN W25  [ get_ports "c0_ddr4_adr[7]" ]
set_property PACKAGE_PIN R20  [ get_ports "c0_ddr4_adr[8]" ]
set_property PACKAGE_PIN Y26  [ get_ports "c0_ddr4_adr[9]" ]
set_property PACKAGE_PIN R25  [ get_ports "c0_ddr4_adr[10]" ]
set_property PACKAGE_PIN V23  [ get_ports "c0_ddr4_adr[11]" ]
set_property PACKAGE_PIN AA24 [ get_ports "c0_ddr4_adr[12]" ]
set_property PACKAGE_PIN W26  [ get_ports "c0_ddr4_adr[13]" ]
set_property PACKAGE_PIN P23  [ get_ports "c0_ddr4_adr[14]" ]
set_property PACKAGE_PIN AA25 [ get_ports "c0_ddr4_adr[15]" ]
set_property PACKAGE_PIN T25  [ get_ports "c0_ddr4_adr[16]" ]

set_property PACKAGE_PIN V24 [ get_ports "c0_ddr4_ck_t[0]" ]
set_property PACKAGE_PIN W24 [ get_ports "c0_ddr4_ck_c[0]" ]

set_property PACKAGE_PIN R22 [ get_ports "c0_ddr4_bg[0]" ]
set_property PACKAGE_PIN P25 [ get_ports "c0_ddr4_cs_n[0]" ]
set_property PACKAGE_PIN P21 [ get_ports "c0_ddr4_ba[0]" ]
set_property PACKAGE_PIN P20 [ get_ports "c0_ddr4_cke[0]" ]
set_property PACKAGE_PIN R23 [ get_ports "c0_ddr4_odt[0]" ]
set_property PACKAGE_PIN P26 [ get_ports "c0_ddr4_ba[1]" ]
set_property PACKAGE_PIN P24 [ get_ports "c0_ddr4_act_n" ]

set_property PACKAGE_PIN P19  [ get_ports "c0_ddr4_reset_n" ]

# Timing waiver for DDR4 calibration IP internal signal
create_waiver -internal -user ddr4_v2_2_19 -scope -type METHODOLOGY -id {TIMING-17} -description "Ignore the TIMING-17 Critical Warning for sl_iport_i" -objects [get_pins -quiet -leaf -of [get_nets -quiet u_ddr4_0/inst/u_ddr4_mem_intfc/u_ddr_cal_top/u_ddr_cal/U_XSDB_SLAVE/sl_iport_i*] -filter {DIRECTION==IN}]

# CVFPU generated-clock and CDC constraints live in cvfpu_timing.tcl.  They
# need normal Tcl conditionals, which Vivado's XDC parser rejects.

# Bitstream configuration
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]

# RGMII to the RTL8211F-CG Ethernet PHY. Pins copied from the board's
# 12_UDP_TEST design (LVCMOS18). No MDIO/MDC or PHY-reset pins — the PHY uses
# strapping defaults, matching the reference design. The MAC datapath runs in
# the recovered RX clock (eth_rxc, 125 MHz), deskewed + phase-centered by the
# MMCM in rgmii_rx (see there for why: unconstrained BUFG capture had ~0.2ns
# of eye margin and broke RX per-build). eth_rxc and its generated clocks are
# declared async to the DDR/core clocks in cvfpu_timing.tcl (eth_tx_engine
# crosses with 2-FF synchronizers + an async-read frame RAM).
create_clock -period 8.000 -name eth_rxc [get_ports eth_rxc]

# RGMII-ID RX inputs: the PHY centers rxc in each data eye; RGMII v2.0 TskewR
# guarantees the data valid >=1.2ns before and after the pin clock edge.
# Standard center-aligned DDR source-synchronous template: data for a capture
# edge is launched by the PREVIOUS (opposite) edge 4ns earlier.
set_input_delay -clock eth_rxc -max 2.800 [get_ports {eth_rxd[*] eth_rx_ctl}]
set_input_delay -clock eth_rxc -min 1.200 [get_ports {eth_rxd[*] eth_rx_ctl}]
set_input_delay -clock eth_rxc -clock_fall -max 2.800 -add_delay [get_ports {eth_rxd[*] eth_rx_ctl}]
set_input_delay -clock eth_rxc -clock_fall -min 1.200 -add_delay [get_ports {eth_rxd[*] eth_rx_ctl}]
set_property IOSTANDARD LVCMOS18 [get_ports eth_rxc]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_rxd[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_rxd[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_rxd[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_rxd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_txd[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_txd[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_txd[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_txd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports eth_rx_ctl]
set_property IOSTANDARD LVCMOS18 [get_ports eth_tx_ctl]
set_property IOSTANDARD LVCMOS18 [get_ports eth_txc]

set_property PACKAGE_PIN K20 [get_ports {eth_txd[3]}]
set_property PACKAGE_PIN L20 [get_ports {eth_txd[2]}]
set_property PACKAGE_PIN L22 [get_ports {eth_txd[1]}]
set_property PACKAGE_PIN L23 [get_ports {eth_txd[0]}]
set_property PACKAGE_PIN K26 [get_ports {eth_rxd[3]}]
set_property PACKAGE_PIN K25 [get_ports {eth_rxd[2]}]
set_property PACKAGE_PIN L25 [get_ports {eth_rxd[1]}]
set_property PACKAGE_PIN L24 [get_ports {eth_rxd[0]}]
set_property PACKAGE_PIN K22 [get_ports eth_rxc]
set_property PACKAGE_PIN K23 [get_ports eth_rx_ctl]
set_property PACKAGE_PIN M26 [get_ports eth_tx_ctl]
set_property PACKAGE_PIN M25 [get_ports eth_txc]

# MMIO clock-bridge FIFO reset is an async crossing (fifo_reset_q on ui_clk ->
# XPM async-FIFO reset synchronizers in the core_clk/ui_clk domains). Because
# core_clk/ui_clk are synchronous BUFGCE_DIV derivatives, Vivado otherwise times
# this reset as a single-cycle path; it was the chronic worst path (WNS=0.000)
# pinned by routing, not logic. The XPM reset block re-synchronizes assertion
# internally, so this path is a false path. Removing it recovers ~0.05 ns and
# lifts the design WNS off zero (next path is the core npc->frontend path).
set_false_path \
    -from [get_cells mmio_clock_bridge_inst/fifo_reset_q_reg] \
    -through [get_pins -hier -filter {NAME =~ *mmio_clock_bridge_inst*xpm_fifo_rst_inst*/D}]


# ---- ddr_line_cdc: MCP payload crossings (src/ddr_line_cdc.v) ----
# The 512-bit line port crosses probe_clk <-> ui_clk with a 4-phase FULL HANDSHAKE:
# only the request/done LEVELS are 2-FF synchronized. The payload -- p_we/p_addr/
# p_wdata/p_wmask going out, m_rdata_q coming back -- is a plain free-running sample
# that the initiator holds STABLE for the entire round trip ("launch: hold req payload
# stable" / "stable: held while m_done asserted"), and the far side consumes it only
# after the synchronized level arrives, >= 2 destination clocks later. So it is a
# multi-cycle path, not a single-cycle transfer.
#
# probe_clk is an MMCM derivative of ui_clk, so without this Vivado times the
# 512-bit bus as one 3 ns ui_clk hop: ZERO logic levels, ~90% routing. That
# over-constraint -- not core logic -- was the sole thing keeping the in-order core
# off 111 MHz (probe_clk->probe_clk met at +0.003 ns while this CDC missed at
# -0.020 ns). Same reasoning as the mmio_clock_bridge false path above.
#
# 6 ns = 2x ui_clk, far inside the guaranteed stability window in both directions.
#
# Find the clock by OBJECT, not by literal name.  An auto-derived clock is named after the
# net at its source pin, so a change to how probe_clk is generated can rename it -- and a
# bare `get_clocks probe_clk` that matches nothing does not fail, it returns an empty list
# and SILENTLY DROPS this constraint, restoring the exact over-constraint that cost us
# 111 MHz.  probe_clk_check.tcl (a hook, because an `if` in an .xdc is ignored with only a
# CRITICAL WARNING) asserts that this lookup finds exactly one clock.
set_max_delay -datapath_only 6.000 \
    -from [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *probe_clk_buf/O}]] \
    -to   [get_pins -hier -filter {NAME =~ *probe_cdc/p_we_m_reg*/D || NAME =~ *probe_cdc/p_addr_m_reg*/D || NAME =~ *probe_cdc/p_wdata_m_reg*/D || NAME =~ *probe_cdc/p_wmask_m_reg*/D}]
# The RETURN half. This one had no -from at all, and `set_max_delay -datapath_only`
# REQUIRES one -- Vivado has been answering it with
#
#   CRITICAL WARNING: [Constraints 18-540] set_max_delay -datapath_only requires
#   -from to be non-empty
#
# and dropping the constraint, in every build there has ever been, including the one
# that shipped the 25-hour Geekbench 5 run. So while the outbound payload was correctly
# treated as multicycle, the 512-bit read data coming BACK has always been timed as a
# single ui_clk -> probe_clk hop: the same over-constraint, on the same bus, in the other
# direction. It met anyway at 9 ns. It is exactly the kind of thing that does not meet
# at 6 ns, and it is not core logic.
#
# -from names the launching flops rather than a clock, which is both more precise (this
# is one specific MCP payload, not the whole ui_clk domain) and immune to the clock
# renaming that silently broke the outbound constraint above.
set_max_delay -datapath_only 6.000 \
    -from [get_cells -hier -filter {NAME =~ *probe_cdc/m_rdata_q_reg*}] \
    -to   [get_pins -hier -filter {NAME =~ *probe_cdc/p_rdata_reg*/D}]

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
