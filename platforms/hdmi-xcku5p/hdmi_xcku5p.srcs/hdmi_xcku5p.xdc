# The HDMI-XCKU5P board's pins. Every ball is from its schematic (0-Schematics/XCKU5P_Schematic.pdf),
# checked against the vendor tutorials' constraints; the SoC's own constraints are
# platforms/xcku5p/smolrv64_shell.xdc. Banks: 64, 65 at 1.2 V (DDR4, and HDMI in 65); 66, 67 at
# +ADJ, 1.9 V (J9 at 1-2); 84, 86, 87 at 3.3 V.

# The DDR4 IP's system clock: X1, 100 MHz, on T24/U24 (bank 65). The IP's XDC creates its clock.
set_property PACKAGE_PIN T24 [get_ports sys_clk_p]
set_property PACKAGE_PIN U24 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {sys_clk_p sys_clk_n}]

# Keys (bank 65, pulled to +1.2 V), the four push buttons.
set_property -dict {PACKAGE_PIN T19 IOSTANDARD LVCMOS12} [get_ports {key[0]}]
set_property -dict {PACKAGE_PIN N22 IOSTANDARD LVCMOS12} [get_ports {key[1]}]
set_property -dict {PACKAGE_PIN N21 IOSTANDARD LVCMOS12} [get_ports {key[2]}]
set_property -dict {PACKAGE_PIN P19 IOSTANDARD LVCMOS12} [get_ports {key[3]}]

# LEDs led0..led7 (bank 67), dark unless a diagnostic build drives the low four.
set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS18} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS18} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS18} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN D15 IOSTANDARD LVCMOS18} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN B16 IOSTANDARD LVCMOS18} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS18} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS18} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN E22 IOSTANDARD LVCMOS18} [get_ports {led[7]}]

# The console: the CP2105's interface 0 (UART2 on the schematic), VIO 1.8 V.
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS18} [get_ports txd]
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS18} [get_ports rxd]

# microSD J8 in SPI mode (bank 84, 3.3 V, 4.7K pull-ups on the board): sd_cmd is MOSI, sd_d[0]
# MISO, sd_d[3] CS#. No card detect.
set_property -dict {PACKAGE_PIN AE15 IOSTANDARD LVCMOS33} [get_ports sd_clk]
set_property -dict {PACKAGE_PIN AD15 IOSTANDARD LVCMOS33} [get_ports sd_cmd]
set_property -dict {PACKAGE_PIN AF13 IOSTANDARD LVCMOS33} [get_ports {sd_d[0]}]
set_property -dict {PACKAGE_PIN AE13 IOSTANDARD LVCMOS33} [get_ports {sd_d[1]}]
set_property -dict {PACKAGE_PIN AF15 IOSTANDARD LVCMOS33} [get_ports {sd_d[2]}]
set_property -dict {PACKAGE_PIN AF14 IOSTANDARD LVCMOS33} [get_ports {sd_d[3]}]
set_property DRIVE 8 [get_ports {sd_clk sd_cmd sd_d[*]}]

# HDMI: the TMDS pairs in bank 65, whose VCCO the DDR4 command/address pins fix at 1.2 V, so
# DIFF_SSTL12; AC-coupled to the connector. hdmi_oe switches the connector's 5 V and HPD buffer.
set_property -dict {PACKAGE_PIN T25 IOSTANDARD DIFF_SSTL12} [get_ports tmds_clk_p]
set_property -dict {PACKAGE_PIN U25 IOSTANDARD DIFF_SSTL12} [get_ports tmds_clk_n]
set_property -dict {PACKAGE_PIN N24 IOSTANDARD DIFF_SSTL12} [get_ports {tmds_p[0]}]
set_property -dict {PACKAGE_PIN P24 IOSTANDARD DIFF_SSTL12} [get_ports {tmds_n[0]}]
set_property -dict {PACKAGE_PIN V24 IOSTANDARD DIFF_SSTL12} [get_ports {tmds_p[1]}]
set_property -dict {PACKAGE_PIN W24 IOSTANDARD DIFF_SSTL12} [get_ports {tmds_n[1]}]
set_property -dict {PACKAGE_PIN V23 IOSTANDARD DIFF_SSTL12} [get_ports {tmds_p[2]}]
set_property -dict {PACKAGE_PIN W23 IOSTANDARD DIFF_SSTL12} [get_ports {tmds_n[2]}]
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports hdmi_oe]
set_false_path -to [get_ports {hdmi_oe led[*]}]

# The pixel clock and hdmi_tx's word clock are frequency-locked (its MMCM runs from the pixel
# clock) but meet only in the gearbox's async FIFO; its reset bridges assert asynchronously.
set hdmi_pix  [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *vga_mmcm_inst/CLKOUT0}]]
set hdmi_word [get_clocks -of_objects [get_pins -hier -filter {NAME =~ hdmi/bufg_word/O}]]
set_max_delay -datapath_only -from $hdmi_pix  -to $hdmi_word 5.000
set_max_delay -datapath_only -from $hdmi_word -to $hdmi_pix  5.000
set_false_path -to [get_pins -hier -filter {NAME =~ hdmi/rb_*/sr_reg[*]/PRE}]
# hdmi_tx's MMCM takes the pixel clock from the shell's MMCM through a BUFG: a cascade the placer
# may not put in one CMT column (its MMCM sits by the serializers in bank 65).
set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets -of_objects [get_pins hdmi/smmcm/CLKIN1]]

# Ethernet: the RTL8211EG, strapped GMII, bank 87 (3.3 V). gmii_phy captures RXD/RXDV in the IOB
# on its MMCM's clock, RX_PHASE placing the sample in the eye. The PHY drives RXD valid 2.5 ns before
# RXCLK's rising edge and holds it 0.5 ns after (GMII at the PHY's pins, IEEE 802.3 35.4.2.3): input
# delay max 8 - 2.5, min 0.5. (The receiver-side 2.0 / 0.0 leaves the IOB flop's capture ~0.01 ns,
# under one MMCM phase step.) Transmit leaves IOB flops with GTX_CLK forwarded inverted.
# named eth_rxc on every board: cvfpu_timing.tcl declares it asynchronous to the rest
create_clock -period 8.000 -name eth_rxc [get_ports eth_rxclk]
set_input_delay -clock eth_rxc -max 5.500 [get_ports {eth_rxd[*] eth_rxdv}]
set_input_delay -clock eth_rxc -min 0.500 [get_ports {eth_rxd[*] eth_rxdv}]
set_property PACKAGE_PIN D14 [get_ports eth_rxclk]
set_property PACKAGE_PIN H12 [get_ports eth_rxdv]
set_property PACKAGE_PIN F12 [get_ports {eth_rxd[0]}]
set_property PACKAGE_PIN G14 [get_ports {eth_rxd[1]}]
set_property PACKAGE_PIN G12 [get_ports {eth_rxd[2]}]
set_property PACKAGE_PIN H14 [get_ports {eth_rxd[3]}]
set_property PACKAGE_PIN J15 [get_ports {eth_rxd[4]}]
set_property PACKAGE_PIN J14 [get_ports {eth_rxd[5]}]
set_property PACKAGE_PIN H13 [get_ports {eth_rxd[6]}]
set_property PACKAGE_PIN J13 [get_ports {eth_rxd[7]}]
set_property PACKAGE_PIN E13 [get_ports eth_gtx_clk]
set_property PACKAGE_PIN F14 [get_ports eth_txen]
set_property PACKAGE_PIN E12 [get_ports {eth_txd[0]}]
set_property PACKAGE_PIN F13 [get_ports {eth_txd[1]}]
set_property PACKAGE_PIN C14 [get_ports {eth_txd[2]}]
set_property PACKAGE_PIN C12 [get_ports {eth_txd[3]}]
set_property PACKAGE_PIN B12 [get_ports {eth_txd[4]}]
set_property PACKAGE_PIN A12 [get_ports {eth_txd[5]}]
set_property PACKAGE_PIN A13 [get_ports {eth_txd[6]}]
set_property PACKAGE_PIN B14 [get_ports {eth_txd[7]}]
set_property PACKAGE_PIN D13 [get_ports eth_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_rxclk eth_rxdv eth_rxd[*] eth_gtx_clk eth_txen eth_txd[*] eth_rst_n}]
set_property IOB TRUE [get_ports {eth_txen eth_txd[*]}]
# RXCLK (D14) is an HD bank's clock pin: it reaches the clock network through that bank's own
# BUFGCE_HDIO, and gmii_phy's MMCM -- no CMT sits beside an HD bank -- is in another column.
set_property LOC BUFGCE_HDIO_X0Y6 [get_cells eth/bufg_in]
set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets eth/rxclk_g]
set_false_path -to [get_ports eth_rst_n]

# Configuration from the quad-SPI flash, a Micron MT25QU128 (JEDEC 20 BB 18; the schematic says
# MT25QU256).
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]

# DDR4 (bank 64 data, bank 65 command/address), from the vendor's DDR4 tutorial; the IP's own
# constraints set the I/O standards.
set_property PACKAGE_PIN AD24 [get_ports {c0_ddr4_dq[0]}]
set_property PACKAGE_PIN AF24 [get_ports {c0_ddr4_dq[1]}]
set_property PACKAGE_PIN AB26 [get_ports {c0_ddr4_dq[2]}]
set_property PACKAGE_PIN AB24 [get_ports {c0_ddr4_dq[3]}]
set_property PACKAGE_PIN AC24 [get_ports {c0_ddr4_dq[4]}]
set_property PACKAGE_PIN AB25 [get_ports {c0_ddr4_dq[5]}]
set_property PACKAGE_PIN AF25 [get_ports {c0_ddr4_dq[6]}]
set_property PACKAGE_PIN AD25 [get_ports {c0_ddr4_dq[7]}]
set_property PACKAGE_PIN AD23 [get_ports {c0_ddr4_dq[8]}]
set_property PACKAGE_PIN AE23 [get_ports {c0_ddr4_dq[9]}]
set_property PACKAGE_PIN AD21 [get_ports {c0_ddr4_dq[10]}]
set_property PACKAGE_PIN AC23 [get_ports {c0_ddr4_dq[11]}]
set_property PACKAGE_PIN AC22 [get_ports {c0_ddr4_dq[12]}]
set_property PACKAGE_PIN AE21 [get_ports {c0_ddr4_dq[13]}]
set_property PACKAGE_PIN AB21 [get_ports {c0_ddr4_dq[14]}]
set_property PACKAGE_PIN AC21 [get_ports {c0_ddr4_dq[15]}]
set_property PACKAGE_PIN AF17 [get_ports {c0_ddr4_dq[16]}]
set_property PACKAGE_PIN AE17 [get_ports {c0_ddr4_dq[17]}]
set_property PACKAGE_PIN AC19 [get_ports {c0_ddr4_dq[18]}]
set_property PACKAGE_PIN AF18 [get_ports {c0_ddr4_dq[19]}]
set_property PACKAGE_PIN AF19 [get_ports {c0_ddr4_dq[20]}]
set_property PACKAGE_PIN AD19 [get_ports {c0_ddr4_dq[21]}]
set_property PACKAGE_PIN AE16 [get_ports {c0_ddr4_dq[22]}]
set_property PACKAGE_PIN AD16 [get_ports {c0_ddr4_dq[23]}]
set_property PACKAGE_PIN AB20 [get_ports {c0_ddr4_dq[24]}]
set_property PACKAGE_PIN AB19 [get_ports {c0_ddr4_dq[25]}]
set_property PACKAGE_PIN AA19 [get_ports {c0_ddr4_dq[26]}]
set_property PACKAGE_PIN AA20 [get_ports {c0_ddr4_dq[27]}]
set_property PACKAGE_PIN Y17  [get_ports {c0_ddr4_dq[28]}]
set_property PACKAGE_PIN AA17 [get_ports {c0_ddr4_dq[29]}]
set_property PACKAGE_PIN Y18  [get_ports {c0_ddr4_dq[30]}]
set_property PACKAGE_PIN AA18 [get_ports {c0_ddr4_dq[31]}]
set_property PACKAGE_PIN P26 [get_ports {c0_ddr4_adr[0]}]
set_property PACKAGE_PIN P25 [get_ports {c0_ddr4_adr[1]}]
set_property PACKAGE_PIN R22 [get_ports {c0_ddr4_adr[2]}]
set_property PACKAGE_PIN AA24 [get_ports {c0_ddr4_adr[3]}]
set_property PACKAGE_PIN T23 [get_ports {c0_ddr4_adr[4]}]
set_property PACKAGE_PIN W20 [get_ports {c0_ddr4_adr[5]}]
set_property PACKAGE_PIN T22 [get_ports {c0_ddr4_adr[6]}]
set_property PACKAGE_PIN W19 [get_ports {c0_ddr4_adr[7]}]
set_property PACKAGE_PIN U21 [get_ports {c0_ddr4_adr[8]}]
set_property PACKAGE_PIN P21 [get_ports {c0_ddr4_adr[9]}]
set_property PACKAGE_PIN V22 [get_ports {c0_ddr4_adr[10]}]
set_property PACKAGE_PIN U19 [get_ports {c0_ddr4_adr[11]}]
set_property PACKAGE_PIN Y25 [get_ports {c0_ddr4_adr[12]}]
set_property PACKAGE_PIN P20 [get_ports {c0_ddr4_adr[13]}]
set_property PACKAGE_PIN Y23 [get_ports {c0_ddr4_adr[14]}]
set_property PACKAGE_PIN U26 [get_ports {c0_ddr4_adr[15]}]
set_property PACKAGE_PIN V26 [get_ports {c0_ddr4_adr[16]}]
set_property PACKAGE_PIN AC26 [get_ports {c0_ddr4_dqs_t[0]}]
set_property PACKAGE_PIN AA22 [get_ports {c0_ddr4_dqs_t[1]}]
set_property PACKAGE_PIN AC18 [get_ports {c0_ddr4_dqs_t[2]}]
set_property PACKAGE_PIN AB17 [get_ports {c0_ddr4_dqs_t[3]}]
set_property PACKAGE_PIN U22 [get_ports {c0_ddr4_ba[0]}]
set_property PACKAGE_PIN R26 [get_ports {c0_ddr4_ba[1]}]
set_property PACKAGE_PIN AE25 [get_ports {c0_ddr4_dm_dbi_n[0]}]
set_property PACKAGE_PIN AE22 [get_ports {c0_ddr4_dm_dbi_n[1]}]
set_property PACKAGE_PIN AD20 [get_ports {c0_ddr4_dm_dbi_n[2]}]
set_property PACKAGE_PIN Y20 [get_ports {c0_ddr4_dm_dbi_n[3]}]
set_property PACKAGE_PIN V21 [get_ports {c0_ddr4_bg[0]}]
set_property PACKAGE_PIN P23 [get_ports c0_ddr4_reset_n]
set_property PACKAGE_PIN V19 [get_ports c0_ddr4_act_n]
set_property PACKAGE_PIN AA25 [get_ports {c0_ddr4_odt[0]}] 
set_property PACKAGE_PIN W25 [get_ports {c0_ddr4_ck_t[0]}]
set_property PACKAGE_PIN Y22 [get_ports {c0_ddr4_cke[0]}]
set_property PACKAGE_PIN Y26 [get_ports {c0_ddr4_cs_n[0]}]
