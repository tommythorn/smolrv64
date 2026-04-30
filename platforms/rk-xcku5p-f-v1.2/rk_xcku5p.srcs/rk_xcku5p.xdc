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
create_clock -period 5.0 [get_ports "sys_clk_p"]

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

# Bitstream configuration
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]
