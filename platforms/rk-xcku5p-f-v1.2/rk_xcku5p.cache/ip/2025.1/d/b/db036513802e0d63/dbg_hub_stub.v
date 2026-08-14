// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.1 (lin64) Build 6140274 Wed May 21 22:58:25 MDT 2025
// Date        : Tue Aug  4 09:26:38 2026
// Host        : coffee running 64-bit Ubuntu 26.04 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ dbg_hub_stub.v
// Design      : dbg_hub
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku5p-ffvb676-2-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* CHECK_LICENSE_TYPE = "dbg_hub,xsdbm_v3_0_4_xsdbm,{}" *) (* CORE_GENERATION_INFO = "dbg_hub,xsdbm_v3_0_4_xsdbm,{x_ipProduct=Vivado 2025.1,x_ipVendor=xilinx.com,x_ipLibrary=ip,x_ipName=xsdbm,x_ipVersion=3.0,x_ipCoreRevision=4,x_ipLanguage=VERILOG,x_ipSimLanguage=MIXED,C_FIFO_STYLE=SUBCORE,C_XDEVICEFAMILY=kintexuplus,C_MAJOR_VERSION=14,C_MINOR_VERSION=1,C_BUILD_REVISION=0,C_CORE_MAJOR_VER=1,C_CORE_MINOR_VER=0,C_CORE_MINOR_ALPHA_VER=97,C_XSDB_NUM_SLAVES=2,C_XSDB_PERIOD_INT=10,C_XSDB_PERIOD_FRC=0,C_USE_BUFR=0,C_DCLK_HAS_RESET=0,C_EN_INT_SIM=1,C_CORE_TYPE=1,C_USER_SCAN_CHAIN=1,C_CLKFBOUT_MULT_F=4.000,C_DIVCLK_DIVIDE=1,C_CLKOUT0_DIVIDE_F=12.000,C_CLK_INPUT_FREQ_HZ=300000000,C_ENABLE_CLK_DIVIDER=0,C_USE_EXT_BSCAN=0,C_USE_STARTUP_CLK=0,C_EN_BSCANID_VEC=0,C_TWO_PRIM_MODE=0,C_USER_SCAN_CHAIN1=1,C_BSCANID=76546592,C_BSCAN_MODE=0,C_BSCAN_MODE_WITH_CORE=0,C_NUM_BSCAN_MASTER_PORTS=0}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) 
(* X_CORE_INFO = "xsdbm_v3_0_4_xsdbm,Vivado 2025.1" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(sl_iport0_o, sl_iport1_o, sl_oport0_i, 
  sl_oport1_i, clk)
/* synthesis syn_black_box black_box_pad_pin="sl_iport0_o[36:0],sl_iport1_o[36:0],sl_oport0_i[16:0],sl_oport1_i[16:0]" */
/* synthesis syn_force_seq_prim="clk" */;
  output [36:0]sl_iport0_o;
  output [36:0]sl_iport1_o;
  input [16:0]sl_oport0_i;
  input [16:0]sl_oport1_i;
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 signal_clock CLK" *) (* X_INTERFACE_MODE = "slave" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME signal_clock, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, INSERT_VIP 0" *) input clk /* synthesis syn_isclock = 1 */;
endmodule
