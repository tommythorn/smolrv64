`timescale 1ns/1ps
`default_nettype none
// tb_ddr_port.v's memory path with probe_clk at ui_clk / 4 (PROBE_CLK_DIV8=96: the half-speed build)
module tb_ddr_port_div4;
   tb #(.DIV(4)) t();
endmodule
`include "tb_ddr_port.v"
`default_nettype wire
