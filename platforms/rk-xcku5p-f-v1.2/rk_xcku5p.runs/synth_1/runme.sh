#!/bin/sh

# 
# Vivado(TM)
# runme.sh: a Vivado-generated Runs Script for UNIX
# Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
# Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
# 

if [ -z "$PATH" ]; then
  PATH=/home/tommy/Xilinx/2025.1/Vitis/bin:/home/tommy/Xilinx/2025.1/Vivado/ids_lite/ISE/bin/lin64:/home/tommy/Xilinx/2025.1/Vivado/bin
else
  PATH=/home/tommy/Xilinx/2025.1/Vitis/bin:/home/tommy/Xilinx/2025.1/Vivado/ids_lite/ISE/bin/lin64:/home/tommy/Xilinx/2025.1/Vivado/bin:$PATH
fi
export PATH

if [ -z "$LD_LIBRARY_PATH" ]; then
  LD_LIBRARY_PATH=
else
  LD_LIBRARY_PATH=:$LD_LIBRARY_PATH
fi
export LD_LIBRARY_PATH

HD_PWD='/home/tommy/smolrv64/platforms/rk-xcku5p-f-v1.2/rk_xcku5p.runs/synth_1'
cd "$HD_PWD"

HD_LOG=runme.log
/bin/touch $HD_LOG

ISEStep="./ISEWrap.sh"
EAStep()
{
     $ISEStep $HD_LOG "$@" >> $HD_LOG 2>&1
     if [ $? -ne 0 ]
     then
         exit
     fi
}

EAStep vivado -log rk_xcku5p.vds -m64 -product Vivado -mode batch -messageDb vivado.pb -notrace -source rk_xcku5p.tcl
