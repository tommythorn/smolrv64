#!/bin/bash
for class in passes fails unsupported
do echo
   echo "$class:"
   for x in riscv-tests/$class/*
   do printf "%-25s " `basename $x .bin`
      hexdump -ve '1/8 "%016x\n"' $x > mem.hex
      cut -c9-16 < mem.hex > mem0.hex
      cut -c1-8 < mem.hex > mem1.hex
      iverilog -s smolrv64_tb -DSIMULATE -DRISCV_TESTS smolrv64.v rs232tx.v
      ./a.out | egrep -v '(WARNING|finish called at)'
   done
done
