#!/bin/bash
# passes fails unsupported
for class in $*
do echo
   echo "$class:"
   for x in riscv-tests/$class/*
   do printf "%-25s " `basename $x .bin`
      ./evenodd.py $x 0 > mem0.hex
      ./evenodd.py $x 1 > mem1.hex
      iverilog -s smolrv64_tb -DSIMULATE -DRISCV_TESTS smolrv64.v rs232tx.v
      ./a.out | egrep -v '(WARNING|finish called at)'
   done
done
