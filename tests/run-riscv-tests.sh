#!/bin/bash
# passes fails unsupported
SRC=../src
iverilog -I$SRC  -o smolrv64-tests -s smolrv64_tb -DSIMULATE -DRISCV_TESTS $SRC/smolrv64.v $SRC/rs232tx.v

for class in $*
do echo
   echo "$class:"
   for x in riscv-tests/$class/*.bin
   do base=`basename $x .bin`
      printf "%-25s " $base
      ./smolrv64-tests +even="riscv-tests/$class/$base.even" +odd="riscv-tests/$class/$base.odd"|egrep -v '(WARNING|finish called at)'
   done
done
