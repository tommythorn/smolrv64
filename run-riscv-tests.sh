#!/bin/bash
# passes fails unsupported
iverilog  -o smolrv64 -s smolrv64_tb -DSIMULATE -DRISCV_TESTS smolrv64.v rs232tx.v

for class in $*
do echo
   echo "$class:"
   for x in riscv-tests/$class/*.bin
   do base=`basename $x .bin`
      printf "%-25s " $base
      ./smolrv64 +even="riscv-tests/$class/$base.even" +odd="riscv-tests/$class/$base.odd"|egrep -v '(WARNING|finish called at)'
   done
done
