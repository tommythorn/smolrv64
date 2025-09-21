#!/bin/bash
# passes fails unsupported

make -C ../src smolrv64-tester

for class in $*
do echo
   echo "$class:"
   for x in riscv-tests/$class/*.bin
   do base=`basename $x .bin`
      path=../tests/riscv-tests/$class/$base
      printf "%-25s " $base
      (cd ../src;./smolrv64-tester +even=$path.even +odd=$path.odd)|egrep -v '(WARNING|finish called at)'
   done
done
