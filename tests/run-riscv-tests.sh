#!/bin/bash
# passes fails unsupported

make -C ../src smolrv64-tester || exit

NM=$(which riscv64-elf-nm 2>/dev/null || which riscv64-linux-gnu-nm 2>/dev/null || echo "")

for class in $*
do echo
   echo "$class:"
   for x in riscv-tests/$class/*.bin
   do base=`basename $x .bin`
      path=../tests/riscv-tests/$class/$base
      printf "%-25s " $base

      # Extract tohost address from ELF if available
      elf=riscv-tests/$class/$base
      tohost=""
      if [ -f "$elf" ] && [ -n "$NM" ]; then
         addr=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}')
         if [ -n "$addr" ]; then
            tohost="+tohost=$addr"
         fi
      fi

      (cd ../src;./smolrv64-tester +even=$path.even +odd=$path.odd $tohost)|egrep -v '(WARNING|finish called at)'
   done
done
