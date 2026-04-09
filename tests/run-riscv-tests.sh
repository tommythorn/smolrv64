#!/bin/bash
# passes fails unsupported

make -C ../src smolrv64-tester || exit

NM=$(which riscv64-elf-nm 2>/dev/null || which riscv64-linux-gnu-nm 2>/dev/null || echo "")

lock=$(mktemp)
trap 'rm -f "$lock"' EXIT

for class in "$@"
do
   echo
   echo "$class:"

   for x in riscv-tests/$class/*.bin
   do base=$(basename "$x" .bin)
      path=../tests/riscv-tests/$class/$base

      tohost=""
      elf=riscv-tests/$class/$base
      if [ -f "$elf" ] && [ -n "$NM" ]; then
         addr=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}')
         [ -n "$addr" ] && tohost="+tohost=$addr"
      fi

      (
         out=$(cd ../src; ./smolrv64-tester +even="$path.even" +odd="$path.odd" $tohost 2>&1 \
               | grep -Ev '(WARNING|finish called at)')
         flock "$lock" printf "%-25s %s\n" "$base" "$out"
      ) &
   done

   wait
done
