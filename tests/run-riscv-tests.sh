#!/bin/bash
# passes fails unsupported

make -C ../src smolrv64-tester smolrv64-cvfpu-tester || exit

NM=$(command -v riscv64-elf-nm 2>/dev/null || command -v riscv64-linux-gnu-nm 2>/dev/null || echo "")
DEFAULT_JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)
JOBS=${JOBS:-$DEFAULT_JOBS}

case "$JOBS" in
   ''|*[!0-9]*|0) JOBS=1 ;;
esac

lock=$(mktemp)
fail=$(mktemp)
trap 'rm -f "$lock" "$fail"' EXIT

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
         tester=./smolrv64-tester
         case "$base" in
            rv64uf-*|rv64ud-*) tester=./smolrv64-cvfpu-tester ;;
         esac
         out=$(cd ../src; "$tester" +even="$path.even" +odd="$path.odd" $tohost 2>&1 \
               | grep -Ev '(WARNING|finish called at)')
         flock "$lock" printf "%-25s %s\n" "$base" "$out"
         case "$out" in
            *"Test Passed"*) ;;
            *) flock "$lock" printf "%s\n" "$base" >> "$fail" ;;
         esac
      ) &

      while [ "$(jobs -pr | wc -l)" -ge "$JOBS" ]; do
         wait -n
      done
   done

   wait
done

if [ -s "$fail" ]; then
   echo
   echo "failed tests:"
   cat "$fail"
   exit 1
fi
