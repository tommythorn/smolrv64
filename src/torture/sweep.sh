cd /home/tommy/smolrv64/probe/torture
riscv64-linux-gnu-gcc -nostdlib -nostartfiles -march=rv64imafdc -mabi=lp64d -T link.ld -o torture.elf torture.S
riscv64-linux-gnu-objcopy -O binary torture.elf torture.bin
od -An -v -tx1 torture.bin > torture.hex
TH=$(riscv64-linux-gnu-nm torture.elf | awk '/ tohost$/{print $1}')
BIN=../obj_dir_vl/tb_vl
for ca in 1 0; do for ml in 4 9 1 0; do
  r=$("$BIN" +hex=torture.hex +tohost=$TH +cycles=250000000 +memlat=$ml +cache=$ca 2>&1 | grep -aE 'RISCV-TEST' | tail -1)
  echo "$(date +%H:%M) cache=$ca memlat=$ml -> $r"
  case "$r" in *FAIL*) echo ">>> BUG EXPOSED (self-check tripped) at cache=$ca memlat=$ml"; exit 0;; esac
done; done
echo "NO FAIL across sweep (all TIMEOUT) -> need heavier pattern or longer run"
