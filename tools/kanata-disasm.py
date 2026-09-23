#!/usr/bin/env python3
"""Put disassembly into a Kanata pipe view (tb_ooo2_linux +kanata=).

  tools/kanata-disasm.py run.kanata program.elf > run-dis.kanata

The testbench labels each instruction `<pc>: <instruction word>`. This replaces the word with
objdump's disassembly for that PC, adding the enclosing symbol, so Konata's left pane reads
`80002062 <Func_1+0xe>: ret`. PCs the ELF does not cover (a kernel, an image loaded as a
blob) keep the raw word. The ELF's march must match the objdump's defaults for extensions
(riscv64-linux-gnu-objdump reads the ELF attributes).
"""
import re, subprocess, sys

def disasm(elf, objdump='riscv64-linux-gnu-objdump'):
    out = subprocess.run([objdump, '-d', '--no-show-raw-insn', elf], capture_output=True, text=True, check=True).stdout
    table, sym = {}, None
    for ln in out.splitlines():
        m = re.match(r'^([0-9a-f]+) <([^>]+)>:', ln)
        if m:
            sym = (int(m.group(1), 16), m.group(2))
            continue
        m = re.match(r'^\s+([0-9a-f]+):\s+(.*)$', ln)
        if m and sym:
            pc = int(m.group(1), 16)
            off = pc - sym[0]
            table[pc] = f'<{sym[1]}{"+%#x" % off if off else ""}>: ' + re.sub(r'\s+', ' ', m.group(2).split('#')[0]).strip()
    return table

def main(kan, elf):
    table = disasm(elf)
    lab = re.compile(r'^(L\t\d+\t0\t)([0-9a-f]+): ([0-9a-f]+)$')
    for ln in open(kan):
        m = lab.match(ln.rstrip('\n'))
        if m:
            pc = int(m.group(2), 16)
            if pc in table:
                ln = f'{m.group(1)}{pc:x} {table[pc]}\n'
        sys.stdout.write(ln)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
