#!/usr/bin/env python3
"""Rule F4: no Verilog function reads an array. Vivado gives a function that indexes a memory
ONE read port -- the last call site's -- and folds every earlier call to constant 0 (rename
port A wrote physical register 0 on four bitstreams, 2026-09-06). Flags every function body
that indexes an identifier declared as an unpacked array in the same file. Exit 1 on a hit.
   tools/check-func-ram-reads.py ooo2/*.v src/*.v
"""
import re, sys
ARRAY_DECL = re.compile(r'^\s*(?:\(\*[^)]*\*\)\s*)?(?:reg|wire|logic)\s*(?:\[[^\]]*\]\s*)*([A-Za-z_]\w*)\s*\[[^\]]*\]\s*(?:,|;)', re.M)
FUNC = re.compile(r'^\s*function\b(.*?)\bendfunction', re.S | re.M)
hits = 0
for path in sys.argv[1:]:
    if path.split('/')[-1].startswith('tb_'): continue   # testbenches are not synthesized
    src = open(path, errors='replace').read()
    src = re.sub(r'//[^\n]*', '', src)
    arrays = set(ARRAY_DECL.findall(src))
    # multi-declarations on one line: `reg [W-1:0] a [0:N], b [0:N];`
    for m in re.finditer(r'^\s*(?:\(\*[^)]*\*\)\s*)?(?:reg|wire|logic)\s*(?:\[[^\]]*\]\s*)*([^;]*);', src, re.M):
        for part in m.group(1).split(','):
            d = re.match(r'\s*([A-Za-z_]\w*)\s*\[', part)
            if d: arrays.add(d.group(1))
    if not arrays: continue
    for f in FUNC.finditer(src):
        body = f.group(1); name = re.search(r'([A-Za-z_]\w*)\s*[(;]', body)
        for a in sorted(arrays):
            if re.search(r'\b' + re.escape(a) + r'\s*\[', body):
                line = src[:f.start()].count('\n') + 1
                print(f"{path}:{line}: function {name.group(1) if name else '?'} reads array {a} (rule F4: one continuous assign per reader instead)")
                hits += 1
sys.exit(1 if hits else 0)
