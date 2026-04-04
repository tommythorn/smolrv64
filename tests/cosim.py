#!/usr/bin/env python3
"""Compare SmolRV64 and Simmerv traces, stopping at first divergence."""

import sys
import re

def parse_smolrv64_line(line):
    """Parse: seq prv pc insn [rd value]"""
    parts = line.split()
    if len(parts) < 4:
        return None
    seq, prv, pc, insn = parts[0], parts[1], parts[2], parts[3]
    rd_val = (parts[4], parts[5]) if len(parts) >= 6 else None
    return (seq, prv, pc, insn, rd_val)

def parse_simmerv_line(line):
    """Parse: seq prv pc insn mnemonic rd, rs1, rs2, imm [value]"""
    parts = line.split()
    if len(parts) < 4:
        return None
    seq, prv, pc, insn = parts[0], parts[1], parts[2], parts[3]
    # parts[4] = mnemonic, parts[5] = "rd," (comma-separated)
    # rd is "," for x0 (no writeback), or "ra," / "t0," etc.
    if len(parts) >= 6:
        rd_name = parts[5].rstrip(',')
        if rd_name and rd_name != '' and rd_name in REG_NAMES and reg_to_num(rd_name) != '0':
            # Has writeback — value is last field
            rd_val = (reg_to_num(rd_name), parts[-1])
        else:
            rd_val = None
    else:
        rd_val = None
    return (seq, prv, pc, insn, rd_val)

# Register name to number mapping
REG_NAMES = {
    'x0': '0', 'ra': '1', 'sp': '2', 'gp': '3', 'tp': '4',
    't0': '5', 't1': '6', 't2': '7', 's0': '8', 's1': '9',
    'a0': '10', 'a1': '11', 'a2': '12', 'a3': '13', 'a4': '14',
    'a5': '15', 'a6': '16', 'a7': '17', 's2': '18', 's3': '19',
    's4': '20', 's5': '21', 's6': '22', 's7': '23', 's8': '24',
    's9': '25', 's10': '26', 's11': '27', 't3': '28', 't4': '29',
    't5': '30', 't6': '31',
}

def reg_to_num(name):
    return REG_NAMES.get(name, name)

def compare(smol_path, simmerv_path, context=3):
    with open(smol_path) as f:
        smol_lines = [l.rstrip() for l in f if l[0:1].isdigit()]
    with open(simmerv_path) as f:
        simmerv_lines = [l.rstrip() for l in f if l.strip()[:1].isdigit()]

    n = min(len(smol_lines), len(simmerv_lines))
    for i in range(n):
        s = parse_smolrv64_line(smol_lines[i])
        r = parse_simmerv_line(simmerv_lines[i])
        if not s or not r:
            continue

        # Compare seq, prv, pc, insn
        mismatch = None
        if s[0] != r[0]:
            mismatch = f"seq: smol={s[0]} simmerv={r[0]}"
        elif s[1] != r[1]:
            mismatch = f"prv: smol={s[1]} simmerv={r[1]}"
        elif s[2] != r[2]:
            mismatch = f"pc: smol={s[2]} simmerv={r[2]}"
        elif r[3] == '<inaccessible>':
            pass  # Simmerv couldn't fetch insn for display, skip
        elif s[3] != r[3]:
            mismatch = f"insn: smol={s[3]} simmerv={r[3]}"
        elif s[4] and r[4]:
            s_rd, s_val = s[4]
            r_rd, r_val = reg_to_num(r[4][0]), r[4][1]
            if s_rd != r_rd:
                mismatch = f"rd: smol=x{s_rd} simmerv={r[4][0]}({r_rd})"
            elif int(s_val, 16) != int(r_val, 16):
                mismatch = f"rd_value: smol={s_val} simmerv={r_val}"
        elif bool(s[4]) != bool(r[4]):
            mismatch = f"writeback: smol={'yes' if s[4] else 'no'} simmerv={'yes' if r[4] else 'no'}"

        if mismatch:
            print(f"DIVERGENCE at instruction {i}: {mismatch}")
            start = max(0, i - context)
            for j in range(start, min(i + context + 1, n)):
                marker = ">>>" if j == i else "   "
                print(f"  {marker} smol:    {smol_lines[j]}")
                print(f"  {marker} simmerv: {simmerv_lines[j]}")
            return 1

    if len(smol_lines) != len(simmerv_lines):
        print(f"Traces match for {n} instructions but differ in length: smol={len(smol_lines)} simmerv={len(simmerv_lines)}")
    else:
        print(f"OK: {n} instructions match")
    return 0

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} smolrv64.trace simmerv.trace")
        sys.exit(1)
    sys.exit(compare(sys.argv[1], sys.argv[2]))
