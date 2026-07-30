#!/usr/bin/env python3
"""Decode an ila_strand.tcl capture into the system-op / mtvec story.

probe5 (probe_csrop) is {pc[31:0], 16'h0, addr[11:0], func[2:0], is_csr} and probe6
(probe_csrop_v) strobes when a system op reaches the CSR unit.  Printing those in order,
next to mtvec and its write strobe, says which instructions of OpenSBI's MPRV-accessor
window actually executed:

    8000e828  csrrw a2, mtvec, a2    install probe handler
    8000e82c  csrrs a5, mstatus, a6  set MPRV
    8000e830  lbu   a7, 0(a0)        the MPRV load (not a system op -- absent here)
    8000e834  csrw  mstatus, a5      MPRV off
    8000e838  csrw  mtvec, a2        restore

A window missing the e838 entry is a lost restore; one where a trap/mret appears between
e830 and e838 says the fault path skipped it.

    Usage: ila_decode.py <capture.csv>
"""
import csv, sys

FUNC = {1: "csrrw", 2: "csrrs", 3: "csrrc", 5: "csrrwi", 6: "csrrsi", 7: "csrrci"}
CSRNAME = {0x300: "mstatus", 0x305: "mtvec", 0x341: "mepc", 0x342: "mcause",
           0x343: "mtval", 0x340: "mscratch", 0x304: "mie", 0x344: "mip",
           0x14d: "stimecmp", 0x30a: "menvcfg", 0x180: "satp", 0x105: "stvec"}
# funct3==0 SYSTEM ops carry a selector in place of a CSR address
SYSOP = {0x000: "ecall", 0x001: "ebreak", 0x102: "sret", 0x302: "mret",
         0x105: "wfi", 0x7f0: "irq-pseudo"}


def num(s):
    s = s.strip()
    if not s:
        return 0
    try:
        return int(s, 16)
    except ValueError:
        return int(s, 2)


def find(cols, *names):
    for n in names:
        for c in cols:
            if n in c:
                return c
    return None


def main(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    # Vivado prefixes the CSV with comment/metadata lines; the header is the row that
    # names the sample index column.
    hdr = next(i for i, r in enumerate(rows) if r and "Sample in Buffer" in r[0])
    cols = rows[hdr]
    data = [r for r in rows[hdr + 1:] if r and len(r) == len(cols)]

    c_samp = cols[0]
    c_op = find(cols, "csrop", "probe5")
    c_opv = find(cols, "csrop_v", "probe6")
    c_mtvec = find(cols, "mtvec_dbg", "probe2")
    c_mtwe = find(cols, "mtvec_we", "probe4")
    c_pc = find(cols, "pc_dbg", "probe1")
    if not (c_op and c_opv):
        sys.exit(f"no csrop probes in {path}; columns: {cols}")
    idx = {c: cols.index(c) for c in cols}

    print(f"{'sample':>7}  {'event':<26} {'mtvec':>16}  {'fetchPA':>16}")
    prev_mtvec = None
    for r in data:
        samp = r[idx[c_samp]]
        mtvec = num(r[idx[c_mtvec]]) if c_mtvec else 0
        fetch = num(r[idx[c_pc]]) if c_pc else 0
        events = []
        if num(r[idx[c_opv]]):
            v = num(r[idx[c_op]])
            pc = (v >> 32) & 0xFFFFFFFF
            addr = (v >> 4) & 0xFFF
            func = (v >> 1) & 0x7
            is_csr = v & 1
            if is_csr:
                what = f"{FUNC.get(func, f'f{func}')} {CSRNAME.get(addr, hex(addr))}"
            else:
                what = SYSOP.get(addr, f"sys:{hex(addr)}")
            events.append(f"{pc:08x} {what}")
        if c_mtwe and num(r[idx[c_mtwe]]):
            events.append("MTVEC-WE")
        if prev_mtvec is not None and mtvec != prev_mtvec:
            events.append(f"mtvec:={mtvec:016x}")
        prev_mtvec = mtvec
        for e in events:
            print(f"{samp:>7}  {e:<26} {mtvec:016x}  {fetch:016x}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ila_strand.csv")
