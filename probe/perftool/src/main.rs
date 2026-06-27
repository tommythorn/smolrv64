// Analyzer for the sharded-OoO performance event trace (docs/perf-observability-plan.md).
// Reads the fixed 32-byte little-endian records emitted by probe/perf_trace.cpp, joins
// events into per-instruction lifecycles keyed by a sim-only uid (monotonic at dispatch),
// and reports the dependency-latency histograms that decide bypass-net vs frontend:
//   WB(producer)->READY(consumer)  = wakeup latency      (READY derived offline)
//   READY->SELECT                  = select-bandwidth pressure
// Usage: perftool <trace.bin> [--waterfall N]
//
// Record layout (must match perf_trace.cpp):
//   [0..8) cyc u64  [8..16) pc u64  [16] kind u8  [17] seqno u8  [18] ckpid u8
//   [19] rdv u8     [20..22) pdst u16  [22..24) ps1 u16  [24..26) ps2 u16  [26..32) pad
// kind: 1=DISPATCH 2=SELECT 3=WRITEBACK 4=COMMIT 5=SQUASH.

use std::collections::HashMap;

const KIND_DISP: u8 = 1;
const KIND_SEL: u8 = 2;
const KIND_WB: u8 = 3;
const KIND_COMMIT: u8 = 4;
const KIND_SQUASH: u8 = 5;

#[derive(Default)]
#[allow(dead_code)] // pdst/ps1/ps2 retained for waterfall + future analyses
struct Insn {
    seqno: u8,
    ckpid: u8,
    pc: u64,
    pdst: u16,
    ps1: u16,
    ps2: u16,
    prod1: Option<u64>, // producer uid of ps1 (at dispatch)
    prod2: Option<u64>,
    disp: u64,
    sel: Option<u64>,
    wb: Option<u64>,
}

fn rd16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn rd64(b: &[u8], o: usize) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(a)
}

// 8 buckets: 0 | 1 | 2-3 | 4-7 | 8-15 | 16-31 | 32-63 | 64+
fn bucket(d: u64) -> usize {
    if d == 0 {
        0
    } else {
        (64 - (d.leading_zeros() as usize)).min(7) // floor(log2(d))+1, capped
    }
}
const BUCKET_LABELS: [&str; 8] = ["0", "1", "2-3", "4-7", "8-15", "16-31", "32-63", "64+"];

struct Hist {
    name: &'static str,
    b: [u64; 8],
    sum: u64,
    n: u64,
    max: u64,
}
impl Hist {
    fn new(name: &'static str) -> Self {
        Hist { name, b: [0; 8], sum: 0, n: 0, max: 0 }
    }
    fn add(&mut self, d: u64) {
        self.b[bucket(d)] += 1;
        self.sum += d;
        self.n += 1;
        if d > self.max {
            self.max = d;
        }
    }
    fn print(&self) {
        let avg = if self.n > 0 { self.sum as f64 / self.n as f64 } else { 0.0 };
        println!("\n{}  (n={}, avg={:.2}, max={})", self.name, self.n, avg, self.max);
        let peak = self.b.iter().copied().max().unwrap_or(1).max(1);
        for i in 0..8 {
            let c = self.b[i];
            let pct = if self.n > 0 { 100.0 * c as f64 / self.n as f64 } else { 0.0 };
            let bar = (40 * c / peak) as usize;
            println!("  {:>6} {:>10} {:5.1}%  {}", BUCKET_LABELS[i], c, pct, "#".repeat(bar));
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: perftool <trace.bin> [--waterfall N]");
        std::process::exit(2);
    }
    let path = &args[1];
    let mut waterfall = 0usize;
    if let Some(p) = args.iter().position(|a| a == "--waterfall") {
        waterfall = args.get(p + 1).and_then(|s| s.parse().ok()).unwrap_or(20);
    }

    let data = std::fs::read(path).unwrap_or_else(|e| {
        eprintln!("perftool: cannot read {path}: {e}");
        std::process::exit(1);
    });
    if data.len() % 32 != 0 {
        eprintln!("perftool: {} bytes is not a multiple of 32", data.len());
    }

    let mut insns: HashMap<u64, Insn> = HashMap::new();
    let mut order: Vec<u64> = Vec::new(); // uids in dispatch order
    let mut seq2uid: [Option<u64>; 256] = [None; 256];
    let mut pdst2uid: HashMap<u16, u64> = HashMap::new();
    let mut next_uid: u64 = 0;

    let (mut n_disp, mut n_sel, mut n_wb, mut n_commit, mut n_squash) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let (mut min_cyc, mut max_cyc) = (u64::MAX, 0u64);

    for rec in data.chunks_exact(32) {
        let cyc = rd64(rec, 0);
        let pc = rd64(rec, 8);
        let kind = rec[16];
        let seqno = rec[17];
        let ckpid = rec[18];
        let rdv = rec[19];
        let pdst = rd16(rec, 20);
        let ps1 = rd16(rec, 22);
        let ps2 = rd16(rec, 24);
        if cyc < min_cyc {
            min_cyc = cyc;
        }
        if cyc > max_cyc {
            max_cyc = cyc;
        }
        match kind {
            KIND_DISP => {
                n_disp += 1;
                let uid = next_uid;
                next_uid += 1;
                let prod1 = if ps1 != 0 { pdst2uid.get(&ps1).copied() } else { None };
                let prod2 = if ps2 != 0 { pdst2uid.get(&ps2).copied() } else { None };
                insns.insert(
                    uid,
                    Insn { seqno, ckpid, pc, pdst, ps1, ps2, prod1, prod2, disp: cyc, sel: None, wb: None },
                );
                order.push(uid);
                seq2uid[seqno as usize] = Some(uid);
                if rdv != 0 {
                    pdst2uid.insert(pdst, uid);
                }
            }
            KIND_SEL => {
                n_sel += 1;
                if let Some(uid) = seq2uid[seqno as usize] {
                    if let Some(i) = insns.get_mut(&uid) {
                        if i.sel.is_none() {
                            i.sel = Some(cyc);
                        }
                    }
                }
            }
            KIND_WB => {
                n_wb += 1;
                if let Some(uid) = seq2uid[seqno as usize] {
                    if let Some(i) = insns.get_mut(&uid) {
                        if i.wb.is_none() {
                            i.wb = Some(cyc);
                        }
                    }
                }
            }
            KIND_COMMIT => {
                n_commit += 1;
                let _ = ckpid;
            }
            KIND_SQUASH => {
                n_squash += 1;
            }
            _ => {}
        }
    }

    let cyc_span = max_cyc.saturating_sub(min_cyc) + 1;
    println!("=== perf trace: {} ===", path);
    println!("records: disp={n_disp} sel={n_sel} wb={n_wb} commit={n_commit} squash={n_squash}");
    println!("cycle span: {} ({}..{})", cyc_span, min_cyc, max_cyc);
    println!(
        "dispatched IPC: {:.3}   selected IPC: {:.3}",
        n_disp as f64 / cyc_span as f64,
        n_sel as f64 / cyc_span as f64
    );

    // Derive READY and the dependency histograms.
    let mut h_dep = Hist::new("DISPATCH->READY  (operands wait: dependency stall)");
    let mut h_wake = Hist::new("READY->SELECT    (select-bandwidth pressure)");
    let mut h_d2s = Hist::new("DISPATCH->SELECT (total in scheduler)");
    let mut h_wb2s = Hist::new("last-producer WB->SELECT (back-to-back dependent latency)");

    for &uid in &order {
        let i = &insns[&uid];
        let sel = match i.sel {
            Some(s) => s,
            None => continue, // never issued in-window (squashed / cross-window)
        };
        // READY = max(disp, producer writebacks). A producer with no in-window WB is
        // treated as ready-at-dispatch (it wrote before the window).
        let wbc = |p: Option<u64>| -> u64 {
            p.and_then(|u| insns.get(&u)).and_then(|pi| pi.wb).unwrap_or(0)
        };
        let pw1 = wbc(i.prod1);
        let pw2 = wbc(i.prod2);
        let last_prod_wb = pw1.max(pw2);
        let ready = i.disp.max(pw1).max(pw2);
        h_dep.add(ready.saturating_sub(i.disp));
        h_wake.add(sel.saturating_sub(ready));
        h_d2s.add(sel.saturating_sub(i.disp));
        if last_prod_wb > 0 && sel >= last_prod_wb {
            h_wb2s.add(sel - last_prod_wb);
        }
    }

    h_dep.print();
    h_wake.print();
    h_d2s.print();
    h_wb2s.print();

    if waterfall > 0 {
        println!("\n=== waterfall (first {waterfall} dispatched) ===");
        println!("{:>6} {:>5} {:>5} {:>16} {:>8} {:>8} {:>8}", "uid", "seq", "ckp", "pc", "disp", "sel", "wb");
        for &uid in order.iter().take(waterfall) {
            let i = &insns[&uid];
            let s = i.sel.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
            let w = i.wb.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
            println!("{:>6} {:>5} {:>5} {:>16x} {:>8} {:>8} {:>8}", uid, i.seqno, i.ckpid, i.pc, i.disp, s, w);
        }
    }
}
