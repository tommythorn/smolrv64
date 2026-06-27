// Analyzer for the sharded-OoO performance event trace (docs/perf-observability-plan.md).
// Reads the fixed 32-byte little-endian records emitted by probe/perf_trace.cpp, joins
// events into per-instruction lifecycles keyed by a sim-only uid (monotonic at dispatch),
// and reports the dependency-latency histograms + a first-approximation diagnosis.
// Run `perftool --help` for how to read the output.
//
// Record layout (must match perf_trace.cpp):
//   [0..8) data u64  -- DISPATCH: pc, WRITEBACK: wb_val
//   [16] kind u8  [17] seqno u8  [18] ckpid u8  [19] rdv u8
//   [20..22) pdst u16  [22..24) ps1 u16  [24..26) ps2 u16  [26..30) insn u32  [30..32) pad
// kind: 1=DISPATCH 2=SELECT 3=WRITEBACK 4=COMMIT 5=SQUASH.

use std::collections::HashMap;

const KIND_DISP: u8 = 1;
const KIND_SEL: u8 = 2;
const KIND_WB: u8 = 3;
const KIND_COMMIT: u8 = 4;
const KIND_SQUASH: u8 = 5;

const HELP: &str = r#"perftool -- sharded-OoO performance event trace analyzer

USAGE
    perftool <trace.bin> [--waterfall N] [--help]

    <trace.bin>     binary trace from a -DPERF_TRACE build (perf_trace.cpp).
                    Produce one with:  PERF_TRACE=1 ./run-vl-tests.sh <test>
                    (output path = $PERF_TRACE_OUT, default /tmp/perf_trace.bin)
    --waterfall N   dump the first N dispatched instructions' lifecycle + disasm
    --help          this text

THE INSTRUCTION TIMELINE
    Every instruction moves through:
        DISPATCH --> READY --> SELECT --> (execute) --> WRITEBACK
    DISPATCH  entered the issue queue (renamed)
    READY     all source operands available (DERIVED offline =
              max(dispatch, producers' writeback cycles))
    SELECT    scheduler picked it to issue
    WRITEBACK result available
    Each histogram measures the gap between two milestones, in cycles, across all
    instructions in the window, bucketed by log2: 0 | 1 | 2-3 | 4-7 | ... | 64+.

THE FOUR HISTOGRAMS
    DISPATCH->READY  "am I waiting on data?"
        piled at 0  -> operands already available = independent work / ILP
        heavy tail  -> waiting on not-yet-computed results = DEPENDENCY-CHAIN-bound
    READY->SELECT    "am I waiting on an issue slot?"
        piled at 0-1 -> scheduler issues promptly (1 = normal wakeup latency)
        heavy tail   -> ready ops competing for too few lanes = ISSUE-WIDTH-bound
    DISPATCH->SELECT total scheduler residency (= the two above combined)
    last-producer WB->SELECT  the per-link cost of a dependency chain. On serial
        code CPI ~= this + producer execute latency. This is THE number for sha256.

HOW TO READ IT (the fork the tool auto-summarizes)
    DISPATCH->READY  READY->SELECT  throughput  =>  conclusion
        big tail        small          low          latency / bypass-bound
        small           big tail       low          issue-width-bound
        small           small          low          frontend-bound (window starved)
        small           small          high         healthy

CAVEATS (step-1 model)
    * READY is derived, not a hardware tap (a true E_READY tap is a refinement).
    * Producers dispatched before the capture window are treated as ready-at-dispatch,
      so only intra-window dependencies show -- widen PERF_TRACE_WIN if tails look cut.
    * No issue-queue OCCUPANCY yet (the cleanest frontend-vs-backend discriminator);
      for now low throughput + everything-at-0 is the frontend-starved tell.
    * On trap-heavy microtests the first ~hundreds of cycles are boot/CSR setup
      (squashes, seqno reuse); skip them with PERF_TRACE_WIN for steady-state numbers.
"#;

#[derive(Default)]
#[allow(dead_code)] // ps1/ps2 retained for future analyses
struct Insn {
    seqno: u8,
    ckpid: u8,
    pc: u64,
    insn: u32,
    pdst: u16,
    ps1: u16,
    ps2: u16,
    prod1: Option<u64>,
    prod2: Option<u64>,
    disp: u64,
    sel: Option<u64>,
    wb: Option<u64>,
    wbval: Option<u64>,
}

fn rd16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn rd32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
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
        (64 - (d.leading_zeros() as usize)).min(7)
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
    fn avg(&self) -> f64 {
        if self.n > 0 {
            self.sum as f64 / self.n as f64
        } else {
            0.0
        }
    }
    fn frac_nonzero(&self) -> f64 {
        if self.n > 0 {
            (self.n - self.b[0]) as f64 / self.n as f64
        } else {
            0.0
        }
    }
    fn print(&self) {
        println!("\n{}  (n={}, avg={:.2}, max={})", self.name, self.n, self.avg(), self.max);
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
    if args.iter().any(|a| a == "--help" || a == "-h") || args.len() < 2 {
        print!("{HELP}");
        std::process::exit(if args.len() < 2 { 2 } else { 0 });
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

    let mut insns: HashMap<u64, Insn> = HashMap::new();
    let mut order: Vec<u64> = Vec::new();
    let mut seq2uid: [Option<u64>; 256] = [None; 256];
    let mut pdst2uid: HashMap<u16, u64> = HashMap::new();
    let mut next_uid: u64 = 0;

    let (mut n_disp, mut n_sel, mut n_wb, mut n_commit, mut n_squash) = (0u64, 0, 0, 0, 0);
    let (mut min_cyc, mut max_cyc) = (u64::MAX, 0u64);

    for rec in data.chunks_exact(32) {
        let cyc = rd64(rec, 0);
        let datum = rd64(rec, 8);
        let kind = rec[16];
        let seqno = rec[17];
        let ckpid = rec[18];
        let rdv = rec[19];
        let pdst = rd16(rec, 20);
        let ps1 = rd16(rec, 22);
        let ps2 = rd16(rec, 24);
        let iword = rd32(rec, 26);
        min_cyc = min_cyc.min(cyc);
        max_cyc = max_cyc.max(cyc);
        match kind {
            KIND_DISP => {
                n_disp += 1;
                let uid = next_uid;
                next_uid += 1;
                let prod1 = if ps1 != 0 { pdst2uid.get(&ps1).copied() } else { None };
                let prod2 = if ps2 != 0 { pdst2uid.get(&ps2).copied() } else { None };
                insns.insert(
                    uid,
                    Insn { seqno, ckpid, pc: datum, insn: iword, pdst, ps1, ps2, prod1, prod2,
                           disp: cyc, sel: None, wb: None, wbval: None },
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
                            i.wbval = Some(datum);
                        }
                    }
                }
            }
            KIND_COMMIT => n_commit += 1,
            KIND_SQUASH => n_squash += 1,
            _ => {}
        }
    }

    let cyc_span = max_cyc.saturating_sub(min_cyc) + 1;
    let sel_ipc = n_sel as f64 / cyc_span as f64;
    println!("=== perf trace: {path} ===");
    println!("records: disp={n_disp} sel={n_sel} wb={n_wb} commit={n_commit} squash={n_squash}");
    println!("cycle span: {cyc_span} ({min_cyc}..{max_cyc})");
    println!("dispatched IPC: {:.3}   selected IPC: {:.3}", n_disp as f64 / cyc_span as f64, sel_ipc);

    let mut h_dep = Hist::new("DISPATCH->READY  (operands wait: dependency stall)");
    let mut h_wake = Hist::new("READY->SELECT    (select-bandwidth pressure)");
    let mut h_d2s = Hist::new("DISPATCH->SELECT (total in scheduler)");
    let mut h_wb2s = Hist::new("last-producer WB->SELECT (back-to-back dependent latency)");
    let mut h_exec = Hist::new("SELECT->WRITEBACK (execute latency)");

    for &uid in &order {
        let i = &insns[&uid];
        let sel = match i.sel {
            Some(s) => s,
            None => continue,
        };
        let wbc = |p: Option<u64>| -> u64 { p.and_then(|u| insns.get(&u)).and_then(|pi| pi.wb).unwrap_or(0) };
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
        if let Some(w) = i.wb {
            if w >= sel {
                h_exec.add(w - sel);
            }
        }
    }

    diagnose(&h_dep, &h_wake, &h_wb2s, &h_exec, sel_ipc, n_disp, n_squash);

    h_dep.print();
    h_wake.print();
    h_d2s.print();
    h_wb2s.print();
    h_exec.print();

    if waterfall > 0 {
        println!("\n=== waterfall (first {waterfall} dispatched) ===");
        println!("{:>6} {:>4} {:>3} {:>10} {:>6} {:>6} {:>6} {:>18}  {}", "uid", "seq", "ck", "pc", "disp", "sel", "wb", "wbval", "insn");
        for &uid in order.iter().take(waterfall) {
            let i = &insns[&uid];
            let s = i.sel.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
            let w = i.wb.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
            let v = i.wbval.map(|v| format!("{v:#018x}")).unwrap_or_else(|| "-".into());
            println!("{:>6} {:>4} {:>3} {:>10x} {:>6} {:>6} {:>6} {:>18}  {}", uid, i.seqno, i.ckpid, i.pc, i.disp, s, w, v, rvdisasm::disasm(i.insn, i.pc));
        }
    }
}

// First-approximation diagnosis from the histograms (docs/perf-observability-plan.md
// decision table). Heuristic, clearly labelled; refine once occupancy lands (step 2).
fn diagnose(dep: &Hist, wake: &Hist, wb2s: &Hist, exec: &Hist, sel_ipc: f64, n_disp: u64, n_squash: u64) {
    let dep_a = dep.avg();
    let wake_a = wake.avg();
    let dep_wait = dep.frac_nonzero();
    let squash_pct = if n_disp > 0 { 100.0 * n_squash as f64 / n_disp as f64 } else { 0.0 };
    let per_link = wb2s.avg() + exec.avg();

    let (verdict, lever) = if sel_ipc >= 2.5 {
        ("HEALTHY -- high issue throughput", "watch the dominant secondary cost below")
    } else if dep_a >= 2.0 && dep_a >= wake_a * 1.5 {
        ("LATENCY / BYPASS-BOUND -- operands wait; the scheduler issues promptly once ready",
         "shorten producer->consumer turnaround: bypass network / faster wakeup->issue")
    } else if wake_a >= 2.0 {
        ("ISSUE-WIDTH-BOUND -- ops are ready but queue for an issue slot",
         "widen issue / improve the select")
    } else if sel_ipc < 1.0 {
        ("FRONTEND-BOUND (tentative) -- neither operands nor issue dominate, yet throughput is low",
         "the issue window is likely starved (fetch / redirects / branch prediction); confirm with occupancy (step 2)")
    } else {
        ("MODERATE -- headroom remains", "address the dominant cost below")
    };

    println!("\n--- first-approximation diagnosis ---");
    println!("  {verdict}");
    println!("  lever: {lever}");
    println!(
        "  evidence: selected IPC {:.2}, DISPATCH->READY avg {:.2}c ({:.0}% of ops wait), READY->SELECT avg {:.2}c,",
        sel_ipc, dep_a, 100.0 * dep_wait, wake_a
    );
    println!(
        "            dependent per-link ~{:.1}c (WB->SELECT {:.1} + execute {:.1}), squash {:.1}% of dispatched",
        per_link, wb2s.avg(), exec.avg(), squash_pct
    );
}
