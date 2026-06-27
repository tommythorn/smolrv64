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
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::process::{Command, Stdio};

const KIND_DISP: u8 = 1;
const KIND_SEL: u8 = 2;
const KIND_WB: u8 = 3;
const KIND_COMMIT: u8 = 4;
const KIND_SQUASH: u8 = 5;
const KIND_STALL: u8 = 6;
const KIND_FEMPTY: u8 = 7;
// dispatch-stall reason bits (mask in the STALL record's seqno field; see backend_top.v)
const STALL_LABELS: [&str; 7] =
    ["checkpoints", "free-regs", "scheduler", "store-buf", "load-queue", "fault", "redirect"];
// frontend-empty reason bits (mask in the FEMPTY record's seqno field; see backend_top.v)
const FEMPTY_LABELS: [&str; 5] = ["redirect", "immu-wait", "immu-fault", "icache-miss", "other"];

const HELP: &str = r#"perftool -- sharded-OoO performance event trace analyzer

USAGE
    perftool <trace.bin> [--waterfall N] [--pipeview N] [--interactive] [--help]

    -i, --interactive
                    scrollable TUI: a terminal-sized window into the trace with a
                    concise stats header (recomputed over the VISIBLE window) above the
                    pipeview/waterfall. Keys:
                      up/down        scroll one instruction
                      pgup/pgdn      scroll one page
                      home/end       jump to start/end
                      tab            toggle pipeview <-> waterfall
                      q              quit
                    All displayed data (histograms, diagnosis, view) is for the window
                    you are looking at, not the whole trace.

    <trace.bin>     binary trace from a -DPERF_TRACE build (perf_trace.cpp).
                    Produce one with:  PERF_TRACE=1 ./run-vl-tests.sh <test>
                    (output path = $PERF_TRACE_OUT, default /tmp/perf_trace.bin)
    --waterfall N   dump the first N dispatched instructions' lifecycle + disasm
    --pipeview N    per-cycle pipeline view (konata/O3PipeView style) of the first N,
                    all rows on a shared cycle axis so overlap/stalls are visible:
                      D=dispatch  ==dep-stall  -=issue-wait  i=issue  e=execute  w=writeback
                    Read down a column to see what every instruction is doing that cycle.
    --include-squashed
                    include wrong-path (squashed) instructions. By default they are
                    hidden from the histograms, waterfall, and pipeview so the stats
                    reflect only retired work; the count is reported in the summary.
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

CYCLE ACCOUNTING ("why aren't we dispatching?")
    A top-down split of EVERY cycle in the window into one of three states:
        dispatch        a bundle was dispatched this cycle (forward progress)
        stall           a bundle was READY at the front but back-pressured (can't
                        dispatch) -- the interesting bucket; see reasons below
        frontend-empty  no bundle was delivered -- a FRONTEND problem, not backend.
                        Broken down by reason: redirect (branch/trap/fence flush),
                        immu-wait (iTLB miss / PTW), immu-fault (fetch fault pending),
                        icache-miss (translated OK but I$ returned nothing), other
                        (fetch/decode/rename pipeline bubble or aligner truncation).
    When stalled, the hardware reports WHICH structure is full (a cycle can have
    several at once, so reason %s may sum past the stall %):
        checkpoints   out of CPR checkpoints (NCHK=4) -- too few in-flight branches
        free-regs     physical register freelist empty
        scheduler     issue queue won't accept the bundle (RS entries full)
        store-buf     store buffer full
        load-queue    load queue full
        fault         dispatch frozen on a pending fault/replay
        redirect      branch/exception redirect in flight (frontend re-steering)
    This is the direct answer to "are we blocked on checkpoints vs free registers
    vs something else?". In --interactive the split is window-local (current view).

    TIME-BINNED ACCOUNTING splits the window into 16 equal slices, one line each, so
    phase changes (hash loop vs call/return vs memcpy) show up instead of averaging out.

    PC HOTSPOTS collapses every dynamic instruction by static PC and ranks the top 25
    by total scheduler residency (DISPATCH->SELECT) -- the few PCs that cost the most
    cycles, with disassembly. avgDep = mean DISPATCH->READY (dependency wait) per hit;
    %res = that PC's share of all residency. This is where to point an optimization.
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
    squashed: bool,
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
    let mut pipeview = 0usize;
    if let Some(p) = args.iter().position(|a| a == "--pipeview") {
        pipeview = args.get(p + 1).and_then(|s| s.parse().ok()).unwrap_or(20);
    }
    let incl_squashed = args.iter().any(|a| a == "--include-squashed");
    let interactive = args.iter().any(|a| a == "--interactive" || a == "-i");

    let data = std::fs::read(path).unwrap_or_else(|e| {
        eprintln!("perftool: cannot read {path}: {e}");
        std::process::exit(1);
    });

    let mut insns: HashMap<u64, Insn> = HashMap::new();
    let mut order: Vec<u64> = Vec::new();
    let mut seq2uid: [Option<u64>; 256] = [None; 256];
    let mut pdst2uid: HashMap<u16, u64> = HashMap::new();
    let mut next_uid: u64 = 0;
    // In-flight FIFO (program order) + per-bundle sizes, to mark squashed instructions
    // exactly like the hardware: a SQUASH(roll_seq=R) kills every in-flight op younger
    // than R (signed8(seq-R)>0); a COMMIT retires the oldest bundle. (Windowed traces
    // are approximate near the window start, where pre-window commits have no bundle.)
    let mut inflight: VecDeque<u64> = VecDeque::new();
    let mut bsize: VecDeque<usize> = VecDeque::new();
    let mut cur_disp_cyc: Option<u64> = None;
    let mut stalls: Vec<(u64, u8)> = Vec::new(); // (cycle, reason mask) per stall cycle
    let mut fe_empty: Vec<(u64, u8)> = Vec::new(); // (cycle, reason mask) per frontend-empty cycle
    let mut disp_cycles: Vec<u64> = Vec::new(); // distinct dispatch cycles (sorted)

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
                           disp: cyc, sel: None, wb: None, wbval: None, squashed: false },
                );
                order.push(uid);
                seq2uid[seqno as usize] = Some(uid);
                if rdv != 0 {
                    pdst2uid.insert(pdst, uid);
                }
                if cur_disp_cyc != Some(cyc) {
                    bsize.push_back(0);
                    cur_disp_cyc = Some(cyc);
                    disp_cycles.push(cyc);
                }
                inflight.push_back(uid);
                *bsize.back_mut().unwrap() += 1;
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
            KIND_COMMIT => {
                n_commit += 1;
                if let Some(nb) = bsize.pop_front() {
                    for _ in 0..nb {
                        inflight.pop_front();
                    }
                }
            }
            KIND_SQUASH => {
                n_squash += 1;
                let r = seqno; // roll_seq: kill in-flight ops younger than R
                while let Some(&u) = inflight.back() {
                    if (insns[&u].seqno.wrapping_sub(r) as i8) > 0 {
                        inflight.pop_back();
                        if let Some(i) = insns.get_mut(&u) {
                            i.squashed = true;
                        }
                        if let Some(b) = bsize.back_mut() {
                            *b -= 1;
                            if *b == 0 {
                                bsize.pop_back();
                            }
                        }
                    } else {
                        break;
                    }
                }
                cur_disp_cyc = None; // next dispatch starts a fresh bundle
            }
            KIND_STALL => stalls.push((cyc, seqno)), // seqno field carries the reason mask
            KIND_FEMPTY => fe_empty.push((cyc, seqno)), // seqno field carries the reason mask
            _ => {}
        }
    }

    let cyc_span = max_cyc.saturating_sub(min_cyc) + 1;
    let sel_ipc = n_sel as f64 / cyc_span as f64;
    if !interactive {
        println!("=== perf trace: {path} ===");
        println!("records: disp={n_disp} sel={n_sel} wb={n_wb} commit={n_commit} squash={n_squash}");
        println!("cycle span: {cyc_span} ({min_cyc}..{max_cyc})");
        println!("dispatched IPC: {:.3}   selected IPC: {:.3}", n_disp as f64 / cyc_span as f64, sel_ipc);
        let n_sq_insn = insns.values().filter(|i| i.squashed).count();
        if n_sq_insn > 0 {
            println!(
                "squashed instructions: {} {}",
                n_sq_insn,
                if incl_squashed { "(shown)" } else { "(hidden; pass --include-squashed to show)" }
            );
        }
    }

    // instructions for stats/views: squashed (wrong-path) hidden unless requested
    let shown: Vec<u64> = order.iter().copied().filter(|u| incl_squashed || !insns[u].squashed).collect();

    if interactive {
        interactive_mode(&shown, &insns, &stalls, &fe_empty, &disp_cycles);
        return;
    }

    let h = build_hists(&shown, &insns);
    diagnose(&h.dep, &h.wake, &h.wb2s, &h.exec, sel_ipc, n_disp, n_squash);
    print_accounting(min_cyc, max_cyc, &stalls, &fe_empty, &disp_cycles);
    print_time_bins(min_cyc, max_cyc, &stalls, &disp_cycles, 16);
    h.dep.print();
    h.wake.print();
    h.d2s.print();
    h.wb2s.print();
    h.exec.print();
    print_pc_hotspots(&shown, &insns, 25);

    if waterfall > 0 {
        println!("\n=== waterfall (first {waterfall} dispatched) ===");
        wf_header();
        for &uid in shown.iter().take(waterfall) {
            wf_row(uid, &insns);
        }
    }

    if pipeview > 0 {
        let v: Vec<u64> = shown.iter().take(pipeview).copied().collect();
        println!("\n=== pipeview (first {pipeview} dispatched) ===");
        println!("legend: D=dispatch  ==dep-stall  -=issue-wait  i=issue  e=execute  w=writeback");
        pipe_render(&v, &insns, usize::MAX);
    }
}

struct Hists {
    dep: Hist,
    wake: Hist,
    d2s: Hist,
    wb2s: Hist,
    exec: Hist,
}

// Build the five lifecycle histograms over an explicit list of uids (READY derived from
// each op's producers' writeback). Used for both the whole trace and an interactive window.
fn build_hists(uids: &[u64], insns: &HashMap<u64, Insn>) -> Hists {
    let mut h = Hists {
        dep: Hist::new("DISPATCH->READY  (operands wait: dependency stall)"),
        wake: Hist::new("READY->SELECT    (select-bandwidth pressure)"),
        d2s: Hist::new("DISPATCH->SELECT (total in scheduler)"),
        wb2s: Hist::new("last-producer WB->SELECT (back-to-back dependent latency)"),
        exec: Hist::new("SELECT->WRITEBACK (execute latency)"),
    };
    for &uid in uids {
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
        h.dep.add(ready.saturating_sub(i.disp));
        h.wake.add(sel.saturating_sub(ready));
        h.d2s.add(sel.saturating_sub(i.disp));
        if last_prod_wb > 0 && sel >= last_prod_wb {
            h.wb2s.add(sel - last_prod_wb);
        }
        if let Some(w) = i.wb {
            if w >= sel {
                h.exec.add(w - sel);
            }
        }
    }
    h
}

fn wf_header() {
    println!("{:>6} {:>3} {:>10} {:>6} {:>6} {:>6} {:>18}  {}", "uid", "ck", "pc", "disp", "sel", "wb", "wbval", "insn");
}
fn wf_row(uid: u64, insns: &HashMap<u64, Insn>) {
    let i = &insns[&uid];
    let s = i.sel.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
    let w = i.wb.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
    let v = i.wbval.map(|v| format!("{v:#018x}")).unwrap_or_else(|| "-".into());
    println!("{:>6} {:>3} {:>10x} {:>6} {:>6} {:>6} {:>18}  {}", uid, i.ckpid, i.pc, i.disp, s, w, v, rvdisasm::disasm(i.insn, i.pc));
}

fn ready_of(insns: &HashMap<u64, Insn>, i: &Insn) -> u64 {
    let wbc = |p: Option<u64>| -> u64 { p.and_then(|u| insns.get(&u)).and_then(|pi| pi.wb).unwrap_or(0) };
    i.disp.max(wbc(i.prod1)).max(wbc(i.prod2))
}

// One character for what instruction (disp/ready/sel/wb) is doing at cycle c.
fn stage_char(c: u64, disp: u64, ready: u64, sel: Option<u64>, wb: Option<u64>) -> char {
    if c < disp {
        return ' ';
    }
    if c == disp {
        return 'D';
    }
    match sel {
        Some(s) => {
            if c < s {
                if c < ready {
                    '='
                } else {
                    '-'
                }
            } else if c == s {
                'i'
            } else {
                match wb {
                    Some(w) if c < w => 'e',
                    Some(w) if c == w => 'w',
                    // no writeback event (jump/branch/store, or wb past window): stop at issue
                    _ => ' ',
                }
            }
        }
        None => {
            if c < ready {
                '='
            } else {
                '-'
            }
        }
    }
}

// konata/O3PipeView-style per-cycle view of an explicit list of instructions, all on a
// shared cycle axis (origin = earliest dispatch in the list), so a vertical column is one
// cycle across all rows -- overlap and stalls read off directly. The timeline is capped to
// max_cols (terminal width) so it never wraps; the caller prints any title/legend.
const PIPE_LEFTW: usize = 48; // must match the row prefix format below
fn pipe_render(vis: &[u64], insns: &HashMap<u64, Insn>, max_cols: usize) {
    if vis.is_empty() {
        return;
    }
    let c0 = vis.iter().map(|u| insns[u].disp).min().unwrap();
    let c1 = vis
        .iter()
        .map(|u| {
            let i = &insns[u];
            i.wb.or(i.sel).unwrap_or(i.disp)
        })
        .max()
        .unwrap();
    let span = (c1 - c0) as usize;
    let tlcols = (span + 1).min(max_cols.saturating_sub(PIPE_LEFTW).max(1));

    let mut ruler = vec![b' '; tlcols];
    for col in 0..tlcols {
        if (c0 + col as u64) % 10 == 0 {
            for (k, ch) in (c0 + col as u64).to_string().bytes().enumerate() {
                if col + k < ruler.len() {
                    ruler[col + k] = ch;
                }
            }
        }
    }
    println!("{}{}", " ".repeat(PIPE_LEFTW), String::from_utf8(ruler).unwrap());

    for u in vis {
        let i = &insns[u];
        let ready = ready_of(insns, i).min(i.sel.unwrap_or(u64::MAX));
        let tl: String = (0..tlcols).map(|col| stage_char(c0 + col as u64, i.disp, ready, i.sel, i.wb)).collect();
        let mut dis = rvdisasm::disasm(i.insn, i.pc);
        dis.truncate(24);
        println!("{:>5} {:>3} {:>10x} {:<24} | {}", u, i.ckpid, i.pc, dis, tl);
    }
}

// The decision-table verdict (docs/perf-observability-plan.md). Heuristic, shared by the
// static diagnosis and the interactive header.
fn verdict(dep: &Hist, wake: &Hist, sel_ipc: f64) -> (&'static str, &'static str) {
    let dep_a = dep.avg();
    let wake_a = wake.avg();
    if sel_ipc >= 2.5 {
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
    }
}

// First-approximation diagnosis from the histograms. Refine once occupancy lands (step 2).
fn diagnose(dep: &Hist, wake: &Hist, wb2s: &Hist, exec: &Hist, sel_ipc: f64, n_disp: u64, n_squash: u64) {
    let dep_a = dep.avg();
    let wake_a = wake.avg();
    let dep_wait = dep.frac_nonzero();
    let squash_pct = if n_disp > 0 { 100.0 * n_squash as f64 / n_disp as f64 } else { 0.0 };
    let per_link = wb2s.avg() + exec.avg();
    let (verdict, lever) = verdict(dep, wake, sel_ipc);

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

// Top-down cycle accounting over [lo,hi]: every cycle is dispatch / stall / frontend-empty.
// Returns (total, dispatch_cyc, stall_cyc, empty_cyc, per-reason stall counts).
fn accounting(lo: u64, hi: u64, stalls: &[(u64, u8)], disp_cycles: &[u64]) -> (u64, u64, u64, u64, [u64; 7]) {
    let total = hi.saturating_sub(lo) + 1;
    let dlo = disp_cycles.partition_point(|&c| c < lo);
    let dhi = disp_cycles.partition_point(|&c| c <= hi);
    let disp = (dhi - dlo) as u64;
    let mut stall = 0u64;
    let mut bits = [0u64; 7];
    for &(c, m) in stalls {
        if c >= lo && c <= hi {
            stall += 1;
            for (b, slot) in bits.iter_mut().enumerate() {
                if m & (1 << b) != 0 {
                    *slot += 1;
                }
            }
        }
    }
    let empty = total.saturating_sub(disp).saturating_sub(stall);
    (total, disp, stall, empty, bits)
}

// Count, per reason bit, how many events in [lo,hi] assert that bit (a cycle may set several).
fn reason_counts(lo: u64, hi: u64, events: &[(u64, u8)]) -> [u64; 8] {
    let mut b = [0u64; 8];
    for &(c, m) in events {
        if c >= lo && c <= hi {
            for (k, slot) in b.iter_mut().enumerate() {
                if m & (1 << k) != 0 {
                    *slot += 1;
                }
            }
        }
    }
    b
}

fn print_accounting(lo: u64, hi: u64, stalls: &[(u64, u8)], fe_empty: &[(u64, u8)], disp_cycles: &[u64]) {
    let (total, disp, stall, empty, bits) = accounting(lo, hi, stalls, disp_cycles);
    let pct = |x: u64| 100.0 * x as f64 / total.max(1) as f64;
    println!("\n--- cycle accounting ({total} cycles): every cycle is one of ---");
    println!("  dispatch       {:>10} ({:4.1}%)  -- a bundle dispatched", disp, pct(disp));
    println!("  stall          {:>10} ({:4.1}%)  -- bundle ready but back-pressured", stall, pct(stall));
    println!("  frontend-empty {:>10} ({:4.1}%)  -- no bundle delivered (I$ miss / redirect / bubble)", empty, pct(empty));
    if stall > 0 {
        println!("  stall by reason (% of all cycles; a cycle may have several):");
        for b in 0..7 {
            if bits[b] > 0 {
                println!("     {:<12} {:>10} ({:4.1}%)", STALL_LABELS[b], bits[b], pct(bits[b]));
            }
        }
    }
    let fe = reason_counts(lo, hi, fe_empty);
    if fe.iter().take(5).any(|&x| x > 0) {
        println!("  frontend-empty by reason (% of all cycles):");
        for b in 0..5 {
            if fe[b] > 0 {
                println!("     {:<12} {:>10} ({:4.1}%)", FEMPTY_LABELS[b], fe[b], pct(fe[b]));
            }
        }
    }
}

// Cycle accounting split into NBINS equal time slices, so phase changes over a long
// window (hash loop vs call/return vs memcpy) show up instead of hiding in one average.
fn print_time_bins(lo: u64, hi: u64, stalls: &[(u64, u8)], disp_cycles: &[u64], nbins: usize) {
    let span = hi.saturating_sub(lo) + 1;
    if span < nbins as u64 {
        return; // window too short to bin meaningfully
    }
    let w = span / nbins as u64; // last bin absorbs the remainder
    println!("\n=== time-binned cycle accounting ({nbins} bins of ~{w}c) ===");
    println!("{:>12} {:>6} {:>6} {:>6}  {}", "cyc", "disp%", "stall%", "fe%", "dominant stall");
    for k in 0..nbins {
        let blo = lo + k as u64 * w;
        let bhi = if k == nbins - 1 { hi } else { blo + w - 1 };
        let (tot, disp, stall, empty, bits) = accounting(blo, bhi, stalls, disp_cycles);
        let pct = |x: u64| 100.0 * x as f64 / tot.max(1) as f64;
        let (mut bi, mut bv) = (0usize, 0u64);
        for (b, &v) in bits.iter().enumerate() {
            if v > bv {
                bv = v;
                bi = b;
            }
        }
        let dom = if bv > 0 { format!("{} {:.0}%", STALL_LABELS[bi], pct(bv)) } else { "-".into() };
        println!("{:>12} {:>5.0}% {:>5.0}% {:>5.0}%  {}", blo, pct(disp), pct(stall), pct(empty), dom);
    }
}

// Collapse millions of dynamic instructions into the few hot STATIC PCs, ranked by total
// scheduler residency (DISPATCH->SELECT) contribution -- "which code costs the most cycles".
fn print_pc_hotspots(uids: &[u64], insns: &HashMap<u64, Insn>, topn: usize) {
    struct Agg {
        cnt: u64,
        dep: u64,   // sum DISPATCH->READY
        resid: u64, // sum DISPATCH->SELECT
        insn: u32,
    }
    let mut m: HashMap<u64, Agg> = HashMap::new();
    let mut tot_resid = 0u64;
    for &uid in uids {
        let i = &insns[&uid];
        let sel = match i.sel {
            Some(s) => s,
            None => continue,
        };
        let resid = sel.saturating_sub(i.disp);
        let dep = ready_of(insns, i).saturating_sub(i.disp);
        tot_resid += resid;
        let e = m.entry(i.pc).or_insert(Agg { cnt: 0, dep: 0, resid: 0, insn: i.insn });
        e.cnt += 1;
        e.dep += dep;
        e.resid += resid;
    }
    let mut v: Vec<(u64, &Agg)> = m.iter().map(|(&pc, a)| (pc, a)).collect();
    v.sort_by(|a, b| b.1.resid.cmp(&a.1.resid));
    println!("\n=== PC hotspots (top {topn} by total scheduler residency = DISPATCH->SELECT) ===");
    println!("{:>10} {:>8} {:>7} {:>7} {:>6}  {}", "pc", "count", "avgDep", "avgRes", "%res", "insn");
    for (pc, a) in v.iter().take(topn) {
        let pctr = 100.0 * a.resid as f64 / tot_resid.max(1) as f64;
        println!(
            "{:>10x} {:>8} {:>7.1} {:>7.1} {:>5.1}%  {}",
            pc, a.cnt, a.dep as f64 / a.cnt as f64, a.resid as f64 / a.cnt as f64, pctr,
            rvdisasm::disasm(a.insn, *pc)
        );
    }
}

// ----------------------------------------------------------------- interactive TUI
// Zero-dependency: raw mode + size via `stty`, ANSI for clear/positioning.

const SPARK: [char; 9] = [' ', '\u{2581}', '\u{2582}', '\u{2583}', '\u{2584}', '\u{2585}', '\u{2586}', '\u{2587}', '\u{2588}'];
fn spark(h: &Hist) -> String {
    let peak = h.b.iter().copied().max().unwrap_or(1).max(1);
    h.b.iter().map(|&c| SPARK[((9 * c / peak) as usize).min(8)]).collect()
}

enum Key {
    Up,
    Down,
    PgUp,
    PgDn,
    Home,
    End,
    Tab,
    Quit,
    Other,
}

fn stty_capture(args: &[&str]) -> Option<String> {
    let out = Command::new("stty").args(args).stdin(Stdio::inherit()).output().ok()?;
    if !out.status.success() {
        return None;
    }
    Some(String::from_utf8_lossy(&out.stdout).into_owned())
}
fn run_stty(args: &[&str]) {
    let _ = Command::new("stty").args(args).stdin(Stdio::inherit()).status();
}
fn term_size() -> (usize, usize) {
    if let Some(s) = stty_capture(&["size"]) {
        let mut it = s.split_whitespace();
        if let (Some(r), Some(c)) = (it.next(), it.next()) {
            if let (Ok(r), Ok(c)) = (r.parse(), c.parse()) {
                return (r, c);
            }
        }
    }
    (24, 80)
}

// Restores terminal modes + cursor on drop (including on early return / Ctrl-C-as-byte).
struct RawGuard {
    saved: String,
}
impl RawGuard {
    fn new() -> Option<RawGuard> {
        let saved = stty_capture(&["-g"])?.trim().to_string();
        if saved.is_empty() {
            return None;
        }
        run_stty(&["-echo", "-icanon", "-isig", "min", "1", "time", "0"]);
        print!("\x1b[?25l"); // hide cursor
        let _ = io::stdout().flush();
        Some(RawGuard { saved })
    }
}
impl Drop for RawGuard {
    fn drop(&mut self) {
        run_stty(&[&self.saved]);
        print!("\x1b[?25h\x1b[2J\x1b[H"); // show cursor, clear
        let _ = io::stdout().flush();
    }
}

fn read_key() -> Key {
    let mut b = [0u8; 1];
    if io::stdin().read(&mut b).unwrap_or(0) == 0 {
        return Key::Quit;
    }
    match b[0] {
        b'q' | 0x03 | 0x04 => Key::Quit, // q, Ctrl-C, Ctrl-D
        b'\t' => Key::Tab,
        0x1b => {
            let mut c = [0u8; 1];
            if io::stdin().read(&mut c).unwrap_or(0) == 0 {
                return Key::Quit; // bare ESC
            }
            if c[0] != b'[' && c[0] != b'O' {
                return Key::Other;
            }
            let mut d = [0u8; 1];
            if io::stdin().read(&mut d).unwrap_or(0) == 0 {
                return Key::Other;
            }
            let eat = || {
                let mut t = [0u8; 1];
                let _ = io::stdin().read(&mut t);
            };
            match d[0] {
                b'A' => Key::Up,
                b'B' => Key::Down,
                b'H' => Key::Home,
                b'F' => Key::End,
                b'5' => { eat(); Key::PgUp } // ESC [ 5 ~
                b'6' => { eat(); Key::PgDn } // ESC [ 6 ~
                b'1' => { eat(); Key::Home } // ESC [ 1 ~
                b'4' => { eat(); Key::End }  // ESC [ 4 ~
                _ => Key::Other,
            }
        }
        _ => Key::Other,
    }
}

fn interactive_mode(shown: &[u64], insns: &HashMap<u64, Insn>, stalls: &[(u64, u8)], fe_empty: &[(u64, u8)], disp_cycles: &[u64]) {
    if shown.is_empty() {
        eprintln!("perftool: no (non-squashed) instructions to show");
        return;
    }
    let guard = RawGuard::new();
    if guard.is_none() {
        eprintln!("perftool: --interactive needs a terminal (stty failed)");
        return;
    }
    let total = shown.len();
    let mut start = 0usize;
    let mut pipe = true;

    loop {
        let (rows, cols) = term_size();
        let header = 8usize; // concise header + ruler
        let win = rows.saturating_sub(header + 1).max(1);
        if start > total.saturating_sub(1) {
            start = total.saturating_sub(1);
        }
        let end = (start + win).min(total);
        let vis = &shown[start..end];

        let h = build_hists(vis, insns);
        let wmin = vis.iter().map(|u| insns[u].disp).min().unwrap();
        let wmax = vis.iter().map(|u| { let i = &insns[u]; i.wb.or(i.sel).unwrap_or(i.disp) }).max().unwrap();
        let wspan = (wmax - wmin + 1) as f64;
        let nsel = vis.iter().filter(|u| insns[u].sel.is_some()).count();
        let ipc = nsel as f64 / wspan;
        let (vd, _lever) = verdict(&h.dep, &h.wake, ipc);

        let mut out = String::new();
        out.push_str("\x1b[H\x1b[2J"); // home + clear
        out.push_str(&format!(
            "perftool  uid {}..{} of {}  |  view: {}  |  cyc {}..{} ({}c)  sel-IPC {:.2}\n",
            start, end, total, if pipe { "pipeview" } else { "waterfall" }, wmin, wmax, wspan as u64, ipc
        ));
        out.push_str(&format!("diagnosis: {}\n", vd));
        out.push_str(&format!(
            "dep[D>R] {:.1} {}   sel[R>i] {:.1} {}   exec[i>w] {:.1} {}   link[w>i] {:.1} {}\n",
            h.dep.avg(), spark(&h.dep), h.wake.avg(), spark(&h.wake), h.exec.avg(), spark(&h.exec), h.wb2s.avg(), spark(&h.wb2s)
        ));
        out.push_str("buckets: 0 1 2-3 4-7 8-15 16-31 32-63 64+    legend: D=disp ==dep -=wait i=issue e=exec w=wb\n");
        {
            let (tot, disp, stall, empty, bits) = accounting(wmin, wmax, stalls, disp_cycles);
            let pct = |x: u64| 100.0 * x as f64 / tot.max(1) as f64;
            let mut reasons = String::new();
            for b in 0..7 {
                if bits[b] > 0 {
                    reasons.push_str(&format!(" {}={:.0}%", STALL_LABELS[b], pct(bits[b])));
                }
            }
            let mut fe = String::new();
            for (b, &c) in reason_counts(wmin, wmax, fe_empty).iter().take(5).enumerate() {
                if c > 0 {
                    fe.push_str(&format!(" {}={:.0}%", FEMPTY_LABELS[b], pct(c)));
                }
            }
            out.push_str(&format!(
                "cycles: disp {:.0}%  stall {:.0}%  fe-empty {:.0}%  | stall:{} | fe:{}\n",
                pct(disp), pct(stall), pct(empty),
                if reasons.is_empty() { " -".into() } else { reasons },
                if fe.is_empty() { " -".into() } else { fe }
            ));
        }
        print!("{out}");
        let _ = io::stdout().flush();

        if pipe {
            pipe_render(vis, insns, cols);
        } else {
            wf_header();
            for &u in vis {
                wf_row(u, insns);
            }
        }

        print!(
            "\x1b[7m up/dn pg home/end scroll | tab view | q quit \x1b[0m"
        );
        let _ = io::stdout().flush();

        match read_key() {
            Key::Up => start = start.saturating_sub(1),
            Key::Down => {
                if end < total {
                    start += 1;
                }
            }
            Key::PgUp => start = start.saturating_sub(win),
            Key::PgDn => start = (start + win).min(total.saturating_sub(1)),
            Key::Home => start = 0,
            Key::End => start = total.saturating_sub(win),
            Key::Tab => pipe = !pipe,
            Key::Quit => break,
            Key::Other => {}
        }
    }
}
