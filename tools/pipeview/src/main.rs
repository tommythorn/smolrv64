// pipeview: a terminal pipe viewer for the testbench's Kanata stream (tb_ooo2_linux
// +kanata=<file>, docs/OOO2-Spec.md §11.2). One row per dispatched instruction and one per run
// of non-producing cycles (a stall row, labelled with its cause); one column per cycle, each cell
// the stage the row is in. Zero dependencies beyond rvdisasm: raw mode via `stty`, ANSI drawing.

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::process::{Command, Stdio};

const LEGEND: &str = "One row per fetched instruction, one column per cycle; each letter is the
stage the instruction is in during that cycle:

    f  fetched: in the bundle register, waiting for room in the decoupling queue
    q  in the decoupling queue (8 entries)
    i  in the instruction register (decode), waiting to dispatch
    d  dispatched (renamed, in the ROB and a scheduler), waiting to issue
    a  issued on ALU a        b  issued on ALU b
    M  in the M port (memory, CSR, mul/div)      F  in the FP port
    c  in the control-flow (branch) unit
    =  complete, waiting to retire
    R  retired in this cycle (committed at the ROB head)
    A B C  executed on ALU a / ALU b / the branch unit AND retired in this same cycle: an
       ALU op writes the ROB the cycle it computes, and the ROB forwards that write to its
       head, so when the op is the head it commits in its execute cycle ('dddA')
    x  flushed in this cycle (a squash, or a frontend redirect before dispatch)

Stall rows ('** cause', dimmed yellow) cover each run of cycles in which nothing was
dispatched, filled with '#', and name the cause:
    bs:redirect, bs:drain             bad speculation: the redirect cycle; a mispredict waiting
                                      for the ROB head
    fe:immu fe:icache fe:align fe:queue  the frontend: iTLB walk; no fetch bytes; bytes but
                                      no whole instruction; an instruction fetched but none in
                                      the IR
    be:M-mem be:M-other be:rob-full be:dep-load be:dep-fp be:iq-full be:rename be:sq-full
    be:lq-full be:serialize           the backend, in the order dispatch checks them

The header counts the visible rows: the cycle span, retired instructions, IPC, flushed rows,
and the cycles each stall cause took.
";

const HELP: &str = "pipeview -- terminal pipe viewer for the ooo2 testbench's Kanata stream

USAGE
    pipeview <run.kanata> [prog.elf]              interactive
    pipeview <run.kanata> [prog.elf] --text [N]   the first N rows as text (default 60)

    Record one with: make run B=<prog> PLUSARGS=\"+kanata=$PWD/run.kanata +trace_from=A +trace_to=B\"
    in workloads/rvbench (or any tb_ooo2_linux run). With the ELF, labels are objdump's
    disassembly and symbols; without it, the instruction word is decoded (rvdisasm).

LEGEND

HEADER (recomputed over the visible rows)
    the cycle span, retired instructions, IPC, flushes, and the cycles each stall cause took

KEYS
    up/down j/k      one row          pgup/pgdn space/b   one page
    home/end g/G     first/last       left/right h/l      shift the cycle columns by 8
    /                search labels (a PC, a mnemonic, a symbol, a cause)   n  next match
    ?                the legend        q  quit
";

struct Row {
    label: String,
    stages: Vec<(u64, u8)>, // (cycle, glyph) in order
    end: Option<u64>,
    flushed: bool,
    stall: bool,
}

impl Row {
    fn start(&self) -> u64 {
        self.stages.first().map(|s| s.0).unwrap_or(0)
    }
    fn last(&self) -> u64 {
        self.end.unwrap_or_else(|| self.stages.last().map(|s| s.0).unwrap_or(0))
    }
    // The row's letter for cycle c: the stage the instruction is in during that cycle. R or x
    // marks the cycle it retired or was flushed in; when it also executed in that cycle (an ALU
    // op commits the cycle it computes, since the ROB forwards the writeback to its head), the
    // cell is the unit's letter in upper case. A stall row fills its own cycles, [start, end).
    fn glyph_at(&self, c: u64) -> u8 {
        if c < self.start() {
            return b' ';
        }
        if self.stall {
            return if self.end.map_or(true, |e| c < e) { b'#' } else { b' ' };
        }
        if let Some(e) = self.end {
            if c > e {
                return b' ';
            }
            if c == e {
                if let Some(&(_, g)) = self.stages.iter().rev().find(|s| s.0 == c) {
                    if !self.flushed && b"abc".contains(&g) {
                        return g.to_ascii_uppercase();
                    }
                }
                return if self.flushed { b'x' } else { b'R' };
            }
        }
        let mut g = b' ';
        for &(sc, sg) in &self.stages {
            if sc > c {
                break;
            }
            g = sg;
        }
        g
    }
}

fn glyph(stage: &str) -> u8 {
    match stage {
        "Fe" => b'f',
        "Dq" => b'q',
        "Ir" => b'i',
        "Ds" => b'd',
        "Xa" => b'a',
        "Xb" => b'b',
        "Xc" | "Ct" => b'c',
        "M" => b'M',
        "F" => b'F',
        "Cm" => b'=',
        "St" => b'#',
        s => s.bytes().next().unwrap_or(b'?'),
    }
}

// pc -> "<sym+off>: text" from objdump, when an ELF is given
fn objdump_table(elf: &str) -> HashMap<u64, String> {
    let mut t = HashMap::new();
    let out = match Command::new("riscv64-linux-gnu-objdump").args(["-d", "--no-show-raw-insn", elf]).output() {
        Ok(o) => String::from_utf8_lossy(&o.stdout).into_owned(),
        Err(e) => {
            eprintln!("pipeview: objdump: {e}; decoding instruction words instead");
            return t;
        }
    };
    let mut sym: Option<(u64, String)> = None;
    for ln in out.lines() {
        if let Some(rest) = ln.strip_suffix(">:") {
            if let Some((a, s)) = rest.split_once(" <") {
                if let Ok(a) = u64::from_str_radix(a.trim(), 16) {
                    sym = Some((a, s.to_string()));
                }
            }
            continue;
        }
        let Some((a, text)) = ln.split_once(":\t") else { continue };
        let Ok(pc) = u64::from_str_radix(a.trim(), 16) else { continue };
        let Some((base, name)) = &sym else { continue };
        let text = text.split('#').next().unwrap_or("").split_whitespace().collect::<Vec<_>>().join(" ");
        let off = pc - base;
        let at = if off == 0 { format!("<{name}>") } else { format!("<{name}+{off:#x}>") };
        t.insert(pc, format!("{at}: {text}"));
    }
    t
}

fn load(path: &str, elf: Option<&str>) -> io::Result<Vec<Row>> {
    let table = elf.map(objdump_table).unwrap_or_default();
    let mut rows: Vec<Row> = Vec::new();
    let mut by_id: HashMap<String, usize> = HashMap::new();
    let mut cyc: u64 = 0;
    for ln in BufReader::new(File::open(path)?).lines() {
        let ln = ln?;
        let f: Vec<&str> = ln.split('\t').collect();
        match f[0] {
            "C" if f.len() > 1 => cyc += f[1].parse::<u64>().unwrap_or(0),
            "I" if f.len() > 1 => {
                by_id.insert(f[1].to_string(), rows.len());
                rows.push(Row { label: String::new(), stages: Vec::new(), end: None, flushed: false, stall: false });
            }
            "L" if f.len() > 3 => {
                let Some(&i) = by_id.get(f[1]) else { continue };
                let r = &mut rows[i];
                if let Some(cause) = f[3].strip_prefix("** ") {
                    r.stall = true;
                    r.label = cause.to_string();
                } else if let Some((pc, word)) = f[3].split_once(": ") {
                    let p = u64::from_str_radix(pc.trim(), 16).unwrap_or(0);
                    let w = u32::from_str_radix(word.trim(), 16).unwrap_or(0);
                    r.label = match table.get(&p) {
                        Some(t) => format!("{p:x} {t}"),
                        // a row flushed before decode carries its raw fetch word; rvdisasm reads 32-bit words
                        None if w & 3 != 3 => format!("{p:x} (rvc {w:04x})"),
                        None => format!("{p:x} {}", rvdisasm::disasm(w, p).split_whitespace().collect::<Vec<_>>().join(" ")),
                    };
                } else {
                    r.label = f[3].to_string();
                }
            }
            "S" if f.len() > 3 => {
                if let Some(&i) = by_id.get(f[1]) {
                    rows[i].stages.push((cyc, glyph(f[3])));
                }
            }
            "R" if f.len() > 3 => {
                if let Some(&i) = by_id.get(f[1]) {
                    rows[i].end = Some(cyc);
                    rows[i].flushed = f[3] == "1";
                }
            }
            _ => {}
        }
    }
    Ok(rows)
}

fn stats(rows: &[Row]) -> (String, String) {
    if rows.is_empty() {
        return (String::new(), String::new());
    }
    let lo = rows.iter().map(|r| r.start()).min().unwrap_or(0);
    let hi = rows.iter().map(|r| r.last()).max().unwrap_or(0);
    let insn = rows.iter().filter(|r| !r.stall);
    let ret = insn.clone().filter(|r| r.end.is_some() && !r.flushed).count();
    let fl = insn.filter(|r| r.flushed).count();
    let mut causes: Vec<(String, u64)> = Vec::new();
    for r in rows.iter().filter(|r| r.stall && r.end.is_some()) {
        let n = r.end.unwrap() - r.start();
        match causes.iter_mut().find(|c| c.0 == r.label) {
            Some(c) => c.1 += n,
            None => causes.push((r.label.clone(), n)),
        }
    }
    causes.sort_by(|a, b| b.1.cmp(&a.1));
    let n = (hi - lo).max(1);
    let head = format!("cycles {lo}..{hi} ({n})  retired {ret}  IPC {:.2}  flushed {fl}", ret as f64 / n as f64);
    let top = causes.iter().take(8).map(|(k, v)| format!("{k} {v}")).collect::<Vec<_>>().join("  ");
    (head, top)
}

fn line(r: &Row, lab_w: usize, c0: u64, ncol: usize) -> String {
    let lab: String = if r.stall { format!("** {}", r.label) } else { r.label.clone() };
    let lab: String = lab.chars().take(lab_w).collect();
    let cells: String = (0..ncol as u64).map(|k| r.glyph_at(c0 + k) as char).collect();
    format!("{:>8} {:<lab_w$} {}", r.start(), lab, cells)
}

// ---- the terminal: raw mode via stty, ANSI escapes for drawing ----
fn stty(args: &[&str]) -> Option<String> {
    let out = Command::new("stty").args(args).stdin(Stdio::inherit()).output().ok()?;
    Some(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn term_size() -> (usize, usize) {
    stty(&["size"])
        .and_then(|s| {
            let mut it = s.split_whitespace().filter_map(|x| x.parse().ok());
            Some((it.next()?, it.next()?))
        })
        .unwrap_or((24, 80))
}

enum Key {
    Up, Down, PgUp, PgDn, Home, End, Left, Right, Search, Next, Legend, Quit, Other,
}

fn read_key(inp: &mut impl Read) -> Key {
    let mut b = [0u8; 1];
    if inp.read(&mut b).unwrap_or(0) == 0 {
        return Key::Quit;
    }
    match b[0] {
        b'q' => Key::Quit,
        b'j' => Key::Down,
        b'k' => Key::Up,
        b' ' => Key::PgDn,
        b'b' => Key::PgUp,
        b'g' => Key::Home,
        b'G' => Key::End,
        b'h' => Key::Left,
        b'l' => Key::Right,
        b'/' => Key::Search,
        b'n' => Key::Next,
        b'?' => Key::Legend,
        0x1b => {
            let mut s = [0u8; 1];
            if inp.read(&mut s).unwrap_or(0) == 0 || s[0] != b'[' {
                return Key::Quit;
            }
            let mut c = [0u8; 1];
            inp.read(&mut c).ok();
            match c[0] {
                b'A' => Key::Up,
                b'B' => Key::Down,
                b'C' => Key::Right,
                b'D' => Key::Left,
                b'H' => Key::Home,
                b'F' => Key::End,
                b'1' | b'4' | b'5' | b'6' => {
                    let mut t = [0u8; 1];
                    inp.read(&mut t).ok(); // the trailing '~'
                    match c[0] {
                        b'1' => Key::Home,
                        b'4' => Key::End,
                        b'5' => Key::PgUp,
                        _ => Key::PgDn,
                    }
                }
                _ => Key::Other,
            }
        }
        _ => Key::Other,
    }
}

fn read_line(inp: &mut impl Read, out: &mut impl Write) -> String {
    let mut s = String::new();
    loop {
        let mut b = [0u8; 1];
        if inp.read(&mut b).unwrap_or(0) == 0 {
            break;
        }
        match b[0] {
            b'\r' | b'\n' => break,
            0x7f | 0x08 => {
                if s.pop().is_some() {
                    let _ = write!(out, "\x08 \x08");
                }
            }
            0x1b => return String::new(),
            c if c >= 0x20 => {
                s.push(c as char);
                let _ = write!(out, "{}", c as char);
            }
            _ => {}
        }
        let _ = out.flush();
    }
    s
}

fn tui(rows: &[Row]) -> io::Result<()> {
    let saved = stty(&["-g"]).unwrap_or_default();
    stty(&["-echo", "-icanon", "-isig", "min", "1", "time", "0"]);
    let mut out = io::stdout();
    let mut inp = io::stdin();
    write!(out, "\x1b[?1049h\x1b[?25l")?;
    let (mut top, mut shift, mut query): (usize, i64, String) = (0, 0, String::new());
    loop {
        let (h, w) = term_size();
        let body = h.saturating_sub(4).max(1);
        let lab_w = (w / 3).clamp(20, 48);
        let ncol = w.saturating_sub(lab_w + 10).max(10);
        let view = &rows[top..(top + body).min(rows.len())];
        let c0 = (view.first().map(|r| r.start()).unwrap_or(0) as i64 + shift).max(0) as u64;
        let (head, causes) = stats(view);
        let clip = |s: &str| s.chars().take(w.saturating_sub(1)).collect::<String>();
        write!(out, "\x1b[H\x1b[2J\x1b[1m{}\x1b[0m\r\n", clip(&head))?;
        write!(out, "{}\r\n", clip(if causes.is_empty() { "(no stall cycles in view)" } else { &causes }))?;
        write!(out, "\x1b[2m{}\x1b[0m\r\n", clip(&format!("{:>8} {:<lab_w$} cycles from {c0}", "cycle", "instruction")))?;
        for r in view {
            let l = clip(&line(r, lab_w, c0, ncol));
            if r.stall {
                write!(out, "\x1b[33m{l}\x1b[0m\r\n")?;
            } else if r.flushed {
                write!(out, "\x1b[2m{l}\x1b[0m\r\n")?;
            } else {
                write!(out, "{l}\r\n")?;
            }
        }
        let status = format!(
            "row {top}/{}  ? legend  arrows pg home end, left/right cycles, / search, n next, q quit{}",
            rows.len(),
            if query.is_empty() { String::new() } else { format!("  [{query}]") }
        );
        write!(out, "\x1b[{h};1H\x1b[7m{}\x1b[0m", clip(&status))?;
        out.flush()?;
        let last = rows.len().saturating_sub(1);
        match read_key(&mut inp) {
            Key::Quit => break,
            Key::Down => { top = (top + 1).min(last); shift = 0; }
            Key::Up => { top = top.saturating_sub(1); shift = 0; }
            Key::PgDn => { top = (top + body).min(last); shift = 0; }
            Key::PgUp => { top = top.saturating_sub(body); shift = 0; }
            Key::Home => { top = 0; shift = 0; }
            Key::End => { top = rows.len().saturating_sub(body); shift = 0; }
            Key::Right => shift += 8,
            Key::Left => shift -= 8,
            k @ (Key::Search | Key::Next) => {
                if matches!(k, Key::Search) {
                    write!(out, "\x1b[{h};1H\x1b[2K/\x1b[?25h")?;
                    out.flush()?;
                    query = read_line(&mut inp, &mut out);
                    write!(out, "\x1b[?25l")?;
                }
                if !query.is_empty() {
                    if let Some(i) = (top + 1..rows.len()).find(|&i| rows[i].label.contains(query.as_str())) {
                        top = i;
                        shift = 0;
                    }
                }
            }
            Key::Legend => {
                write!(out, "\x1b[H\x1b[2J")?;
                for l in LEGEND.lines().take(h.saturating_sub(2)) {
                    write!(out, "{}\r\n", clip(l))?;
                }
                write!(out, "\x1b[{h};1H\x1b[7m{}\x1b[0m", clip("any key returns"))?;
                out.flush()?;
                read_key(&mut inp);
            }
            Key::Other => {}
        }
    }
    write!(out, "\x1b[?25h\x1b[?1049l")?;
    out.flush()?;
    if !saved.is_empty() {
        stty(&[saved.as_str()]);
    }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        print!("{HELP}\n{LEGEND}");
        return;
    }
    let text = args.iter().position(|a| a == "--text");
    let mut pos: Vec<&String> = args.iter().filter(|a| !a.starts_with("--")).collect();
    let mut n = 60usize;
    if let Some(t) = text {
        if let Some(v) = args.get(t + 1).and_then(|v| v.parse().ok()) {
            n = v;
            pos.retain(|a| a.parse::<usize>().is_err());
        }
    }
    let rows = match load(pos[0], pos.get(1).map(|s| s.as_str())) {
        Ok(r) if !r.is_empty() => r,
        Ok(_) => {
            eprintln!("{}: no rows (was the run inside +trace_from/+trace_to?)", pos[0]);
            std::process::exit(1)
        }
        Err(e) => {
            eprintln!("{}: {e}", pos[0]);
            std::process::exit(1)
        }
    };
    if text.is_some() {
        let v = &rows[..n.min(rows.len())];
        let (head, causes) = stats(v);
        println!("{head}\n{causes}");
        for r in v {
            println!("{}", line(r, 44, v[0].start(), 110).trim_end());
        }
    } else if let Err(e) = tui(&rows) {
        eprintln!("pipeview: {e}");
    }
}
