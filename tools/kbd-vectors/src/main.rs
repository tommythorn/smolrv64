#[allow(dead_code)]
#[path = "../../../../simmerv/sim/src/term_keys.rs"]
mod term_keys;
use term_keys::translate;
fn main() {
    bitmap();
    // 1. every single byte except ESC: the byte table
    for b in 0u8..=255 {
        if b == 0x1b { continue; }
        let k = translate(&[b]);
        match k.as_slice() {
            [] => println!("B {b:02x} none"),
            [(code, mods)] => println!("B {b:02x} {code} {mods}"),
            _ => panic!("byte {b:02x} gave {k:?}"),
        }
    }
    // 2. sequences: each line is the input in hex, then the keys
    let seqs: Vec<&[u8]> = vec![
        b"\x1b", b"\x1b[A", b"\x1b[B", b"\x1b[C", b"\x1b[D", b"\x1b[H", b"\x1b[F", b"\x1bOA", b"\x1bOD",
        b"\x1bOP", b"\x1bOQ", b"\x1bOR", b"\x1bOS", b"\x1b[Z", b"\x1b[1~", b"\x1b[2~", b"\x1b[3~",
        b"\x1b[4~", b"\x1b[5~", b"\x1b[6~", b"\x1b[7~", b"\x1b[8~", b"\x1b[9~", b"\x1b[11~", b"\x1b[15~",
        b"\x1b[17~", b"\x1b[21~", b"\x1b[22~", b"\x1b[23~", b"\x1b[24~", b"\x1b[25~", b"\x1b[1;5A",
        b"\x1b[1;2D", b"\x1b[1;3C", b"\x1b[1;8B", b"\x1b[3;5~", b"\x1b[15;2~", b"\x1bx", b"\x1bX",
        b"\x1b\x1b", b"\x1b\r", b"\x1b\x01", b"\x1b[", b"\x1b[1;", b"\x1b[1;5", b"\x1b[1x", b"\x1b[1;5Ax",
        b"\x1b[A\x1b[B", b"ab\x1b[Ac", b"\x1b[999~", b"\x1b[65535A", b"\x1b[1;65535A", b"\x1b[1;2;3A",
        b"\x1b[;5A", b"\x1bO5P", b"\x1b[1;1A", b"\x1b[1;0A", b"\x1b[1;9A", b"\x1b[1;17A", b"hello, World!\r",
    ];
    for s in seqs {
        let hex: String = s.iter().map(|b| format!("{b:02x}")).collect();
        let keys: Vec<String> = translate(s).iter().map(|(c, m)| format!("{c}:{m}")).collect();
        println!("S {hex} {}", if keys.is_empty() { "-".into() } else { keys.join(",") });
    }
}
#[allow(dead_code)]
fn bitmap() {
    let mut bits = [0u8; 16];
    for code in (0..0x100).filter_map(simmerv::device::virtio_input::hid_to_linux) {
        bits[usize::from(code / 8)] |= 1 << (code % 8);
    }
    let s: String = bits.iter().map(|b| format!("{b:02x}")).collect();
    println!("EVKEY {s}");
}
