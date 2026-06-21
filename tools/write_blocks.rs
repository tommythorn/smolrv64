use std::fs::File;
use std::io::{self, Write};

fn main() -> io::Result<()> {
    let path = "output.bin";
    let mut file = File::create(path)?;

    // 256 blocks: bytes 65..=255 then 0..=64, each block is 1 KiB
    let sequence: Vec<u8> = (65u8..=255).chain(0u8..=64).collect();

    for byte_val in sequence {
        let block = [byte_val; 1024];
        file.write_all(&block)?;
    }

    println!(
        "Written {} blocks ({} bytes) to {}",
        256,
        256 * 1024,
        path
    );
    Ok(())
}
