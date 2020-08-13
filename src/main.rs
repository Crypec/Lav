#![allow(dead_code)]
use anyhow::Result;

mod instr;
mod intrp;

fn main() -> Result<()> {
    println!("Hello, world!");
    Ok(())
}
