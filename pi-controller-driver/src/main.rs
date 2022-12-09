use std::error::Error;
use std::thread;
use std::time::Duration;

use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

const POLL: u8 = 0b0010; // poll current button state 
const RDY: u8 = 0b0001; // ready to start

fn main() -> Result<(), Box<dyn Error>> {
    // Configure the SPI peripheral.
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 8_000_000, Mode::Mode0)?;

    let mut buffer = [0u8; 1];

    spi.write(&[RDY])?;

    loop {
        spi.write(&[POLL])?;
        thread::sleep(Duration::from_millis(5));
        spi.read(&mut buffer)?;

        println!("bytes received: {:#08b}", buffer[0]);
        
        buffer = [0x0];
        thread::sleep(Duration::from_millis(10));
    }

    Ok(())
}
