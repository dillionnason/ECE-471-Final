#![no_std]
#![no_main]

#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
extern crate cortex_m;
extern crate embedded_hal as ehal;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;

use crate::ehal::spi::{Mode, Phase, Polarity};
use crate::hal::prelude::*;
use crate::hal::spi::Spi;
use crate::rt::ExceptionFrame;
use cortex_m::asm;

// Instruction set
const POLL: u8 = 0b0010; // poll current button state 
const RDY: u8 = 0b0001; // ready to start

#[entry]
fn main() -> ! {

    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    let _clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    // for SPI
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb2);

    // for buttons
    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    // SCLK = PE13
    // MISO = PE14
    // MOSI = PE15
    let sck = gpioe
        .pe13
        .into_alternate::<5>(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrh);
    let miso = gpioe
        .pe14
        .into_alternate::<5>(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrh);
    let mosi = gpioe
        .pe15
        .into_alternate::<5>(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrh);

    // A = PA5
    // B = PA1
    let button_a = gpioa.pa5.into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let button_b = gpioa.pa1.into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);
    // left = PB3
    // down = PB2
    // up = PE8
    // right = PA0
    let button_left = gpioa.pa3.into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let button_down = gpiob.pb2.into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let button_up = gpioe.pe8.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    let button_right = gpioa.pa0.into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);

    let mut spi = Spi::spi1_slave(
        p.SPI1, 
        (sck, miso, mosi), 
        Mode { 
            phase: Phase::CaptureOnFirstTransition,
            polarity: Polarity::IdleLow,
        },
        &mut rcc.apb2
    );

    let mut data = [0x0];
    spi.transfer(&mut data).unwrap();

    let mut button_state;
    if data[0] == RDY {
        loop {
            data = [0x0];
            while spi.transfer(&mut data).is_err() {}

            button_state = [0x0];
            if button_a.is_high() {
                button_state[0] += (1 << 0);
            }
            if button_b.is_high() {
                button_state[0] += (1 << 1);
            }
            if button_down.is_high() {
                button_state[0] += (1 << 3);
            }
            if button_left.is_high() {
                button_state[0] += (1 << 2);
            }
            if button_up.is_high() {
                button_state[0] += (1 << 4);
            }
            if button_right.is_high() {
                button_state[0] += (1 << 5);
            }
            while spi.write(&button_state).is_err() {}
        }
    }

    asm::bkpt();

    loop {
        continue;
    }
}


#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
