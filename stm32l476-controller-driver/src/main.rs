#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(unused_mut)]

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;

use stm32l4xx_hal as hal;

use hal::prelude::*;
use hal::gpio::{ gpioa::PA0, gpiob::PB7, Output, PushPull };

use cortex_m::interrupt::{ Mutex, free };

use core::cell::RefCell;
use core::ops::DerefMut;

static CLOCK: Mutex<RefCell<Option<PA0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static DATA_OUT: Mutex<RefCell<Option<PB7<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let _clocks = rcc.cfgr.sysclk(80.MHz()).hclk(8.MHz()).freeze(&mut flash.acr, &mut pwr);

    // pin pa0 = shift clock, toggled in systick interrupt
    // systick reload in microseconds (HCLK/8=1MHz) 
    cp.SYST.set_reload(500000_u32);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt(); 

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut clk = gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    // this allows the interrupt to borrow the pin
    // the free block executes code in an "interrupt-free context"
    free(|cs| {
        CLOCK.borrow(cs).replace(Some(clk));
    }); 

    // pin pb7 = serial out
    // pin pb2 = storage clock (data latch)
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    // let mut latch = gpiob.pb2.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut data_out = gpiob.pb7.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    data_out.set_low();

    free(|cs| {
        DATA_OUT.borrow(cs).replace(Some(data_out));
    });

    loop {
        continue;
    }
}

#[exception]
fn SysTick() {
    static mut DATA: u32 = 0x55;
    static mut COUNT: u32 = 0;

    if *COUNT < 15 {
        toggle_clock();
        if *COUNT % 2 == 0 {
            free(|cs| {
                let mut data_pin_ref = DATA_OUT.borrow(cs).borrow_mut();
                if let Some(ref mut data_pin) = data_pin_ref.deref_mut() {
                    if (*DATA & 1) == 1 {
                        data_pin.set_high();
                    } else {
                        data_pin.set_low();
                    }
                    *DATA >>= 1;
                }
            });
        }
    }

    *COUNT += 1;
}

/// Grab the global clock pin and toggle it
fn toggle_clock() {
    free(|cs| {
        let mut clk_ref = CLOCK.borrow(cs).borrow_mut();
        if let Some(ref mut clk) = clk_ref.deref_mut() {
            clk.toggle();
        }
    });
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
