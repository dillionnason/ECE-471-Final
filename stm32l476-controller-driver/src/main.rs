/*
 *  ECE 471 Final Project
 *  STM32L476 Controller Driver
 *  Author: Dillion Nason
 *
 *  Takes the clock and latch signals from the Raspberry Pi
 *  and sends the data serially. Button presses are encoded in an 8-Bit
 *  unsigned integer in this order:
 *
 *  B A X Y D_up D_down D_left D_right
 *
 *  GPIO Pins:
 *      PC14 - Data Clock
 *      PB6 - Data Latch
 *      PA0 - Data
 *
 *      PA1 - D_left
 *      PA5 - D_down
 *      PA2 - D_right
 *      PA3 - D_up
 *
 *      PE11 - B
 *      PE10 - A
 *      PE12 - Y
 *      PE13 - X
 *
 *  Clock and latch signals correspond to interrupts EXTI15_10 and EXTI9_5 respectively 
 *
 */

#![no_std]
#![no_main]
#![allow(unused_mut)]
#![allow(non_snake_case)]

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;

use stm32l4xx_hal as hal;

use hal::prelude::*;
use hal::interrupt;
use hal::delay::Delay;

use hal::gpio::{
    gpioc::PC14,
    gpiob::PB6,
    gpioe::PE11,
    gpioe::PE10,
    gpioe::PE12,
    gpioe::PE13,
    gpioa::PA0,
    gpioa::PA1,
    gpioa::PA5,
    gpioa::PA2,
    gpioa::PA3,
    Input,
    Output,
    PullUp,
    PullDown,
    PushPull
};

use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{ 
    free, 
    Mutex 
};

/* These are needed to allow reading and writing the pins in the interrupts */
static LATCH: Mutex<RefCell<Option<PB6<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static CLOCK: Mutex<RefCell<Option<PC14<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static DATA_PIN: Mutex<RefCell<Option<PA0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

static B_BUTTON: Mutex<RefCell<Option<PE11<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static A_BUTTON: Mutex<RefCell<Option<PE10<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static Y_BUTTON: Mutex<RefCell<Option<PE12<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static X_BUTTON: Mutex<RefCell<Option<PE13<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

static D_LEFT:  Mutex<RefCell<Option<PA1<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
static D_DOWN:  Mutex<RefCell<Option<PA5<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
static D_RIGHT: Mutex<RefCell<Option<PA2<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
static D_UP:    Mutex<RefCell<Option<PA3<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

static DATA:    Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));

static SYST:    Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.sysclk(80.MHz()).hclk(8.MHz()).freeze(&mut flash.acr, &mut pwr);

    /* Set up clock and latch pins/interrupts */
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    
    let mut clock = gpioc.pc14.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
    let mut latch = gpiob.pb6.into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

    clock.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
    clock.enable_interrupt(&mut dp.EXTI);
    clock.trigger_on_edge(&mut dp.EXTI, hal::gpio::Edge::Rising);

    latch.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
    latch.enable_interrupt(&mut dp.EXTI);
    latch.trigger_on_edge(&mut dp.EXTI, hal::gpio::Edge::Rising);

    unsafe { 
        cortex_m::peripheral::NVIC::unmask(hal::stm32::Interrupt::EXTI9_5); 
        cortex_m::peripheral::NVIC::unmask(hal::stm32::Interrupt::EXTI15_10); 
    }

    /* Set up GPIO pins for buttons */
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb2);

    let b_button = gpioe.pe11.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    let a_button = gpioe.pe10.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    let y_button = gpioe.pe12.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    let x_button = gpioe.pe13.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);

    /* Set up joystick inputs */
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    
    let d_left = gpioa.pa1.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let d_down = gpioa.pa5.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let d_right = gpioa.pa2.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let d_up = gpioa.pa3.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

    /* Pin to send data on */
    let data_pin = gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    /* Inputs will be written to this value */
    let mut data: u8 = 0b0000_0000;

    /* Timer for setting pulses */
    let mut timer = Delay::new(cp.SYST, clocks);

    /* cs is "critical section", it makes sure this block won't be interrupted. 
     * This wraps all of the inputs in Mutexs so they can be read in the interrupts */
    free(|cs| {
        LATCH.borrow(cs).replace(Some(latch));
        CLOCK.borrow(cs).replace(Some(clock));
        DATA_PIN.borrow(cs).replace(Some(data_pin));

        B_BUTTON.borrow(cs).replace(Some(b_button));
        A_BUTTON.borrow(cs).replace(Some(a_button));
        Y_BUTTON.borrow(cs).replace(Some(y_button));
        X_BUTTON.borrow(cs).replace(Some(x_button));

        D_LEFT.borrow(cs).replace(Some(d_left));
        D_DOWN.borrow(cs).replace(Some(d_down));
        D_RIGHT.borrow(cs).replace(Some(d_right));
        D_UP.borrow(cs).replace(Some(d_up));

        DATA.borrow(cs).replace(Some(data));

        SYST.borrow(cs).replace(Some(timer));
    });


    loop {
        continue;
    }
}

// Data clock interrupt
#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        let mut clock_ref = CLOCK.borrow(cs).borrow_mut();
        let mut data_pin_ref = DATA_PIN.borrow(cs).borrow_mut();
        let mut data_ref = DATA.borrow(cs).borrow_mut();
        let mut syst_ref = SYST.borrow(cs).borrow_mut();

        if let Some(ref mut clock) = clock_ref.deref_mut() {
            if let Some(ref mut data_pin) = data_pin_ref.deref_mut() {
                if let Some(ref mut data) = data_ref.deref_mut() {
                    /* On each data clock pulse, set data line high if the LSB 
                     * of data is a 1. Shift data right to read next bit on next
                     * clock pulse */
                    if *data & 0b0000_0001 == 0b0000_0001 {
                        data_pin.set_high();
                    } else {
                        data_pin.set_low();
                    }
                    *data >>= 1;

                    /* Use SysTick to delay 12us */
                    if let Some(ref mut syst) = syst_ref.deref_mut() {
                        syst.delay_us(12_u32);
                    }
                }

                if clock.check_interrupt() {
                    clock.clear_interrupt_pending_bit();
                }
            }
        }
    });
}

// Data latch interrupt
// Currently will need a fairly big delay to ensure all of the borrows
// and checks can happen before the data clock starts
#[interrupt]
fn EXTI9_5() {
    free(|cs| {
        let mut latch_ref = LATCH.borrow(cs).borrow_mut();
        
        let mut data_ref = DATA.borrow(cs).borrow_mut();
        let b_button_ref = B_BUTTON.borrow(cs).borrow();
        let a_button_ref = A_BUTTON.borrow(cs).borrow();
        let y_button_ref = Y_BUTTON.borrow(cs).borrow();
        let x_button_ref = X_BUTTON.borrow(cs).borrow();

        let d_left_ref = D_LEFT.borrow(cs).borrow();
        let d_down_ref = D_DOWN.borrow(cs).borrow();
        let d_right_ref = D_RIGHT.borrow(cs).borrow();
        let d_up_ref = D_UP.borrow(cs).borrow();

        /* Checks each input pin and toggles the corresponding data bit.
         * Probably a better way to do this, maybe just check interrupt flag
         * in the main function? */
        if let Some(ref mut latch) = latch_ref.deref_mut() {
            if let Some(ref mut data) = data_ref.deref_mut() {
                if b_button_ref.as_ref().unwrap().is_high() {
                    *data += 0b1000_0000;
                }
                if a_button_ref.as_ref().unwrap().is_high() {
                    *data += 0b0100_0000;
                }
                if y_button_ref.as_ref().unwrap().is_high() {
                    *data += 0b0010_0000;
                }
                if x_button_ref.as_ref().unwrap().is_high() {
                    *data += 0b0001_0000;
                }
                if d_up_ref.as_ref().unwrap().is_low() {
                    *data += 0b0000_1000;
                }
                if d_down_ref.as_ref().unwrap().is_low() {
                    *data += 0b0000_0100;
                }
                if d_left_ref.as_ref().unwrap().is_low() {
                    *data += 0b0000_0010;
                }
                if d_right_ref.as_ref().unwrap().is_low() {
                    *data += 0b0000_0001;
                }
            }

            if latch.check_interrupt() {
                latch.clear_interrupt_pending_bit();
            }
        }
    });
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
