//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use fugit::{MicrosDurationU32, RateExtU32};
use mcp4725::*;
use panic_probe as _;
use rp2040_hal::gpio::Interrupt::EdgeHigh;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::Pins,
    pac,
    pac::interrupt,
    timer::{Alarm, Alarm0, Alarm1, Alarm2, Alarm3, Timer},
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
    Clock, Sio, I2C,
};
use rp_pico::entry;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
mod types;
use rotary_encoder_embedded::{standard::StandardMode, RotaryEncoder};
use rp2040_hal::{
    gpio::bank0::Gpio0,
    gpio::{FunctionSioInput, Pin, PullUp},
};
use types::{Counter, ModuleState};

static mut MODULE_STATE: Mutex<RefCell<Option<ModuleState>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let pac = pac::Peripherals::take().unwrap();
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let external_xtal_freq_hz = 12_000_000u32;
    info!("Setting clocks");
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    info!("creating timer");
    let mut timer = Timer::new(pac.TIMER, &mut resets, &clocks);

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    let clock_in: Pin<Gpio0, FunctionSioInput, PullUp> = pins.gpio0.reconfigure();
    clock_in.set_interrupt_enabled(rp2040_hal::gpio::Interrupt::EdgeHigh, true);

    let counter = Counter {
        pin0: pins.gpio19.into_push_pull_output(),
        pin1: pins.gpio18.into_push_pull_output(),
        pin2: pins.gpio17.into_push_pull_output(),
        pin3: pins.gpio16.into_push_pull_output(),
    };

    info!("set up i2c1");
    let scl = pins.gpio3.into_function();
    let sda = pins.gpio2.into_function();
    let i2c1_device =
        I2C::new_controller(pac.I2C1, sda, scl, 400.kHz(), &mut resets, 125_000_000.Hz());

    info!("create dac");
    let mut dac = MCP4725::new(i2c1_device, 0b000);
    dac.set_dac(PowerDown::Normal, 0x0).unwrap();
    info!("dac made");

    // Gate pins

    info!("setting state");
    critical_section::with(|cs| {
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                clock_in,
                counter,
                dac,
                note: 0,
            }));
        }
        // Don't unmask the interrupts until the Module State is in place
        unsafe { pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0) }
    });

    loop {}
}
#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut note,
            mut clock_in,
            mut counter,
            mut dac,
            ..
        } = module_state;
        if clock_in.interrupt_status(EdgeHigh) {
            note = (note + 208) % 2496;
            dac.set_dac_fast(PowerDown::Normal, note);
            clock_in.clear_interrupt(EdgeHigh);
        }
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                note,
                dac,
                clock_in,
                counter,
                ..module_state
            }));
        }
    });
}
