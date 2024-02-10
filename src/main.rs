//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use fugit::{MicrosDurationU32, RateExtU32};
use mcp4725::*;
use panic_probe as _;
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
mod clocked_interrupts;
mod display_messages;
mod encoder_interrupt;
mod io_irq_bank0;
mod types;
mod uart_interrupt;
use rotary_encoder_embedded::{standard::StandardMode, RotaryEncoder};
use types::ModuleState;

static mut MODULE_STATE: Mutex<RefCell<Option<ModuleState>>> = Mutex::new(RefCell::new(None));
static INITIAL_ALARM_DURATION: MicrosDurationU32 = MicrosDurationU32::micros(100000);
static INITIAL_ENCODER_POLL_DURATION: MicrosDurationU32 = MicrosDurationU32::micros(2000);

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

    info!("creating alarms");
    let mut alarm_0 = timer.alarm_0().unwrap();
    let mut alarm_1 = timer.alarm_1().unwrap();
    let mut alarm_2 = timer.alarm_2().unwrap();
    let mut encoder_poll_alarm = timer.alarm_3().unwrap();

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    info!("set up i2c0");
    let scl = pins.gpio19.into_function();
    let sda = pins.gpio18.into_function();
    let i2c1_device =
        I2C::new_controller(pac.I2C1, sda, scl, 400.kHz(), &mut resets, 125_000_000.Hz());

    info!("create dac");
    let mut dac = MCP4725::new(i2c1_device, 0b010);
    dac.set_dac(PowerDown::Normal, 0x0);

    info!("set up i2c1");
    let display_scl = pins.gpio21.into_function();
    let display_sda = pins.gpio20.into_function();
    let i2c0_device = I2C::new_controller(
        pac.I2C0,
        display_sda,
        display_scl,
        400.kHz(),
        &mut resets,
        125_000_000.Hz(),
    );
    info!("set up display");
    let interface = I2CDisplayInterface::new(i2c0_device);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().ok();

    info!("set up uart");
    let uart_pins = (pins.gpio8.into_function(), pins.gpio9.into_function());
    let mut uart_1 = UartPeripheral::new(pac.UART1, uart_pins, &mut resets)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart_1.enable_rx_interrupt();

    info!("set up rotary 1");
    let rotary_1_dt = pins.gpio15.into_pull_up_input();
    let rotary_1_clk = pins.gpio14.into_pull_up_input();
    let encoder_1 = RotaryEncoder::new(rotary_1_dt, rotary_1_clk).into_standard_mode();
    let encoder_1_button = pins.gpio11.reconfigure();
    encoder_1_button.set_interrupt_enabled(rp2040_hal::gpio::Interrupt::EdgeHigh, true);

    info!("set up rotary 2");
    let rotary_2_dt = pins.gpio13.into_pull_up_input();
    let rotary_2_clk = pins.gpio12.into_pull_up_input();
    let encoder_2 = RotaryEncoder::new(rotary_2_dt, rotary_2_clk).into_standard_mode();
    let encoder_2_button = pins.gpio10.reconfigure();
    encoder_2_button.set_interrupt_enabled(rp2040_hal::gpio::Interrupt::EdgeHigh, true);

    // Gate pins

    critical_section::with(|cs| {
        alarm_0.schedule(INITIAL_ALARM_DURATION).ok();
        alarm_0.enable_interrupt();
        alarm_1.schedule(INITIAL_ALARM_DURATION).ok();
        alarm_1.enable_interrupt();
        alarm_2.schedule(INITIAL_ALARM_DURATION).ok();
        alarm_2.enable_interrupt();
        encoder_poll_alarm
            .schedule(INITIAL_ENCODER_POLL_DURATION)
            .ok();
        encoder_poll_alarm.enable_interrupt();
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                alarm_0_duration: INITIAL_ALARM_DURATION,
                alarm_1_duration: INITIAL_ALARM_DURATION,
                alarm_2_duration: INITIAL_ALARM_DURATION,
                encoder_poll_duration: INITIAL_ENCODER_POLL_DURATION,
                alarm_0,
                alarm_1,
                alarm_2,
                encoder_poll_alarm,
                encoder_1,
                encoder_2,
                encoder_1_button,
                encoder_2_button,
                dac,
                uart_1,
                display,
            }));
        }
        // Don't unmask the interrupts until the Module State is in place
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_1);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_2);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_3);
            pac::NVIC::unmask(pac::Interrupt::UART1_IRQ);
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0)
        }
    });

    loop {}
}
