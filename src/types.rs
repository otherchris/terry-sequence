use crate::MicrosDurationU32;
use crate::{pac::I2C0, pac::I2C1, Alarm0, Alarm1, Alarm2, Alarm3, I2C};
use crate::{RotaryEncoder, StandardMode};
use mcp4725::MCP4725;
use rp2040_hal::gpio::FunctionSioOutput;
use rp2040_hal::{
    gpio::bank0::{
        Gpio0, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio18, Gpio19,
        Gpio2, Gpio20, Gpio21, Gpio3, Gpio8, Gpio9,
    },
    gpio::{FunctionI2c, FunctionPio0, FunctionSioInput, FunctionUart, Pin, PullDown, PullUp},
    pac::UART1,
    uart::{Enabled, UartPeripheral},
};
use ssd1306::mode::TerminalMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::size::DisplaySize128x32;
use ssd1306::Ssd1306;

pub struct Counter {
    pub pin0: Pin<Gpio16, FunctionSioInput, PullUp>,
    pub pin1: Pin<Gpio17, FunctionSioInput, PullUp>,
    pub pin2: Pin<Gpio18, FunctionSioInput, PullUp>,
    pub pin3: Pin<Gpio19, FunctionSioInput, PullUp>,
}

pub type ClockIn = Pin<Gpio0, FunctionSioInput, PullUp>;
pub type ModePin = Pin<Gpio20, FunctionSioInput, PullUp>;
pub type EnterPin = Pin<Gpio21, FunctionSioInput, PullUp>;
pub enum Mode {
    Input,
    Run,
}

pub type SdaPin = Pin<Gpio2, FunctionI2c, PullDown>;
pub type SclPin = Pin<Gpio3, FunctionI2c, PullDown>;
pub type I2C1Type = I2C<I2C1, (SdaPin, SclPin)>;

pub type DacType = MCP4725<I2C1Type>;

pub type DisplaySdaPin = Pin<Gpio20, FunctionI2c, PullDown>;
pub type DisplaySclPin = Pin<Gpio21, FunctionI2c, PullDown>;
pub type DisplayI2c = I2C<I2C0, (DisplaySdaPin, DisplaySclPin)>;

pub type TxPin = rp2040_hal::gpio::Pin<Gpio8, FunctionUart, PullDown>;
pub type RxPin = rp2040_hal::gpio::Pin<Gpio9, FunctionUart, PullDown>;
pub type Uart1Type = UartPeripheral<Enabled, UART1, (TxPin, RxPin)>;

pub type RotaryEncoder1 = RotaryEncoder<
    StandardMode,
    Pin<Gpio15, FunctionSioInput, PullUp>,
    Pin<Gpio14, FunctionSioInput, PullUp>,
>;

pub type RotaryEncoder2 = RotaryEncoder<
    StandardMode,
    Pin<Gpio13, FunctionSioInput, PullUp>,
    Pin<Gpio12, FunctionSioInput, PullUp>,
>;

pub type RotaryEncoder1Button = Pin<Gpio11, FunctionSioInput, PullUp>;
pub type RotaryEncoder2Button = Pin<Gpio10, FunctionSioInput, PullUp>;

pub struct ModuleState {
    pub mode: Mode,
    pub mode_pin: ModePin,
    pub enter_pin: EnterPin,
    pub note: u16,
    pub counter: Counter,
    pub clock_in: ClockIn,
    pub dac: DacType,
}
