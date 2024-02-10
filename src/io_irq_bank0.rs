use crate::{info, interrupt, pac, ModuleState, Pins, Sio, MODULE_STATE};
use rp2040_hal::gpio::Interrupt::EdgeHigh;

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut encoder_1_button,
            mut encoder_2_button,
            mut display,
            ..
        } = module_state;
        if encoder_1_button.interrupt_status(EdgeHigh) {
            display.clear().ok();
            for c in ['b', 'u', 't', 't', '1'] {
                display.print_char(c).ok();
            }
            encoder_1_button.clear_interrupt(EdgeHigh);
        }
        if encoder_2_button.interrupt_status(EdgeHigh) {
            display.clear().ok();
            for c in ['b', 'u', 't', 't', '2'] {
                display.print_char(c).ok();
            }
            encoder_2_button.clear_interrupt(EdgeHigh);
        }
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                encoder_1_button,
                encoder_2_button,
                display,
                ..module_state
            }))
        }
    });
}
