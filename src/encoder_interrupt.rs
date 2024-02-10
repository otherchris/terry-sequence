use rotary_encoder_embedded::Direction;
use rp2040_hal::timer::Alarm;

use crate::{info, interrupt, pac, ModuleState, Pins, Sio, MODULE_STATE};

#[interrupt]
fn TIMER_IRQ_3() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut encoder_poll_duration,
            mut encoder_poll_alarm,
            mut encoder_1,
            mut encoder_2,
            mut display,
            ..
        } = module_state;
        encoder_poll_alarm.clear_interrupt();
        encoder_poll_alarm.schedule(encoder_poll_duration).ok();

        encoder_1.update();
        encoder_2.update();
        match encoder_1.direction() {
            Direction::Clockwise => {
                display.clear().ok();
                for c in ['c', 'l', 'o', 'c', 'k', '1'] {
                    display.print_char(c).ok();
                }
            }
            Direction::Anticlockwise => {
                display.clear().ok();
                for c in ['a', 'n', 't', 'i', '1'] {
                    display.print_char(c).ok();
                }
            }
            Direction::None => {
                // info!("None")
            }
        }
        match encoder_2.direction() {
            Direction::Clockwise => {
                display.clear().ok();
                for c in ['c', 'l', 'o', 'c', 'k', '2'] {
                    display.print_char(c).ok();
                }
            }
            Direction::Anticlockwise => {
                display.clear().ok();
                for c in ['a', 'n', 't', 'i', '2'] {
                    display.print_char(c).ok();
                }
            }
            Direction::None => {
                // info!("None")
            }
        }
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                encoder_poll_duration,
                encoder_poll_alarm,
                encoder_1,
                encoder_2,
                display,
                ..module_state
            }))
        }
    });
}
