use rotary_encoder_embedded::Direction;
use rp2040_hal::timer::Alarm;

use crate::{interrupt, ModuleState, MODULE_STATE};

#[interrupt]
fn TIMER_IRQ_0() {
    // info!("alarm 0 fired and caught");
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut alarm_0_duration,
            mut alarm_0,
            ..
        } = module_state;
        alarm_0.clear_interrupt();
        alarm_0.schedule(alarm_0_duration);
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                alarm_0_duration,
                alarm_0,
                ..module_state
            }))
        }
    });
}

#[interrupt]
fn TIMER_IRQ_1() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut alarm_1_duration,
            mut alarm_1,
            ..
        } = module_state;
        alarm_1.clear_interrupt();
        alarm_1.schedule(alarm_1_duration);
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                alarm_1_duration,
                alarm_1,
                ..module_state
            }))
        }
    });
}

#[interrupt]
fn TIMER_IRQ_2() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut alarm_2_duration,
            mut alarm_2,
            ..
        } = module_state;
        alarm_2.clear_interrupt();
        alarm_2.schedule(alarm_2_duration);
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                alarm_2_duration,
                alarm_2,
                ..module_state
            }))
        }
    });
}
