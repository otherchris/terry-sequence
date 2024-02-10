use crate::{interrupt, ModuleState, MODULE_STATE};

//Handle UART data
#[interrupt]
fn UART1_IRQ() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState { mut uart_1, .. } = module_state;
        // if uart.uart_is_readable() {
        //     let buf = singleton!(: [u8; 5] = [0; 5]).unwrap();
        //     uart.read_full_blocking(buf).unwrap();
        //     info!("{}", buf);
        // }

        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                uart_1,
                ..module_state
            }))
        };
    })
}
