// Raspberry Pi Pico specific core functions

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use embedded_hal::digital::OutputPin;
use mal::printer::pr_seq;
use mal::types::MalVal::{Bool, Nil};
use mal::types::{MalArgs, MalRet, error};

/// Create a closure that controls the LED pin
/// Usage: (led true) to turn on, (led false) to turn off
pub fn create_led_func<P: OutputPin>(
    led_pin: &'static RefCell<P>,
) -> impl Fn(MalArgs) -> MalRet + 'static {
    move |args: MalArgs| {
        if args.len() != 1 {
            return error("led expects 1 argument (true/false)");
        }
        match &args[0] {
            Bool(true) => {
                let _ = led_pin.borrow_mut().set_high();
                Ok(Nil)
            }
            Bool(false) => {
                let _ = led_pin.borrow_mut().set_low();
                Ok(Nil)
            }
            _ => error("led expects boolean argument"),
        }
    }
}

/// Print values to UART with readability
/// Usage: (prn arg1 arg2 ...)
pub fn prn(args: MalArgs) -> MalRet {
    use crate::GLOBAL_UART0;

    let s = pr_seq(&args, true, "", "", " ");
    defmt::info!("{}", &s.as_str());

    // SAFETY: Safe because GLOBAL_UART0 is initialized before prn is called
    unsafe {
        let uart0 = &mut *addr_of_mut!(GLOBAL_UART0).read().unwrap();
        uart0.write_full_blocking(s.as_bytes());
    }
    Ok(Nil)
}

/// Print values to UART with newline
/// Usage: (println arg1 arg2 ...)
pub fn println(args: MalArgs) -> MalRet {
    use crate::GLOBAL_UART0;

    let s = pr_seq(&args, false, "", "", " ");
    defmt::info!("{}", &s.as_str());

    // SAFETY: Safe because GLOBAL_UART0 is initialized before println is called
    unsafe {
        let uart0 = &mut *addr_of_mut!(GLOBAL_UART0).read().unwrap();
        uart0.write_full_blocking(s.as_bytes());
        uart0.write_full_blocking(b"\r\n");
    }
    Ok(Nil)
}
