// Raspberry Pi Pico specific core functions

use core::cell::RefCell;
use embedded_hal::digital::OutputPin;
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
