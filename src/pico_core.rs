// Raspberry Pi Pico specific core functions

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use cortex_m::interrupt::free;
use embedded_hal::digital::OutputPin;
use mal::printer::pr_seq;
use mal::types::MalVal::{Bool, Int, Nil};
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

/// Report free heap memory
/// Usage: (mem-free)
pub fn mem_free(_args: MalArgs) -> MalRet {
    Ok(Int(1024 * 128))
}

// ============================================================================
// PID Controller Functions
// ============================================================================

/// Set PID gains
/// Usage: (pid-set-gains kp ki kd)
pub fn pid_set_gains(args: MalArgs) -> MalRet {
    if args.len() != 3 {
        return error("pid-set-gains expects 3 arguments (kp ki kd)");
    }

    let kp = match &args[0] {
        Int(i) => *i as f32,
        _ => return error("kp must be a number"),
    };

    let ki = match &args[1] {
        Int(i) => *i as f32,
        _ => return error("ki must be a number"),
    };

    let kd = match &args[2] {
        Int(i) => *i as f32,
        _ => return error("kd must be a number"),
    };

    free(|cs| {
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.set_gains(kp, ki, kd);
    });

    Ok(Nil)
}

/// Set PID setpoint
/// Usage: (pid-set-setpoint value)
pub fn pid_set_setpoint(args: MalArgs) -> MalRet {
    if args.len() != 1 {
        return error("pid-set-setpoint expects 1 argument");
    }

    let setpoint = match &args[0] {
        Int(i) => *i as f32,
        _ => return error("setpoint must be a number"),
    };

    free(|cs| {
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.setpoint = setpoint;
    });

    Ok(Nil)
}

/// Enable/disable PID controller
/// Usage: (pid-enable true) or (pid-enable false)
pub fn pid_enable(args: MalArgs) -> MalRet {
    if args.len() != 1 {
        return error("pid-enable expects 1 argument (true/false)");
    }

    let enabled = match &args[0] {
        Bool(b) => *b,
        _ => return error("argument must be boolean"),
    };

    free(|cs| {
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.enabled = enabled;
    });

    Ok(Nil)
}

/// Reset PID controller state
/// Usage: (pid-reset)
pub fn pid_reset(_args: MalArgs) -> MalRet {
    free(|cs| {
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.reset();
    });

    Ok(Nil)
}

/// Set PID output limits
/// Usage: (pid-set-limits min max)
pub fn pid_set_limits(args: MalArgs) -> MalRet {
    if args.len() != 2 {
        return error("pid-set-limits expects 2 arguments (min max)");
    }

    let min = match &args[0] {
        Int(i) => *i as f32,
        _ => return error("min must be a number"),
    };

    let max = match &args[1] {
        Int(i) => *i as f32,
        _ => return error("max must be a number"),
    };

    free(|cs| {
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.set_output_limits(min, max);
    });

    Ok(Nil)
}
