// Raspberry Pi Pico specific core functions

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use cortex_m::interrupt::free;
use embedded_hal::digital::OutputPin;
use mal::printer::pr_seq;
use mal::types::MalVal::{Bool, Float, Int, Nil};
use mal::types::{MalArgs, MalRet, error};

/// Create a closure that controls the LED pin
/// Usage: (led true) to turn on, (led false) to turn off
#[allow(dead_code)]
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

/// Set PID delta time (timer period)
/// Usage: (pid-set-dt 0.02) for 20ms
pub fn pid_set_dt(args: MalArgs) -> MalRet {
    if args.len() != 1 {
        return error("pid-set-dt expects 1 argument (dt in seconds)");
    }

    let dt = match &args[0] {
        Int(i) => *i as f32,
        Float(f) => *f,
        _ => return error("dt must be a number"),
    };

    if dt <= 0.0 {
        return error("dt must be positive");
    }

    if dt < 0.0005 {
        return error("dt too small (min 0.0005s / 2kHz)");
    }

    if dt > 0.5 {
        return error("dt too large (max 0.5s / 2Hz)");
    }

    // Convert seconds to microseconds for the timer
    let period_us = (dt * 1_000_000.0) as u32;

    free(|cs| {
        // Update timer period
        let mut timer_period = crate::TIMER_PERIOD_US.borrow(cs).borrow_mut();
        *timer_period = period_us;

        // Update PID dt
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.dt = dt;
    });

    Ok(Nil)
}

/// Set PID gains
/// Usage: (pid-set-gains kp ki kd)
pub fn pid_set_gains(args: MalArgs) -> MalRet {
    if args.len() != 3 {
        return error("pid-set-gains expects 3 arguments (kp ki kd)");
    }

    let kp = match &args[0] {
        Int(i) => *i as f32,
        Float(f) => *f,
        _ => return error("kp must be a number"),
    };

    let ki = match &args[1] {
        Int(i) => *i as f32,
        Float(f) => *f,
        _ => return error("ki must be a number"),
    };

    let kd = match &args[2] {
        Int(i) => *i as f32,
        Float(f) => *f,
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
        Float(f) => *f,
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
        Float(f) => *f,
        _ => return error("min must be a number"),
    };

    let max = match &args[1] {
        Int(i) => *i as f32,
        Float(f) => *f,
        _ => return error("max must be a number"),
    };

    free(|cs| {
        let mut pid = crate::pid::GLOBAL_PID.borrow(cs).borrow_mut();
        pid.set_output_limits(min, max);
    });

    Ok(Nil)
}

// ============================================================================
// Motor Control Functions
// ============================================================================

/// Move a single motor
/// Usage: (motor-move motor-index level)
/// motor-index: 0=FL, 1=FR, 2=RL, 3=RR
/// level: -4999 to 4999 (negative = reverse, positive = forward, 0 = stop)
pub fn motor_move(args: MalArgs) -> MalRet {
    if args.len() != 2 {
        return error("motor-move expects 2 arguments (motor-index level)");
    }

    let motor_idx = match &args[0] {
        Int(i) => *i,
        _ => return error("motor-index must be an integer"),
    };

    if motor_idx < 0 || motor_idx > 3 {
        return error("motor-index must be 0-3 (FL, FR, RL, RR)");
    }

    let level = match &args[1] {
        Int(i) => *i as i32,
        Float(f) => *f as i32,
        _ => return error("level must be a number"),
    };

    if level < -4999 || level > 4999 {
        return error("level must be between -4999 and 4999");
    }

    free(|_cs| {
        crate::motor::set_motor_level(motor_idx as usize, level);
    });

    Ok(Nil)
}

/// Move all motors at once
/// Usage: (motors-move fl fr rl rr)
/// Each parameter: -4999 to 4999
pub fn motors_move(args: MalArgs) -> MalRet {
    if args.len() != 4 {
        return error("motors-move expects 4 arguments (fl fr rl rr)");
    }

    let mut levels = [0i32; 4];
    for i in 0..4 {
        levels[i] = match &args[i] {
            Int(val) => *val as i32,
            Float(f) => *f as i32,
            _ => return error("all levels must be numbers"),
        };

        if levels[i] < -4999 || levels[i] > 4999 {
            return error("all levels must be between -4999 and 4999");
        }
    }

    free(|_cs| {
        crate::motor::set_all_motors(levels[0], levels[1], levels[2], levels[3]);
    });

    Ok(Nil)
}

/// Get encoder count for a motor
/// Usage: (encoder-get motor-index)
/// motor-index: 0=FL, 1=FR, 2=RL, 3=RR
pub fn encoder_get(args: MalArgs) -> MalRet {
    if args.len() != 1 {
        return error("encoder-get expects 1 argument (motor-index)");
    }

    let motor_idx = match &args[0] {
        Int(i) => *i,
        _ => return error("motor-index must be an integer"),
    };

    if motor_idx < 0 || motor_idx > 3 {
        return error("motor-index must be 0-3 (FL, FR, RL, RR)");
    }

    let count = free(|_cs| crate::motor::get_encoder_count(motor_idx as usize));

    Ok(Int(count as i64))
}

/// Reset encoder count for a motor
/// Usage: (encoder-reset motor-index)
pub fn encoder_reset(args: MalArgs) -> MalRet {
    if args.len() != 1 {
        return error("encoder-reset expects 1 argument (motor-index)");
    }

    let motor_idx = match &args[0] {
        Int(i) => *i,
        _ => return error("motor-index must be an integer"),
    };

    if motor_idx < 0 || motor_idx > 3 {
        return error("motor-index must be 0-3 (FL, FR, RL, RR)");
    }

    free(|_cs| {
        crate::motor::reset_encoder(motor_idx as usize);
    });

    Ok(Nil)
}

/// Reset all encoder counts
/// Usage: (encoders-reset)
pub fn encoders_reset(_args: MalArgs) -> MalRet {
    free(|_cs| {
        crate::motor::reset_all_encoders();
    });

    Ok(Nil)
}
