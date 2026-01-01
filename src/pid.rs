// PID Controller implementation

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

/// PID controller state
pub struct PidController {
    // PID gains
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    // Setpoint and limits
    pub setpoint: f32,
    pub output_min: f32,
    pub output_max: f32,

    // Time delta (dt) in seconds
    pub dt: f32,

    // Internal state
    integral: f32,
    prev_error: f32,

    // Enable/disable
    pub enabled: bool,
}

impl PidController {
    /// Create a new PID controller with default values
    pub const fn new() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            setpoint: 0.0,
            output_min: -100.0,
            output_max: 100.0,
            dt: 0.02, // Default 20ms
            integral: 0.0,
            prev_error: 0.0,
            enabled: false,
        }
    }

    /// Reset the PID controller state
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    /// Update the PID controller with current measurement
    /// Returns the control output
    #[allow(dead_code)]
    pub fn update(&mut self, measurement: f32, dt: f32) -> f32 {
        if !self.enabled {
            return 0.0;
        }

        // Calculate error
        let error = self.setpoint - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term
        self.integral += error * dt;
        let i_term = self.ki * self.integral;

        // Derivative term
        let derivative = (error - self.prev_error) / dt;
        let d_term = self.kd * derivative;

        // Store error for next iteration
        self.prev_error = error;

        // Calculate output
        let output = p_term + i_term + d_term;

        // Clamp output
        output.clamp(self.output_min, self.output_max)
    }

    /// Set PID gains
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Set output limits
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.output_min = min;
        self.output_max = max;
    }
}

/// Global PID controller instance
pub static GLOBAL_PID: Mutex<RefCell<PidController>> =
    Mutex::new(RefCell::new(PidController::new()));

/// PID update function to be called from ISR
/// dt should be in seconds (e.g., 0.2 for 200ms)
#[allow(dead_code)]
pub fn pid_update_isr(measurement: f32, dt: f32) -> f32 {
    cortex_m::interrupt::free(|cs| {
        let mut pid = GLOBAL_PID.borrow(cs).borrow_mut();
        pid.update(measurement, dt)
    })
}
