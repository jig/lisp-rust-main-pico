//! Motor control with PWM and encoder support for Raspberry Pi Pico
//!
//! This module provides control for 4 DC motors with:
//! - PWM speed control (25kHz)
//! - Direction control via dual enable pins
//! - Quadrature encoder support with GPIO interrupts
//!
//! NOTE: This is a simplified version. Full PWM and GPIO initialization
//! will be added in a future update. Currently provides encoder tracking
//! and motor control API stubs.

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};

// Pin definitions matching the C implementation
// Front Left Motor
pub const ENABLE_MOTOR_FL_PWM: u8 = 6; // PWM_3A
pub const ENABLE_MOTOR_FL_ENA_0: u8 = 7;
pub const ENABLE_MOTOR_FL_ENA_1: u8 = 8;

// Rear Left Motor
pub const ENABLE_MOTOR_RL_PWM: u8 = 9; // PWM_4B
pub const ENABLE_MOTOR_RL_ENA_0: u8 = 10;
pub const ENABLE_MOTOR_RL_ENA_1: u8 = 11;

// Front Right Motor
pub const ENABLE_MOTOR_FR_PWM: u8 = 3; // PWM_1B
pub const ENABLE_MOTOR_FR_ENA_0: u8 = 4;
pub const ENABLE_MOTOR_FR_ENA_1: u8 = 5;

// Rear Right Motor
pub const ENABLE_MOTOR_RR_PWM: u8 = 20; // PWM_2A
pub const ENABLE_MOTOR_RR_ENA_0: u8 = 21;
pub const ENABLE_MOTOR_RR_ENA_1: u8 = 22;

// Encoder pins
pub const ENCODER_MOTOR_FL_A: u8 = 12;
pub const ENCODER_MOTOR_FL_B: u8 = 13;
pub const ENCODER_MOTOR_RL_A: u8 = 14;
pub const ENCODER_MOTOR_RL_B: u8 = 15;
pub const ENCODER_MOTOR_FR_A: u8 = 19;
pub const ENCODER_MOTOR_FR_B: u8 = 18;
pub const ENCODER_MOTOR_RR_A: u8 = 17;
pub const ENCODER_MOTOR_RR_B: u8 = 16;

// PWM frequency for motors (25kHz)
pub const MOTOR_PWM_FREQ_HZ: u32 = 25000;

// Maximum PWM level (to match C implementation limit of 4999)
pub const MAX_PWM_LEVEL: i32 = 4999;

/// Global encoder counters [FL, FR, RL, RR]
static ENCODERS: Mutex<RefCell<[i32; 4]>> = Mutex::new(RefCell::new([0, 0, 0, 0]));

/// Motor index enum for clearer API
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorIndex {
    FrontLeft = 0,
    FrontRight = 1,
    RearLeft = 2,
    RearRight = 3,
}

/// Motor control structure
#[allow(dead_code)]
pub struct MotorController {
    _private: (),
}

#[allow(dead_code)]
impl MotorController {
    /// Create a new motor controller instance
    pub const fn new() -> Self {
        Self { _private: () }
    }

    /// Get encoder count for a specific motor
    pub fn get_encoder(&self, motor: MotorIndex) -> i32 {
        free(|cs| ENCODERS.borrow(cs).borrow()[motor as usize])
    }

    /// Reset encoder count for a specific motor
    pub fn reset_encoder(&mut self, motor: MotorIndex) {
        free(|cs| ENCODERS.borrow(cs).borrow_mut()[motor as usize] = 0);
    }

    /// Reset all encoder counts
    pub fn reset_all_encoders(&mut self) {
        free(|cs| *ENCODERS.borrow(cs).borrow_mut() = [0, 0, 0, 0]);
    }
}

// ============================================================================
// Global Motor Control Functions (callable from interrupts and Lisp)
// ============================================================================

/// Set motor level for a specific motor
/// Safe to call from interrupts
///
/// NOTE: This is currently a stub. Full PWM implementation will be added
/// when proper pin management is implemented.
pub fn set_motor_level(motor_idx: usize, level: i32) {
    if motor_idx > 3 {
        return;
    }

    // For now, just log the intent
    defmt::info!("Motor {} set to level {}", motor_idx, level);

    // TODO: Implement actual PWM and GPIO control
    // This requires storing Pin references in global static
    // which needs careful design with type-erased pins or DynPin
}

/// Set all motor levels at once
pub fn set_all_motors(fl: i32, fr: i32, rl: i32, rr: i32) {
    set_motor_level(0, fl);
    set_motor_level(1, fr);
    set_motor_level(2, rl);
    set_motor_level(3, rr);
}

/// Get encoder count for a motor
pub fn get_encoder_count(motor_idx: usize) -> i32 {
    if motor_idx > 3 {
        return 0;
    }

    free(|cs| ENCODERS.borrow(cs).borrow()[motor_idx])
}

/// Reset encoder for a specific motor
pub fn reset_encoder(motor_idx: usize) {
    if motor_idx > 3 {
        return;
    }

    free(|cs| ENCODERS.borrow(cs).borrow_mut()[motor_idx] = 0);
}

/// Reset all encoders
pub fn reset_all_encoders() {
    free(|cs| *ENCODERS.borrow(cs).borrow_mut() = [0, 0, 0, 0]);
}

// TODO: Full motor initialization function to be added
// This will include:
// - PWM slice configuration for all 4 motors
// - GPIO pin setup for enable signals (ENA_0, ENA_1)
// - Encoder GPIO configuration with interrupts
// - Quadrature encoder decoding in ISR
