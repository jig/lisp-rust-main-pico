//! Motor control with PWM and encoder support for Raspberry Pi Pico
//!
//! This module provides control for 4 DC motors with:
//! - PWM speed control (25kHz)
//! - Direction control via dual enable pins
//! - Quadrature encoder support with GPIO interrupts

use core::cell::RefCell;
use cortex_m::interrupt::{Mutex, free};

#[cfg(rp2350)]
use crate::hal;
#[cfg(rp2040)]
use crate::hal;

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

/// Previous encoder state for quadrature decoding [motor_idx][0=A, 1=B]
static ENCODER_PREV_STATE: Mutex<RefCell<[[bool; 2]; 4]>> =
    Mutex::new(RefCell::new([[false; 2]; 4]));

/// Store PWM TOP values for duty cycle calculations
/// These are set during initialization and used in set_motor_level
static PWM_TOP_VALUES: Mutex<RefCell<[u16; 4]>> = Mutex::new(RefCell::new([0, 0, 0, 0]));

/// Track if motors have been initialized
static MOTORS_INITIALIZED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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

/// Calculate optimal PWM parameters for target frequency
fn calculate_pwm_params(sys_freq_hz: u32, target_freq_hz: u32) -> (u8, u16) {
    // Try different dividers to find optimal resolution
    let dividers = [1u32, 2, 4, 8, 16, 32, 64, 128, 256];

    for &div in &dividers {
        let pwm_clock = sys_freq_hz / div;
        let top = pwm_clock / target_freq_hz;

        if top > 0 && top <= 65536 {
            return (div as u8, (top - 1) as u16);
        }
    }

    // Fallback
    (1, 4999)
}

/// Initialize motor control hardware
/// Must be called before using motor control functions
#[cfg(rp2350)]
pub fn init_motors(
    pac_pwm: hal::pac::PWM,
    pac_io: hal::pac::IO_BANK0,
    pac_pads: hal::pac::PADS_BANK0,
    resets: &mut hal::pac::RESETS,
    sys_freq_hz: u32,
) {
    defmt::info!("Initializing motors...");

    // Calculate PWM parameters for 25kHz
    let (divider, top) = calculate_pwm_params(sys_freq_hz, MOTOR_PWM_FREQ_HZ);
    defmt::info!(
        "PWM: div={}, top={}, freq={}Hz",
        divider,
        top,
        sys_freq_hz / (divider as u32 * (top as u32 + 1))
    );

    // Store TOP values
    free(|cs| {
        let mut tops = PWM_TOP_VALUES.borrow(cs).borrow_mut();
        for i in 0..4 {
            tops[i] = top;
        }
    });

    // Reset PWM
    resets.reset().modify(|_, w| w.pwm().set_bit());
    resets.reset().modify(|_, w| w.pwm().clear_bit());
    while resets.reset_done().read().pwm().bit_is_clear() {}

    // Configure GPIO functions for PWM (funcsel=4 for PWM on RP2350)
    // GPIO6 -> PWM3A (FL)
    pac_io
        .gpio(6)
        .gpio_ctrl()
        .write(|w| unsafe { w.funcsel().bits(4) });
    pac_pads.gpio(6).write(|w| unsafe { w.bits(0x56) });

    // GPIO9 -> PWM4B (RL)
    pac_io
        .gpio(9)
        .gpio_ctrl()
        .write(|w| unsafe { w.funcsel().bits(4) });
    pac_pads.gpio(9).write(|w| unsafe { w.bits(0x56) });

    // GPIO3 -> PWM1B (FR)
    pac_io
        .gpio(3)
        .gpio_ctrl()
        .write(|w| unsafe { w.funcsel().bits(4) });
    pac_pads.gpio(3).write(|w| unsafe { w.bits(0x56) });

    // GPIO20 -> PWM2A (RR)
    pac_io
        .gpio(20)
        .gpio_ctrl()
        .write(|w| unsafe { w.funcsel().bits(4) });
    pac_pads.gpio(20).write(|w| unsafe { w.bits(0x56) });

    // Configure PWM slices
    // PWM1 (FR)
    pac_pwm
        .ch(1)
        .div()
        .write(|w| unsafe { w.bits((divider as u32) << 4) });
    pac_pwm.ch(1).top().write(|w| unsafe { w.bits(top as u32) });
    pac_pwm.ch(1).cc().write(|w| unsafe { w.bits(0) });
    pac_pwm.ch(1).csr().write(|w| unsafe { w.bits(0x0001) });

    // PWM2 (RR)
    pac_pwm
        .ch(2)
        .div()
        .write(|w| unsafe { w.bits((divider as u32) << 4) });
    pac_pwm.ch(2).top().write(|w| unsafe { w.bits(top as u32) });
    pac_pwm.ch(2).cc().write(|w| unsafe { w.bits(0) });
    pac_pwm.ch(2).csr().write(|w| unsafe { w.bits(0x0001) });

    // PWM3 (FL)
    pac_pwm
        .ch(3)
        .div()
        .write(|w| unsafe { w.bits((divider as u32) << 4) });
    pac_pwm.ch(3).top().write(|w| unsafe { w.bits(top as u32) });
    pac_pwm.ch(3).cc().write(|w| unsafe { w.bits(0) });
    pac_pwm.ch(3).csr().write(|w| unsafe { w.bits(0x0001) });

    // PWM4 (RL)
    pac_pwm
        .ch(4)
        .div()
        .write(|w| unsafe { w.bits((divider as u32) << 4) });
    pac_pwm.ch(4).top().write(|w| unsafe { w.bits(top as u32) });
    pac_pwm.ch(4).cc().write(|w| unsafe { w.bits(0) });
    pac_pwm.ch(4).csr().write(|w| unsafe { w.bits(0x0001) });

    // Configure GPIO enable pins as outputs (using SIO)
    let enable_pins = [7u8, 8, 4, 5, 10, 11, 21, 22];

    unsafe {
        let sio = &*hal::pac::SIO::PTR;

        for &pin in &enable_pins {
            // Set as output
            pac_io
                .gpio(pin as usize)
                .gpio_ctrl()
                .write(|w| w.funcsel().sio());
            pac_pads
                .gpio(pin as usize)
                .write(|w| unsafe { w.bits(0x56) });

            // Set output enable
            sio.gpio_oe_set().write(|w| w.bits(1 << pin));
            // Set low initially
            sio.gpio_out_clr().write(|w| w.bits(1 << pin));
        }
    }

    // Configure encoder pins as inputs with interrupts
    let encoder_pins = [
        (12u8, 13u8), // FL: A, B
        (19u8, 18u8), // FR: A, B
        (14u8, 15u8), // RL: A, B
        (17u8, 16u8), // RR: A, B
    ];

    unsafe {
        let sio = &*hal::pac::SIO::PTR;
        let _io = &*hal::pac::IO_BANK0::PTR;

        for (motor_idx, &(pin_a, pin_b)) in encoder_pins.iter().enumerate() {
            // Configure pin A
            pac_io
                .gpio(pin_a as usize)
                .gpio_ctrl()
                .write(|w| w.funcsel().bits(5)); // SIO
            pac_pads
                .gpio(pin_a as usize)
                .write(|w| unsafe { w.bits(0x5E) }); // IE + pullup + 8mA

            // Configure pin B
            pac_io
                .gpio(pin_b as usize)
                .gpio_ctrl()
                .write(|w| w.funcsel().bits(5)); // SIO
            pac_pads
                .gpio(pin_b as usize)
                .write(|w| unsafe { w.bits(0x5E) }); // IE + pullup + 8mA

            // Set as input (clear output enable)
            sio.gpio_oe_clr()
                .write(|w| w.bits((1 << pin_a) | (1 << pin_b)));

            // Enable edge-detect interrupts for both edges on both pins
            // Use PROC0_INTE registers for CPU0
            let shift_a = (pin_a as usize % 8) * 4;
            let mask_a = 0b1111u32 << shift_a;
            pac_io
                .proc0_inte(pin_a as usize / 8)
                .modify(|r, w| w.bits(r.bits() | mask_a));
            let shift_b = (pin_b as usize % 8) * 4;
            let mask_b = 0b1111u32 << shift_b;
            pac_io
                .proc0_inte(pin_b as usize / 8)
                .modify(|r, w| w.bits(r.bits() | mask_b));

            // Read initial state
            let gpio_in = sio.gpio_in().read().bits();
            let state_a = (gpio_in & (1 << pin_a)) != 0;
            let state_b = (gpio_in & (1 << pin_b)) != 0;

            cortex_m::interrupt::free(|cs| {
                let mut prev = ENCODER_PREV_STATE.borrow(cs).borrow_mut();
                prev[motor_idx][0] = state_a;
                prev[motor_idx][1] = state_b;
            });
        }
    }

    // Mark as initialized
    free(|cs| {
        *MOTORS_INITIALIZED.borrow(cs).borrow_mut() = true;
    });

    defmt::info!("Motors and encoders initialized successfully");
}

/// Initialize motor control hardware for RP2040
#[cfg(rp2040)]
pub fn init_motors(
    pac_pwm: hal::pac::PWM,
    pac_io: hal::pac::IO_BANK0,
    pac_pads: hal::pac::PADS_BANK0,
    resets: &mut hal::pac::RESETS,
    sys_freq_hz: u32,
) {
    defmt::info!("Initializing motors for RP2040...");

    // Calculate PWM parameters for 25kHz
    let (divider, top) = calculate_pwm_params(sys_freq_hz, MOTOR_PWM_FREQ_HZ);
    defmt::info!(
        "PWM: div={}, top={}, freq={}Hz",
        divider,
        top,
        sys_freq_hz / (divider as u32 * (top as u32 + 1))
    );

    // Store TOP values
    free(|cs| {
        let mut tops = PWM_TOP_VALUES.borrow(cs).borrow_mut();
        for i in 0..4 {
            tops[i] = top;
        }
    });

    // Reset PWM
    resets.reset().modify(|_, w| w.pwm().set_bit());
    resets.reset().modify(|_, w| w.pwm().clear_bit());
    while resets.reset_done().read().pwm().bit_is_clear() {}

    // Reset IO_BANK0 to ensure GPIO registers are writable
    resets.reset().modify(|_, w| w.io_bank0().set_bit());
    resets.reset().modify(|_, w| w.io_bank0().clear_bit());
    while resets.reset_done().read().io_bank0().bit_is_clear() {}

    // Reset PADS_BANK0
    resets.reset().modify(|_, w| w.pads_bank0().set_bit());
    resets.reset().modify(|_, w| w.pads_bank0().clear_bit());
    while resets.reset_done().read().pads_bank0().bit_is_clear() {}

    // Configure GPIO functions for PWM (funcsel=4 for PWM on RP2040)
    unsafe {
        // GPIO6 -> PWM3A (FL) - ONLY funcsel=4, NO override bits
        pac_io.gpio(6).gpio_ctrl().write(|w| w.bits(0x0004));
        pac_pads.gpio(6).write(|w| {
            w.od().clear_bit();
            w.ie().set_bit()
        });

        // GPIO9 -> PWM4B (RL)
        pac_io.gpio(9).gpio_ctrl().write(|w| w.bits(0x0004));
        pac_pads.gpio(9).write(|w| {
            w.od().clear_bit();
            w.ie().set_bit()
        });

        // GPIO3 -> PWM1B (FR)
        pac_io.gpio(3).gpio_ctrl().write(|w| w.bits(0x0004));
        pac_pads.gpio(3).write(|w| {
            w.od().clear_bit();
            w.ie().set_bit()
        });

        // GPIO20 -> PWM2A (RR)
        pac_io.gpio(20).gpio_ctrl().write(|w| w.bits(0x0004));
        pac_pads.gpio(20).write(|w| {
            w.od().clear_bit();
            w.ie().set_bit()
        });

        // Configure PWM slices
        // PWM1 (FR)
        pac_pwm.ch(1).div().write(|w| w.int().bits(divider));
        pac_pwm.ch(1).top().write(|w| w.bits(top as u32));
        pac_pwm.ch(1).cc().write(|w| w.bits(0));
        pac_pwm.ch(1).csr().write(|w| w.en().set_bit());

        // PWM2 (RR)
        pac_pwm.ch(2).div().write(|w| w.int().bits(divider));
        pac_pwm.ch(2).top().write(|w| w.bits(top as u32));
        pac_pwm.ch(2).cc().write(|w| w.bits(0));
        pac_pwm.ch(2).csr().write(|w| w.en().set_bit());

        // PWM3 (FL)
        pac_pwm.ch(3).div().write(|w| w.int().bits(divider));
        pac_pwm.ch(3).top().write(|w| w.bits(top as u32));
        pac_pwm.ch(3).cc().write(|w| w.bits(0));
        pac_pwm.ch(3).csr().write(|w| w.en().set_bit());

        // PWM4 (RL)
        pac_pwm.ch(4).div().write(|w| w.int().bits(divider));
        pac_pwm.ch(4).top().write(|w| w.bits(top as u32));
        pac_pwm.ch(4).cc().write(|w| w.bits(0));
        pac_pwm.ch(4).csr().write(|w| w.en().set_bit());
    }

    // Configure GPIO enable pins as outputs (using SIO)
    let enable_pins = [7u8, 8, 4, 5, 10, 11, 21, 22];

    unsafe {
        let sio = &*hal::pac::SIO::PTR;

        for &pin in &enable_pins {
            // Set as output
            pac_io
                .gpio(pin as usize)
                .gpio_ctrl()
                .write(|w| w.funcsel().sio());
            pac_pads.gpio(pin as usize).write(|w| {
                w.od().clear_bit();
                w.ie().set_bit()
            });

            // Set output enable
            sio.gpio_oe_set().write(|w| w.bits(1 << pin));
            // Set low initially
            sio.gpio_out_clr().write(|w| w.bits(1 << pin));
        }
    }

    // Configure encoder pins as inputs with interrupts
    let encoder_pins = [
        (12u8, 13u8), // FL: A, B
        (19u8, 18u8), // FR: A, B
        (14u8, 15u8), // RL: A, B
        (17u8, 16u8), // RR: A, B
    ];

    unsafe {
        let sio = &*hal::pac::SIO::PTR;
        let _io = &*hal::pac::IO_BANK0::PTR;

        for (motor_idx, &(pin_a, pin_b)) in encoder_pins.iter().enumerate() {
            // Configure pin A (funcsel=5 for SIO)
            pac_io
                .gpio(pin_a as usize)
                .gpio_ctrl()
                .write(|w| unsafe { w.bits(0x0005) });
            pac_pads.gpio(pin_a as usize).write(|w| {
                w.od().clear_bit();
                w.ie().set_bit();
                w.pue().set_bit() // Pull-up enable
            });

            // Configure pin B (funcsel=5 for SIO)
            pac_io
                .gpio(pin_b as usize)
                .gpio_ctrl()
                .write(|w| unsafe { w.bits(0x0005) });
            pac_pads.gpio(pin_b as usize).write(|w| {
                w.od().clear_bit();
                w.ie().set_bit();
                w.pue().set_bit() // Pull-up enable
            });

            // Set as input (clear output enable)
            sio.gpio_oe_clr()
                .write(|w| w.bits((1 << pin_a) | (1 << pin_b)));

            // Enable edge-detect interrupts for both edges on both pins
            // Use PROC0_INTE registers for CPU0
            let shift_a = (pin_a as usize % 8) * 4;
            let mask_a = 0b1111u32 << shift_a;
            pac_io
                .proc0_inte(pin_a as usize / 8)
                .modify(|r, w| w.bits(r.bits() | mask_a));
            let shift_b = (pin_b as usize % 8) * 4;
            let mask_b = 0b1111u32 << shift_b;
            pac_io
                .proc0_inte(pin_b as usize / 8)
                .modify(|r, w| w.bits(r.bits() | mask_b));

            // Read initial state
            let gpio_in = sio.gpio_in().read().bits();
            let state_a = (gpio_in & (1 << pin_a)) != 0;
            let state_b = (gpio_in & (1 << pin_b)) != 0;

            cortex_m::interrupt::free(|cs| {
                let mut prev = ENCODER_PREV_STATE.borrow(cs).borrow_mut();
                prev[motor_idx][0] = state_a;
                prev[motor_idx][1] = state_b;
            });
        }
    }

    // Mark as initialized
    free(|cs| {
        *MOTORS_INITIALIZED.borrow(cs).borrow_mut() = true;
    });

    defmt::info!("Motors and encoders initialized successfully (RP2040)");
}

/// Set motor level for a specific motor using direct PAC access
/// Safe to call from interrupts
pub fn set_motor_level(motor_idx: usize, level: i32) {
    if motor_idx > 3 {
        defmt::warn!("Invalid motor index: {}", motor_idx);
        return;
    }

    // Check if initialized
    let initialized = free(|cs| *MOTORS_INITIALIZED.borrow(cs).borrow());
    if !initialized {
        defmt::warn!("Motors not initialized, call init_motors first!");
        return;
    }

    defmt::debug!("set_motor_level: motor={} level={}", motor_idx, level);

    #[cfg(rp2350)]
    unsafe {
        let pwm = &*hal::pac::PWM::PTR;
        let sio = &*hal::pac::SIO::PTR;

        // Map motor index to PWM slice and channel, GPIO pins
        let (slice_num, channel, ena0_pin, ena1_pin) = match motor_idx {
            0 => (3, 0, 7, 8),   // FL: PWM3A, GPIO7, GPIO8
            1 => (1, 1, 4, 5),   // FR: PWM1B, GPIO4, GPIO5
            2 => (4, 1, 10, 11), // RL: PWM4B, GPIO10, GPIO11
            3 => (2, 0, 21, 22), // RR: PWM2A, GPIO21, GPIO22
            _ => return,
        };

        if level == 0 {
            // Stop motor
            sio.gpio_out_clr()
                .write(|w| w.bits((1 << ena0_pin) | (1 << ena1_pin)));

            // Set PWM to 0
            if channel == 0 {
                pwm.ch(slice_num).cc().modify(|_, w| w.a().bits(0));
            } else {
                pwm.ch(slice_num).cc().modify(|_, w| w.b().bits(0));
            }
        } else if level < 0 {
            // Reverse direction
            let abs_level = if level < -MAX_PWM_LEVEL {
                MAX_PWM_LEVEL as u16
            } else {
                (-level) as u16
            };

            // ENA0=high, ENA1=low
            sio.gpio_out_set().write(|w| w.bits(1 << ena0_pin));
            sio.gpio_out_clr().write(|w| w.bits(1 << ena1_pin));

            // Set PWM level
            if channel == 0 {
                pwm.ch(slice_num).cc().modify(|_, w| w.a().bits(abs_level));
            } else {
                pwm.ch(slice_num).cc().modify(|_, w| w.b().bits(abs_level));
            }
        } else {
            // Forward direction
            let abs_level = if level > MAX_PWM_LEVEL {
                MAX_PWM_LEVEL as u16
            } else {
                level as u16
            };

            // ENA0=low, ENA1=high
            sio.gpio_out_clr().write(|w| w.bits(1 << ena0_pin));
            sio.gpio_out_set().write(|w| w.bits(1 << ena1_pin));

            // Set PWM level
            if channel == 0 {
                pwm.ch(slice_num).cc().modify(|_, w| w.a().bits(abs_level));
            } else {
                pwm.ch(slice_num).cc().modify(|_, w| w.b().bits(abs_level));
            }
        }
    }

    #[cfg(rp2040)]
    unsafe {
        defmt::info!(
            "set_motor_level rp2040: motor={} level={}",
            motor_idx,
            level
        );
        let pwm = &*hal::pac::PWM::PTR;
        let sio = &*hal::pac::SIO::PTR;

        // Map motor index to PWM slice and channel, GPIO pins
        let (slice_num, channel, ena0_pin, ena1_pin) = match motor_idx {
            0 => (3, 0, 7, 8),   // FL: PWM3A, GPIO7, GPIO8
            1 => (1, 1, 4, 5),   // FR: PWM1B, GPIO4, GPIO5
            2 => (4, 1, 10, 11), // RL: PWM4B, GPIO10, GPIO11
            3 => (2, 0, 21, 22), // RR: PWM2A, GPIO21, GPIO22
            _ => return,
        };

        if level == 0 {
            // Stop motor
            sio.gpio_out_clr()
                .write(|w| w.bits((1 << ena0_pin) | (1 << ena1_pin)));

            // Set PWM to 0
            let cc_val = pwm.ch(slice_num).cc().read().bits();
            if channel == 0 {
                // Clear A, preserve B
                pwm.ch(slice_num)
                    .cc()
                    .write(|w| w.bits(cc_val & 0xFFFF0000));
            } else {
                // Clear B, preserve A
                pwm.ch(slice_num)
                    .cc()
                    .write(|w| w.bits(cc_val & 0x0000FFFF));
            }
        } else if level < 0 {
            // Reverse direction
            let abs_level = if level < -MAX_PWM_LEVEL {
                MAX_PWM_LEVEL as u16
            } else {
                (-level) as u16
            };

            // ENA0=high, ENA1=low
            sio.gpio_out_set().write(|w| w.bits(1 << ena0_pin));
            sio.gpio_out_clr().write(|w| w.bits(1 << ena1_pin));

            // Set PWM level
            let cc_val = pwm.ch(slice_num).cc().read().bits();
            if channel == 0 {
                // Set A, preserve B
                pwm.ch(slice_num)
                    .cc()
                    .write(|w| w.bits((cc_val & 0xFFFF0000) | (abs_level as u32)));
            } else {
                // Set B, preserve A
                pwm.ch(slice_num)
                    .cc()
                    .write(|w| w.bits((cc_val & 0x0000FFFF) | ((abs_level as u32) << 16)));
            }
        } else {
            // Forward direction
            let abs_level = if level > MAX_PWM_LEVEL {
                MAX_PWM_LEVEL as u16
            } else {
                level as u16
            };

            // ENA0=low, ENA1=high
            sio.gpio_out_clr().write(|w| w.bits(1 << ena0_pin));
            sio.gpio_out_set().write(|w| w.bits(1 << ena1_pin));

            // Set PWM level
            let cc_val = pwm.ch(slice_num).cc().read().bits();
            if channel == 0 {
                // Set A, preserve B
                let new_val = (cc_val & 0xFFFF0000) | (abs_level as u32);
                pwm.ch(slice_num).cc().write(|w| w.bits(new_val));

                // DEBUG: Read back to verify
                let verify = pwm.ch(slice_num).cc().read().bits();
                let gpio_state = sio.gpio_out().read().bits();
                defmt::debug!(
                    "PWM written: new_val={:#010x}, verify={:#010x}, gpio={:#010x}",
                    new_val,
                    verify,
                    gpio_state
                );
            } else {
                // Set B, preserve A
                let new_val = (cc_val & 0x0000FFFF) | ((abs_level as u32) << 16);
                pwm.ch(slice_num).cc().write(|w| w.bits(new_val));

                // DEBUG: Read back to verify
                let verify = pwm.ch(slice_num).cc().read().bits();
                let gpio_state = sio.gpio_out().read().bits();
                defmt::debug!(
                    "PWM written: new_val={:#010x}, verify={:#010x}, gpio={:#010x}",
                    new_val,
                    verify,
                    gpio_state
                );
            }
        }
    }
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

/// Get all encoder counts
pub fn get_all_encoders() -> [i32; 4] {
    free(|cs| *ENCODERS.borrow(cs).borrow())
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

/// Process encoder interrupts and update quadrature counters
/// Must be called from IO_IRQ_BANK0 interrupt handler
pub fn process_encoder_interrupts() {
    unsafe {
        let sio = &*hal::pac::SIO::PTR;
        let io = &*hal::pac::IO_BANK0::PTR;

        // Encoder pin mappings [motor_idx] -> (pin_a, pin_b)
        let encoder_pins = [
            (12u8, 13u8), // FL: A, B
            (19u8, 18u8), // FR: A, B
            (14u8, 15u8), // RL: A, B
            (17u8, 16u8), // RR: A, B
        ];

        // Read current GPIO state
        let gpio_in = sio.gpio_in().read().bits();

        free(|cs| {
            let mut encoders = ENCODERS.borrow(cs).borrow_mut();
            let mut prev_state = ENCODER_PREV_STATE.borrow(cs).borrow_mut();

            for (motor_idx, &(pin_a, pin_b)) in encoder_pins.iter().enumerate() {
                // Check if interrupt is pending for either pin using PROC0_INTS
                let ints_reg_idx = pin_a as usize / 8;
                let ints_status = io.proc0_ints(ints_reg_idx).read().bits();

                let pin_a_shift = (pin_a as usize % 8) * 4;
                let pin_b_shift = (pin_b as usize % 8) * 4;

                let pin_a_int = (ints_status & (0b1111 << pin_a_shift)) != 0;
                let pin_b_int = (ints_status & (0b1111 << pin_b_shift)) != 0;

                if !pin_a_int && !pin_b_int {
                    continue; // No interrupt for this encoder
                }

                // Read current state
                let curr_a = (gpio_in & (1 << pin_a)) != 0;
                let curr_b = (gpio_in & (1 << pin_b)) != 0;

                // Get previous state
                let prev_a = prev_state[motor_idx][0];
                let prev_b = prev_state[motor_idx][1];

                // Quadrature decoding state machine
                // Standard gray code sequence: 00 -> 10 -> 11 -> 01 -> 00 (forward)
                //                             00 -> 01 -> 11 -> 10 -> 00 (reverse)
                let state = ((prev_a as u8) << 3)
                    | ((prev_b as u8) << 2)
                    | ((curr_a as u8) << 1)
                    | (curr_b as u8);

                match state {
                    // Forward transitions
                    0b0010 | 0b1011 | 0b1101 | 0b0100 => encoders[motor_idx] += 1,
                    // Reverse transitions
                    0b0001 | 0b0111 | 0b1110 | 0b1000 => encoders[motor_idx] -= 1,
                    // No change or invalid transitions
                    _ => {}
                }

                // Update previous state
                prev_state[motor_idx][0] = curr_a;
                prev_state[motor_idx][1] = curr_b;

                // Clear interrupts for these pins by writing to INTR (edge status register)
                io.intr(ints_reg_idx)
                    .write(|w| w.bits((0b1111 << pin_a_shift) | (0b1111 << pin_b_shift)));
            }
        });
    }
}
