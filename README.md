# Borinot Firmware - Motor Control & PID

**Lisp-based motor control firmware for Raspberry Pi Pico/Pico 2 with integrated PID controller.**

This firmware provides a complete motor control system with quadrature encoder feedback and PID control, all programmable through a Lisp REPL over UART.

> **Note:** For Lisp language documentation, see [github.com/jig/lisp-rust](https://github.com/jig/lisp-rust)

## Importantant Note

This is work in progress! While the core features are implemented and functional, expect ongoing improvements, bug fixes, and additional features in future releases.

## Features

- **4-Motor PWM Control** - Independent control of 4 DC motors with direction
- **Quadrature Encoder Support** - Hardware interrupt-driven encoder decoding
- **PID Controller** - Closed-loop position/velocity control
- **Lisp REPL** - Interactive programming via UART (115200 baud)
- **Real-time Operation** - 20ms timer interrupt for control loops
- **Platform Support** - RP2040 (Pico) and RP2350 (Pico 2)

## Install

### Build

```bash
cargo build --release
```

### Flash Firmware

#### Option 1: Raspberry Pi GPIO (OpenOCD)

When developing on a Raspberry Pi 4 connected to the Pico via GPIO:

**Wiring (RPi 4 → Pico):**
- Pin 23 (GPIO 11) → SWCLK (Pico pin 24/GP18)
- Pin 22 (GPIO 25) → SWDIO (Pico pin 25/GP19)
- GND → GND

**Flash:**
```bash
cargo run-gpio           # or just: cargo run
```

**Debug in VS Code:**
- Press F5 and select **"Pico Debug (OpenOCD/RPi GPIO)"**

#### Option 2: Debug Probe (probe-rs)

When using a Raspberry Pi Debug Probe or similar SWD debugger:

**Flash:**
```bash
cargo run-probe
```

**Debug in VS Code:**
- Press F5 and select **"Pico Debug (probe-rs)"**

## Hardware Configuration

### Motor Pins

4 motors with PWM output and dual H-bridge enable control:

| Motor | PWM Pin | Enable Pins | Encoder Pins |
|-------|---------|-------------|--------------|
| Front Left (FL)  | GPIO6 (PWM3A) | GPIO7, GPIO8  | GPIO12, GPIO13 (A, B) |
| Front Right (FR) | GPIO3 (PWM1B) | GPIO4, GPIO5  | GPIO19, GPIO18 (A, B) |
| Rear Left (RL)   | GPIO9 (PWM4B) | GPIO10, GPIO11 | GPIO14, GPIO15 (A, B) |
| Rear Right (RR)  | GPIO20 (PWM2A) | GPIO21, GPIO22 | GPIO17, GPIO16 (A, B) |

### UART Communication

- **TX:** GPIO0
- **RX:** GPIO1
- **Baud Rate:** 115200
- **Status LED:** GPIO25 (onboard LED, toggles every timer interrupt)

### PWM Specifications

- **Frequency:** 25 kHz
- **Resolution:** 16-bit (0-4999 PWM counts, mapped from -1.0 to +1.0 API range)
- **Direction Control:** H-bridge dual enable (forward: ENA0=low/ENA1=high, reverse: ENA0=high/ENA1=low)

### Encoder Configuration

- **Type:** Incremental quadrature encoders
- **Pull-ups:** Enabled on all encoder pins
- **Interrupt Mode:** Edge-triggered on both rising and falling edges
- **Decoding:** Standard Gray code state machine

## Motor Control API

### Basic Motor Control

#### `(motor-move motor-idx level)`

Control a single motor.

- `motor-idx`: Motor index (0=FL, 1=FR, 2=RL, 3=RR)
- `level`: Speed and direction (-1.0 to +1.0)
  - Positive = forward
  - Negative = reverse
  - 0 = stop

```lisp
(motor-move 0 0.2)    ; Front left forward at 20% power
(motor-move 1 -0.5)   ; Front right reverse at 50% power
(motor-move 2 0)      ; Rear left stop
```

#### `(motors-move fl fr rl rr)`

Control all 4 motors simultaneously.

```lisp
(motors-move 0.4 0.4 0.4 0.4)    ; All forward at 40%
(motors-move 0.4 -0.4 0.4 -0.4)  ; Turn in place (tank turn)
(motors-move 0 0 0 0)            ; Stop all
```

## Encoder API

### Reading Encoders

#### `(encoder-get motor-idx)`

Get encoder count for a single motor.

- Returns: Current encoder count (signed integer)

```lisp
(encoder-get 0)        ; => 1234 (encoder ticks)
```

#### `(encoders-get)`

Get all encoder counts as a list.

- Returns: List of 4 encoder values `(fl fr rl rr)`

```lisp
(encoders-get)         ; => (1234 -567 890 432)
```

### Resetting Encoders

#### `(encoder-reset motor-idx)`

Reset a single encoder to zero.

```lisp
(encoder-reset 0)      ; Reset front left encoder
```

#### `(encoders-reset)`

Reset all encoders to zero.

```lisp
(encoders-reset)       ; Reset all to 0
```

## PID Controller API

The firmware includes a PID controller for closed-loop control. Typical use case: position or velocity control using encoder feedback.

### Configuration

#### `(pid-set-dt delta-time)`

Set the PID controller time step (in seconds).

- `delta-time`: Time step as floating-point (e.g., 0.02 for 20ms)

```lisp
(pid-set-dt 0.02)      ; 20ms control loop (50 Hz)
```

#### `(pid-set-gains kp ki kd)`

Set PID gains.

- `kp`: Proportional gain
- `ki`: Integral gain
- `kd`: Derivative gain

```lisp
(pid-set-gains 1.0 0.1 0.05)   ; Typical starting values
(pid-set-gains 2.5 0.0 0.0)    ; Pure P controller
```

#### `(pid-set-limits min-output max-output)`

Set output limits (anti-windup).

- `min-output`: Minimum PID output (-1.0 to 0)
- `max-output`: Maximum PID output (0 to +1.0)

```lisp
(pid-set-limits -1.0 1.0)    ; Full motor range
(pid-set-limits -0.4 0.4)    ; Limit to 40% power
```

#### `(pid-set-setpoint target)`

Set the target setpoint.

- `target`: Desired position/velocity (matches encoder units)

```lisp
(pid-set-setpoint 1000.0)   ; Target 1000 encoder ticks (you might prefer to use integer values: 1000)
```

### Operation

#### `(pid-enable state)`

Enable or disable the PID controller.

- `state`: `true` to enable, `false` to disable

```lisp
(pid-enable true)       ; Start PID control
(pid-enable false)      ; Stop PID control
```

#### `(pid-reset)`

Reset PID internal state (clear integral, derivative).

```lisp
(pid-reset)             ; Clear accumulated error
```

### PID Usage Example

```lisp
; Configure PID for position control
(pid-set-dt 0.02)                    ; 20ms update rate
(pid-set-gains 2.0 0.5 0.1)          ; Tuned gains
(pid-set-limits -1.0 1.0)            ; Full power range
(pid-set-setpoint 5000.0)            ; Target: 5000 ticks  (you might use integer ticks insted: 5000)
(encoders-reset)                      ; Start from zero
(pid-enable true)                     ; Activate controller

; In your control loop (called every 20ms):
; (let ((pos (encoder-get 0)))
;   (let ((output (pid-compute pos)))
;     (motor-move 0 output)))
```

## Advanced Features

### Timer Control

The firmware runs a 20ms periodic timer interrupt. You can adjust this:

```lisp
; Timer period is stored in TIMER_PERIOD_US global
; Currently cannot be changed at runtime (requires firmware modification)
```

### Utility Functions

#### `(time-us)` / `(time-ms)` / `(time-s)`

Get current system time.

- Returns: Microseconds / milliseconds / seconds since boot

```lisp
(time-ms)              ; => 12345 (12.345 seconds since boot)
```

## Encoder Quadrature Decoding

The firmware implements a hardware interrupt-driven quadrature decoder with the following characteristics:

### State Machine

- **Forward sequence:** 00 → 10 → 11 → 01 → 00 (increment)
- **Reverse sequence:** 00 → 01 → 11 → 10 → 00 (decrement)
- **Edge detection:** Both rising and falling edges on A and B channels
- **Processing:** `IO_IRQ_BANK0` interrupt handler

### Performance

- **Zero CPU overhead** when motors stopped (no polling)
- **Sub-millisecond latency** from edge to count update
- **No missed transitions** (hardware interrupt-driven)

## Building and Flashing

### Prerequisites

```bash
# Install Rust and targets
rustup target add thumbv6m-none-eabi      # For RP2040
rustup target add thumbv8m.main-none-eabihf  # For RP2350

# Install probe-rs for flashing/debugging
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
```

### Build

```bash
cargo build --release
```

### Flash

```bash
probe-rs run --chip RP2350 --protocol swd target/thumbv8m.main-none-eabihf/release/lisp-rust-main-pico
```

Or use VS Code tasks (see `.vscode/tasks.json`).

## Debugging

### Logging

The firmware uses `defmt` for structured logging over RTT (Real-Time Transfer):

```bash
# View defmt logs in real-time
probe-rs attach --chip RP2350
```

### Common Issues

**Motors not responding:**

- Check `init_motors()` was called successfully
- Verify power supply to motor driver
- Check H-bridge enable pins are connected

**Encoders not counting:**

- Verify `IO_IRQ_BANK0` interrupt is enabled
- Check encoder wiring (A, B, GND)
- Test with `(encoders-get)` while manually rotating motor

**UART not working:**

- Confirm 115200 baud rate
- Check GPIO0 (TX) and GPIO1 (RX) connections
- Ensure ground reference shared

## Example: Velocity Control

```lisp
; Simple velocity control using time-based position delta
(def target-velocity 100)  ; ticks per 100ms
(def last-pos 0)
(def last-time (time-ms))

(defn velocity-control []
  (let ((now (time-ms))
        (pos (encoder-get 0)))
    (if (> (- now last-time) 100)
      (let ((delta-pos (- pos last-pos))
            (error (- target-velocity delta-pos))
            (output (* 0.01 error)))  ; Simple P controller, scaled to -1.0..+1.0
        (motor-move 0 output)
        (def last-pos pos)
        (def last-time now)))))

; Call velocity-control periodically from your main loop
```

## Example: Position Synchronization

```lisp
; Keep two motors synchronized in position
(defn sync-motors (idx1 idx2 power)
  (let ((pos1 (encoder-get idx1))
        (pos2 (encoder-get idx2))
        (error (- pos1 pos2))
        (adjust (* 0.001 error)))  ; Proportional correction
    (motor-move idx1 (- power adjust))
    (motor-move idx2 (+ power adjust))))

; Use in loop:
(sync-motors 0 1 0.4)  ; Synchronize FL and FR at 40% power
```

## Project Structure

```
src/
├── main.rs          # Entry point, REPL, interrupt handlers
├── motor.rs         # Motor control & encoder decoding
├── pid.rs           # PID controller implementation
└── pico_core.rs     # Lisp-callable functions

AGENTS.md            # Development guidelines
Cargo.toml          # Dependencies and build config
.pico-rs            # Target chip selection (rp2040/rp2350)
```

## License

See LICENSE file for details.

## Contributing

Contributions welcome! Please ensure:

- Code compiles for both RP2040 and RP2350
- No dynamic allocation in interrupt handlers
- Follow existing code style (see `AGENTS.md`)

---

**Powered by Rust 🦀 | Made for Robotics 🤖**
