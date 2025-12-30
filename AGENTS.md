# AGENTS.md

## Project Overview
This is a Rust project for Raspberry Pi Pico (RP2040) and Pico 2 (RP2350) microcontrollers.
It uses the `rp-rs` ecosystem, including `cortex-m`, `embedded-hal`, and `defmt` for logging.

## Build Instructions
- **Debug Build:** `cargo build`
- **Release Build:** `cargo build --release`
- **SBOM Generation:** `cargo sbom > target/.../sbom.spdx.json` (handled by VS Code tasks)

## Configuration
- The target chip (RP2040 or RP2350) is configured via the `.pico-rs` file.
- `build.rs` automatically handles linker scripts (`memory.x`) and target architecture configuration based on `.pico-rs`.

## Debugging
- Uses `probe-rs` for flashing and debugging.
- **Important:** `haltAfterReset` is set to `true` in `launch.json` to ensure breakpoints are hit after reset.
- `defmt` is used for logging over RTT. `info!`, `warn!`, `error!` macros send compressed logs to the host.

## Code Conventions
- **Environment:** `#![no_std]` and `#![no_main]`.
- **Allocations:** Avoid dynamic allocation. Use `heapless` for fixed-capacity data structures (e.g., `heapless::String`).
- **Async/Blocking:** The project currently uses blocking or polling models.
- **UART Handling:**
  - Use `nb` crate for non-blocking I/O.
  - Handle `nb::Error::WouldBlock` for polling loops.
  - Use `cortex_m::asm::nop()` in tight polling loops to prevent over-optimization and allow debuggers to attach properly.

## Common Issues & Fixes
- **Breakpoints not working:** Ensure `haltAfterReset` is `true` in `launch.json`.
- **"Source not found" for lib.rs:** This is normal when halted at the reset handler in precompiled libraries. Press Continue (F5).
- **UART Read Errors:** `nb::Error` must be matched explicitly to separate `WouldBlock` from actual errors (`Other`).
