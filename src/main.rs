//! # UART Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

#[cfg(target_arch = "riscv32")]
use panic_halt as _;
#[cfg(target_arch = "arm")]
use panic_probe as _;

// Some things we need

#[cfg(rp2350)]
use rp235x_hal as hal;

#[cfg(rp2040)]
use rp2040_hal as hal;

// use bsp::entry;
// use bsp::hal;
// use rp_pico as bsp;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[unsafe(link_section = ".boot2")]
#[used]
#[cfg(rp2040)]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};
use heapless::String;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
#[cfg(rp2350)]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    #[cfg(rp2040)]
    let mut delay = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    #[cfg(rp2350)]
    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let uart0_pins = (
        // UART TX (characters sent from rp235x) on pin 4 (GPIO2) in Aux mode
        pins.gpio0.into_function(),
        // UART RX (characters received by rp235x) on pin 5 (GPIO3) in Aux mode
        pins.gpio1.into_function(),
    );
    let uart0 = hal::uart::UartPeripheral::new(pac.UART0, uart0_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart0.write_full_blocking(b"\r\nBorinot Firmware\r\n");

    const BUF_LINE_LENGHT: usize = 256;
    let mut command: String<BUF_LINE_LENGHT> = String::new();
    loop {
        let mut buf = [0u8; BUF_LINE_LENGHT];
        match uart0.read_raw(&mut buf) {
            Err(nb::Error::WouldBlock) => {
                delay.delay_us(1u32);
                // cortex_m::asm::nop();
                continue;
            }
            Err(nb::Error::Other(err)) => {
                error!("uart0.read_raw: {:?}", err);
                delay.delay_us(1u32);
                continue;
            }
            Ok(0) => {
                info!("uart0.read_raw: Ok(0)");
                delay.delay_us(1u32);
                continue;
            }
            Ok(count) => {
                led_pin.set_high().unwrap();
                // cortex_m::asm::bkpt();
                let value = core::str::from_utf8(&buf[0..count]).unwrap_or("<invalid utf8>");
                uart0.write_full_blocking(value.as_bytes());

                for c in value.chars() {
                    if c == '\r' {
                        info!("uart0.read_raw: Ok(\\r)");
                        exec(&command);
                        command.clear();
                    } else if c == '\n' {
                        info!("uart0.read_raw: Ok(\\n)");
                        exec(&command);
                        command.clear();
                    } else {
                        info!("uart0.read_raw: Ok({})", &value);
                        let _ = command.push(c);
                    }
                }
                led_pin.set_low().unwrap();
            }
        }
    }
}

fn exec(command: &str) {
    warn!("EXEC: {}", command);
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Borinot Firmware"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
