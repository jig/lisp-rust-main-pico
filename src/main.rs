#![no_std]
#![no_main]

mod pico_core;
mod pid;

extern crate alloc;
use alloc::{boxed::Box, vec};
use core::cell::RefCell;
use cortex_m::interrupt::{Mutex, free};
use cortex_m::peripheral::NVIC;

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use hal::clocks::Clock;
use hal::fugit::{MicrosDurationU32, RateExtU32};
use hal::timer::Alarm;

use mal::types::MalVal::Int;
use mal::types::{MalArgs, MalRet, error, func, func_closure};

#[cfg(target_arch = "riscv32")]
use panic_halt as _;
#[cfg(target_arch = "arm")]
use panic_probe as _;

#[cfg(rp2350)]
use hal::pac::interrupt;
#[cfg(rp2040)]
use hal::pac::interrupt;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

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

static mut GLOBAL_ENV: Option<&'static mal::Env> = None;

type UartType = hal::uart::UartPeripheral<
    hal::uart::Enabled,
    hal::pac::UART0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
    ),
>;

static mut GLOBAL_UART0: Option<&'static mut UartType> = None;

// GPIO pin for timer interrupt test (GPIO25 - onboard LED)
type TestPinType =
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;
static GLOBAL_TEST_PIN: Mutex<RefCell<Option<TestPinType>>> = Mutex::new(RefCell::new(None));

// Alarm for periodic interrupts
#[cfg(rp2350)]
type AlarmType = hal::timer::Alarm0<hal::timer::CopyableTimer0>;
#[cfg(rp2040)]
type AlarmType = hal::timer::Alarm0<hal::timer::Timer<hal::timer::TimerDevice>>;
static GLOBAL_ALARM: Mutex<RefCell<Option<AlarmType>>> = Mutex::new(RefCell::new(None));

// Counter for ISR debugging
static ISR_COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

fn eval_wrapper(a: MalArgs) -> MalRet {
    if a.len() != 1 {
        return error("eval requires exactly 1 argument");
    }
    // SAFETY: Safe because GLOBAL_ENV is initialized before eval_wrapper is called
    let env = unsafe { GLOBAL_ENV.unwrap() };
    mal::eval(&a[0], env)
}

fn slurp(a: MalArgs) -> MalRet {
    if a.len() != 1 {
        return error("read-file expects 1 argument");
    }
    error("unimplemented: slurp")
}

fn create_time_us_func<'a>(
    timer: &'a hal::Timer<impl hal::timer::TimerDevice>,
) -> impl Fn(MalArgs) -> MalRet + 'a {
    move |args: MalArgs| {
        if args.len() != 0 {
            return error("time-us expects 0 arguments");
        }
        Ok(Int(timer.get_counter().ticks() as i64))
    }
}

fn create_time_ms_func<'a>(
    timer: &'a hal::Timer<impl hal::timer::TimerDevice>,
) -> impl Fn(MalArgs) -> MalRet + 'a {
    move |args: MalArgs| {
        if args.len() != 0 {
            return error("time-ms expects 0 arguments");
        }
        Ok(Int((timer.get_counter().ticks() / 1_000) as i64))
    }
}

fn create_time_s_func<'a>(
    timer: &'a hal::Timer<impl hal::timer::TimerDevice>,
) -> impl Fn(MalArgs) -> MalRet + 'a {
    move |args: MalArgs| {
        if args.len() != 0 {
            return error("time-s expects 0 arguments");
        }
        Ok(Int((timer.get_counter().ticks() / 1_000_000) as i64))
    }
}

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

    // Configure GPIO25 as output for timer interrupt test (onboard LED)
    let test_pin = pins.gpio25.into_push_pull_output();
    free(|cs| {
        GLOBAL_TEST_PIN.borrow(cs).replace(Some(test_pin));
    });

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

    // Leak uart0 to get 'static lifetime and store in GLOBAL_UART0
    let uart0_static: &'static mut UartType = Box::leak(Box::new(uart0));
    unsafe {
        GLOBAL_UART0 = Some(uart0_static);
    }
    info!("Borinot Firmware started!");

    // Configure alarm for 200ms periodic interrupt
    let mut alarm = delay.alarm_0().unwrap();
    let _ = alarm.schedule(MicrosDurationU32::from_ticks(200_000));
    alarm.enable_interrupt();
    free(|cs| {
        GLOBAL_ALARM.borrow(cs).replace(Some(alarm));
    });

    // Enable timer interrupt in NVIC
    #[cfg(rp2350)]
    unsafe {
        NVIC::unmask(hal::pac::Interrupt::TIMER0_IRQ_0);
    }
    #[cfg(rp2040)]
    unsafe {
        NVIC::unmask(hal::pac::Interrupt::TIMER_IRQ_0);
    }

    info!("Timer alarm configured for 200ms");

    // Initialize heap
    {
        use core::mem::MaybeUninit;
        use core::ptr::addr_of_mut;
        const HEAP_SIZE: usize = 1024 * 64; // 64 KB
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    use mal::{env_sets, initialize_mal_env, mal_env, rep};

    // Create environment - readline is no longer part of core
    let env = mal_env();
    initialize_mal_env(&env, vec![]);

    env_sets(&env, "slurp", func(slurp));

    // Leak delay to get 'static lifetime for the closure
    let delay_static = Box::leak(Box::new(delay.clone()));
    env_sets(
        &env,
        "time-us",
        func_closure(create_time_us_func(delay_static)),
    );
    env_sets(
        &env,
        "time-ms",
        func_closure(create_time_ms_func(delay_static)),
    );

    env_sets(
        &env,
        "time-s",
        func_closure(create_time_s_func(delay_static)),
    );

    fn readline(a: MalArgs) -> MalRet {
        if a.len() != 1 {
            return error("readline expects 1 argument");
        }
        error("readline not supported on embedded")
    }
    env_sets(&env, "readline", func(readline));
    env_sets(&env, "prn", func(pico_core::prn));
    env_sets(&env, "println", func(pico_core::println));
    env_sets(&env, "mem-free", func(pico_core::mem_free));

    // PID functions
    env_sets(&env, "pid-set-gains", func(pico_core::pid_set_gains));
    env_sets(&env, "pid-set-setpoint", func(pico_core::pid_set_setpoint));
    env_sets(&env, "pid-enable", func(pico_core::pid_enable));
    env_sets(&env, "pid-reset", func(pico_core::pid_reset));
    env_sets(&env, "pid-set-limits", func(pico_core::pid_set_limits));

    // Leak env to get a 'static reference, then set eval
    let env_static: &'static mal::Env = Box::leak(Box::new(env.clone()));
    unsafe {
        GLOBAL_ENV = Some(env_static);
    }
    env_sets(&env, "eval", func(eval_wrapper));

    const BUF_LINE_LENGHT: usize = 256;
    let mut command: String<BUF_LINE_LENGHT> = String::new();
    loop {
        let mut buf = [0u8; BUF_LINE_LENGHT];
        // SAFETY: Safe because GLOBAL_UART0 is initialized before this loop
        let uart0 = unsafe { &mut *core::ptr::addr_of_mut!(GLOBAL_UART0).read().unwrap() };
        match uart0.read_raw(&mut buf) {
            Err(nb::Error::WouldBlock) => {
                delay.delay_us(1u32);
                // cortex_m::asm::nop();
                continue;
            }
            Err(nb::Error::Other(err)) => {
                warn!("uart0.read_raw: Err: {:?}", err);
                delay.delay_us(1u32);
                continue;
            }
            Ok(0) => {
                info!("uart0.read_raw: Ok: 0");
                delay.delay_us(1u32);
                continue;
            }
            Ok(count) => {
                // cortex_m::asm::bkpt();
                let value = core::str::from_utf8(&buf[0..count]).unwrap_or("<invalid utf8>");
                uart0.write_full_blocking(value.as_bytes());

                for c in value.chars() {
                    if c == '\r' {
                        uart0.write_full_blocking(b"\r\n");
                        match rep(command.as_str(), &env) {
                            Ok(out) => {
                                info!("> {}", &command.as_str());
                                uart0.write_full_blocking(b"=> ");
                                uart0.write_full_blocking(out.as_bytes());
                                uart0.write_full_blocking(b"\r\n");
                                info!("=> {}", out.as_str())
                            }
                            Err(err) => {
                                let e = err.pr_str(true);
                                if e.as_str() != "\"no input\"" {
                                    uart0.write_full_blocking(b"ERR> ");
                                    uart0.write_full_blocking(e.as_bytes());
                                    uart0.write_full_blocking(b"\r\n");
                                    error!("Error: {}", e.as_str())
                                } else {
                                    warn!("No input")
                                }
                            }
                        }
                        command.clear();
                    } else {
                        debug!("uart0.read_raw: Ok({})", &c);
                        let _ = command.push(c);
                    }
                }
            }
        }
    }
}

/// Timer interrupt handler - runs every 200ms
/// Toggles GPIO25 (onboard LED) to validate the timer is working
#[cfg(rp2350)]
#[interrupt]
fn TIMER0_IRQ_0() {
    free(|cs| {
        // Increment counter
        let mut counter = ISR_COUNTER.borrow(cs).borrow_mut();
        *counter = counter.wrapping_add(1);

        // Log every 10 interrupts (2 seconds)
        if *counter % 10 == 0 {
            defmt::info!("ISR executed {} times", *counter);
        }
        // Clear the interrupt
        if let Some(ref mut alarm) = *GLOBAL_ALARM.borrow(cs).borrow_mut() {
            alarm.clear_interrupt();
            // Schedule next interrupt (200ms from now)
            let _ = alarm.schedule(MicrosDurationU32::from_ticks(200_000));
        }

        // Toggle test pin
        if let Some(ref mut pin) = *GLOBAL_TEST_PIN.borrow(cs).borrow_mut() {
            let _ = pin.toggle();
        }
    });
}

#[cfg(rp2040)]
#[interrupt]
fn TIMER_IRQ_0() {
    free(|cs| {
        // Increment counter
        let mut counter = ISR_COUNTER.borrow(cs).borrow_mut();
        *counter = counter.wrapping_add(1);

        // Log every 10 interrupts (2 seconds)
        if *counter % 10 == 0 {
            defmt::info!("ISR executed {} times", *counter);
        }

        // Clear the interrupt
        if let Some(ref mut alarm) = *GLOBAL_ALARM.borrow(cs).borrow_mut() {
            alarm.clear_interrupt();
            // Schedule next interrupt (200ms from now)
            let _ = alarm.schedule(MicrosDurationU32::from_ticks(200_000));
        }

        // Toggle test pin
        if let Some(ref mut pin) = *GLOBAL_TEST_PIN.borrow(cs).borrow_mut() {
            let _ = pin.toggle();
        }
    });
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
