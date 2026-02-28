#![no_std]
#![no_main]

use core::fmt::Write;
use defmt_rtt as _;
use embedded_hal::i2c::{I2c};
use fugit::RateExtU32;
use heapless::String;
use panic_probe as _;
use rp235x_hal::{
    self as hal, Clock,
    clocks::init_clocks_and_plls,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
};
use rtic_monotonics::rp235x::prelude::*;

rp235x_timer_monotonic!(Mono);

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 RTIC hello world"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

#[rtic::app(device = rp235x_hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_1])]
mod app {
    use super::*;

    type Uart0 = hal::uart::UartPeripheral<
        hal::uart::Enabled,
        hal::pac::UART0,
        (
            hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        ),
    >;

    type I2c0 = hal::i2c::I2C<
        hal::pac::I2C0,
        (
            hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionI2C, hal::gpio::PullUp>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::FunctionI2C, hal::gpio::PullUp>,
        ),
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        uart: Uart0,
        i2c: I2c0,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        // External high-speed crystal on the pico board is 12Mhz
        let external_xtal_freq_hz = 12_000_000u32;
        let clocks = init_clocks_and_plls(
            external_xtal_freq_hz,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(ctx.device.SIO);

        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let uart_pins = (
            pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
            pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
        );

        let baudrate: u32 = 115_200;

        let uart =
            hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(baudrate.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();

        let sda_pin = pins
            .gpio4
            .into_function::<hal::gpio::FunctionI2C>()
            .into_pull_type::<hal::gpio::PullUp>();
        let scl_pin = pins
            .gpio5
            .into_function::<hal::gpio::FunctionI2C>()
            .into_pull_type::<hal::gpio::PullUp>();

        let mut i2c = hal::i2c::I2C::i2c0(
            ctx.device.I2C0,
            sda_pin,
            scl_pin,
            10.kHz(),
            &mut ctx.device.RESETS,
            clocks.system_clock.freq(),
        );

        uart.write_full_blocking(b"Hello RTIC mpu6050!\r\n");

        let sensor_adr: u8 = 0x68u8;
        // Wake up MPU6050 (it starts in sleep mode)
        match i2c.write(sensor_adr, &[0x6B, 0x00]) {
            Ok(_) => uart.write_full_blocking(b"Sensor wakeup OK\r\n"),
            Err(_) => uart.write_full_blocking(b"Sensor wakeup failed\r\n"),
        }

        read_sensor::spawn().ok();

        (Shared {}, Local { uart, i2c })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [uart, i2c], priority = 1)]
    async fn read_sensor(ctx: read_sensor::Context) {
        let mut s: String<64> = String::new();

        let sensor_adr: u8 = 0x68u8;

        Mono::delay(100.millis()).await;

        // Read accelerometer and gyro data
        let mut buffer = [0u8; 14];
        ctx.local.uart.write_full_blocking(b"Reading sensor...\r\n");
        loop {
            Mono::delay(1000.millis()).await;
            match ctx.local.i2c.write_read(sensor_adr, &[0x3B], &mut buffer) {
                Ok(_) => {
                    let ax = i16::from_be_bytes([buffer[0], buffer[1]]);
                    let ay = i16::from_be_bytes([buffer[2], buffer[3]]);
                    let az = i16::from_be_bytes([buffer[4], buffer[5]]);
                    let gx = i16::from_be_bytes([buffer[8], buffer[9]]);
                    let gy = i16::from_be_bytes([buffer[10], buffer[11]]);
                    let gz = i16::from_be_bytes([buffer[12], buffer[13]]);
                    write!(
                        s,
                        "Accel [{},{},{}]\r\nGyro  [{},{},{}]\r\n",
                        ax, ay, az, gx, gy, gz
                    )
                        .unwrap();
                    ctx.local.uart.write_full_blocking(s.as_bytes());
                    s.clear();
                }
                Err(hal::i2c::Error::Abort(code)) => {
                    write!(s, "Read Abort: 0x{:06X}\r\n", code).unwrap();
                    ctx.local.uart.write_full_blocking(s.as_bytes());
                    s.clear();
                }
                Err(_) => ctx.local.uart.write_full_blocking(b"Read other error\r\n"),
            }
        }
    }
}

// End of file
