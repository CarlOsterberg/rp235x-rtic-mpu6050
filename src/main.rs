#![no_std]
#![no_main]

pub mod complementary_filter;
pub mod constants;
pub mod type_defs;

use core::fmt::Write;
use defmt_rtt as _;
use embedded_hal::i2c::I2c;
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

use crate::complementary_filter::*;
use crate::constants::*;
use crate::type_defs::*;

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
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 RTIC mpu6050 complementary filter"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

#[rtic::app(device = rp235x_hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        uart: Uart0,
        i2c: I2c0,
        interrupt: InterruptPin,
        complementary_filter: ComplementaryFilter,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        let clocks = init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ_HZ,
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

        let uart =
            hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(BAUD_RATE.Hz(), DataBits::Eight, None, StopBits::One),
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
            100.kHz(),
            &mut ctx.device.RESETS,
            clocks.system_clock.freq(),
        );

        // Enable the interrupts on this specific pin,
        // the sensor will pull this high when data is available
        let interrupt = pins.gpio6.into_pull_up_input();
        interrupt.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        // Wake up the sensor, MPU6050 (it starts in sleep mode)
        match i2c.write(SENSOR_I2C_ADDR, &[0x6B, 0x00]) {
            Ok(_) => uart.write_full_blocking(b"Sensor wakeup OK\r\n"),
            Err(_) => uart.write_full_blocking(b"Sensor wakeup failed\r\n"),
        }
        // Enable Digital Low Pass Filter (DLPF), this also sets sample rate to 1 kHz,
        // without DLPF the sample rate is 8 kHz.
        match i2c.write(SENSOR_I2C_ADDR, &[0x1A, 0x06]) {
            Ok(_) => uart.write_full_blocking(b"DLPF configuration OK\r\n"),
            Err(_) => uart.write_full_blocking(b"DLPF configuration failed\r\n"),
        }
        // Set sample rate divider - slows output to 100 Hz ( 1000Hz / (1 + 9) )
        let sample_rate_divider = 0x9;
        match i2c.write(SENSOR_I2C_ADDR, &[0x19, sample_rate_divider]) {
            Ok(_) => uart.write_full_blocking(b"Sample rate set OK\r\n"),
            Err(_) => uart.write_full_blocking(b"Sample rate set failed\r\n"),
        }
        let sample_rate_hz: u32 = 1000 / (1 + sample_rate_divider as u32);
        // Enable sensor to generate an interrupt when new data is available
        match i2c.write(SENSOR_I2C_ADDR, &[0x38, 0x01]) {
            Ok(_) => uart.write_full_blocking(b"Sensor interrupt enable OK\r\n"),
            Err(_) => uart.write_full_blocking(b"Sensor interrupt enable failed\r\n"),
        }

        let alpha: f32 = 0.05;
        let complementary_filter = ComplementaryFilter::new(sample_rate_hz as f32, alpha);

        (
            Shared {},
            Local {
                uart,
                i2c,
                interrupt,
                complementary_filter,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [interrupt], priority = 2)]
    fn gpio_irq(ctx: gpio_irq::Context) {
        if ctx
            .local
            .interrupt
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            ctx.local
                .interrupt
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            read_i2c::spawn().ok();
        }
    }

    #[task(local = [i2c, uart, complementary_filter], priority = 1)]
    async fn read_i2c(ctx: read_i2c::Context) {
        let mut s: String<64> = String::new();

        // Read sensor data
        let mut buffer = [0u8; 14];
        match ctx.local.i2c.write_read(SENSOR_I2C_ADDR, &[0x3B], &mut buffer) {
            Ok(_) => {
                // ---------------------- ACCEL -----------------------
                let raw_accelerometer_x = i16::from_be_bytes([buffer[0], buffer[1]]);
                let raw_accelerometer_y = i16::from_be_bytes([buffer[2], buffer[3]]);
                let raw_accelerometer_z = i16::from_be_bytes([buffer[4], buffer[5]]);
                //  Convert accelerometer sensor values to m/s^2
                let ax = raw_accelerometer_x as f32 / ACCEL_LSB * GRAVITY;
                let ay = raw_accelerometer_y as f32 / ACCEL_LSB * GRAVITY;
                let az = raw_accelerometer_z as f32 / ACCEL_LSB * GRAVITY;
                // due to sensor orientation on breadboard,
                // -ax = z
                // ay = y
                // az = x
                // ---------------------- ACCEL -----------------------

                // ----------------------- GYRO -----------------------
                let gx_degrees_ps = i16::from_be_bytes([buffer[8], buffer[9]]);
                let gy_degrees_ps = i16::from_be_bytes([buffer[10], buffer[11]]);
                let gz_degrees_ps = i16::from_be_bytes([buffer[12], buffer[13]]);
                // Convert from gyro sensor reading to rad/s
                let gx_rad_ps = (gx_degrees_ps as f32 / GYRO_LSB).to_radians();
                let gy_rad_ps = (gy_degrees_ps as f32 / GYRO_LSB).to_radians();
                let gz_rad_ps = (gz_degrees_ps as f32 / GYRO_LSB).to_radians();
                // Due to sensor orientation on breadboard,
                // -gx = z
                // gy = y
                // gz = x
                // ----------------------- GYRO -----------------------

                let sensor_values =
                    SensorValues::new(az, ay, -ax, gz_rad_ps, gy_rad_ps, -gx_rad_ps);
                ctx.local.complementary_filter.timestep(sensor_values);

                write!(
                    s,
                    "roll:{:.3}\tpitch:{:.3}\r\n",
                    ctx.local.complementary_filter.get_roll().to_degrees(),
                    ctx.local.complementary_filter.get_pitch().to_degrees()
                )
                .unwrap();
                ctx.local.uart.write_full_blocking(s.as_bytes());
                s.clear();
            }
            Err(_) => {
                ctx.local.uart.write_full_blocking(b"Read error\r\n");
            }
        }
    }
}

// End of file
