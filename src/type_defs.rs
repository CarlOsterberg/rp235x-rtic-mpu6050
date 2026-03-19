
use rp235x_hal::{self as hal};

pub type Uart0 = hal::uart::UartPeripheral<
    hal::uart::Enabled,
    hal::pac::UART0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
    ),
>;

pub type I2c0 = hal::i2c::I2C<
    hal::pac::I2C0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionI2C, hal::gpio::PullUp>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::FunctionI2C, hal::gpio::PullUp>,
    ),
>;

pub type InterruptPin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio6,
    hal::gpio::FunctionSio<hal::gpio::SioInput>,
    hal::gpio::PullUp,
>;