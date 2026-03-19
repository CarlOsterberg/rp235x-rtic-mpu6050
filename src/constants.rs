
// External high-speed crystal on the pico board is 12Mhz
pub const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000;
pub const BAUD_RATE: u32 = 460_800;
pub const GRAVITY: f32 = 9.818f32; // m/s^2
pub const ACCEL_LSB: f32 = 16_384.0; // 16384/g
pub const GYRO_LSB: f32 = 131.0;

pub const SENSOR_I2C_ADDR: u8 = 0x68;
pub const SENSOR_DATA_REG: u8 = 0x3B;
