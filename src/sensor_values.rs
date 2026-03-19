use crate::constants::{ACCEL_LSB, GRAVITY, GYRO_LSB};

pub struct SensorValues {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
}

impl SensorValues {
    // Accelerometer values must be in m/s^2
    // Gyroscope values must be in rad/s
    pub fn new(buffer: &[u8;14]) -> Self {

        let raw_accelerometer_x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let raw_accelerometer_y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let raw_accelerometer_z = i16::from_be_bytes([buffer[4], buffer[5]]);
        //  Convert accelerometer sensor values to m/s^2
        let ax_m_ps = raw_accelerometer_x as f32 / ACCEL_LSB * GRAVITY;
        let ay_m_ps = raw_accelerometer_y as f32 / ACCEL_LSB * GRAVITY;
        let az_m_ps = raw_accelerometer_z as f32 / ACCEL_LSB * GRAVITY;
        // ---------------------- ACCEL -----------------------

        // ----------------------- GYRO -----------------------
        let gx_degrees_ps = i16::from_be_bytes([buffer[8], buffer[9]]);
        let gy_degrees_ps = i16::from_be_bytes([buffer[10], buffer[11]]);
        let gz_degrees_ps = i16::from_be_bytes([buffer[12], buffer[13]]);
        // Convert from gyro sensor reading to rad/s
        let gx_rad_ps = (gx_degrees_ps as f32 / GYRO_LSB).to_radians();
        let gy_rad_ps = (gy_degrees_ps as f32 / GYRO_LSB).to_radians();
        let gz_rad_ps = (gz_degrees_ps as f32 / GYRO_LSB).to_radians();
        // ----------------------- GYRO -----------------------
        
        SensorValues {
            // Due to sensor orientation on breadboard,
            // az = x
            // ay = y
            // -ax = z
            ax: az_m_ps,
            ay: ay_m_ps,
            az: -ax_m_ps,
            // Due to sensor orientation on breadboard,
            // gz = x
            // gy = y
            // -gx = z
            gx: gz_rad_ps,
            gy: gy_rad_ps,
            gz: -gx_rad_ps,
        }
    }
}