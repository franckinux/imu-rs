#[derive(Debug)]
pub enum ImuError {
    AccelerometerError(&'static str),
    GyroscopeError(&'static str),
    MagnetometerError(&'static str),
}


pub trait Accelerometer {
    fn acc_configure(&mut self) -> Result<(), ImuError>;
    fn acc_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>;
}


pub trait Gyroscope {
    fn gyro_configure(&mut self) -> Result<(), ImuError>;
    fn gyro_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>;
}


pub trait Magnetometer {
    fn mag_configure(&mut self) -> Result<(), ImuError>;
    fn mag_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>;
}
