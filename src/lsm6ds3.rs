use embedded_hal::blocking::i2c::{Write, WriteRead};
use crate::types::{Accelerometer, Gyroscope, ImuError};
use lsm6ds33::{
    AccelerometerBandwidth,
    AccelerometerOutput,
    AccelerometerScale,
    GyroscopeFullScale,
    GyroscopeOutput,
    Lsm6ds33,
    Error as Lsm6ds3Error,
};


impl<E> From<Lsm6ds3Error<E>> for ImuError
{
    fn from(err: Lsm6ds3Error<E>) -> Self
    {
        match err {
            Lsm6ds3Error::CommunicationError(_) => ImuError::AccelerometerError("acc / gyro: communication error"),
            Lsm6ds3Error::ChipDetectFailed => ImuError::AccelerometerError("acc / gyro: chip detection error"),
            Lsm6ds3Error::RegisterReadFailed => ImuError::AccelerometerError("acc / gyro: invalid input data"),
        }
    }
}


impl<I2C, E> From<(I2C, Lsm6ds3Error<E>)> for ImuError
{
    fn from(_err: (I2C, Lsm6ds3Error<E>)) -> Self
    {
        ImuError::AccelerometerError("acc / gyro: creation error")
    }
}


pub struct Lsm6ds3Imu<I2C>
{
    sensor: Lsm6ds33<I2C>,
}


impl<E, I2C> Lsm6ds3Imu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug
{
    pub fn new(i2c_bus: I2C) -> Result<Self, ImuError>
    {
        let address = 0x6b;
        let lsm = Lsm6ds3Imu {
            sensor: Lsm6ds33::new(i2c_bus, address)?,
        };
        Ok(lsm)
    }

    fn acc_swap(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32)
    {
        (
            x.into(),
            y.into(),
            (-z).into(),
        )
    }

    fn gyro_swap(&self, p: f32, q: f32, r: f32) -> (f32, f32, f32)
    {
        (
            (-p).into(),
            (-q).into(),
            r.into(),
        )
    }
}


impl<E, I2C> Accelerometer for Lsm6ds3Imu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug,
{
    fn acc_configure(&mut self) -> Result<(), ImuError>
    {
        self.sensor.set_accelerometer_bandwidth(AccelerometerBandwidth::Freq400)?;
        self.sensor.set_accelerometer_output(AccelerometerOutput::Rate416)?;
        self.sensor.set_accelerometer_scale(AccelerometerScale::G08)?;
        Ok(())
    }

    fn acc_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>
    {
        let (x, y, z)= self.sensor.read_accelerometer()?;
        Ok(self.acc_swap(x, y, z))
    }
}


impl<E, I2C> Gyroscope for Lsm6ds3Imu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug,
{
    fn gyro_configure(&mut self) -> Result<(), ImuError>
    {
        self.sensor.set_gyroscope_output(GyroscopeOutput::Rate416)?;
        self.sensor.set_gyroscope_scale(GyroscopeFullScale::Dps245)?;
        Ok(())
    }

    fn gyro_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>
    {
        let (p, q, r) = self.sensor.read_gyro()?;
        Ok(self.gyro_swap(p, q, r))
    }
}
