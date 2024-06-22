use embedded_hal::blocking::i2c::{Write, WriteRead};
use crate::types::{Accelerometer, Gyroscope, ImuError};
use bmi160::{
    AccelerometerPowerMode,
    Bmi160,
    GyroscopePowerMode,
    interface::I2cInterface,
    SensorSelector,
    SlaveAddr as BmiSlaveAddr,
    Error as Bmi160Error,
};


impl<CommE, PinE> From<Bmi160Error<CommE, PinE>> for ImuError
{
    fn from(err: Bmi160Error<CommE, PinE>) -> Self
    {
        match err {
            Bmi160Error::Comm(_) => ImuError::AccelerometerError("acc / gyro: communication error"),
            Bmi160Error::Pin(_) => ImuError::AccelerometerError("acc / gyro: pin error"),
            Bmi160Error::InvalidInputData => ImuError::AccelerometerError("acc / gyro: invalid input data"),
        }
    }
}


pub struct Bmi160Imu<I2C> {
    sensor: Bmi160<I2cInterface<I2C>>,
}


impl<E, I2C> Bmi160Imu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug
{
    pub fn new(i2c_bus: I2C) -> Result<Self, ()>
    {
        let address = BmiSlaveAddr::Alternative(true);
        let bmi = Bmi160Imu {
            sensor: Bmi160::new_with_i2c(i2c_bus, address),
        };
        Ok(bmi)
    }

    fn acc_swap(x: f32, y: f32, z: f32) -> (f32, f32, f32)
    {
        (x, y, -z)
    }

    fn gyro_swap(p: f32, q: f32, r: f32) -> (f32, f32, f32)
    {
        (-p, -q, r)
    }
}


impl<E, I2C> Accelerometer for Bmi160Imu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug,
{
    fn acc_configure(&mut self) -> Result<(), ImuError>
    {
        self.sensor.set_accel_power_mode(AccelerometerPowerMode::Normal)?;
        // default accel conf : ODR = 100 Hz / RANGE = +/- 2g
        Ok(())
    }

    fn acc_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>
    {
        let data = self.sensor.data(SensorSelector::new().accel())?;
        let acc = data.accel.unwrap();
        // convert register value to m/s^2
        // if +/- 2g : reg / 16384 * 9.80665
        // if +/- 4g : reg / 8192 * 9.80665
        // if +/- 8g : reg / 4096 * 9.80665
        // if +/- 16g : reg / 2048 * 9.80665
        // for 2g :
        let x: f32 = acc.x as f32 / 1670.703;
        let y: f32 = acc.y as f32 / 1670.703;
        let z: f32 = acc.z as f32 / 1670.703;
        Ok(Self::acc_swap(x, y, z))
    }
}


impl<E, I2C> Gyroscope for Bmi160Imu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug,
{
    fn gyro_configure(&mut self) -> Result<(), ImuError>
    {
        self.sensor.set_gyro_power_mode(GyroscopePowerMode::Normal)?;
        // default gyro conf : ODR = 100 Hmz / RANGE = 2000°/s
        Ok(())
    }

    fn gyro_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>
    {
        let data = self.sensor.data(SensorSelector::new().gyro())?;
        let gyro = data.gyro.unwrap();
        // convert register value to °/s^2
        // if +/- 2000°/s^2 : reg / 16.4
        // if +/- 1000°/s^2 : reg / 32.8
        // if +/- 500°/s^2 : reg / 65.6
        // if +/- 250°/s^2 : reg / 131.2
        // if +/- 125°/s^2 : reg / 262.4
        // for 2000°/s^2 (default value) :
        let p = gyro.x as f32 / 16.4;
        let q = gyro.y as f32 / 16.4;
        let r = gyro.z as f32 / 16.4;
        Ok(Self::gyro_swap(p, q, r))
    }
}
