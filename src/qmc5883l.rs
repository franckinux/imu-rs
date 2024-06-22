use embedded_hal::blocking::i2c::{Write, WriteRead};
use stm32f4xx_hal::i2c::Error as I2cError;
use qmc5883l::{
    FieldRange,
    OutputDataRate,
    OversampleRate,
    QMC5883L,
    Error as Qmc5883lError,
};
use crate::types::{ImuError, Magnetometer};


impl<E> From<Qmc5883lError<E>> for ImuError
{
    fn from(err: Qmc5883lError<E>) -> Self
    {
        match err {
            Qmc5883lError::InvalidDevice(_) => ImuError::MagnetometerError("mag: invalid device"),
            Qmc5883lError::NotReady => ImuError::MagnetometerError("mag: not ready"),
            Qmc5883lError::Overflow => ImuError::MagnetometerError("mag: overflow"),
            Qmc5883lError::BusError(_) => ImuError::MagnetometerError("mag: bus error"),
        }
    }
}


impl From<I2cError> for ImuError
{
    fn from(_err: I2cError) -> Self
    {
        ImuError::MagnetometerError("mag: hal i2c error")
    }
}


pub struct Qmc5883lImu<I2C> {
    sensor: QMC5883L<I2C>,
}


impl<E, I2C> Qmc5883lImu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug, ImuError: From<E>
{
    pub fn new(i2c_bus: I2C) -> Result<Self, ImuError>
    {
        let qmc = Qmc5883lImu {
            sensor: QMC5883L::new(i2c_bus)?,
        };
        Ok(qmc)
    }

    fn mag_swap(x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        (y, -x, z)
    }
}


impl<E, I2C> Magnetometer for Qmc5883lImu<I2C>
    where I2C: WriteRead<Error = E> + Write<Error = E>, E: core::fmt::Debug, ImuError: From<E>
{
    fn mag_configure(&mut self) -> Result<(), ImuError>
    {
        self.sensor.set_field_range(FieldRange::Range8Gauss)?;
        self.sensor.set_output_data_rate(OutputDataRate::Rate100Hz)?;
        self.sensor.set_oversample(OversampleRate::Rate256)?;
        self.sensor.continuous().unwrap();
        Ok(())
    }

    fn mag_read_values(&mut self) -> Result<(f32, f32, f32), ImuError>
    {
        let (x, y, z) = self.sensor.mag()?;
        // convert register value to G
        // if +/- 8G : reg / 3000
        // if +/- 2G : reg / 12000
        // for 8G (default value) :
        let x = x as f32 / 3000.0;
        let y = y as f32 / 3000.0;
        let z = z as f32 / 3000.0;
        Ok(Self::mag_swap(x, y, z))
    }
}
