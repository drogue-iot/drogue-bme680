use crate::AmbientTemperatureProvider;
use core::fmt::Formatter;
use embedded_hal::blocking::i2c::{Write, WriteRead};

/// Error for sensor operations.
pub enum Error<I2C>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
{
    WriteError(<I2C as Write>::Error),
    WriteReadError(<I2C as WriteRead>::Error),
    WrongDevice,
}

impl<I2C> core::fmt::Debug for Error<I2C>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            Error::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Error::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            Error::WrongDevice => f.write_str("WrongDevice"),
        }
    }
}

pub enum ControllerError<I2C, ATP>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
    ATP: AmbientTemperatureProvider,
    ATP::Error: core::fmt::Debug,
{
    SensorError(Error<I2C>),
    ProviderError(ATP::Error),
}

impl<I2C, ATP> core::fmt::Debug for ControllerError<I2C, ATP>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
    ATP: AmbientTemperatureProvider,
    ATP::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            ControllerError::SensorError(e) => f.debug_tuple("SensorError").field(e).finish(),
            ControllerError::ProviderError(e) => f.debug_tuple("ProviderError").field(e).finish(),
        }
    }
}

impl<I2C, ATP> From<Error<I2C>> for ControllerError<I2C, ATP>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
    ATP: AmbientTemperatureProvider,
    ATP::Error: core::fmt::Debug,
{
    fn from(e: Error<I2C>) -> Self {
        ControllerError::SensorError(e)
    }
}
