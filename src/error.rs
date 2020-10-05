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
