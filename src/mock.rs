//! Implementations just for the sake of creating compilable documentation.

use embedded_hal::blocking::i2c::{Write, WriteRead};

pub use crate::delay::mock::*;

pub struct MockTimer {}
pub struct MockI2C {}

pub fn blocking_i2c() -> MockI2C {
    MockI2C {}
}

impl Write for MockI2C {
    type Error = ();

    fn write(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

impl WriteRead for MockI2C {
    type Error = ();

    fn write_read(&mut self, _: u8, _: &[u8], _: &mut [u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }
}
