//! Implementations just for the sake of creating compilable documentation.

use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::timer::CountDown;
use embedded_time::duration::Milliseconds;
use void::Void;

pub struct MockTimer {}
pub struct MockI2C {}

pub fn create_timer() -> MockTimer {
    MockTimer {}
}

impl CountDown for MockTimer {
    type Time = Milliseconds;

    fn start<T>(&mut self, _: T)
    where
        T: Into<Self::Time>,
    {
        unimplemented!()
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        unimplemented!()
    }
}

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
