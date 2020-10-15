use core::cell::Cell;
use embedded_hal::blocking::delay::DelayMs;
use embedded_time::duration::Milliseconds;

pub trait Delay {
    /// Pause execution for `duration_ms` milliseconds.
    fn delay_ms(&self, duration_ms: u16);
}

pub struct DelayWrapper<'a, Clock>(pub drogue_embedded_timer::Delay<'a, Clock>)
where
    Clock: embedded_time::Clock;

impl<'a, Clock> Delay for DelayWrapper<'a, Clock>
where
    Clock: embedded_time::Clock,
{
    fn delay_ms(&self, duration_ms: u16) {
        self.0.delay(Milliseconds(duration_ms as u32))
    }
}

pub struct DelayMsWrapper<D>
where
    D: DelayMs<u16> + Sized,
{
    delay: Cell<Option<D>>,
}

impl<D> DelayMsWrapper<D>
where
    D: DelayMs<u16> + Sized,
{
    pub fn new(delay: D) -> Self {
        DelayMsWrapper {
            delay: Cell::new(Some(delay)),
        }
    }
}

impl<D> Delay for DelayMsWrapper<D>
where
    D: DelayMs<u16> + Sized,
{
    fn delay_ms(&self, duration: u16) {
        let delay = self.delay.take();
        let mut delay = delay.expect("Delay instance gone missing");
        delay.delay_ms(duration);
        self.delay.set(Some(delay));
    }
}

impl<D> DelayMs<u16> for DelayMsWrapper<D>
where
    D: DelayMs<u16> + Sized,
{
    fn delay_ms(&mut self, ms: u16) {
        let delay = self.delay.take();
        let mut delay = delay.expect("Delay instance gone missing");
        delay.delay_ms(ms);
        self.delay.set(Some(delay));
    }
}

impl<D> DelayMs<u8> for DelayMsWrapper<D>
where
    D: DelayMs<u16> + Sized,
{
    fn delay_ms(&mut self, ms: u8) {
        let delay = self.delay.take();
        let mut delay = delay.expect("Delay instance gone missing");
        delay.delay_ms(ms as u16);
        self.delay.set(Some(delay));
    }
}

#[doc(hidden)]
pub mod mock {
    use super::Delay;

    pub struct MockDelay;

    impl Delay for MockDelay {
        fn delay_ms(&self, _: u16) {
            unimplemented!()
        }
    }
}
