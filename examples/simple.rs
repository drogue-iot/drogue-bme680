#![deny(unsafe_code)]
#![no_main]
#![no_std]

//! Example for using the BME680 sensor.
//!
//! ## STM32F411RE - Nucleo 64
//!
//! Connect the BME680 sensor to the I2C port #1 using pins PB8/PB9.
//!
//! ## STM32F723E - DISCOVERY
//!
//! Connect the BME680 to the STM32F723E DISCOVERY board via the Grove I2C connector
//! on the extension board.

#[cfg(not(feature = "env_logging"))]
use panic_rtt_target as _;
use rtt_target::rtt_init_print;

#[cfg(any(feature = "stm32f7xx", feature = "stm32f4xx"))]
use cortex_m_rt::entry;

#[cfg(feature = "stm32f4xx")]
use stm32f4 as _;
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::stm32::Peripherals as DevicePeripherals;

#[cfg(feature = "stm32f4xx")]
use hal::i2c::I2c;

#[cfg(feature = "stm32f7xx")]
use stm32f7 as _;
#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal as hal;
#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::device::Peripherals as DevicePeripherals;

#[cfg(feature = "stm32f7xx")]
use hal::i2c::{BlockingI2c, Mode};

use hal::{delay::Delay, gpio::GpioExt, rcc::RccExt, time::U32Ext};

use drogue_bme680::{
    Address, Bme680Controller, Bme680Sensor, Configuration, DelayMsWrapper, StaticProvider,
};

use log::LevelFilter;
use rtt_logger::RTTLogger;

use embedded_time::duration::Milliseconds;

static LOGGER: RTTLogger = RTTLogger::new(LevelFilter::Info);

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockSkip, 4096);
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(log::LevelFilter::Trace);
    log::info!("Starting up...");

    let p = DevicePeripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    #[cfg(feature = "stm32f4xx")]
    let rcc = p.RCC.constrain();
    #[cfg(feature = "stm32f7xx")]
    let mut rcc = p.RCC.constrain();

    #[cfg(feature = "stm32f4xx")]
    let clocks = rcc.cfgr.sysclk(50.mhz()).freeze();
    #[cfg(feature = "stm32f7xx")]
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    #[cfg(feature = "stm32f4xx")]
    let gpiob = p.GPIOB.split();
    #[cfg(feature = "stm32f7xx")]
    let gpioh = p.GPIOH.split();

    // delay implementation

    let delay = Delay::new(cp.SYST, clocks);
    let delay = DelayMsWrapper::new(delay);

    // init

    #[cfg(feature = "stm32f4xx")]
    let (sda, scl) = {
        let sda = gpiob.pb9.into_alternate_af4_open_drain();
        let scl = gpiob.pb8.into_alternate_af4_open_drain();
        (sda, scl)
    };
    #[cfg(feature = "stm32f7xx")]
    let (sda, scl) = {
        let sda = gpioh.ph5.into_alternate_af4();
        let scl = gpioh.ph4.into_alternate_af4();
        (sda, scl)
    };

    // Initialize I2C

    #[cfg(feature = "stm32f4xx")]
    let i2c = I2c::i2c1(p.I2C1, (scl, sda), 100.khz(), clocks);
    #[cfg(feature = "stm32f7xx")]
    let i2c = BlockingI2c::i2c2(
        p.I2C2,
        (scl, sda),
        Mode::standard(100_000.hz()),
        clocks,
        &mut rcc.apb1,
        100,
    );

    let bme680 = Bme680Sensor::from(i2c, Address::Secondary).unwrap();

    let mut controller =
        Bme680Controller::new(bme680, delay, Configuration::standard(), StaticProvider(25))
            .unwrap();

    let mut cnt = 0;

    loop {
        if cnt == 0 {
            log::info!(
                "Measured: {:?}",
                controller.measure(5, Milliseconds(50)).unwrap()
            );
        }
        cnt += 1;
        if cnt > 30 {
            cnt = 0;
        }

        controller.delay(Milliseconds(1_000));
    }
}
