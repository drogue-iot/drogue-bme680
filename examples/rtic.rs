#![deny(unsafe_code)]
#![no_main]
#![no_std]

//! Example for using the BME680 sensor with an RTIC application.
//!
//! ## STM32F411RE - Nucleo 64
//!
//! Connect the BME680 sensor to the I2C port #1 using pins PB8/PB9.
//!
//! ## STM32F723E - DISCOVERY
//!
//! Connect the BME680 to the STM32F723E DISCOVERY board via the Grove I2C connector
//! on the extension board.
//!
//! **NOTE:** Currently I2C is broken in combination with RTIC and STM32F7xx. So this example
//! is expected to fail at the moment.
//!

#[cfg(not(feature = "env_logging"))]
use panic_rtt_target as _;
use rtt_target::rtt_init_print;

#[cfg(feature = "stm32f4xx")]
use stm32f4 as _;
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::stm32 as device;

#[cfg(feature = "stm32f4xx")]
use hal::{gpio::AlternateOD, i2c::I2c};

#[cfg(feature = "stm32f7xx")]
use stm32f7 as _;
#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal as hal;
#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::device;

#[cfg(feature = "stm32f7xx")]
use hal::{
    gpio::Alternate,
    i2c::{BlockingI2c, Mode},
};

use hal::{gpio::GpioExt, gpio::AF4, rcc::RccExt, time::U32Ext};

#[cfg(feature = "stm32f7xx")]
use rtic::export::DWT;

use drogue_bme680::{Address, Bme680Sensor, CalibrationInformation};

use log::LevelFilter;
use rtt_logger::RTTLogger;

static LOGGER: RTTLogger = RTTLogger::new(LevelFilter::Info);

pub trait I2cConfig<Instance> {
    type SclPin;
    type SdaPin;
}

#[cfg(feature = "stm32f4xx")]
type I2CInstance = stm32f4::stm32f411::I2C1;
#[cfg(feature = "stm32f4xx")]
impl<Instance> I2cConfig<Instance> for I2CInstance {
    type SclPin = hal::gpio::gpiob::PB8<AlternateOD<AF4>>;
    type SdaPin = hal::gpio::gpiob::PB9<AlternateOD<AF4>>;
}
#[cfg(feature = "stm32f4xx")]
type I2cBmeInstance = I2c<
    I2CInstance,
    (
        <I2CInstance as I2cConfig<I2CInstance>>::SclPin,
        <I2CInstance as I2cConfig<I2CInstance>>::SdaPin,
    ),
>;

#[cfg(feature = "stm32f7xx")]
type I2CInstance = stm32f7::stm32f7x3::I2C2;
#[cfg(feature = "stm32f7xx")]
impl<Instance> I2cConfig<Instance> for I2CInstance {
    type SclPin = hal::gpio::gpioh::PH4<Alternate<AF4>>;
    type SdaPin = hal::gpio::gpioh::PH5<Alternate<AF4>>;
}

#[cfg(feature = "stm32f7xx")]
type I2cBmeInstance = BlockingI2c<
    I2CInstance,
    <I2CInstance as I2cConfig<I2CInstance>>::SclPin,
    <I2CInstance as I2cConfig<I2CInstance>>::SdaPin,
>;

#[rtic::app(device = crate::device, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        i2c: Option<I2cBmeInstance>,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        rtt_init_print!(NoBlockSkip, 4096);
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Trace);
        log::info!("Starting up...");

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        #[cfg(feature = "stm32f7xx")]
        DWT::unlock();
        cx.core.DWT.enable_cycle_counter();

        let device: device::Peripherals = cx.device;

        // semantically, the monotonic timer is frozen at time "zero" during `init`
        // NOTE do *not* call `Instant::now` in this context; it will return a nonsense value
        // let now = cx.start; // the start time of the system

        // init sensor

        #[cfg(feature = "stm32f4xx")]
        let rcc = device.RCC.constrain();
        #[cfg(feature = "stm32f7xx")]
        let mut rcc = device.RCC.constrain();

        #[cfg(feature = "stm32f4xx")]
        let clocks = rcc.cfgr.sysclk(50.mhz()).freeze();
        #[cfg(feature = "stm32f7xx")]
        let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

        // let gpioa = p.GPIOA.split();
        let gpiob = device.GPIOB.split();
        // let gpioc = p.GPIOC.split();
        // let gpiod = p.GPIOD.split();
        // let gpiog = p.GPIOG.split();
        let gpioh = device.GPIOH.split();
        // let gpioi = p.GPIOI.split();

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

        log::info!("Init I2C ...");

        #[cfg(feature = "stm32f4xx")]
        let i2c = I2c::i2c1(device.I2C1, (scl, sda), 100.khz(), clocks);
        #[cfg(feature = "stm32f7xx")]
        let i2c = BlockingI2c::i2c2(
            device.I2C2,
            (scl, sda),
            Mode::standard(100_000.hz()),
            clocks,
            &mut rcc.apb1,
            100,
        );

        init::LateResources { i2c: Some(i2c) }
    }

    #[idle(resources=[i2c])]
    fn idle(cx: idle::Context) -> ! {
        let i2c = cx.resources.i2c.take().unwrap();

        let mut bme680 = Bme680Sensor::from(i2c, Address::Secondary).unwrap();
        let mut calib = bme680.get_calibration_data().unwrap();
        log::info!("Calib: {:#?}", CalibrationInformation(&mut calib.0[..]));

        loop {
            cortex_m::asm::nop();
        }
    }
};
