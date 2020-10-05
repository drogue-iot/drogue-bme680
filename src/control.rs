use crate::data::*;
use crate::{calc_profile_duration, Error};
use crate::{Bme680Sensor, BME680_CALIB_SIZE};

use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::timer::CountDown;

use embedded_time::duration::Milliseconds;
use nb::block;

/// Processes measurement data.
#[derive(Copy, Clone, Debug)]
pub struct Data {
    pub temperature: f32,
    pub pressure: Option<f32>,
    pub humidity: f32,
    pub gas_resistance: f32,
}

/// Controller for taking measurements with the sensor.
///
/// Compared to [`Bme680Sensor`], this controller proceeds through the necessary setup and
/// measurement steps.
pub struct Bme680Controller<'clock, I2C, T, C, ATP>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
    T: From<Milliseconds>,
    C: CountDown<Time = T>,
    ATP: Fn() -> i8,
{
    sensor: Bme680Sensor<I2C>,
    calibration: CalibrationInformation<[u8; BME680_CALIB_SIZE]>,
    config: Configuration,
    timer: &'clock mut C,
    ambient_temperature_provider: ATP,
    profile_duration: u32,
}

/// Current state.
#[derive(Copy, Clone, Debug)]
pub struct State {
    pub measuring: bool,
    pub gas_measuring: bool,
    pub heat_stable: bool,
    pub new_data: bool,
}

/// Configuration for the controller.
#[derive(Copy, Clone, Debug)]
pub struct Configuration {
    pub filter: Filter,
    pub temperature: Oversampling,
    pub pressure: Oversampling,
    pub humidity: Oversampling,
    pub heater_temperature: u16,
    pub heater_duration: Milliseconds,
}

impl Configuration {
    /// Provides a reasonable, default configuration.
    pub fn standard() -> Self {
        Configuration {
            filter: Filter::Coefficient3,
            pressure: Oversampling::By4,
            temperature: Oversampling::By8,
            humidity: Oversampling::By2,
            heater_temperature: 300,
            heater_duration: Milliseconds(150),
        }
    }
}

impl<'clock, I2C, T, C, ATP> Bme680Controller<'clock, I2C, T, C, ATP>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
    T: From<Milliseconds>,
    C: CountDown<Time = T>,
    ATP: Fn() -> i8,
{
    /// Create a new instance of a controller.
    ///
    /// # Arguments
    ///
    /// * `sensor` the sensor this controller wil use. It will take ownership of the sensor.
    /// * `timer` a timer which will be used for waiting.
    /// * `config` the configuration of the controller.
    /// * `ambient_temperature_provider` a provider for the ambient temperature (in degrees Celsius).
    ///
    /// Initially the ambient temperature can be acquired by doing some initial (temperature only)
    /// measurements. Or simply by using some reasonable default value, like 20 °C.
    pub fn new(
        mut sensor: Bme680Sensor<I2C>,
        timer: &'clock mut C,
        config: Configuration,
        ambient_temperature_provider: ATP,
    ) -> Result<Self, Error<I2C>> {
        let calibration = sensor.get_calibration_data()?;
        #[cfg(feature = "dump")]
        {
            let mut calibration = sensor.get_calibration_data()?;
            let calibration = CalibrationInformation(&mut calibration.0[..]);
            log::info!("par_t1 = {:?}", calibration.par_t1());
            log::info!("par_t2 = {:?}", calibration.par_t2());
            log::info!("par_t3 = {:?}", calibration.par_t3());

            log::info!("par_h1 = {:?}", calibration.par_h1());
            log::info!("par_h2 = {:?}", calibration.par_h2());
            log::info!("par_h3 = {:?}", calibration.par_h3());
            log::info!("par_h4 = {:?}", calibration.par_h4());
            log::info!("par_h5 = {:?}", calibration.par_h5());
            log::info!("par_h6 = {:?}", calibration.par_h6());
            log::info!("par_h7 = {:?}", calibration.par_h7());

            log::info!("par_p1 = {:?}", calibration.par_p1());
            log::info!("par_p2 = {:?}", calibration.par_p2());
            log::info!("par_p3 = {:?}", calibration.par_p3());
            log::info!("par_p4 = {:?}", calibration.par_p4());
            log::info!("par_p5 = {:?}", calibration.par_p5());
            log::info!("par_p6 = {:?}", calibration.par_p6());
            log::info!("par_p7 = {:?}", calibration.par_p7());
            log::info!("par_p8 = {:?}", calibration.par_p8());
            log::info!("par_p9 = {:?}", calibration.par_p9());
            log::info!("par_p10 = {:?}", calibration.par_p10());

            log::info!("par_gh1 = {:?}", calibration.par_gh1());
            log::info!("par_gh2 = {:?}", calibration.par_gh2());
            log::info!("par_gh3 = {:?}", calibration.par_gh3());

            log::info!("res_heat_range = {:?}", calibration.res_heat_range());
            log::info!("res_heat_val = {:?}", calibration.res_heat_val());
            log::info!("range_sw_err = {:?}", calibration.range_sw_err());
        }

        let profile_duration = calc_profile_duration(
            config.temperature,
            config.pressure,
            config.humidity,
            true,
            GasWaitTime::from(config.heater_duration.0 as u16),
        );

        Bme680Controller {
            sensor,
            calibration,
            config,
            timer,
            ambient_temperature_provider,
            profile_duration,
        }
        .init()
    }

    fn init(mut self) -> Result<Self, Error<I2C>> {
        // perform a soft reset
        self.sensor.soft_reset()?;
        self.delay(Milliseconds(10));

        // send to sleep
        self.send_to_sleep()?;

        // start configuration

        // Filter first

        let mut cfg = self.sensor.get_config()?;
        cfg.set_filter(self.config.filter);

        // control - temp, press

        let mut ctrl_meas = self.sensor.get_control_measurement()?;

        ctrl_meas.set_pressure(self.config.pressure);
        ctrl_meas.set_temperature(self.config.temperature);
        self.sensor.set_control_measurement(ctrl_meas)?;

        // control - hum

        let mut ctrl_hum = self.sensor.get_control_humidity()?;

        ctrl_hum.set_humidity(self.config.humidity);
        self.sensor.set_control_humidity(ctrl_hum)?;

        // gas sensor

        let mut gas = self.sensor.get_control_gas()?;
        // we always work with profile #0 here
        gas.set_nb_conv(HeaterProfile::Profile0);
        gas.set_run_gas(true);
        self.sensor.set_control_gas(gas)?;

        // set config

        self.set_gas_config(
            self.config.heater_temperature,
            self.config.heater_duration.0 as u16,
        )?;

        // done

        Ok(self)
    }

    fn delay<D>(&mut self, duration: D)
    where
        D: Into<T>,
    {
        self.timer.start(duration.into());
        block!(self.timer.wait()).ok();
    }

    fn send_to_sleep(&mut self) -> Result<(), Error<I2C>> {
        while self.sensor.get_power_mode()? != Mode::Sleep {
            self.sensor.set_power_mode(Mode::Sleep)?;
            self.delay(Milliseconds(10));
        }

        Ok(())
    }

    pub fn power_mode(&mut self) -> Result<Mode, Error<I2C>> {
        let cfg = self.sensor.get_control_measurement()?;
        Ok(cfg.mode())
    }

    fn calc_heater_res(&mut self, temperature: u16) -> u8 {
        // FIXME: blocked by: https://github.com/dzamlo/rust-bitfield/issues/29
        let mut calib = CalibrationInformation(&mut self.calibration.0[..]);

        calc_heater_res(
            &mut calib,
            (self.ambient_temperature_provider)(),
            temperature,
        )
    }

    /// Set the configuration for the gas sensor.
    ///
    /// # Arguments
    ///
    /// * `temperature` - temperature in degree Celsius
    /// * `duration` - duration in milliseconds
    ///
    fn set_gas_config(&mut self, temperature: u16, duration: u16) -> Result<(), Error<I2C>> {
        let resistance = self.calc_heater_res(temperature);
        let gas_wait = GasWaitTime::from(duration);

        #[cfg(feature = "dump")]
        {
            log::info!("heater_res: {:x?}, gas_wait: {:x?}", resistance, gas_wait.0);
        }

        self.sensor
            .set_raw_gas_config(HeaterProfile::Profile0, resistance, gas_wait)?;

        Ok(())
    }

    pub fn get_profile_duration(&self) -> u32 {
        self.profile_duration
    }

    fn read_data(&mut self) -> Result<(Data, State), Error<I2C>> {
        let data = self.sensor.read_raw_sensor_data()?;

        // FIXME: blocked by: https://github.com/dzamlo/rust-bitfield/issues/29
        let calib = CalibrationInformation(&mut self.calibration.0[..]);

        Ok(from_raw(calib, &data))
    }

    /// Perform a measurement with default re-try settings.
    ///
    /// This will:
    ///   * Set the power mode to "forced".
    ///   * Wait until the measurement is expected to be complete.
    ///   * Read the data until the "new data" flag is detected.
    pub fn measure_default(&mut self) -> Result<Option<Data>, Error<I2C>> {
        self.measure(5, Milliseconds(10))
    }

    /// Perform a measurement.
    ///
    /// This will:
    ///   * Set the power mode to "forced".
    ///   * Wait until the measurement is expected to be complete.
    ///   * Read the data until the "new data" flag is detected.
    pub fn measure(
        &mut self,
        retries: u8,
        delay: Milliseconds,
    ) -> Result<Option<Data>, Error<I2C>> {
        self.sensor.set_power_mode(Mode::Forced)?;

        #[cfg(feature = "dump")]
        {
            log::info!("Expected duration: {:} ms", self.profile_duration);
            log::info!("Gas control: {:?}", self.sensor.get_control_gas());
            log::info!(
                "Gas config: {:?}",
                self.sensor.get_raw_gas_config(HeaterProfile::Profile0)
            );
        }

        #[cfg(not(feature = "dump"))]
        self.delay(Milliseconds(self.profile_duration));

        let mut cnt = retries;

        loop {
            match self.read_data() {
                Ok(result) if result.1.new_data => {
                    #[cfg(feature = "dump")]
                    {
                        log::info!("State: {:?}", result.1);
                    }
                    return Ok(Some(result.0));
                }
                Ok(result) => {
                    #[cfg(feature = "dump")]
                    {
                        log::info!("State: {:?}", result.1);
                    }
                    {
                        cnt -= 1;
                        self.delay(delay);
                    }
                }
                Err(e) => {
                    return Err(e);
                }
            }
            if cnt == 0 {
                return Ok(None);
            }
        }
    }
}

fn for_state(raw: &RawData<[u8; 15]>) -> State {
    let gas_measuring = raw.gas_measuring();
    let measuring = raw.measuring();
    let heat_stable = raw.heat_stab();
    let new_data = raw.new_data();

    State {
        measuring,
        gas_measuring,
        heat_stable,
        new_data,
    }
}

/// Convert the raw sensor data into actual measurements, using the calibration information.
pub fn from_raw(
    calib: CalibrationInformation<&mut [u8]>,
    data: &RawData<[u8; 15]>,
) -> (Data, State) {
    #[cfg(feature = "dump")]
    {
        log::info!("Raw: {:?}", data);
    }

    let temperature = calc_temperature(&calib, data.temperature().0);
    let pressure = calc_pressure(&calib, data.pressure().0, temperature.1);
    let humidity = calc_humidity(&calib, data.humidity().0, temperature.1);
    let gas_resistance = calc_gas_resistance(&calib, data.gas_resistance().0, data.gas_range());

    #[cfg(feature = "dump")]
    {
        log::info!("t_fine: {}", temperature.1);
    }

    (
        Data {
            temperature: temperature.0,
            pressure,
            humidity,
            gas_resistance,
        },
        for_state(data),
    )
}

fn calc_heater_res(
    calib: &CalibrationInformation<&mut [u8]>,
    ambient_temperature: i8,
    temperature: u16,
) -> u8 {
    // cap the temperature at 400 degrees Celsius
    let temperature = if temperature > 400 { 400 } else { temperature };

    let var1 = (calib.par_gh1() as f32 / (16.0)) + 49.0;
    let var2 = ((calib.par_gh2() as f32 / (32768.0)) * (0.0005)) + 0.00235;
    let var3 = calib.par_gh3() as f32 / (1024.0);
    let var4 = var1 * (1.0 + (var2 * temperature as f32));
    let var5 = var4 + (var3 * ambient_temperature as f32);
    let res_heat = 3.4
        * ((var5
            * (4.0 / (4.0 + calib.res_heat_range() as f32))
            * (1.0 / (1.0 + (calib.res_heat_val() as f32 * 0.002))))
            - 25.0);

    // convert f32 to i8 inside a u8

    let res_heat_r = res_heat as i16;
    let res_heat_r = res_heat_r as u8;

    #[cfg(feature = "dump")]
    {
        log::info!(
            "var1 = {}, var2 = {}, var3 = {}, var4 = {}, var5 = {}, res_heat = {} ({})",
            var1,
            var2,
            var3,
            var4,
            var5,
            res_heat,
            res_heat_r,
        );
    }

    res_heat_r
}

fn calc_temperature(calib: &CalibrationInformation<&mut [u8]>, temp_adc: u32) -> (f32, f32) {
    let temp_adc = temp_adc as f32;

    let var1 = ((temp_adc / 16384.0) - (calib.par_t1() as f32 / 1024.0)) * (calib.par_t2() as f32);

    let var2 = (((temp_adc / 131072.0) - (calib.par_t1() as f32 / 8192.0))
        * ((temp_adc / 131072.0) - (calib.par_t1() as f32 / 8192.0)))
        * (calib.par_t3() as f32 * 16.0);

    let t_fine = var1 + var2;
    let calc_temp = t_fine / 5120.0;

    (calc_temp, t_fine)
}

fn calc_pressure(
    calib: &CalibrationInformation<&mut [u8]>,
    pres_adc: u32,
    t_fine: f32,
) -> Option<f32> {
    let pres_adc = pres_adc as f32;

    let var1 = (t_fine / 2.0) - 64000.0;
    let var2 = var1 * var1 * ((calib.par_p6() as f32) / (131072.0));
    let var2 = var2 + (var1 * (calib.par_p5() as f32) * 2.0);
    let var2 = (var2 / 4.0) + ((calib.par_p4() as f32) * 65536.0);
    let var1 = (((calib.par_p3() as f32 * var1 * var1) / 16384.0) + (calib.par_p2() as f32 * var1))
        / 524288.0;
    let var1 = (1.0 + (var1 / 32768.0)) * (calib.par_p1() as f32);
    let calc_pres = 1048576.0 - (pres_adc);

    if var1 != 0.0 {
        let calc_pres = ((calc_pres - (var2 / 4096.0)) * 6250.0) / var1;
        let var1 = ((calib.par_p9() as f32) * calc_pres * calc_pres) / 2147483648.0;
        let var2 = calc_pres * ((calib.par_p8() as f32) / 32768.0);
        let var3 = (calc_pres / 256.0)
            * (calc_pres / 256.0)
            * (calc_pres / 256.0)
            * (calib.par_p10() as f32 / 131072.0);
        let calc_pres = calc_pres + (var1 + var2 + var3 + (calib.par_p7() as f32 * 128.0)) / 16.0;

        Some(calc_pres)
    } else {
        None
    }
}

fn calc_humidity(calib: &CalibrationInformation<&mut [u8]>, hum_adc: u16, t_fine: f32) -> f32 {
    let temp_comp = t_fine / 5120.0;
    let hum_adc = hum_adc as f32;

    let var1 = (hum_adc)
        - ((calib.par_h1().0 as f32 * 16.0) + ((calib.par_h3() as f32 / 2.0) * temp_comp));

    let var2 = var1
        * ((calib.par_h2().0 as f32 / 262144.0)
            * (1.0
                + ((calib.par_h4() as f32 / 16384.0) * temp_comp)
                + ((calib.par_h5() as f32 / 1048576.0) * temp_comp * temp_comp)));

    let var3 = calib.par_h6() as f32 / 16384.0;
    let var4 = calib.par_h7() as f32 / 2097152.0;

    let calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    if calc_hum > 100.0 {
        100.0
    } else if calc_hum < 0.0 {
        0.0
    } else {
        calc_hum
    }
}

fn calc_gas_resistance(
    calib: &CalibrationInformation<&mut [u8]>,
    gas_res_adc: u16,
    gas_range: u8,
) -> f32 {
    let gas_res_adc = gas_res_adc as f32;

    const LOOKUP_K1_RANGE: [f32; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0,
    ];
    const LOOKUP_K2_RANGE: [f32; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];

    let var1 = 1340.0 + (5.0 * calib.range_sw_err() as f32);
    let var2 = var1 * (1.0 + LOOKUP_K1_RANGE[gas_range as usize] / 100.0);
    let var3 = 1.0 + (LOOKUP_K2_RANGE[gas_range as usize] / 100.0);

    let calc_gas_res = 1.0
        / (var3 * 0.000000125 * (1 << gas_range) as f32 * ((((gas_res_adc) - 512.0) / var2) + 1.0));

    calc_gas_res
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_temp() {
        crate::test::init();

        let mut buf = [0u8; BME680_CALIB_SIZE];
        let mut calib = CalibrationInformation(&mut buf[..]);
        calib.set_par_t1(1);
        calib.set_par_t2(2);
        calib.set_par_t3(3);
        let temp = calc_temperature(&calib, 10_000);

        assert_eq!((0.00029243232, 1.4972534), temp);
    }

    #[test]
    fn calc_heater_res_1() {
        crate::test::init();

        let mut buf = [0u8; BME680_CALIB_SIZE];
        let calib = CalibrationInformation(&mut buf[..]);
        let heat = calc_heater_res(&calib, 25, 300) as u8;

        assert_eq!(199, heat);
    }

    #[test]
    fn calc_heater_res_2() {
        crate::test::init();

        let mut buf = [0u8; BME680_CALIB_SIZE];
        let mut calib = CalibrationInformation(&mut buf[..]);
        calib.set_par_gh1(-21);
        calib.set_par_gh2(-11054);
        calib.set_par_gh3(18);
        calib.set_res_heat_range(22);
        calib.set_res_heat_val(46);
        let heat = calc_heater_res(&calib, 25, 300);

        log::info!("{}", heat);

        assert_eq!(210, heat as u8);
    }
}
