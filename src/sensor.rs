use embedded_hal::blocking::i2c::{Write, WriteRead};

use crate::data::*;
use crate::error::Error;
use embedded_time::duration::Milliseconds;

const BME680_SOFT_RESET_ADDR: u8 = 0xE0;
const BME680_CHIP_ID_ADDR: u8 = 0xD0;
const BME680_CONFIG_ADDR: u8 = 0x75;
const BME680_CTRL_MEAS_ADDR: u8 = 0x74;
const BME680_CTRL_HUM_ADDR: u8 = 0x72;
const BME680_CTRL_GAS_1_ADDR: u8 = 0x71;
const BME680_CTRL_GAS_0_ADDR: u8 = 0x70;

const BME680_CTRL_GAS_WAIT_0_ADDR: u8 = 0x64;
const BME680_CTRL_RES_HEAT_0_ADDR: u8 = 0x5A;
const BME680_CTRL_IDAC_HEAT_0_ADDR: u8 = 0x50;

const BME680_ADDR_RANGE_SW_ERR_ADDR: u8 = 0x04;
const BME680_ADDR_RES_HEAT_RANGE_ADDR: u8 = 0x02;
const BME680_ADDR_RES_HEAT_VAL_ADDR: u8 = 0x00;

pub(crate) const BME680_COEFF_SIZE: usize = 41;
pub(crate) const BME680_CALIB_SIZE: usize = BME680_COEFF_SIZE + 3;

pub(crate) const BME680_DATA_SIZE: usize = 15;

const BME680_COEFF_ADDR1: u8 = 0x89;
const BME680_COEFF_ADDR2: u8 = 0xE1;
const BME680_COEFF_ADDR1_LEN: usize = 25;
const BME680_COEFF_ADDR2_LEN: usize = 16;

const BME680_FIELD0_ADDR: u8 = 0x1D;

const BME680_CHIP_ID: u8 = 0x61;
const BME680_SOFT_RESET_CMD: u8 = 0xB6;

/// General purpose sensor access.
pub struct Bme680Sensor<I2C>
where
    I2C: WriteRead,
{
    id: u8,
    i2c: I2C,
}

impl<I2C> Bme680Sensor<I2C>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
{
    pub fn from(i2c: I2C, address: Address) -> Result<Self, Error<I2C>> {
        let id = match address {
            Address::Primary => 0b1110110u8,
            Address::Secondary => 0b1110111u8,
        };
        Bme680Sensor { id, i2c }.init()
    }

    fn init(mut self) -> Result<Self, Error<I2C>> {
        let chip_id = self.get_register(BME680_CHIP_ID_ADDR)?;

        if chip_id != BME680_CHIP_ID {
            Err(Error::WrongDevice)
        } else {
            Ok(self)
        }
    }

    fn get_register(&mut self, register: u8) -> Result<u8, Error<I2C>> {
        let mut buffer = [0u8; 1];
        self.get_registers(register, &mut buffer).map(|b| b[0])
    }

    fn get_registers<'b>(
        &mut self,
        start_register: u8,
        mut buffer: &'b mut [u8],
    ) -> Result<&'b [u8], Error<I2C>> {
        self.i2c
            .write_read(self.id, &[start_register], &mut buffer)
            .map_err(|e| Error::WriteReadError(e))?;
        Ok(buffer)
    }

    fn set_register(&mut self, register: u8, value: u8) -> Result<(), Error<I2C>> {
        self.write(&[register, value])
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error<I2C>> {
        self.i2c
            .write(self.id, data)
            .map_err(|e| Error::WriteError(e))
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<I2C>> {
        self.set_register(BME680_SOFT_RESET_ADDR, BME680_SOFT_RESET_CMD)
    }

    pub fn get_control_measurement(&mut self) -> Result<ControlMeasurement, Error<I2C>> {
        self.get_register(BME680_CTRL_MEAS_ADDR)
            .map(|b| ControlMeasurement(b))
    }

    pub fn set_control_measurement(
        &mut self,
        ctrl_meas: ControlMeasurement,
    ) -> Result<(), Error<I2C>> {
        self.set_register(BME680_CTRL_MEAS_ADDR, ctrl_meas.0)
    }

    pub fn set_power_mode(&mut self, mode: Mode) -> Result<(), Error<I2C>> {
        let mut ctrl_meas = self.get_control_measurement()?;
        ctrl_meas.set_mode(mode);
        self.set_control_measurement(ctrl_meas)
    }

    pub fn get_power_mode(&mut self) -> Result<Mode, Error<I2C>> {
        let ctrl_meas = self.get_control_measurement()?;
        Ok(ctrl_meas.mode())
    }

    pub fn get_control_humidity(&mut self) -> Result<ControlHumidity, Error<I2C>> {
        self.get_register(BME680_CTRL_HUM_ADDR)
            .map(|b| ControlHumidity(b))
    }

    pub fn set_control_humidity(&mut self, ctrl_hum: ControlHumidity) -> Result<(), Error<I2C>> {
        self.set_register(BME680_CTRL_HUM_ADDR, ctrl_hum.0)
    }

    pub fn get_config(&mut self) -> Result<Config, Error<I2C>> {
        self.get_register(BME680_CONFIG_ADDR).map(|b| Config(b))
    }

    pub fn set_config(&mut self, ctrl_hum: Config) -> Result<(), Error<I2C>> {
        self.set_register(BME680_CONFIG_ADDR, ctrl_hum.0)
    }

    pub fn get_control_gas(&mut self) -> Result<ControlGas, Error<I2C>> {
        self.get_register(BME680_CTRL_GAS_1_ADDR)
            .map(|b| ControlGas(b))
    }

    pub fn set_control_gas(&mut self, ctrl_gas: ControlGas) -> Result<(), Error<I2C>> {
        self.set_register(BME680_CTRL_GAS_1_ADDR, ctrl_gas.0)
    }

    pub fn get_control_gas_heater(&mut self) -> Result<ControlGasHeater, Error<I2C>> {
        self.get_register(BME680_CTRL_GAS_0_ADDR)
            .map(|b| ControlGasHeater(b))
    }

    pub fn set_control_gas_heater(
        &mut self,
        ctrl_gas_heater: ControlGasHeater,
    ) -> Result<(), Error<I2C>> {
        self.set_register(BME680_CTRL_GAS_0_ADDR, ctrl_gas_heater.0)
    }

    pub fn set_gas_heater_off(&mut self, value: bool) -> Result<(), Error<I2C>> {
        self.get_control_gas_heater().and_then(|mut state| {
            state.set_heat_off(value);
            self.set_control_gas_heater(state)
        })
    }

    fn get_profile_data(&mut self, start_register: u8) -> Result<[u8; 10], Error<I2C>> {
        let mut buffer = [0u8; 10];
        self.get_registers(start_register, &mut buffer)?;
        Ok(buffer)
    }

    fn set_profile_data(
        &mut self,
        start_register: u8,
        values: &[u8; 10],
    ) -> Result<(), Error<I2C>> {
        let mut data = [0u8; 11];

        data[0] = start_register;
        data[1..].copy_from_slice(values);

        self.write(&mut data)
    }

    pub fn get_gas_wait(&mut self) -> Result<[GasWaitTime; 10], Error<I2C>> {
        self.get_profile_data(BME680_CTRL_GAS_WAIT_0_ADDR).map(|b| {
            let mut result = [GasWaitTime(0); 10];
            for i in 0..10 {
                result[i] = GasWaitTime(b[i]);
            }
            result
        })
    }

    pub fn set_gas_wait(&mut self, values: &[GasWaitTime; 10]) -> Result<(), Error<I2C>> {
        let mut buffer = [0u8; 11];
        buffer[0] = BME680_CTRL_GAS_WAIT_0_ADDR;
        for i in 0..10 {
            buffer[i + 1] = values[i].0;
        }
        self.write(&buffer)
    }

    pub fn get_target_heater_resistance(&mut self) -> Result<[u8; 10], Error<I2C>> {
        self.get_profile_data(BME680_CTRL_RES_HEAT_0_ADDR)
    }

    pub fn set_target_heater_resistance(&mut self, values: &[u8; 10]) -> Result<(), Error<I2C>> {
        self.set_profile_data(BME680_CTRL_RES_HEAT_0_ADDR, values)
    }

    pub fn get_heater_current(&mut self) -> Result<[u8; 10], Error<I2C>> {
        self.get_profile_data(BME680_CTRL_IDAC_HEAT_0_ADDR)
    }

    pub fn set_heater_current(&mut self, values: &[u8; 10]) -> Result<(), Error<I2C>> {
        self.set_profile_data(BME680_CTRL_IDAC_HEAT_0_ADDR, values)
    }

    pub fn set_raw_gas_config(
        &mut self,
        profile: HeaterProfile,
        resistance: u8,
        gas_wait: GasWaitTime,
    ) -> Result<(), Error<I2C>> {
        let offset: u8 = profile.into();

        self.set_register(BME680_CTRL_RES_HEAT_0_ADDR + offset, resistance)?;
        self.set_register(BME680_CTRL_GAS_WAIT_0_ADDR + offset, gas_wait.0)?;

        Ok(())
    }

    pub fn get_raw_gas_config(
        &mut self,
        profile: HeaterProfile,
    ) -> Result<(u8, GasWaitTime), Error<I2C>> {
        let offset: u8 = profile.into();

        let r = self.get_register(BME680_CTRL_RES_HEAT_0_ADDR + offset)?;
        let g = self.get_register(BME680_CTRL_GAS_WAIT_0_ADDR + offset)?;

        Ok((r, GasWaitTime(g)))
    }

    pub fn get_calibration_data(
        &mut self,
    ) -> Result<CalibrationInformation<[u8; BME680_CALIB_SIZE]>, Error<I2C>> {
        let mut buffer = [0u8; BME680_CALIB_SIZE];

        self.get_registers(BME680_COEFF_ADDR1, &mut buffer[0..BME680_COEFF_ADDR1_LEN])?;
        self.get_registers(
            BME680_COEFF_ADDR2,
            &mut buffer[BME680_COEFF_ADDR1_LEN..BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN],
        )?;

        buffer[41] = self.get_register(BME680_ADDR_RES_HEAT_RANGE_ADDR)?;
        buffer[42] = self.get_register(BME680_ADDR_RES_HEAT_VAL_ADDR)?;
        buffer[43] = self.get_register(BME680_ADDR_RANGE_SW_ERR_ADDR)?;

        #[cfg(feature = "dump")]
        {
            log::info!("calib = {:?}", &buffer[0..41]);
            log::info!("res_heat_range = {:?}", buffer[41]);
            log::info!("res_heat_val = {:?}", buffer[42]);
            log::info!("range_sw_err = {:?}", buffer[43]);
        }

        Ok(CalibrationInformation(buffer))
    }

    pub fn read_raw_sensor_data(&mut self) -> Result<RawData<[u8; BME680_DATA_SIZE]>, Error<I2C>> {
        let mut buf = [0u8; 15];
        self.get_registers(BME680_FIELD0_ADDR, &mut buf)?;
        #[cfg(feature = "dump")]
        {
            log::info!("sensor_data = {:?}", &buf);
        }
        Ok(RawData(buf))
    }
}

/// Calculate how a measurement will take, based on the provided settings.
pub fn calc_profile_duration(
    temperature: Oversampling,
    pressure: Oversampling,
    humidity: Oversampling,
    run_gas: bool,
    gwt: GasWaitTime,
) -> Milliseconds {
    let mut result = 0u32;

    let mut meas_cycles = temperature.cycles();
    meas_cycles += pressure.cycles();
    meas_cycles += humidity.cycles();

    result += meas_cycles as u32 * 1963;
    result += 477 * 4; /* TPH switching duration */
    result += 477 * 5; /* Gas measurement duration */
    result += 500; /* Get it to the closest whole number.*/
    result /= 1000; /* Convert to ms */

    result += 1; /* Wake up duration of 1ms */

    if run_gas {
        result += gwt.as_millis();
    }

    Milliseconds(result)
}
