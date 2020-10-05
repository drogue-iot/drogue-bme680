/// Interface for a BME680 sensor using I2C.
use bitfield::bitfield;

macro_rules! sb {
    ($start:expr) => {
        ($start * 8)
    };
}

macro_rules! eb {
    ($end:expr) => {
        (sb!($end) + 7)
    };
}

#[derive(Copy, Clone, Debug)]
pub enum Address {
    Primary,
    Secondary,
}

#[derive(Copy, Clone, Debug)]
pub enum Oversampling {
    Skip,
    By1,
    By2,
    By4,
    By8,
    By16,
}

impl Oversampling {
    pub fn cycles(&self) -> u8 {
        match self {
            Oversampling::Skip => 0,
            Oversampling::By1 => 1,
            Oversampling::By2 => 2,
            Oversampling::By4 => 4,
            Oversampling::By8 => 8,
            Oversampling::By16 => 16,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Mode {
    Sleep,
    Forced,
    Unknown,
}

#[derive(Copy, Clone, Debug)]
pub enum Filter {
    Coefficient0,
    Coefficient1,
    Coefficient3,
    Coefficient7,
    Coefficient15,
    Coefficient31,
    Coefficient63,
    Coefficient127,
}

#[derive(Copy, Clone, Debug)]
pub enum HeaterProfile {
    Profile0,
    Profile1,
    Profile2,
    Profile3,
    Profile4,
    Profile5,
    Profile6,
    Profile7,
    Profile8,
    Profile9,
}

#[derive(Copy, Clone, Debug)]
pub enum Multiplier {
    By1,
    By4,
    By16,
    By64,
}

impl Multiplier {
    pub fn as_u8(&self) -> u8 {
        match self {
            Multiplier::By1 => 1,
            Multiplier::By4 => 4,
            Multiplier::By16 => 16,
            Multiplier::By64 => 64,
        }
    }
}

bitfield! {
    /// The raw sensor data.
    pub struct RawData([u8]);
    impl Debug;

    pub u8, gas_meas_index, _ : 3, 0;
    pub measuring, _ : 5;
    pub gas_measuring, _ : 6;
    pub new_data, _ : 7;

    pub u32, from into Measurement, pressure, _ : eb!(0x21-0x1D), sb!(0x1F-0x1D);
    pub u32, from into Measurement, temperature, _ : eb!(0x24-0x1D), sb!(0x22-0x1D);
    pub u16, from into SwapU16, humidity, _ : eb!(0x26-0x1D), sb!(0x25-0x1D);
    pub u16, from into GasResistance, gas_resistance, _ : eb!(0x2B-0x1D), sb!(0x2A-0x1D);

    pub gas_valid, _ : sb!(0x2B-0x1D) + 5;
    pub heat_stab, _ : sb!(0x2B-0x1D) + 4;
    pub u8, gas_range, _ : sb!(0x2B-0x1D) + 3, sb!(0x2B-0x1D) + 0;
}

bitfield! {
    /// Control measurement and temperature, pressure oversampling.
    pub struct ControlMeasurement(u8);
    impl Debug;

    pub u8, from into Oversampling, temperature, set_temperature : 7, 5;
    pub u8, from into Oversampling, pressure, set_pressure : 4, 2;
    pub u8, from into Mode, mode, set_mode : 1, 0;
}

bitfield! {
    /// Control humidity oversampling.
    pub struct ControlHumidity(u8);
    impl Debug;

    pub u8, from into Oversampling, humidity, set_humidity : 2, 0;
}

bitfield! {
    /// Control gas measurement.
    pub struct ControlGas(u8);
    impl Debug;

    pub u8, from into HeaterProfile, nb_conv, set_nb_conv : 3, 0;
    pub run_gas, set_run_gas : 4;
}

bitfield! {
    /// Control gas sensor heater.
    pub struct ControlGasHeater(u8);
    impl Debug;

    pub heat_off, set_heat_off : 3;
}

bitfield! {
    pub struct Config(u8);
    impl Debug;

    /// Control temperature and pressure sensor filter.
    pub u8, from into Filter, filter, set_filter : 2, 0;
}

bitfield! {
    #[derive(Copy, Clone)]
    pub struct GasWaitTime(u8);
    impl Debug;

    pub u8, time, set_time : 5, 0;
    pub u8, from into Multiplier, multiplier, set_multiplier : 7, 6;
}

bitfield! {
    /// Sensor calibration information.
    pub struct CalibrationInformation([u8]);
    impl Debug;

    pub u16, par_t1, set_par_t1 : eb!(34), sb!(33);
    pub i16, par_t2, set_par_t2 : eb!(2), sb!(1);
    pub i8, par_t3, set_par_t3 : eb!(3), sb!(3);

    pub u16, par_p1, set_par_p1 : eb!(6), sb!(5);
    pub i16, par_p2, set_par_p2 : eb!(8), sb!(7);
    pub i8, par_p3, set_par_p3 : eb!(9), sb!(9);
    pub i16, par_p4, set_par_p4 : eb!(12), sb!(11);
    pub i16, par_p5, set_par_p5 : eb!(14), sb!(13);
    pub i8, par_p6, set_par_p6 : eb!(16), sb!(16);
    pub i8, par_p7, set_par_p7 : eb!(15), sb!(15);
    pub i16, par_p8, set_par_p8 : eb!(20), sb!(19);
    pub i16, par_p9, set_par_p9 : eb!(22), sb!(21);
    pub u8, par_p10, set_par_p10 : eb!(23), sb!(23);

    pub u16, from into ParameterH1, par_h1, _ : eb!(27), sb!(26);
    pub u16, from into ParameterH2, par_h2, _ : eb!(26), sb!(25);

    pub i8, par_h3, set_par_h3 : eb!(28), sb!(28);
    pub i8, par_h4, set_par_h4 : eb!(29), sb!(29);
    pub i8, par_h5, set_par_h5 : eb!(30), sb!(30);
    pub u8, par_h6, set_par_h6 : eb!(31), sb!(31);
    pub i8, par_h7, set_par_h7 : eb!(32), sb!(32);

    pub i8, par_gh1, set_par_gh1 : eb!(37), sb!(37);
    pub i16, par_gh2, set_par_gh2 : eb!(36), sb!(35);
    pub i8, par_gh3, set_par_gh3 : eb!(38), sb!(38);

    /// Heater resistance range
    pub u8, res_heat_range, set_res_heat_range : eb!(41), sb!(41);
    /// Heater resistance value
    pub i8, res_heat_val, set_res_heat_val : eb!(42), sb!(42);
    /// Error range
    pub i8, range_sw_err, set_range_sw_err : eb!(43), sb!(43);
}

#[derive(Copy, Clone, Debug)]
pub struct ParameterH1(pub u16);

impl From<u16> for ParameterH1 {
    fn from(value: u16) -> Self {
        let value = value.to_be_bytes();
        // log::info!("H1: {:x?}", value);
        ParameterH1(((value[0] as u16) << 4) | ((value[1] as u16) & 0x0F))
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ParameterH2(pub u16);

impl From<u16> for ParameterH2 {
    fn from(value: u16) -> Self {
        let value = value.to_be_bytes();
        // log::info!("H2: {:x?}", value);
        ParameterH2(((value[1] as u16) << 4) | ((value[0] as u16) >> 4))
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GasResistance(pub u16);

impl From<u16> for GasResistance {
    fn from(value: u16) -> Self {
        let value = value.to_be_bytes();
        #[cfg(feature = "dump")]
        {
            log::info!("GasResistance: {:x?}", value);
        }
        let v = (value[1] as u16) << 2 | ((value[0] & 0b11000000) as u16) >> 6;
        GasResistance(v)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Measurement(pub u32);

impl From<u32> for Measurement {
    fn from(value: u32) -> Self {
        let value = value.to_be_bytes();
        let v = (value[3] as u32) << 12 | (value[2] as u32) << 4 | (value[1] as u32) >> 4;
        Measurement(v)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SwapU16(pub u16);

impl From<u16> for SwapU16 {
    fn from(value: u16) -> Self {
        SwapU16(u16::from_le_bytes(value.to_be_bytes()))
    }
}

impl From<u8> for Mode {
    fn from(value: u8) -> Self {
        match value {
            0b00 => Mode::Sleep,
            0b01 => Mode::Forced,
            _ => Mode::Unknown,
        }
    }
}

impl Into<u8> for Mode {
    fn into(self) -> u8 {
        match self {
            Mode::Sleep => 0b00,
            Mode::Forced => 0b01,
            Mode::Unknown => 0b00,
        }
    }
}

impl From<u8> for Oversampling {
    fn from(value: u8) -> Self {
        match value {
            0b000 => Oversampling::Skip,
            0b001 => Oversampling::By1,
            0b010 => Oversampling::By2,
            0b011 => Oversampling::By4,
            0b100 => Oversampling::By8,
            0b101 | _ => Oversampling::By16,
        }
    }
}

impl Into<u8> for Oversampling {
    fn into(self) -> u8 {
        match self {
            Oversampling::Skip => 0b000,
            Oversampling::By1 => 0b001,
            Oversampling::By2 => 0b010,
            Oversampling::By4 => 0b011,
            Oversampling::By8 => 0b100,
            Oversampling::By16 => 0b101,
        }
    }
}

impl From<u8> for Filter {
    fn from(value: u8) -> Self {
        match value {
            0b000 => Filter::Coefficient0,
            0b001 => Filter::Coefficient1,
            0b010 => Filter::Coefficient3,
            0b011 => Filter::Coefficient7,
            0b100 => Filter::Coefficient15,
            0b101 => Filter::Coefficient31,
            0b110 => Filter::Coefficient63,
            0b111 | _ => Filter::Coefficient127,
        }
    }
}

impl Into<u8> for Filter {
    fn into(self) -> u8 {
        match self {
            Filter::Coefficient0 => 0b000,
            Filter::Coefficient1 => 0b001,
            Filter::Coefficient3 => 0b010,
            Filter::Coefficient7 => 0b011,
            Filter::Coefficient15 => 0b100,
            Filter::Coefficient31 => 0b101,
            Filter::Coefficient63 => 0b110,
            Filter::Coefficient127 => 0b111,
        }
    }
}

impl From<u8> for HeaterProfile {
    fn from(value: u8) -> Self {
        match value {
            0b0000 => HeaterProfile::Profile0,
            0b0001 => HeaterProfile::Profile1,
            0b0010 => HeaterProfile::Profile2,
            0b0011 => HeaterProfile::Profile3,
            0b0100 => HeaterProfile::Profile4,
            0b0101 => HeaterProfile::Profile5,
            0b0110 => HeaterProfile::Profile6,
            0b0111 => HeaterProfile::Profile7,
            0b1000 => HeaterProfile::Profile8,
            0b1001 | _ => HeaterProfile::Profile9,
        }
    }
}

impl Into<u8> for HeaterProfile {
    fn into(self) -> u8 {
        match self {
            HeaterProfile::Profile0 => 0b0000,
            HeaterProfile::Profile1 => 0b0001,
            HeaterProfile::Profile2 => 0b0010,
            HeaterProfile::Profile3 => 0b0011,
            HeaterProfile::Profile4 => 0b0100,
            HeaterProfile::Profile5 => 0b0101,
            HeaterProfile::Profile6 => 0b0110,
            HeaterProfile::Profile7 => 0b0111,
            HeaterProfile::Profile8 => 0b1000,
            HeaterProfile::Profile9 => 0b1001,
        }
    }
}

impl From<u8> for Multiplier {
    fn from(value: u8) -> Self {
        match value {
            0b00 => Multiplier::By1,
            0b01 => Multiplier::By4,
            0b10 => Multiplier::By16,
            0b11 | _ => Multiplier::By64,
        }
    }
}

impl Into<u8> for Multiplier {
    fn into(self) -> u8 {
        match self {
            Multiplier::By1 => 0b00,
            Multiplier::By4 => 0b01,
            Multiplier::By16 => 0b10,
            Multiplier::By64 => 0b11,
        }
    }
}

impl GasWaitTime {
    pub fn from(mut duration: u16) -> Self {
        if duration > 0xFC0 {
            GasWaitTime(0xFF)
        } else {
            let mut multiplier = 0u8;
            while duration > 0x3F {
                duration = duration >> 2;
                multiplier += 1;
            }
            let mut result = GasWaitTime(0);
            result.set_time(duration as u8);
            result.set_multiplier(multiplier.into());
            result
        }
    }

    pub fn as_millis(&self) -> u32 {
        self.time() as u32 * self.multiplier().as_u8() as u32
    }
}

#[cfg(test)]
mod test {
    use super::*;

    bitfield! {
        pub struct TestMeasurement([u8]);
        impl Debug;

        pub u32, from into Measurement, v, _ : eb!(2), sb!(0);
    }

    bitfield! {
        pub struct TestGasResistance([u8]);
        impl Debug;

        pub u16, from into GasResistance, v, _ : eb!(1), sb!(0);
    }

    bitfield! {
        pub struct TestH1H2([u8]);
        impl Debug;

        pub u16, from into ParameterH1, par_h1, _ : eb!(2), sb!(1);
        pub u16, from into ParameterH2, par_h2, _ : eb!(1), sb!(0);
    }

    #[test]
    fn test_1() {
        crate::test::init();

        let mut buf = [0u8; 15];
        buf[2] = 0x80;
        log::info!("Bytes: {:x?}", buf);
        let data = RawData(buf);
        log::info!("Bytes: {:?}", data);
    }

    #[test]
    fn test_h1h2_0() {
        crate::test::init();

        let t1 = TestH1H2([0u8, 0, 0]);
        assert_eq!(0, t1.par_h1().0);
        assert_eq!(0, t1.par_h2().0);
    }

    #[test]
    fn test_h1h2_1() {
        crate::test::init();

        assert_h1_h2([1, 2, 3], 50, 16);
    }

    #[test]
    fn test_h1h2_2() {
        crate::test::init();

        assert_h1_h2([0xFF, 0xFF, 0xFF], 4095, 4095);
    }

    #[test]
    fn test_h1h2_3() {
        crate::test::init();

        assert_h1_h2([0x00, 0xFF, 0xFF], 4095, 15);
    }

    #[test]
    fn test_h1h2_4() {
        crate::test::init();

        assert_h1_h2([0xFF, 0x00, 0xFF], 4080, 4080);
    }

    #[test]
    fn test_h1h2_5() {
        crate::test::init();

        assert_h1_h2([0xFF, 0xFF, 0x00], 15, 4095);
    }

    fn assert_h1_h2(input: [u8; 3], h1: u16, h2: u16) {
        let t1 = TestH1H2(input);
        assert_eq!(h1, t1.par_h1().0);
        assert_eq!(h2, t1.par_h2().0);
    }

    #[test]
    fn test_gasres_1() {
        crate::test::init();

        assert_gasres([0x1, 0x2], 4);
    }

    #[test]
    fn test_gasres_2() {
        crate::test::init();

        assert_gasres([0x2, 0x1], 8);
    }

    fn assert_gasres(input: [u8; 2], v: u16) {
        let t1 = TestGasResistance(input);
        assert_eq!(v, t1.v().0);
    }

    #[test]
    fn test_gas_wait_time_0() {
        assert_eq!(GasWaitTime::from(0).0, 0);
    }

    #[test]
    fn test_gas_wait_time_1() {
        assert_eq!(GasWaitTime::from(1).0, 1);
    }

    #[test]
    fn test_gas_wait_time_300() {
        assert_eq!(GasWaitTime::from(300).0, 146);
    }

    #[test]
    fn test_gas_wait_time_100() {
        assert_eq!(GasWaitTime::from(100).0, 0x59);
    }

    #[test]
    fn test_data() {
        let mut data: &mut [u8] =
            &mut [128, 0, 97, 94, 0, 124, 195, 160, 92, 170, 128, 0, 0, 0, 32];

        let data = RawData(&mut data);

        assert_eq!(data.temperature().0, 511034);
        assert_eq!(data.pressure().0, 398816);
        assert_eq!(data.humidity().0, 23722);
        assert_eq!(data.gas_resistance().0, 0);
        assert_eq!(data.gas_range(), 0);
    }

    #[test]
    fn test_calib_data() {
        let mut data: &mut [u8] = &mut [
            64, 64, 103, 3, 254, 147, 138, 148, 215, 88, 255, 131, 28, 157, 255, 27, 30, 0, 0, 135,
            252, 12, 243, 30, 1, 62, 52, 53, 0, 45, 20, 120, 156, 237, 102, 210, 212, 235, 18, 43,
            0, 22, 46, 243,
        ];

        let calib = CalibrationInformation(&mut data);

        assert_eq!(calib.par_t1(), 26349);
        assert_eq!(calib.par_t2(), 26432);
        assert_eq!(calib.par_t3(), 3);
        assert_eq!(calib.par_gh1(), -21);
        assert_eq!(calib.par_gh2(), -11054);
        assert_eq!(calib.par_gh3(), 18);
        assert_eq!(calib.par_h1().0, 852);
        assert_eq!(calib.par_h2().0, 995);
        assert_eq!(calib.par_h3(), 0);
        assert_eq!(calib.par_h4(), 45);
        assert_eq!(calib.par_h5(), 20);
        assert_eq!(calib.par_h6(), 120);
        assert_eq!(calib.par_h7(), -100);
        assert_eq!(calib.par_p1(), 35475);
        assert_eq!(calib.par_p2(), -10348);
        assert_eq!(calib.par_p3(), 88);
        assert_eq!(calib.par_p4(), 7299);
        assert_eq!(calib.par_p5(), -99);
        assert_eq!(calib.par_p6(), 30);
        assert_eq!(calib.par_p7(), 27);
        assert_eq!(calib.par_p8(), -889);
        assert_eq!(calib.par_p9(), -3316);
        assert_eq!(calib.par_p10(), 30);
    }

    #[test]
    fn test_run_gas() {
        let mut cg = ControlGas(0);
        cg.set_run_gas(true);
        assert_eq!(cg.0, 0x10);
    }
}
