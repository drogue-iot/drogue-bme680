# Drogue IoT BME680 Sensor driver

[![CI](https://github.com/drogue-iot/drogue-bme680/workflows/CI/badge.svg)](https://github.com/drogue-iot/drogue-bme680/actions?query=workflow%3A%22CI%22)
[![crates.io](https://img.shields.io/crates/v/drogue-bme680.svg)](https://crates.io/crates/drogue-bme680)
[![docs.rs](https://docs.rs/drogue-bme680/badge.svg)](https://docs.rs/drogue-bme680)
[![Matrix](https://img.shields.io/matrix/drogue-iot:matrix.org)](https://matrix.to/#/#drogue-iot:matrix.org)

The BME680 sensor is a "low power gas, pressure, temperature & humidity sensor" from Bosch Sensortec.

This crate helps to interface with the sensor. Offering general purpose access to the sensor, as well as a more
opinionated controller logic, simplifying the process of taking measurements.

## More information

- [BME680 product page](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors-bme680/)
- [C version of the driver](https://github.com/BoschSensortec/BME680_driver)

## Simple example

```rust
fn main() -> ! {
  let i2c = mock::blocking_i2c();

  let bme680 = Bme680Sensor::from(i2c, Address::Secondary).unwrap();
  let delay = mock::MockDelay;

  let mut controller = Bme680Controller::new(
    bme680,
    delay,
    Configuration::standard(),
    StaticProvider(25),,  // fixed 25 degrees Celsius as ambient temperature
  ).unwrap();

  loop {
    let result = controller.measure_default().unwrap();
    log::info!("Measured: {:?}", result);
  }
}
```

See: [examples/](examples/)

## BSEC

This crate can work together with the [Bosch Sensortec Environmental Cluster (BSEC)](https://www.bosch-sensortec.com/software-tools/software/bsec/) library.

However, as the BSEC library is not open source, this crate does not include any dependencies, headers or other related
artifacts of it. The BSEC interface is part of [drogue-bsec](https://github.com/drogue-iot/drogue-bsec).

## Run unit tests

You can run the unit tests on simple host machine using:

    cargo test --features env_logging test

## Run examples

Run with (STM32F411):

    cargo embed --release --features stm32f4xx --target thumbv7em-none-eabihf --example simple

Run with (STM32F723):

    cargo embed --release --features stm32f7xx --target thumbv7em-none-eabihf --example simple

## Dump data

You can enable logging of raw data, by enabling the feature `dump`, using `--features dump`.
Note that this requires a logging implementation, such as `rtt-logger`. Note that this may have a
huge negative impact on performance.
