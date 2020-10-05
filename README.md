# Drogue IoT BME680 Sensor driver

[![CI](https://github.com/drogue-iot/drogue-bme680/workflows/CI/badge.svg)](https://github.com/drogue-iot/drogue-bme680/actions?query=workflow%3A%22CI%22)
[![crates.io](https://img.shields.io/crates/v/drogue-bme680.svg)](https://crates.io/crates/drogue-bme680)
[![docs.rs](https://docs.rs/drogue-bme680/badge.svg)](https://docs.rs/drogue-bme680)
[![Matrix](https://img.shields.io/matrix/drogue-iot:matrix.org)](https://matrix.to/#/#drogue-iot:matrix.org)

~~~rust
fn main() -> ! {
  let i2c = create_blocking_i2c();
  let mut timer = create_timer();
   
  let bme680 = Bme680Sensor::from(i2c, Address::Secondary).unwrap();
  
  let mut controller = Bme680Controller::new(
    bme680,
    &mut timer,
    Configuration::standard(),
    || 25,  // fixed 25 degrees Celsius as ambient temperature
  ).unwrap();
       
  loop {
    let result = controller.measure_default().unwrap();
    log::info!("Measured: {:?}", result);
  }
}
~~~

Also, see: [examples/simple.rs](examples/simple.rs)

## Run unit tests

You can run the unit tests on simple host machine using:

    cargo test --feature logging

## Run example

Run with:

    cargo embed --release --feature stm32f7xx --target thumbv7em-none-eabihf --example simple

## Dump data

You can enable logging of raw data, by enabling the feature `dump`, using `--features dump`.
Note that this requires a logging implementation, such as `rtt-logger`. Also note that this may have a
huge negative impact on performance.

