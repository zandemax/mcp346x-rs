# MCP346x Sigma-Delta ADC 

[![crates.io](https://img.shields.io/crates/v/mcp346x)](https://crates.io/crates/mcp346x)
[![docs.rs](https://img.shields.io/docsrs/mcp346x)](https://docs.rs/mcp346x)

This crate contains a platform-agnostic driver for the MCP346[1/2/4] Sigma-Delta ADC, using the [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

Currently, you can:

- Initialize the device
- Configure Input MUX and IRQ Pullup
- Set the conversion or power down mode
- Obtain measurements

### [Documentation](https://docs.rs/mcp346x)


## Usage

First of all, in order to get correct readings, the ADC needs to be calibrated according to the following procedure:

1. Set the calibration offsets and multipliers to their default values.
2. Remove any load from the ADC.
3. Calculate the offsets as the average of the ADC readings with no load applied.
4. Set the calculated calibration offsets, leaving the multipliers to their default value.
5. Apply a known load to the ADC.
6. Calculate the multipliers by dividing the known load by the average of the ADC readings with the known load applied.

The followings are two minimal examples to get readings from the ADC, both in a single-phase ADC configuration and a poly-phase multi ADCs configuration.

### Single Measurement

```rust ignore
use mcp346x::*;

let spi_pins = (sck, miso, mosi);
let spi = Spi::new(p.SPI0, spi_pins, Frequency::K500, MODE_0);

let address = 0b01; // This is the default address for most chips
let mut adc = MCP346x::new(spi, address).into_continuous_mode().unwrap();

adc.set_clock_source(mcp346x::ClockSource::Internal).unwrap();
adc.set_irq_internal_pullup(true).unwrap();

let voltage = adc.measure().unwrap();
```

## Status

- [x] Initialization
- [x] Powerdown/wakeup/conversion
- [x] Take Measurements
- [x] Input MUX configuration
- [ ] SCAN mode
- [ ] Expose other Configurations
- [ ] Communication CRC checksums

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
