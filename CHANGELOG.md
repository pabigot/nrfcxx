# Changelog

Notable API and feature changes for the nrfcxx framework.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- sd::Beacon implemented missing mutator for transmit power
- board/xenon reworked scope pins to avoid use of pin connected to LED
- board/xenon added missing battery level lookup
- board/papyr added supported target
- core ability to determine whether soft-device is active
- periph::TEMP added ability to measure die temperature
- periph::TWI added general call reset
- periph::TWI added ability to check whether a device is present
- sensor generalized support for information-carrying invalid values
- sensor::SHT31 added sensor support
- misc/regulator added controlled voltage sources
- sensor::adc::voltage_divider support a controlled voltage support
- sensor::adc::calibrator add state machine to maintain SAADC
  calibration as temperature changes
- sensor::adc::light_sensor added sensor for phototransistor relative
  intensity
- meson options build-examples and build-apps
- periph API to control Peripheral Power Control register for
  workarounds like SAADC PAN-212
- board/pca10059 added supported target

### Changed
- sensor::SHT21 switch observation units to cCel from cK
- periph::ADC changed default options and added API to better manage
  resolution and oversampling configuration to avoid SAADC PAN-212
- examples/sensor/ccs811-fw update to default-install firmware 2.0.1
- periph::ADC correctly scale single-ended (unsigned) and differential
  (signed) SAADC results so values near ground that measure as negative
  due to tolerance failures are not provided as insanely high voltages.

### Deprecated

### Removed
- misc/lipo_monitor: replace internal ADC calibration with requirement
  to provide ADC calibration externally

### Fixed
- periph::GPIOTE configuration fixed to correctly handle P1 pins
- passim assumptions that pin indexes were always on P0
- periph::GPIOTE sense listener protected against invalid enable/disable
  operations
- sensor::adc::lpsm_wrapper provide better control of internal and
  external calibration support

### Security

## [0.1.0] - 2019-03-13

### Added
- Initial public release

[0.1.0]: https://github.com/pabigot/nrfcxx/releases/tag/v0.1.0

<!---
# Local Variables:
# mode:markdown
# End:
-->
