/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/** Interface to vendor BME280 code.
 *
 * @file */

#ifndef NRFCXX_SENSOR_BME280_HPP
#define NRFCXX_SENSOR_BME280_HPP
#pragma once

#include <climits>

#include "bme280.h"

#include <nrfcxx/lpm.hpp>
#include <nrfcxx/periph.hpp>

namespace nrfcxx {
namespace sensor {

/** Abstraction around BME280 temperature/humidity/pressure sensor.
 *
 * Interface to the [Bosch Sensortec
 * BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
 * sensor, wrapping the [vendor-supplied
 * library](https://github.com/BoschSensortec/BME280_driver).
 *
 * This uses forced (one-shot) sampling only, hard-coded with the
 * parameters recommended for weather monitoring.
 *
 * @note Due to limitations in the vendor-provided API only one
 * SPI-based and one TWI-based instance may be present in a single
 * application.
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_OBSERVATION
 */
class bme280 : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;
public:
  /** Lower bound for wait after sample() before get_sensor_data().
   *
   * This value is designed for the weather monitoring configuration
   * hard-coded into the implementation.  Nominal maximum 9.3 ms. */
  static constexpr unsigned int SAMPLE_DELAY_utt = clock::uptime::from_ms(10);

  /** Value used for invalid temperature and humidity observations. */
  static constexpr uint16_t INVALID_TEMPHUMID = 30000;

  /** Value used for invalid pressure observations.
   *
   * This is sized to fit in a 24-bit unsigned integer, with a maximum
   * representable value of 1258.2911 hPa. */
  static constexpr unsigned int INVALID_PRESSURE = 0x00C00000;

  /** Structure used to return sampled values. */
  struct observations_type {
    /** A recently retrieved valid temperature or
     * #INVALID_TEMPHUMID. */
    int16_t temperature_cCel{INVALID_TEMPHUMID};

    /** A recently retrieved valid relative humidity or
     * #INVALID_TEMPHUMID. */
    uint16_t humidity_pptt{INVALID_TEMPHUMID};

    /** A recently retrieved valid pressure or #INVALID_PRESSURE. */
    unsigned int pressure_cPa{INVALID_PRESSURE};
  };

  /** Construct an instance that controls through the SPI interface.
   *
   * @param notify the notifier to inform the application of LPM
   * processing needs.
   *
   * @param spi reference to the SPI peripheral to use.
   *
   * @param csn_psel GPIO pin selector (index) identifying the GPIo
   * connected to the BME280 CSn signal.*/
  bme280 (notifier_type notify,
          nrfcxx::periph::SPI& spi,
          int csn_psel);

  /** Construct an instance that controls through the I2C interface.
   *
   * @param notify the notifier to inform the application of LPM
   * processing needs.
   *
   * @param twi reference to the TWI peripheral to use.
   *
   * @param addr_sec `false` to use the primary I2C address, `true` to
   * use the secondary I2C address. */
  bme280 (notifier_type notify,
          nrfcxx::periph::TWI& twi,
          bool addr_sec = false);

  /** Initialize the BME280.
   *
   * Issue this only once for each instance.  This confirms
   * communication with the device and configures the sample options.
   *
   * @note The SPI/TWI peripheral must be enabled before this method
   * is invoked.
   *
   * @warning If the BME280 is non-responsive this may delay
   * internally for up to 15 ms waiting to successfully connect to the
   * device.  This is due to busy-wait calls inside the
   * vendor-supplied library.
   *
   * @return zero on success, or a non-zero error code. */
  int initialize ();

  /** Request a new sample from the device.
   *
   * @note Wait for #SAMPLE_DELAY_utt ticks before attempting to
   * invoke fetch().
   *
   * @note The SPI/TWI peripheral must be enabled before this method
   * is invoked.
   *
   * @return zero on success, or a non-zero error code. */
  int sample ()
  {
    return ::bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
  }

  /** Retrieve the results from the most recent observation.
   *
   * @note The SPI/TWI peripheral must be enabled before this method
   * is invoked.
   *
   * @return zero on success, or a non-zero error code. */
  int fetch (observations_type& obs);

  /** Retrieve the most recent observations calculated through
   * the LPM infrastructure. */
  const observations_type& observations () const
  {
    return observations_;
  }

private:
  /* Initialization shared between SPI and TWI interfaces. */
  void init_dev_ (bool spi, bool addr_sec = false);

  /* Pointer to the SPI peripheral, if a SPI instance exists. */
  static periph::SPI* spip_;

  /* Pointer to the TWI peripheral, if an I2C interface exists. */
  static periph::TWI* twip_;

  /* Pin selector for SPI CSn signal. */
  static uint8_t csn_psel_;

  /* Driver functions are static so they can peek at #spip_, #twip_,
   * and #csn_psel_ to do what they need to do. */
  static int8_t spi_read (uint8_t dev_id,
                          uint8_t reg_addr,
                          uint8_t* data,
                          uint16_t len);

  static int8_t spi_write (uint8_t dev_id,
                           uint8_t reg_addr,
                           uint8_t* data,
                           uint16_t len);

  static int8_t twi_read (uint8_t dev_id,
                          uint8_t reg_addr,
                          uint8_t* data,
                          uint16_t len);

  static int8_t twi_write (uint8_t dev_id,
                           uint8_t reg_addr,
                           uint8_t* data,
                           uint16_t len);

  /* Instance-specific device configuration. */
  struct bme280_dev dev;

  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

  /* Last observations captured by LPM. */
  observations_type observations_;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_BME280_HPP */
