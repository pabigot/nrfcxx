/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Abstraction of Sensirion SHT21 temperature/humidity sensor.
 *
 * @file */
#ifndef NRFCXX_SENSOR_SHT21_HPP
#define NRFCXX_SENSOR_SHT21_HPP
#pragma once

#include <nrfcxx/clock.hpp>
#include <nrfcxx/lpm.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/utils.hpp>

namespace nrfcxx {
namespace sensor {

/** Abstraction around SHT21/HTU21D temperature/humidity sensor.
 *
 * Interface to the <a
 * href="http://www.sensirion.com/en/products/humidity-temperature/humidity-sensor-sht21/">Sensirion
 * SHT21 Digital Humidity Sensor</a> or the <a
 * href="http://www.meas-spec.com/product/humidity/HTU21D.aspx">Measurement
 * Specialties HTU21D</a> clone.
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_OBSERVATION
 */
class sht21 : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;
public:
  /** Storage for the Electronic Identification Code */
  using eic_type = uint8_t[8];

  /** The maximum time required by the sensor to stabilize after
   * reset(), in milliseconds. */
  constexpr static unsigned int RESET_DELAY_ms = 15;

  /** The maximum time required for a 14-bit measurement, in
   * milliseconds.
   *
   * This is for a SHT21; the HTU21D is about 30% faster. */
  constexpr static unsigned int SAMPLE14_DELAY_ms = 85;

  /** The maximum time required for a 12-bit measurement, in
   * milliseconds.
   *
   * This is for a SHT21; the HTU21D is about 25% faster. */
  constexpr static unsigned int SAMPLE12_DELAY_ms = 22;

  /** Bits for parameter to configure() specifying 12-bit humidity and
   * 14-bit temperature measurements. */
  constexpr static uint8_t CONFIG_RES_H12T14 = 0x00;

  /** Bits for parameter to configure() specifying 8-bit humidity and
   * 12-bit temperature measurements. */
  constexpr static uint8_t CONFIG_RES_H8T12 = 0x01;

  /** Bits for parameter to configure() specifying 10-bit humidity and
   * 13-bit temperature measurements. */
  constexpr static uint8_t CONFIG_RES_H10T13 = 0x80;

  /** Bits for parameter to configure() specifying for 11-bit humidity
   * and 11-bit temperature measurements. */
  constexpr static uint8_t CONFIG_RES_H11T11 = 0x81;

  /** Mask to isolate the bits of the SHT21 user register that
   * identify sampling resolution. */
  constexpr static uint8_t CONFIG_RES_Msk = 0x81;

  /** Bit set in configure() parameter/result to disable configuration
   * reset on each measurement. */
  constexpr static uint8_t CONFIG_OTPRn = 0x02;

  /** Bit set in configure() parameter/result to enable the on-chip
   * heater. */
  constexpr static uint8_t CONFIG_HEATER = 0x04;

  /** Bit set in configure() result indicating Vdd is below 2.25V. */
  constexpr static uint8_t CONFIG_EOB = 0x40;

  /** Value for observations that are known to be invalid.
   *
   * This fits in a 16-bit signed value and is outside the valid range
   * for both temperature_cK() and humidity_pptt(). */
  constexpr static int INVALID_OBSERVATION = sensor::INVALID_stdenv;

  /** Storage for cached results. */
  struct observations_type
  {
    /** A recently calculated valid temperature_cK() converted to
     * centiCelsius, or #INVALID_OBSERVATION */
    int16_t temperature_cCel = INVALID_OBSERVATION;

    /** A recently calculated valid humidity_pptt() or
     * #INVALID_OBSERVATION */
    uint16_t humidity_pptt = INVALID_OBSERVATION;
  };

  /** Information required to communicate with a sensor instance.
   *
   * Instances of this must be defined at the same scope as the sensor
   * instance, as the sensor references the external definition. */
  struct iface_config_type
  {
    /** Reference to TWI device used to communicate with sensor. */
    periph::TWI& twi;

    /** The I2C address used to communicate with the device.
     *
     * @note This is assigned by the infrastructure and need not be
     * populated in the instance passed to the sensor constructor. */
    uint8_t address;

    iface_config_type (const iface_config_type&) = delete;
    iface_config_type& operator= (const iface_config_type&) = delete;
    iface_config_type (iface_config_type&&) = delete;
    iface_config_type& operator= (iface_config_type&&) = delete;
  };

  /** Access the interface configuration for the sensor. */
  const iface_config_type& iface_config () const
  {
    return iface_config_;
  }

  /** Construct a sensor interface around a TWI device and events. */
  sht21 (notifier_type setter,
         iface_config_type& ifc);

  /* No copying or moving. */
  sht21 (const sht21&) = delete;
  sht21& operator= (const sht21&) = delete;
  sht21 (sht21&& ) = delete;
  sht21& operator= (sht21&) = delete;

  /** Send a soft-reset command to the SHT21.
   *
   * If successful this is equivalent to a power-on reset except that
   * the #CONFIG_HEATER bit remains unaffected.
   *
   * @return a non-negative value on success, or a negative @link
   * periph:TWI::error_decoded encoded error@endlink. */
  int reset ();

  /** Configure resolution and heater.
   *
   * @param config A nonnegative configuration byte comprising a
   * resolution setting (e.g. #CONFIG_RES_H12T14) and optionally
   * #CONFIG_HEATER, or a negative value to query the current
   * configuration without changing it.
   *
   * @return The non-negative content of the configuration register
   * prior to invoking this function, or a negative @link
   * periph:TWI::error_decoded encoded error@endlink.
   *
   * @note #CONFIG_OTPRn is unconditionally set in any non-negative @p
   * config value. */
  int configure (int config);

  /** Read the Electronic Identification Code from the device.
   *
   * @param eic Storage for the EIC, which is device-specific and can
   * also be used to distinguish Sensirion SHT21 from HTU21D
   * devices. */
  int read_eic (eic_type& eic);

  /** Return `true` iff the @p eic indicates a Sensirion SHT21
   * device */
  static bool eic_is_sht21 (const eic_type& eic)
  {
    return ((0x00 == eic[0]) && (0x80 == eic[1]));
  }

  /** Return `true` iff the @p eic indicates a Measurement
   * Specialties HTU21D device */
  static bool eic_is_htu21d (const eic_type& eic)
  {
    return (('H' == eic[0]) && ('T' == eic[1]));
  }

  /** Initiate a temperature measurement.
   *
   * The time required for completion of the measurement is device-
   * and resolution-specific, but may be as long as 85 ms.
   *
   * @return Zero on success, or a negative @link
   * periph:TWI::error_decoded encoded error@endlink. */
  int trigger_temperature ()
  {
    return trigger_(false);
  }

  /** Initiate a temperature measurement.
   *
   * The time required for completion of the measurement is device-
   * and resolution-specific, but may be as long as 29 ms.
   *
   * @return Zero on success, or anegative @link
   * periph:TWI::error_decoded encoded error@endlink. */
  int trigger_humidity ()
  {
    return trigger_(true);
  }

  /** The error code returned from temperature_cK() and
   * humidity_pptt() when the device is still processing a
   * measurement. */
  constexpr static int NOT_READY = periph::TWI::error_encoded(periph::TWI::ERR_ANACK);

  /** Retrieve the last measurement as a temperature measurement.
   *
   * @return the non-negative temperature in hundredths of a degree
   * Kelvin, or a negative error code such as #NOT_READY or an encoded
   * TWI error or TWI::ERR_INVALID if the last measurement was not
   * temperature. */
  int temperature_cK ()
  {
    return fetch_(false);
  }

  /** Retrieve the last measurement as a humidity measurement.
   *
   * @return the non-negative relative humidity in
   * parts-per-ten-thousand, or a negative error code such as
   * #NOT_READY or an encoded TWI error or TWI::ERR_INVALID if the
   * last measurement was not humidity. */
  int humidity_pptt ()
  {
    return fetch_(true);
  }

  /** Retrieve the most recent observations calculated through
   * the LPM infrastructure. */
  const observations_type& observations () const
  {
    return observations_;
  }

protected:
  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

  int trigger_ (bool humidity);

  /** Fetch the latest result as a raw value.
   *
   * NOT_READY indicates that the collection is still in progress.
   *
   * The caller is responsible for ensuring that @p humidity is
   * consistent with the previous invocation of trigger_(). */
  int fetch_ (bool humidity);

  iface_config_type& iface_config_;
  observations_type observations_;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_SHT21_HPP */
