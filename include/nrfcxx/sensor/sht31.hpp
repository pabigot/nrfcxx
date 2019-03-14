/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2019 Peter A. Bigot */

/** Abstraction of Sensirion SHT31 temperature/humidity sensor.
 *
 * @file */
#ifndef NRFCXX_SENSOR_SHT31_HPP
#define NRFCXX_SENSOR_SHT31_HPP
#pragma once

#include <nrfcxx/lpm.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/utils.hpp>

namespace nrfcxx {
namespace sensor {

/** Abstraction around SHT31 temperature/humidity sensor.
 *
 * Interface to the [Sensirion
 * SHT3x](https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-various-applications).
 *
 * This is a basic implementation designed for on-demand sampling with
 * highest repeatability without an alert infrastructure or support
 * for plausibility checking via the internal heater.
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_STARTED indicates that it is safe to
 *   invoke lpsm_sample().
 * * lpm::state_machine::PF_OBSERVATION indicates that observations()
 *   has been updated with new data.
 */
class sht31 : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;
public:
  /** Value for observations that are known to be invalid. */
  constexpr static int INVALID_OBSERVATION = sensor::INVALID_stdenv;

  /** Storage for cached results. */
  struct observations_type
  {
    /** A recently calculated valid temperature, or
     * #INVALID_OBSERVATION.
     *
     * Measurements are in hundredths of a degree Celsius. */
    int16_t temperature_cCel = INVALID_OBSERVATION;

    /** A recently calculated valid humidity, or
     * #INVALID_OBSERVATION.
     *
     * Measurements are in parts per ten thousand. */
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
     * @note This is assigned by the infrastructure and should not be
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

  /** Instantiate the device.
   *
   * @param notify the mechanism by which the instance notifies the
   * application that lpsm_process() should be invoked.
   *
   * @param ifc reference to an externally owned struct providing the
   * resources required to communicate with the device.
   *
   * @param secondary_addr `true` if the target device uses the
   * secondary I2C address. */
  sht31 (notifier_type setter,
         iface_config_type& ifc,
         bool secondary_addr = false);

  /* No copying or moving. */
  sht31 (const sht31&) = delete;
  sht31& operator= (const sht31&) = delete;
  sht31 (sht31&& ) = delete;
  sht31& operator= (sht31&) = delete;

  /** Send a soft-reset command to the SHT31.
   *
   * @note This command is not particularly useful: the fact-of reset
   * is not reflected in the status register (contrary to
   * documentation), and what it does is little more than what's done
   * before every measurement by default.
   *
   * @return a non-negative value on success, or a negative @link
   * periph:TWI::error_decoded encoded error@endlink.  A non-negative
   * return value provides the number of milliseconds required for the
   * reset to complete. */
  int reset ();

  /** Initiate a temperature measurement.
   *
   * This invalidates the current observations() cache and issues a
   * command to sample the temperature and humidity.
   *
   * @return A non-negative integer on success, or a negative @link
   * periph:TWI::error_decoded encoded error@endlink.  A non-negative
   * return value provides the number of milliseconds required for the
   * collection to complete. */
  int trigger ();

  /** Fetch the most recent observation from the device.
   *
   * @return zero on successful fetch, `-EBUSY` if the previous
   * trigger() has not completed, or another negative error code. */
  int fetch ();

  /** Fetch the value of the sensor status register.
   *
   * If the read is successful the status register will be cleared.
   *
   * @return a non-negative value corresponding to the 16-bit SHT31
   * status register, or a negative error code. */
  int status ();

  /** Access the most recent observations retrieved through fetch() or
   * the LPM infrastructure. */
  const observations_type& observations () const
  {
    return observations_;
  }

protected:
  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

  iface_config_type& iface_config_;
  observations_type observations_;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_SHT31_HPP */
