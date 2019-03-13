/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Abstraction of Sensirion SDP8xx differential pressure sensor.
 * @file */

#ifndef NRFCXX_SENSOR_SDP8xx_HPP
#define NRFCXX_SENSOR_SDP8xx_HPP
#pragma once

#include <nrfcxx/clock.hpp>
#include <nrfcxx/lpm.hpp>
#include <nrfcxx/periph.hpp>

namespace nrfcxx {
namespace sensor {

/** Abstraction around SDP8xx differential pressure sensor.
 *
 * Interface to the
 * [SDP8xx](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf)
 * digital differential pressure sensor.
 *
 * The device supports either polled reading at a maximum rate of
 * about 200 Hz, or continuous measurement at up to 2 kHz.  The
 * interface currently only supports polled reading, using an @link
 * sdp8xx::lpsm_start LPM state machine@endlink.
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_OBSERVATION
 */
class sdp8xx : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;

public:
  /** The maximum time required by the sensor to stabilize
   * after reset(), in milliseconds. */
  constexpr static unsigned int RESET_DELAY_ms = 2;

  /** The maximum time required for a one-shot measurement, in
   * milliseconds. */
  constexpr static unsigned int SAMPLE_DELAY_ms = 50;

  /** Bit set in configure() parameter/result to disable
   * configuration reset on each measurement. */
  constexpr static uint8_t CONFIG_OTPRn = 0x02;

  /** Bit set in configure() parameter/result to enable the
   * on-chip heater */
  constexpr static uint8_t CONFIG_HEATER = 0x04;

  /** Bit set in configure() result indicating Vdd is below
   * 2.25V */
  constexpr static uint8_t CONFIG_EOB = 0x40;

  /** Value for observations that are known to be invalid.
   *
   * This fits in a 16-bit signed value and is outside the valid range
   * for both temperature_cCel() and diffpres_cPa(). */
  constexpr static int INVALID_OBSERVATION = -30000;

  /** Storage for cached results. */
  struct observations_type
  {
    /** A recently retrieved valid diffpres_cPa() or
     * #INVALID_OBSERVATION */
    int16_t diffpres_cPa{INVALID_OBSERVATION};

    /** A recently retrieved valid temperature_cCel() or
     * #INVALID_OBSERVATION */
    int16_t temperature_cCel{INVALID_OBSERVATION};
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

    /** @cond DOXYGEN_EXCLUDE */
    iface_config_type (const iface_config_type&) = delete;
    iface_config_type& operator= (const iface_config_type&) = delete;
    iface_config_type (iface_config_type&&) = delete;
    iface_config_type& operator= (iface_config_type&&) = delete;
    /** @endcond */
  };

  /** Access the interface configuration for the sensor. */
  const iface_config_type& iface_config () const
  {
    return iface_config_;
  }

  /** Construct a sensor interface around a TWI device and events. */
  sdp8xx (notifier_type notify,
          iface_config_type& ifc);

  /* No copying or moving. */
  sdp8xx (const sdp8xx&) = delete;
  sdp8xx& operator= (const sdp8xx&) = delete;
  sdp8xx (sdp8xx&& ) = delete;
  sdp8xx& operator= (sdp8xx&) = delete;

  /** Send a soft-reset command to the SDP8xx.
   *
   * @warning The SDP8xx is reset using the I2C bus standard broadcast
   * hardware reset-and-write command.  If other devices are on the
   * bus they may be affected.
   *
   * @return a non-negative value on success, or an @link
   * periph:TWI::error_encoded encoded error@endlink. */
  int reset ();

  /** Product number matcher for 500 Pa range manifold-connection
   * device. */
  static constexpr uint32_t SDP800_500Pa_PN = 0x03020100;

  /** Product number matcher for 500 Pa range tube-connection
   * device. */
  static constexpr uint32_t SDP810_500Pa_PN = 0x03020A00;

  /** Product number matcher for 125 Pa range manifold-connection
   * device. */
  static constexpr uint32_t SDP800_125Pa_PN = 0x03020200;

  /** Product number matcher for 125 Pa range tube-connection
   * device. */
  static constexpr uint32_t SDP810_125Pa_PN = 0x03020B00;

  /** Read the device information.
   *
   * @param[out] product a 32-bit product identifier.  The upper 24 bits
   * match one of the product numbers (e.g. @ref SDP810_125Pa_PN)
   * while the low 8 bits are a revision number.
   *
   * @param[out] serial a 64-bit unique serial number for the
   * device. */
  int read_device_info (uint32_t& product,
                        uint64_t& serial);

  /** Retrieve the most recent observations calculated through
   * the LPM infrastructure. */
  const observations_type& observations () const
  {
    return observations_;
  }

private:
  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

  iface_config_type& iface_config_;
  observations_type observations_;
  uint16_t scaleFactor_ = 0;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_SDP8xx_HPP */
