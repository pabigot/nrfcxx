/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Abstraction around the LPS22HB temperature/humidity sensor.
 *
 * @file */

#ifndef NRFCXX_SENSOR_LPS22HB_HPP
#define NRFCXX_SENSOR_LPS22HB_HPP
#pragma once

#include <climits>

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/lpm.hpp>

namespace nrfcxx {
namespace sensor {

/** Interface to the [ST
 * LPS22HB](https://www.st.com/en/mems-and-sensors/lps22hb.html)
 * piezzoresistave absolute pressure sensor.
 *
 * @note The application infrastructure support described in @ref
 * GPIOTE::irq_handler must be provided when using lps22hb, and the
 * following put in the main routine after the lps22hb instance has
 * been constructed:
 *
 *     NVIC_EnableIRQ(GPIOTE_IRQn);
 *     periph::GPIOTE::enable_sense();
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_OBSERVATION
 */
class lps22hb : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;
public:
  /** Information required to communicate with a sensor instance.
   *
   * Instances of this must be defined at the same scope as the sensor
   * instance, as the sensor references the external definition. */
  struct iface_config_type
  {
    /** Reference to TWI device used to communicate with sensor. */
    periph::TWI& twi;

    /** GPIO pin selector usable for interrupt-driven notification of
     * available measurements.
     *
     * At this time a valid GPIO must be provided: poll-based
     * observation is not supported. */
    int8_t drdy_psel;

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

  /** Bit fields in the LPS22HB `STATUS_REG` register. */
  enum STATUS_REG_e : uint8_t
  {
    /** Bit set to indicate that new pressure data is available. */
    SR_P_DA = 0x01,

    /** Bit set to indicate that new temperature data is available. */
    SR_T_DA = 0x02,

    /** Value when all expected data is available. */
    SR_READY = (SR_P_DA | SR_T_DA),
  };

  /** Return the value of the `STATUS` register. */
  int status () const;

  /** Value used for invalid temperature and humidity observations. */
  static constexpr uint16_t INVALID_TEMPHUMID = 30000;

  /** Value used for invalid pressure observations.
   *
   * This is sized to fit in a 24-bit unsigned integer, with a maximum
   * representable value of 1258.2911 hPa. */
  static constexpr unsigned int INVALID_PRESSURE = 0x00C00000;

  /** Structure used to return sampled values. */
  struct observations_type
  {
    /** A recently retrieved valid pressure or #INVALID_PRESSURE. */
    unsigned int pressure_cPa = INVALID_PRESSURE;

    /** A recently retrieved valid temperature or
     * #INVALID_TEMPHUMID. */
    int16_t temperature_cCel = INVALID_TEMPHUMID;
  };

  /** Access the most recent completed observation. */
  const observations_type& observations () const
  {
    return observations_;
  }

  /** Bit field values in `CTRL_REG1` for observation data rate.
   *
   * These may be configured using odr() when the LPM machine is
   * off. */
  enum ODR_e : uint8_t
  {
    /** On-demand observations.
     *
     * Invoke lpsm_start() to initiate an observation. */
    ODR_OneShot = 0x00,

    /** Observations generated at 1 Hz. */
    ODR_1_Hz = 0x01,

    /** Observations generated at 10 Hz. */
    ODR_10_Hz = 0x02,

    /** Observations generated at 25 Hz. */
    ODR_25_Hz = 0x03,

    /** Observations generated at 50 Hz. */
    ODR_50_Hz = 0x04,

    /** Observations generated at 75 Hz. */
    ODR_75_Hz = 0x05,
  };

  /** The default output data rate. */
  static constexpr auto ODR_DEFAULT = ODR_1_Hz;

  /** Retrieve the configured output data rate.
   *
   * @return a value from @ref ODR_e.
   *
   * @see ODR_DEFAULT */
  uint8_t odr () const
  {
    return odr_;
  }

  /** Set the output data rate.
   *
   * @note This function will error if invoked while the LPM
   * infrastructure is running.
   *
   * @return non-negative on success, or a negative error code. */
  int odr (uint8_t dr);

  /** Instantiate the device.
   *
   * @param notify the mechanism by which the instance notifies the
   * application that lpsm_process() should be invoked.
   *
   * @param ifc reference to an externally owned struct providing the
   * resources required to communicate with the device.
   *
   * @param addr the device address as controlled by the SA0 signal,
   * value in the range 0 through 1. */
  lps22hb (notifier_type notify,
           iface_config_type& ifc,
           unsigned int addr = 0);

  /** Programmatic test for whether `DRDY` is asserted. */
  bool drdy_asserted () const
  {
    return drdy_.read();
  }

  /** Post-extend to signal machine if start failed due to undetected
   * drdy. */
  int lpsm_sample () override;

  /** Count of the number of times a DRDY signal was detected to be
   * lost. */
  uint16_t lost_drdy () const
  {
    return lost_drdy_;
  }

private:
  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

  void drdy_callback_ (const periph::GPIOTE::sense_status_type*);

  iface_config_type& iface_config_;
  observations_type observations_{};
  periph::GPIOTE::sense_listener drdy_listener_;
  gpio::pin_reference drdy_;
  uint8_t odr_ = ODR_DEFAULT;
  uint16_t lost_drdy_ = 0;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_LPS22HB_HPP */
