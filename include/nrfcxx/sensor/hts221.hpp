/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Abstraction around the HTS221 temperature/humidity sensor.
 *
 * @file */

#ifndef NRFCXX_SENSOR_HTS221_HPP
#define NRFCXX_SENSOR_HTS221_HPP
#pragma once

#include <climits>

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/lpm.hpp>
#include <nrfcxx/sensor/utils.hpp>

namespace nrfcxx {
namespace sensor {

/** Interface to the [ST
 * HTS221](https://www.st.com/en/mems-and-sensors/hts221.html)
 * capacitive digital sensor for relative humidity and temperature.
 *
 * @note The application infrastructure support described in @ref
 * GPIOTE::irq_handler must be provided when using hts221, and the
 * following put in the main routine after the hts221 instance has
 * been constructed:
 *
 *     NVIC_EnableIRQ(GPIOTE_IRQn);
 *     periph::GPIOTE::enable_sense();
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_OBSERVATION
 */
class hts221 : public lpm::lpsm_capable
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

  /** Structure for calibration information.
   *
   * If #cal_cCel contains two zero values the calibration data is not
   * valid.
   *
   * @note The values here are unit-converted and pre-calculated from
   * the raw calibration data read from the sensor. */
  struct calibration_type
  {
    /** Temperatures used for two-point calibration. */
    int16_t cal_cCel[2];

    /** Relative humidity measures used for two-point calibration. */
    uint16_t cal_pptt[2];

    /** Pre-calculated two-point temperature difference. */
    int16_t td_cCel;

    /** Pre-calculated two-point relative humidity difference. */
    int16_t hd_pptt;

    /** Raw measurement corresponding to #cal_cCel[0]. */
    int16_t t0_out;

    /** Raw measurement corresponding to #cal_pptt[0]. */
    int16_t h0_out;

    /** Pre-calculated two-point raw temperature measurement difference. */
    int16_t td_out;

    /** Pre-calculated two-point raw relative humidity measurement
     * difference. */
    int16_t hd_out;

    /** Convert a measured raw temperature value to centi-Celsius. */
    int16_t conv_cCel (int16_t t_out) const
    {
      return cal_cCel[0] + td_cCel * (t_out - t0_out) / td_out;
    }

    /** Convert a measured raw relative humidity value to parts-per-ten-thousand. */
    uint16_t conv_pptt (int16_t h_out) const
    {
      return cal_pptt[0] + hd_pptt * (h_out - h0_out) / hd_out;
    }
  };

  /** Access the calibration constants.
   *
   * This is useful for diagnostics. */
  const calibration_type& calibration () const
  {
    return calibration_;
  }

  /** Bit fields in the HTS221 `STATUS_REG` register. */
  enum STATUS_REG_e : uint8_t
  {
    /** Bit set to indicate that new temperature data is available. */
    SR_T_DA = 0x01,

    /** Bit set to indicate that new humidity data is available. */
    SR_H_DA = 0x02,

    /** Value when all expected data is available. */
    SR_READY = (SR_T_DA | SR_H_DA),
  };

  /** Return the value of the `STATUS` register. */
  int status () const;

  /** Value used to indicate that observations are not valid. */
  static constexpr uint16_t INVALID_OBSERVATION = sensor::INVALID_stdenv;

  /** Structure used to return sampled values. */
  struct observations_type
  {
    /** A measured temperature in centi-Celsius.
     *
     * If the value is #INVALID_OBSERVATION the temperature is not
     * available. */
    int16_t temperature_cCel{INVALID_OBSERVATION};

    /** A measured relative humidity in parts-per-ten-thousand (c%).
     *
     * If the value is #INVALID_OBSERVATION the relative humidity is
     * not available. */
    uint16_t humidity_pptt{INVALID_OBSERVATION};
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

    /** Observations generated at 7 Hz. */
    ODR_7_Hz = 0x02,

    /** Observations generated at 12.5 Hz. */
    ODR_12p5_Hz = 0x03,
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
   * @param ifc reference to an externally owned struct providing the
   * resources required to communicate with the device.
   *
   * @param notify the mechanism by which the instance notifies the
   * application that lpsm_process() should be invoked. */
  hts221 (notifier_type notify,
          iface_config_type& ifc);

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
  calibration_type calibration_{};
  observations_type observations_{};
  periph::GPIOTE::sense_listener drdy_listener_;
  gpio::pin_reference drdy_;
  uint8_t odr_ = ODR_DEFAULT;
  uint16_t lost_drdy_ = 0;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_HTS221_HPP */
