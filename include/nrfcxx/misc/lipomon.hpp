/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Support for monitoring LiPo batteries.
 *
 * @file */

#ifndef NRFCXX_MISC_LIPOMON_HPP
#define NRFCXX_MISC_LIPOMON_HPP
#pragma once

#include <nrfcxx/periph.hpp>
#include <nrfcxx/lpm.hpp>
#include <nrfcxx/sensor/adc.hpp>

namespace nrfcxx {
namespace misc {

/** A @link lpm::lpsm_capable state machine@endlink to monitor a LiPo battery and charger.
 *
 * This class does several things:
 *
 * * Monitors a VCHG_DETECT signal that indicates whether a charging
 *   voltage is available;
 * * Monitors a CHGn_STATE signal that is low when charging is active;
 * * Maintains the current @link power_source power source status@endlink;
 * * Supports @link lpsm_sample on-request sampling@endlink of the
 *   LiPo output voltage.
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags:
 * * lpm::state_machine::PF_STARTED when BAT_MON_EN has been asserted.
 *   On receipt calibrate() should be invoked;
 * * lpm::state_machine::PF_OBSERVATION when a new battery voltage
 *   measurement is available from batt_mV();
 * * #PF_LIPO when the system is discovered to be running on battery;
 * * #PF_MAINS when the system is discovered to be running on external
 *    power;
 * * #PF_CHARGING when the LiPo battery begins charging from external
 *    power;
 * * #PF_CHARGED when the LiPo battery completes charging from
 *    external power;
 * * #PF_CALIBRATED when ADC calibration completes, allowing
 *   lpm::lpsm_capable::lpsm_sample() to be successfully invoked.
 *
 * calibrate() should be invoked by the application on startup and
 * whenever the ambient temperature changes by more than 10 Cel from
 * the last calibration. */
class lipo_monitor : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;

public:

  /** State of power source as inferred from VCHG_DETECT and
   * BAT_CHG_STAT signals. */
  enum PowerSource_e : uint8_t
  {
    /** No information available. */
    PS_Unknown = 0,
    /** V_BUS is zero.  Values greater than this have V_BUS non-zero. */
    PS_OnBattery = 1,
    /** V_BUS is non-zero and TCK106 charging active. */
    PS_Charging = 2,
    /** V_BUS is non-zero and TCK106 charging not active. */
    PS_Charged = 3,
  };

private:
  using flags_type = uint8_t;

  /** Defined flags for #flags_bi_. */
  enum flags_enum : flags_type
  {
    /** Position in #flags_bi_ for power source captured due to GPIO
     * updates. */
    FL_PWRSRCCAP_Pos = 0U,
    FL_PWRSRCCAP_Msk = 0x03 << FL_PWRSRCCAP_Pos,

    /** Position in #flags_bi_ for power source last processed by the
     * LPSM machine. */
    FL_PWRSRCPRC_Pos = 2U,
    FL_PWRSRCPRC_Msk = 0x03 << FL_PWRSRCPRC_Pos,

    /** Bit set in #flags_bi_ to indicate that a calibrate() operation has
     * completed. */
    FL_CALIBRATED = 0x10,

    /** Bit set in #flags_bi_ to indicate that the ADC is collecting a
     * sample of the battery voltage. */
    FL_SAMPLING = 0x20,

    /** Bit set in #flags_bi_ to indicate that the power source changed
     * while #FL_SAMPLING was set. */
    FL_SAMPLE_CORRUPTED = 0x40,
  };

  /* Power source changes affecting #PF_LIPO, #PF_MAINS, #PF_CHARGING,
   * #PF_CHARGED are decoupled from the battery sampling state.
   *
   * State transitions:
   *
   * * ENTRY_START enables change detection on VCHG_DETECT and
   *   CHGn_STATE, captures their initial state, and sets PF_STARTED.
   *   It falls through to ENTRY_CALIBRATE.
   *
   * * ENTRY_CALIBRATE queues an ADC calibration and falls into
   *   EXIT_CALIBRATE.
   *
   * * EXIT_CALIBRATE loops until calibration resolved, then records
   *   state of calibration and falls into ENTRY_VBATT.
   *
   * * ENTRY_VBATT if not calibrated jumps to ENTRY_CALIBRATE.
   *   Otherwise it invokes sample_setup() and transitions to VBATT
   *   (after a setup delay if necessary).
   *
   * * VBATT queues an ADC collection and transitions to EXIT_VBATT.
   *
   * * EXIT_VBATT is blocked until the ADC completes, then it invokes
   *   sample_teardown() and processes the sample, emitting
   *   #PF_OBSERVATION if the collection was valid.  It then
   *   transitions to IDLE.
   */
  static constexpr auto MS_ENTRY_CALIBRATE = lpm::state_machine::MS_ENTRY_SAMPLE + 1;
  static constexpr auto MS_EXIT_CALIBRATE = lpm::state_machine::MS_EXIT_SAMPLE + 1;
  static constexpr auto MS_ENTRY_VBATT = lpm::state_machine::MS_ENTRY_SAMPLE;
  static constexpr auto MS_VBATT = lpm::state_machine::MS_SAMPLE;
  static constexpr auto MS_EXIT_VBATT = lpm::state_machine::MS_EXIT_SAMPLE;

public:
  static constexpr auto PF_STARTED = lpm::state_machine::PF_STARTED;

  /** Sensor-specific indication from
   * lpm::lpsm_capable::lpsm_process() that a powered USB cable has
   * been removed and the system is running from battery power. */
  static constexpr auto PF_LIPO = lpm::state_machine::PF_APP_BASE;

  /** Sensor-specific indication from
   * lpm::lpsm_capable::lpsm_process() that a powered USB cable has
   * been inserted and the system is running from mains power.
   *
   * If this flag is returned either #PF_CHARGING or #PF_CHARGED will
   * appear with it. */
  static constexpr auto PF_MAINS = lpm::state_machine::PF_APP_BASE << 1;

  /** Sensor-specific indication from
   * lpm::lpsm_capable::lpsm_process() that the LiPo battery is being
   * charged from a mains power source. */
  static constexpr auto PF_CHARGING = lpm::state_machine::PF_APP_BASE << 2;

  /** Sensor-specific indication from
   * lpm::lpsm_capable::lpsm_process() that the LiPo battery has
   * completed charging. */
  static constexpr auto PF_CHARGED = lpm::state_machine::PF_APP_BASE << 3;

  /** Sensor-specific indication from
   * lpm::lpsm_capable::lpsm_process() that the battery voltage sensor
   * has been calibrated. */
  static constexpr auto PF_CALIBRATED = lpm::state_machine::PF_APP_BASE << 5;

  /** Construct an instance.
   *
   * @param notify the setter used to indicate to the application that
   * lpsm_process() should be invoked.
   *
   * @param vchg_detect a GPIO reference that is high when a charging
   * voltage is available, and low when it is not.
   *
   * @param chgn_state a GPIO reference that is low when the battery
   * is being charged, and high when it is not being charged.  The
   * signal is considered valid only when a charging voltage is available.
   *
   * @param battmeas the ADC client used to measure the voltage of the
   * LiPo battery.  A reference to this instance will be retained.
   */
  lipo_monitor (notifier_type notify,
                const gpio::gpio_pin& vchg_detect,
                const gpio::gpio_pin& chgn_state,
                sensor::adc::voltage_divider& battmeas);

  /** Read the live USB detection signal. */
  bool vchg_detected_live ()
  {
    return vchg_detect_.read();
  }

  /** Read the live charging signal. */
  bool charging_live ()
  {
    return vchg_detect_.read() && !chgn_state_.read();
  }

  /** Initiate a calibration of the ADC used to measure voltage.
   *
   * @return zero on success, negative if the state machine is
   * busy. */
  int calibrate ();

  /** The most recently collected LiPo output voltage.
   *
   * @note A negative value indicates that the collection occurred
   * when the system was externally powered, a condition in which the
   * measured power may inaccurately represent the true state of the
   * LiPo. */
  int batt_mV () const
  {
    return batt_mV_;
  }

  /** Get the latest processed power source.
   *
   * @note The value returned here is not live, but the state as
   * observed in the last invocation of lpsm_process().
   *
   * @return An enumeration value from #PowerSource_e or a negative
   * error code. */
  int power_source () const;

protected:
  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

private:
  using mutex_type = periph::GPIOTE::sense_listener::mutex_type;
  using adc_mutex_type = periph::ADCClient::mutex_type;

  void callback_bi_ (const periph::GPIOTE::sense_status_type* sp);

  periph::GPIOTE::sense_listener listener_;

  gpio::gpio_pin vchg_detect_;
  gpio::gpio_pin chgn_state_;
  sensor::adc::voltage_divider& battmeas_;
  int16_t batt_mV_ = 0;

  // Protected by mutex_type
  flags_type volatile flags_bi_ = 0;

  // These three protected by adc_mutex_type, if necessary.
  bool volatile adc_pending_ = false;
  int8_t volatile adc_calibrating_ = 0;
  int8_t volatile adc_sampling_ = 0;
};

} // ns misc
} // namespace

#endif /* NRFCXX_MISC_LIPOMON_HPP */
