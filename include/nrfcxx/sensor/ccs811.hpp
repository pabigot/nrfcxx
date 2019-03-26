/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/** Abstraction around the CCS811 indoor air quality sensor.
 *
 * @file */

#ifndef NRFCXX_SENSOR_CCS811_HPP
#define NRFCXX_SENSOR_CCS811_HPP
#pragma once

#include <climits>

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/lpm.hpp>

namespace nrfcxx {
namespace sensor {

/** Interface to the [ams CCS811](https://ams.com/ccs811) indoor air
 * quality sensor.
 *
 * @note This device assumes that it can stretch the I2C clock, and it
 * does so.  The 200 us timeout that suffices for most I2C sensors
 * used in nrfcxx is insufficient and will result in aborted
 * transactions if separate register operations are initiated too
 * close together.  The I2C specification does not limit the duration
 * of a stretched clock (SMBus specifies 35 ms), and the CCS811 data
 * sheet does not provide a maximum stretch.  500 us TWI timeout
 * appears to be adequate.
 *
 * @note The application infrastructure support described in @ref
 * GPIOTE::irq_handler must be provided when using ccs811, and the
 * following put in the main routine after the ccs811 instance has
 * been constructed:
 *
 *     NVIC_EnableIRQ(GPIOTE_IRQn);
 *     periph::GPIOTE::enable_sense();
 *
 * lpm::lpsm_capable::lpsm_process() for this sensor returns the
 * following flags in addition to the diagnostic flags:
 * * lpm::state_machine::PF_OBSERVATION
 * * lpm::state_machine::PF_RESET
 * * #PF_CCS811_ERROR
 * * #PF_REPORTING
 * * #PF_CONDITIONED
 * * #PF_RESTORED
 * * #PF_BASELINED
 */
class ccs811 : public lpm::lpsm_capable
{
  using super = lpm::lpsm_capable;

  // One-shot not available for this sensor.
  using super::lpsm_sample;

public:
  /** Value for maximum t_APP_START in uptime ticks. */
  static constexpr unsigned int APP_START_DELAY_utt = clock::uptime::from_ms(1U);

  /** Value for maximum t_WAKE in microseconds. */
  static constexpr unsigned int WAKE_DELAY_us = 50U;

  /** Value for maximum t_START after power-on in uptime ticks. */
  static constexpr unsigned int COLD_START_DELAY_utt = clock::uptime::from_ms(20U);

  /** Value for maximum t_START after reset in uptime ticks. */
  static constexpr unsigned int WARM_START_DELAY_utt = clock::uptime::from_ms(2U);

  /** Value for minimum t_DWAKE in microseconds. */
  static constexpr unsigned int DWAKE_DELAY_us = 20U;

  /** Value for minimum t_DRESET in microseconds. */
  static constexpr unsigned int DRESET_DELAY_us = 20U;

  /** Value for minimum t_RESET in microseconds. */
  static constexpr unsigned int RESET_DELAY_us = 15U;

  /** Duration of the conditioning period, in uptime ticks.
   *
   * This is the period after initial power-up or measurement start
   * during which the sensor is warming up.  The `BASELINE` register
   * should not be written until after this period elapses.
   *
   * See [AN370: Baseline Save and
   * Restore](https://ams.com/ccs811#tab/documents). */
  static constexpr unsigned int CONDITIONING_DELAY_utt = 20U * 60U * clock::uptime::Frequency_Hz;

  /** Duration between retention of the `BASELINE` register. */
  static constexpr unsigned int BASELINE_CAPTURE_INTERVAL_utt = 24U * 60U * 60U * clock::uptime::Frequency_Hz;

  /** Structure providing the system resources necessary to interact
   * with the CCS811.
   *
   * Instances of this must be defined at the same scope as the sensor
   * instance, as the sensor references the external definition. */
  struct iface_config_type
  {
    /** Reference to TWI device used to communicate with sensor. */
    periph::TWI& twi;

    /** GPIO pin selector for WAKEn signal. */
    gpio::generic_pin& waken;

    /** GPIO pin selector for RESETn signal. */
    gpio::generic_pin& resetn;

    /** GPIO pin selector for INTn signal. */
    int8_t intn_psel;

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

  /** Structure holding retained state of the sensor.
   *
   * An instance of this should be defined within a `.noinit.ccs811`
   * section, and ccs811::state_setup() invoked to maintain its
   * contents across resets. */
  struct retained_state_type
  {
    /** Type used to hold state flags. */
    using flags_type = uint8_t;

    /** The @link systemState::total_now aggregate uptime@endlink at
     * which the sensor was last reset.
     *
     * This is required to determine whether the sensor is still
     * within the @link CONDITIONING_DELAY_utt conditioning
     * period@endlink and to manage the frequency of preserving the
     * baseline information.
     *
     * The value is cleared when the device is reset. */
    uint64_t reset_utt;

    /** The @link systemState::total_now aggregate uptime@endlink at
     * which the `BASELINE` register was last stored in #baseline.
     *
     * The value is cleared when the device is switched to a different
     * measurement mode. */
    uint64_t baselined_utt;

    /** The most recently validated `BASELINE` register value.
     *
     * When the sensor completes its conditioning period this value is
     * written to the `BASELINE` register.  Periodically afterwards
     * the value here is updated.
     *
     * This is valid iff @ref FL_BASELINED is set in #flags. */
    uint16_t baseline;

    /** Defined values for #flags */
    enum flags_enum : flags_type
    {
      /** Flag set when the first valid (but unconditioned) observation
       * has been provided to the application. */
      FL_REPORTING = 0x01,

      /** Flag set on the first observation received more than @ref
       * CONDITIONING_DELAY_utt after #reset_utt.
       *
       * This is the point at which the LPM machine restores a baseline
       * if one is available, or stores the initial baseline if one is
       * not available.  If this flag is not set the readings may not be
       * valid. */
      FL_CONDITIONED = 0x02,

      /** Flag set when #baseline has been set to a valid value.
       *
       * This may be set when the live register is periodically stored
       * in the retained state, or when a restore_from_persisted() is
       * successfully invoked. */
      FL_BASELINED = 0x04,

      /** Position in #flags where the active drive mode is stored.
       *
       * The value stored at this position should be one of @ref DM_1_s,
       * @ref DM_10_s, or @ref DM_60_s.  A value of @ref DM_IDLE
       * indicates the device is not active. */
      FL_DRIVE_MODE_Pos = 6U,

      /** Mask to isolate active drive mode in #flags. */
      FL_DRIVE_MODE_Msk = (0x03 << FL_DRIVE_MODE_Pos),
    };

    /** Flags recording the stage of processing of the device.
     *
     * * Bit 0 is #FL_REPORTING.
     * * Bit 1 is #FL_CONDITIONED.
     * * Bit 2 is #FL_BASELINED.
     * * Bits 3..5 are reserved and should be zero.
     * * Bits 6..7 encode the @link active_drive_mode drive
     *   mode@endlink for #baseline
     *
     * @see FL_CONDITIONED
     * @see FL_BASELINED
     * @see FL_DRIVE_MODE_Pos
     */
    uint8_t flags;
  };

  /** State that should be persisted in non-volatile memory using the
   * utility::Persist infrastructure. */
  struct persisted_state_type
  {
    /** Recorded baseline values indexed by `MEAS_MODE` drive mode. */
    uint16_t baseline[4];
  };

  /** Update a persisted state with information from retained state.
   *
   * If retained_state_type::FL_BASELINED is set then the baseline for
   * the current mode is copied into the appropriate section of @p ps.
   *
   * @note This should be invoked by the application whenever
   * #PF_BASELINED is produced, or when save_baseline() is executed
   * under application control.  It is the application's
   * responsibility to persist the updated structure.
   *
   * @param ps where the persisted state should be stored.
   *
   * @return zero on successful update, or a negative error code. */
  int update_persisted (persisted_state_type& ps) const;

  /** Update the retained state with information from persisted state.
   *
   * If @p ps has a non-zero baseline for the drive mode configured in
   * retained_state_type::meas_mode and
   * retained_state_type::FL_BASELINED is not currently set then
   * retained_state_type::baseline is set from the persisted state and
   * retained_state_type::FL_BASELINED is set.
   *
   * This function should be invoked on receipt of
   * lpm::state_machine::PF_STARTED so that when #PF_CONDITIONED
   * occurs the LPM machine will restore the `BASELINE` register in
   * the CCS811.
   *
   * @param ps the persisted state that may contain a relevant baseline.
   *
   * @return zero on successful update, or a negative error code. */
  int restore_from_persisted (const persisted_state_type& ps);

  /** Access the retained state. */
  static const retained_state_type& retained_state ()
  {
    return retained_state_;
  }

  /** Function to maintain CCS811 state across resets.
   *
   * This must be invoked from the application's systemState @link
   * systemState::state_type::app_handler app_handler@endlink to
   * ensure @link Beacon::telemetry_state telemetry state@endlink is
   * properly managed. */
  static void state_setup (const systemState::state_type& ss,
                           bool is_reset,
                           bool retained);

  /** Instantiate the device.
   *
   * @param setter the mechanism by which the LPM machine notifies the
   * application that it needs to be serviced.
   *
   * @param ifc reference to an externally owned struct providing the
   * resources required to communicate with the device.
   *
   * @param addr_sec true indicates that the secondary I2C address
   * should be used instead of the primary I2C address. */
  ccs811 (notifier_type setter,
          iface_config_type& ifc,
          bool addr_sec = false);

  /** Directly access the TWI peripheral used to communicate with the
   * device. */
  periph::TWI& twi () const
  {
    return iface_config_.twi;
  }

  /** Class obtained from scoped_enable(). */
  class scoped_enabler
  {
    // ccs811 needs access to construct these things.
    friend class ccs811;

    /** Bit set in #flags_ if constructor must enable TWI. */
    static constexpr uint8_t FL_ENABLE_TWI = 0x01;

    /** Bit set in #flags_ if constructor must assert WAKEn. */
    static constexpr uint8_t FL_ASSERT_WAKEn = 0x02;

    scoped_enabler (periph::TWI& twi,
                    const gpio::active_low& waken) :
      twi{twi},
      waken{waken},
      flags((twi.enabled() ? 0U : FL_ENABLE_TWI)
            | (waken.asserted() ? 0U : FL_ASSERT_WAKEn))
    {
      if (FL_ENABLE_TWI & flags) {
        result_ = twi.enable();
        if (0 <= result_) {
          result_ = 1;
        }
      }
      // Only assert WAKEn if we don't know that twi enable failed.
      if ((0 <= result_)
          && (FL_ASSERT_WAKEn & flags)) {
        assert();
      }
    }

    periph::TWI& twi;
    const gpio::active_low& waken;
    int result_ = 0;
    const uint8_t flags;

  public:
    ~scoped_enabler ()
    {
      if (FL_ASSERT_WAKEn & flags) {
        deassert();
      }
      if (FL_ENABLE_TWI & flags) {
        twi.disable();
      }
    }

    /** Boolean value of the enabler is `true` iff the interface is
     * enabled. */
    operator bool () const
    {
      return 0 <= result_;
    }

    /** Return the result of invoking periph::TWI::enable() in the constructor.
     *
     * If this is negative the attempt to enable TWI failed, and most
     * likely subsequent operations will fail or hang. */
    int result () const
    {
      return result_;
    }

    /** Assert `WAKEn` and delay until I2C operations are allowed. */
    void assert ()
    {
      waken.assert();
      delay_us(WAKE_DELAY_us);
    }

    /** De-assert `WAKEn` and delay as required. */
    void deassert ()
    {
      waken.deassert();
      delay_us(DWAKE_DELAY_us);
    }
  };

  /** Programmatic test for whether `INTn` is asserted. */
  bool intn_asserted () const
  {
    return !intn_.read();
  }

  /** Construct and return an RAII object that supports TWI
   * interaction.
   *
   * Use this within a scope to assert `WAKEn` and enable the
   * associated TWI peripheral.
   *
   * Success of the enable can be assessed from
   * scoped_enabler::result() or treating the enabler as a bool:
   *
   *     if (auto enabler = sensor.scoped_enable()) {
   *       // invoke methods that interact with sensor
   *       rc = twi...;
   *     } else {
   *       // handle interface failure
   *       rc = enabler.result();
   *     }
   *
   * @note Public methods on this class always ensure TWI operations
   * occur within an enabled scope. */
  scoped_enabler scoped_enable () const
  {
    return {twi(), waken_};
  }

  /** The value expected to be read back in version_s::hw_id. */
  static constexpr uint8_t HARDWARE_ID = 0x81;

  /** Aggregate identity and version from hardware and firmware
   * registers. */
  struct version_s
  {
    /** Native byte order boot firmware version identifier.
     *
     * * Bits 12..15 are the major version number.
     * * Bits 8..11 are the minor version number.
     * * Bits 0..7 encode a sub-minor identifier. */
    uint16_t fw_boot_version;

    /** Native byte order application firmware version identifier.
     *
     * * Bits 12..15 are the major version number.
     * * Bits 8..11 are the minor version number.
     * * Bits 0..7 encode a sub-minor identifier. */
    uint16_t fw_app_version;

    /** Hardware identifer, which should be @ref HARDWARE_ID.
     *
     * If the value does not match @ref HARDWARE_ID assume the sensor
     * is not present. */
    uint8_t hw_id;

    /** Hardware version identifier.
     *
     * The high nybble should be 1; the low nybble identifies a build
     * variant. */
    uint8_t hw_version;
  };

  /** Toggle the RESETn line to reset the sensor.
   *
   * @return a non-negative value is the delay before the the sensor
   * may be accessed, in uptime ticks.  A negative value indicates
   * failure to reset. */
  int reset () const;

  /** Value for a valid CCS811 status.
   *
   * The low byte corresponds to a value from the `STATUS` register.
   * If `ST_ERROR` is cleared, the upper byte is zero, otherwise it is
   * the value of the `ERROR_ID` register. */
  using status_type = uint16_t;

  enum STATUS_e : uint16_t
  {
    /** If set the sensor or I2C bus has produced an error.
     *
     * Consult the upper byte of status() for the cause. */
    ST_ERROR = 0x01,

    /** If set new data is available in the ALG_RESULT_DATA register. */
    ST_DATA_READY = 0x08,

    /** If set a valid application image is available in the sensor. */
    ST_APP_VALID = 0x10,

    /** (Boot mode only) Set indicates that a verify operation succeeded. */
    ST_APP_VERIFY = 0x20,

    /** (Boot mode only) Set indicates that an erase operation succeeded. */
    ST_APP_ERASE = 0x40,

    /** Clear indicates device is in boot mode; set indicates device
     * is in application mode. */
    ST_FW_MODE = 0x80,

    /** If set an I2C write was attempted to an unwritable register. */
    EI_WRITE_REG_INVALID = 0x0100,

    /** If set an I2C read was attempted from an unreadable register. */
    EI_READ_REG_INVALID = 0x0200,

    /** If set an I2C write to `MEAS_MODE` provided an unacceptable
     * value. */
    EI_MEASMODE_INVALID = 0x0400,

    /** If set a the sensor has reached the maximum range. */
    EI_MAX_RESISTANCE = 0x0800,

    /** If set the heater supply current is out of range. */
    EI_HEATER_FAULT = 0x1000,

    /** If set the heater supply voltage is not correct. */
    EI_HEATER_SUPPLY = 0x2000,
  };

  /** Captured results after an observation.
   *
   * If @link STATUS_e ST_DATA_READY@endlink is not set in #status
   * then the #eCO2, #eTVOC, and #baseline values are not valid. */
  struct observations_type
  {
    /** The `BASELINE` register read immediately before the
     * `ALG_RESULT_DATA` register. */
    uint16_t baseline;

    /** Equivalent CO2 estimate, in ppm. */
    uint16_t eCO2;

    /** Equivalent total volatile organic compound estimate, in ppb.
     *
     * @note This value appears to be derived from eCO2 or vice-versa.
     * The exact relationship is undocumented, but it appears that 1
     * eTVOC corresponds to about 7 eCO2. */
    uint16_t eTVOC;

    /** The combined `STATUS` and `ERROR_ID` register values.
     *
     * If @link STATUS_e ST_DATA_READY@endlink is not set then the
     * #eCO2, #eTVOC, and #baseline values are not valid.
     *
     * @see is_ready() */
    status_type status;

    /** Test whether the @link STATUS_e ST_DATA_READY@endlink flag is
     * set. */
    bool is_ready () const
    {
      return (ST_DATA_READY & status);
    }

    /** Optional raw data.  This is generally not filled. */
    uint16_t raw;
  };

  /** Read the device hardware and firmware identity and version.
   *
   * @param [out] vid where the version information will be stored.
   *
   * @return non-negative on success, or a negative error code. */
  int id_version (version_s& vid) const;

  /** Access the version state managed by the LPM machine. */
  const version_s& id_version() const
  {
    return version_;
  }

  /** Access the most recent collected observations. */
  const observations_type& observations () const
  {
    return observations_;
  }

  /** Structure that can be broadcast as a beacon to provide detailed
   * system state. */
  struct system_beacon_type
  {
    /** How long the sensor has been running, in deciseconds. */
    uint32_t active_ds = 0;

    /** How long since the baseline register was updated, in deciseconds.
     *
     * A zero value indicates that the baseline is not current. */
    uint32_t bl_age_ds = 0;

    /** The most recently retained value of the `BASELINE` register. */
    uint16_t bl_retained = 0;

    /** The most recently observed value of the `BASELINE` register. */
    uint16_t bl_latest = 0;

    /** Value from version_s::fw_app_version. */
    uint16_t app_version = 0;

    /** Value from version_s::hw_version. */
    uint8_t hw_version = 0;

    /** A copy of retained_state_type::flags. */
    uint8_t flags = 0;

    /** The transmitted length of this structure, excluding trailing
     * padding. */
    static constexpr size_t SPAN = 4+4+2+2+2+1+1;
  };

  /** Set the contents of a system beacon frame.
   *
   * @param [out] fr where the frame information should be stored.
   *
   * @return zero on success, or a negative error code. */
  int fill_system_beacon (system_beacon_type& fr) const;

  /** Structure that can be broadcast as a beacon to provide
   * observation data.
   *
   * @note It is intended that the non-negative value of #status be
   * used as the sd::Beacon::frame_prefix_s::flags value so the drive
   * mode associated with #baseline can be identified. */
  struct observation_beacon_type
  {
    /** Copied from ccs811::env_data(). */
    uint32_t env_data = -1;

    /** Copied from observations_type::baseline. */
    uint16_t baseline = 0;

    /** Copied from observations_type::eCO2. */
    uint16_t eCO2 = 0;

    /** Copied from observations_type::eTVOC. */
    uint16_t eTVOC = 0;

    /** The transmitted length of this structure, excluding trailing
     * padding and untransmitted status. */
    static constexpr size_t SPAN = 4+2+2+2;

    /** Untransmitted content to simplify @link
     * threshold_s::observation_beacon_changed change
     * detection@endlink.
     *
     * When negative this indicates that the source observation was
     * not @link observations_type::is_ready ready@endlink, and so the
     * beacon must not be transmitted.  A non-negative value conveys
     * the current retained_state_type::flags value in its low byte,
     * which is copied into the beacon frame_prefix_s::flags field for
     * transmission. */
    int16_t status;
  };

  /** Populate and return an observation beacon structure from the
   * current state.
   *
   * @note It is the caller's responsibility to ensure that the state
   * of the observation is valid by revewing the content of
   * observation_beacon_type::status. */
  observation_beacon_type observation_beacon () const;

  /** Thresholds for detecting significant changes in readings.
   *
   * @note Thresholds are inclusive of the alerting value.  In other
   * words, if a field is zero then readings with the same value are
   * considered to have met the threshold for change.  This allows
   * trivial configuration to disable transmission limits for
   * mains-powered devices in sparse environments. */
  struct threshold_s
  {
    /** Threshold for changes in observations_type::eCO2. */
    uint16_t eCO2_ppm = 20;

    /** Threshold for changes in observations_type::eTVOC. */
    uint16_t eTVOC_ppb = 5;

    /** Threshold for changes to the temperature value in env_data(). */
    uint8_t temperature_Cel = 1;

    /** Threshold for changes to the relative humidity value in
     * env_data(). */
    uint8_t humidity_pph = 2;

    /** Compare two encoded environmental values for closeness.
     *
     * The HTS221 sensor on the Thingy:52 has abysmal repeatability and
     * frequently bounces back and forth across half-unit boundaries,
     * resulting in unnecessary updates to the environment setting and
     * possibly contributing to variation in eCO2 estimates.
     *
     * Determine whether the difference between two values is large
     * enough to warrant doing an update, using the threshold limits
     * specified in temperature_Cel and humidity_pph.
     *
     * @return nonzero if the two encoded environment values differ in
     * at least the threshold value for at least one encoded
     * measurement. */
    int env_data_changed (uint32_t ed1,
                          uint32_t ed2) const;

    /** Assess whether two observations differ significantly.
     *
     * A change is diagnosed if:
     * * the internal observation_beacon_type::status values differ;
     * * the observation_beacon_type::baseline values differ;
     * * #eCO2 differs by at least threshold_s::eCO2_ppm;
     * * #eTVOC differs by at least threshold_s::eTVOC_ppb;
     * * #env_data differs *at all*.  (It is assumed that the sensor
     *   value would not have been updated if it weren't considered
     *   significant.)
     *
     * @param from a previous instance against which differences are
     * calculated.
     *
     * @param thr the thresholds for detecting differences.
     *
     * @return zero` iff all measurements are equally present/valid in
     * each instance and the differences are less than the measurement
     * threshold.  If a significant change is detected it is indicated
     * by non-zero return value. */
    int observation_beacon_changed (const observation_beacon_type& ob1,
                                    const observation_beacon_type& ob2) const;
  };

  /** Gain read-only access to the LPM machine state. */
  const lpm::state_machine& machine () const
  {
    return machine_;
  }

  /** Bit set in non-negative lpsm_process() result when the sensor has
   * started providing observations.
   *
   * This is set coincident with the first occurrence of
   * lpm::state_machine::PF_OBSERVATION where STATUS_e::ST_DATA_READY is
   * set. */
  static constexpr auto PF_REPORTING = lpm::state_machine::PF_APP_BASE << 0;

  /** Bit set in non-negative lpsm_process() result when the machine
   * updated the CCS811 baseline from retained state.
   *
   * This will occur in conjunction with #PF_CONDITIONED when a
   * baseline is available. */
  static constexpr auto PF_RESTORED = lpm::state_machine::PF_APP_BASE << 1;

  /** Bit set in non-negative lpsm_process() result when the sensor has
   * completed its conditioning period. */
  static constexpr auto PF_CONDITIONED = lpm::state_machine::PF_APP_BASE << 2;

  /** Bit set in non-negative lpsm_process() result when the machine
   * saved the `BASELINE` register to retained state.
   *
   * On receipt of this applications should use update_persisted() to
   * update the non-volatile persisted state in case the volatile
   * retained state is lost due to a power-cycle or uncontrolled
   * reset. */
  static constexpr auto PF_BASELINED = lpm::state_machine::PF_APP_BASE << 3;

  /** Bit set in non-negative lpsm_process() result when a new
   * observation identifies a CCS811 error in
   * observations_type::status. */
  static constexpr auto PF_CCS811_ERROR = lpm::state_machine::PF_APP_BASE << 4;

  /** Convert temperature and humidity to CCS811 `ENV_DATA` format.
   *
   * @note The encoded values are rounded to the nearest half-unit
   * since the CCS811 firmware ignores any finer variation.  See
   * threshold_s::env_data_changed() for change detection support.
   *
   * @param temp_cCel a temperature reading in centi-Celsius.
   *
   * @param rh_pptt a relative humidity reading, in %RH * 100.
   *
   * @return a uint32_t that, when written to the `ENV_DATA` register
   * records the corresponding environment values. */
  static uint32_t encode_env (int16_t temp_cCel,
                              uint16_t rh_pptt);

  /** Read the `STATUS` and, if necessary, `ERROR_ID` registers.
   *
   * If successful, the low 8 bits of the return value correspond to
   * the value of the STATUS register.  If ST_ERROR is set then bits
   * 8..15 of the returned value have the corresponding ERROR_ID
   * register value.
   *
   * @note reading ERROR_ID clears that register, so pay attention to
   * the ST_ERROR bit on what you get back: this is the only chance
   * you'll have to see it.
   *
   * @return a non-negative value as documented for @ref status_type,
   * or a negative error code. */
  int status () const;

  /** Set the environment data structure.
   *
   * @param enc the temperature and humidity encoded by encode_env().
   *
   * @return non-negative on success, or a negative error code. */
  int env_data (uint32_t enc);

  /** Read the last written environment data setting.
   *
   * @note This comes from the driver, as the register cannot be read
   * from the CCS811.  A value with all 1s set indicates that the
   * driver has not set the register.
   *
   * @return as specified. */
  uint32_t env_data () const
  {
    return env_data_;
  }

  /** Read the current baseline value.
   *
   * @return the non-negative value of the baseline register, or a
   * negative error code. */
  int baseline () const;

  /** Update the baseline value.
   *
   * @note After setting this the read-back value appears to be
   * unchanged from the previous setting, whether written after
   * readings start being produced (about 30 s), or after the
   * conditioning period completes (20 min). */
  int baseline (uint16_t value) const;

  /** Record the current baseline in retained state.
   *
   * This may be invoked under external control to capture the
   * baseline when it is known the sensor is in clean air.
   *
   * The invocation fails if the CCS811 has not completed the
   * conditioning period, or does not have an @link observations
   * observation@endlink from which the baseline can be taken.
   *
   * @return a non-negative baseline on success, or a negative error
   * code.
   *
   * @see update_persisted() */
  int retain_baseline ();

  /** Allowed drive modes for the `MEAS_MODE` register. */
  enum DRIVE_MODE_e : uint8_t
  {
    /** Idle mode, no measurements will be produced. */
    DM_IDLE = 0,
    /** Constant power, measurements every second. */
    DM_1_s = 1,
    /** Pulse mode, measurements every 10 seconds. */
    DM_10_s = 2,
    /** Low-pulse model, measurements every 60 seconds. */
    DM_60_s = 3,
  };

  /** Default value for drive_mode() */
  static constexpr auto DEFAULT_DRIVE_MODE = DM_10_s;

  /** Read the drive mode from the active state.
   *
   * This need not match the value from drive_mode() if the sensor is
   * not active.
   *
   * @return a value from @ref DRIVE_MODE_e extracted from
   * retained_state_type::meas_mode (matching the `MEAS_MODE`
   * register), or a negative error code if the sensor is not
   * active. */
  int active_drive_mode () const;

  /** Read the configured drive mode. */
  int drive_mode () const
  {
    return drive_mode_;
  }

  /** Set the drive mode.
   *
   * @param dm a valid value from @ref DRIVE_MODE_e.  All other values
   * are rejected.
   *
   * @return the previous drive mode on success, or a negative error
   * code. */
  int drive_mode (unsigned int dm);

  /** Access the I2C address used for the device. */
  unsigned int i2c_address () const
  {
    return addr_;
  }

private:
  int lpsm_process_ (int& delay,
                     process_flags_type& pf) override;

  /** Read an 8-bit value from the register, returning it or a
   * negative error code.
   *
   * twi must be enabled before this function is invoked. */
  int read_u8_ (uint8_t reg) const;

  /* Other implementations that require enable_scoped(). */
  int status_ () const;
  int baseline_ () const;
  int baseline_ (uint16_t value) const;
  int fetch_ (observations_type& obs);

  /* Fetch the retained (expected) drive mode regardless of machine
   * state. */
  int active_drive_mode_ () const
  {
    const auto rsp = &retained_state_;
    return (retained_state_type::FL_DRIVE_MODE_Msk & rsp->flags) >> retained_state_type::FL_DRIVE_MODE_Pos;
  }

  void intn_callback_ (const periph::GPIOTE::sense_status_type*);

  static retained_state_type retained_state_;

  iface_config_type& iface_config_;

  gpio::active_low waken_;
  gpio::active_low resetn_;
  gpio::pin_reference intn_;

  periph::GPIOTE::sense_listener intn_listener_;
  observations_type observations_{};
  version_s version_{};
  uint64_t baseline_saved_utt_{};
  uint32_t env_data_ = -1;

  uint8_t addr_;
  uint8_t drive_mode_ = DEFAULT_DRIVE_MODE;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_CCS811_HPP */
