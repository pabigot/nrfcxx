/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Infrastructure supporting BLE beacons.
 * @file */

#ifndef NRFCXX_SD_BEACON
#define NRFCXX_SD_BEACON
#pragma once

#include <pabigot/ble/gap.hpp>

#include <nrfcxx/clock.hpp>

#include "ble.h"

namespace nrfcxx {
namespace sd {

/** Helper to schedule and construct beacons.
 *
 * This is a base class associated with a specific beacon, e.g. sensor
 * values or telemetry.  The beacon interval is configured to range
 * from a minimum to a maximum, with exponential back-off until a
 * reset_interval() operation causes the interval to be reduced to its
 * minimum.  Prior to transmission an optional callback is invoked to
 * update the beacon content.  Beacons can be disabled.  Multiple
 * beacons can be active simultaneously.
 *
 * Subclasses should override populate_().  When designing a beacon
 * payload, consider that the standard BLE 4 advertising packet is 31
 * octets, which may be used this way:
 * * 3 octets FLAG
 * * 3 octets TX_POWER (if dt_tx_power() is not zero)
 * * 4 octets MFG_DATA header (length, type, company id) or other
 * * 19 octets MFG_DATA payload available, starting with frame_prefix_s
 * * 2 octets MFG_DATA suffix for CRC-16/DNP provided by update_crc().
 *
 * Applications must configure the Beacon infrastructure using
 * set_notify(), then invoke process_event() when the corresponding
 * notification event is received.  They must also register for
 * soft-device radio notifications and invoke process_completion()
 * when the radio off event is received.
 *
 * Currently defined infrastructure frame types include
 * * 0x00 @link TelemetryBeacon::frame_s Telemetry@endlink carrying power
 *   source voltage, uptime and advertisement count, and information
 *   on duty cycle and activity.
 * * 0x01 @link SystemStateBeacon::frame_s System State@endlink carrying
 *   material from systemState::state_type including reset reasons and
 *   diagnostics, as well as current resource allocations.
 * * 0x02 @link EnvSensorBeacon::frame_s Environmental Sensor@endlink
 *   carrying temperature, humidity, and related environment measures.
 * * 0x03 @link ApplicationIdBeacon::frame_s Application Id@endlink
 *   carrying the application checksum, soft-device version, processor
 *   identification, and related information.
 * * 0x04 through 0x7F reserved for common infrastructure use
 * * 0x80 through 0xBF available for registered application use
 * * 0xC0 through 0xEF available for unregistered application use
 * * 0xF0 through 0xFE reserved
 * * 0xFF is reserved for test purposes
 *
 * Registered application beacons supported by various examples include:
 * * 0x80 @link sensor::ccs811::beacon_frame_type CCS811
 *   status@endlink providing CCS811 version information and baseline
 *   maintenance status.
 */
class Beacon
{
  struct ref_next
  {
    using pointer_type = Beacon*;
    pointer_type& operator() (Beacon& bcn) noexcept
    {
      return bcn.next_ready_;
    }
  };

  using queue_type = pabigot::container::forward_chain<Beacon, ref_next>;

public:
  /** Natively-supported checksum is CRC-16/DNP. */
  using crc_type = uint16_t;

  /** The maximum length of a Manufacturer Specific Data PDU.
   *
   * This excludes the the length, flag, and company id, but includes
   * the terminating CRC-16/DNP present in some beacon payloads. */
  static constexpr size_t MAX_MSD_LENGTH = 19 + sizeof(crc_type);

  /** Values representing the beacon state.
   *
   * Most transitions are performed under application control, either
   * during setup or in response to events.  The following transitions
   * will occur asynchronously and may be delayed using
   * nrfcxx::clock::uptime::mutex_type:
   * * #SCHEDULED_PREPARE to #SCHEDULED
   * * #SCHEDULED to #READY
   */
  enum state_e : uint8_t
  {
    /** Indicates that the beacon is unusable.
     *
     * Generally this means some configurable parameter such as
     * interval is not valid.
     *
     * Transition out of this state can only occur if the invalid
     * state is corrected. */
    INVALID = 0,

    /** Indicates that an attempt to transmit the beacon failed.
     *
     * This may be due to invalid parameters or an overflow in packing
     * the beacon content.  In either case, transition out of this
     * state is not supported. */
    FAILED,

    /** Indicates the beacon is not scheduled to transmit, but may be
     * activated in the future.
     *
     * Transitions to SCHEDULED_PREPARE or SCHEDULED on activate(). */
    INACTIVE,

    /** Indicates a beacon that was deactivated while it was active.
     *
     * Transitions to INACTIVE when the transmission completes. */
    CANCELLED,

    /** Indicates that the beacon is scheduled for a pre-transmission
     * notification.
     *
     * Transitions to INACTIVE on deactivate(), and to SCHEDULED when
     * the alarm fires.
     *
     * @note Transition from SCHEDULED_PREPARE to SCHEDULED will occur
     * asynchronously. */
    SCHEDULED_PREPARE,

    /** Indicates the beacon is scheduled for a future transmission.
     *
     * Transitions to INACTIVE on deactivate(), and to READY when the
     * alarm fires.
     *
     * @note Transition from SCHEDULED to READY will occur
     * asynchronously. */
    SCHEDULED,

    /** Indicates the beacon should transmit as soon as the radio is
     * available.
     *
     * Transitions to ACTIVE or FAILED when the radio becomes
     * available and all earlier READY beacons complete, and to
     * INACTIVE if cancelled before transmission starts. */
    READY,

    /** Indicates the beacon is actively transmitting.
     *
     * Transitions to INACTIVE, SCHEDULED_PREPARE, or SCHEDULED when
     * the transmission completes, and to CANCELLED on
     * deactivate(). */
    ACTIVE,
  };

  /** State retained across resets that is used in telemetry beacons.
   * @see telemetry_state_setup */
  struct telemetry_state_type
  {
    /** Aggregate uptime from all previous sessions.
     *
     * This is initialized on application start, via
     * telemetry_state_setup().  To get instantanteous total uptime
     * you must add uptime::now() to this value. */
    uint64_t total_uptime;

    /** Aggregate number of advertisements sent over all previous sessions.
     *
     * This is only updated on application start.  To get the
     * instanteous total advertisements you must add #adv_count. */
    unsigned int previous_adv;

    /** Number of advertisements sent during this session. */
    unsigned int current_adv;

    /** Number of advertisements sent since reset.
     *
     * Matches the expectation for Eddystone TLM beacons. */
    uint32_t adv_cnt () const
    {
      return current_adv;
    }

    /** Total number of advertisements sent since installation.
     *
     * Matches the expectation for Eddystone TLM beacons. */
    uint32_t aggr_adv_cnt () const
    {
      return previous_adv + current_adv;
    }

    /** Uptime since reset, in deciseconds.
     *
     * Matches the expectation for Eddystone TLM beacons. */
    uint32_t sec_cnt () const
    {
      using clock::uptime;
      return (10 * uptime::now()) / uptime::Frequency_Hz;
    }

    /** Total uptime since reset, in deciseconds.
     *
     * Matches the expectation for Eddystone TLM beacons. */
    uint32_t aggr_sec_cnt () const
    {
      using clock::uptime;
      return (10 * (total_uptime + uptime::now())) / uptime::Frequency_Hz;
    }
  };

  /** Access the current telemetry state. */
  static const telemetry_state_type& telemetry_state ()
  {
    return telemetry_state_;
  }

  /** The value to be used as the company ID for manufacturer specific data.
   *
   * The standard beacons convey information in an MSD data instance,
   * where the first two octets correspond to frame_prefix_s. */
  static constexpr uint16_t COMPANY_ID = -1;

  /** Minimum value for frame_prefix_s::frame_type available for
   * application beacons.
   *
   * Values less than this are reserved for infrastructure-defined
   * beacons. */
  static constexpr uint8_t APP_FRAME_TYPE_BASE = 0x80;

  /** Maximum value for frame_prefix_s::frame_type available for
   * application beacons.
   *
   * Values less than this are reserved for infrastructure-defined
   * beacons. */
  static constexpr uint8_t APP_FRAME_TYPE_LIMIT = 0xEF;

  /** Frame type reserved for test applications. */
  static constexpr uint8_t FRAME_TYPE_TEST = 0xFF;

  /** Structure providing the prefix for most beacon content.
   *
   * This is intended to be used for Manufacturer Specific Data
   * contents.  The #frame_type field is used to identify the content
   * that follows this prefix. */
  struct frame_prefix_s
  {
    /** Identifies packet content.
     *
     * @see TelemetryBeacon::FRAME_TYPE
     * @see SystemStateBeacon::FRAME_TYPE
     */
    uint8_t frame_type = FRAME_TYPE_TEST;

    /** Flags that provide additional information about the beacon content.
     *
     * Generally zero, but in some cases presence of optional fields
     * may be indicated by bits in this value. */
    uint8_t flags = 0;
  } __attribute__((__packed__));

  /** @cond DOXYGEN_EXCLUDE */
  /* You can't move or copy these because that would invalidate the
   * links in the ready queue. */
  Beacon (const Beacon&) = delete;
  Beacon& operator= (const Beacon&) = delete;
  Beacon (Beacon&&) = delete;
  Beacon& operator= (Beacon&&) = delete;
  /** @endcond */

  /** The beacon is stopped on destruction. */
  virtual ~Beacon ();

  /** Return the current beacon state.
   *
   * @note If the returned state is SCHEDULED the actual state may be
   * updated to READY asynchronously by the alarm infrastructure.  See
   * clock::alarm::mutex_type. */
  state_e state () const
  {
    return state_;
  }

  /** Test whether the beacon is in a valid state.
   *
   * This should return 0 if the configuration would allow the beacon
   * to be activated, and a negative error code if they do not.
   * External users should see a non-zero value only if state() is
   * #INVALID or #FAILED. */
  int validate () const;

  /** Minimum interval between transmissions, in uptime ticks. */
  unsigned int min_interval_utt () const
  {
    return min_interval_;
  }

  /** Current interval between transmissions, in uptime ticks. */
  unsigned int interval_utt () const
  {
    return interval_;
  }

  /** Maximum interval between transmissions, in uptime ticks. */
  unsigned int max_interval_utt () const
  {
    return max_interval_;
  }

  /** Duration before beacon transmission that a @link
   * set_prepare_notify pre-transmission notification@endlink will
   * occur, in uptime ticks. */
  unsigned int prepare_backoff_utt () const
  {
    return prepare_backoff_;
  }

  /** Configure a fixed interval.
   *
   * @note This function must be invoked while the beacon is inactive.
   *
   * @param utt the interval between scheduled beacons, in uptime
   * ticks.  The value must be nonzero.
   *
   * @return zero on success, or a negative error code*/
  int set_interval (unsigned int utt)
  {
    return set_interval(utt, utt);
  }

  /** Configure an exponential back-off interval.
   *
   * Invoking reset_interval() sets the interval back to the minimum
   * and transmits ASAP; afterwards the interval doubles until it
   * reachs the maximum, where it stays until reset again.
   *
   * @note This function must be invoked while the beacon is inactive.
   *
   * @param min_utt the minimum interval between scheduled beacons, in
   * uptime ticks.  The value must be nonzero and strictly greater
   * than prepare_backoff_utt().
   *
   * @param max_utt the maximum interval between scheduled beacons, in
   * uptime ticks.  The value must be no less than @p min_utt.
   *
   * @return zero on success, or a negative error code */
  int set_interval (unsigned int min_utt,
                    unsigned int max_utt);

  /** Configure notification that a beacon will transmit soon.
   *
   * This can be used to collect information for an upcoming beacon,
   * e.g. measuring Vdd just before a telemetry beacon will be
   * generated.
   *
   * There is no coordination between this notification and the
   * transmission; if the application does not react to the
   * notification in a timely manner the beacon will still transmit on
   * schedule.
   *
   * @note This function must be invoked while the beacon is inactive.
   *
   * @note See reset_interval() for its impact on prepare
   * notifications.
   *
   * @param notify the notifier that the beacon will transmit soon.
   * Pass a null notifier to disable the feature.
   *
   * @param backoff_utt the duration before the next transmission at
   * which the notifier will be invoked.  This parameter must be
   * strictly less than min_interval_utt().
   *
   * @return zero on successful disable, positive on successful
   * enable, or a negative error code. */
  int set_prepare_backoff (notifier_type notify,
                           unsigned int backoff_utt);

  /** Configure notification that a beacon is being sent.
   *
   * @p notify will be invoked when the soft-device is successfully
   * invoked to transmit the beacon.
   *
   * @param notify the notifier that the beacon will transmit soon. */
  void set_tx_notify (notifier_type notify)
  {
    tx_notify_ = notify;
  }

  /** Get the GAP ASD Flags data type value to use in the beacon. */
  uint8_t dt_flags () const
  {
    return dt_flags_;
  }

  /** Set the GAP ASD Flags data type value to use in the beacon. */
  void dt_flags (uint8_t v);

  /** Get the GAP ASD Tx Power data type value to use in the beacon. */
  int8_t dt_tx_power () const
  {
    return dt_tx_power_;
  }

  /** Set the GAP ASD Tx Power data type value to use in the beacon. */
  void dt_tx_power (int8_t v);

  /** Reset the interval so the beacon is retransmitted as quickly as
   * possible.
   *
   * min_interval_utt() still applies: a reset immediately after a
   * transmission will incur a delay of at least min_interval_utt()
   * until the next transmission.  A reset at least min_interval_utt()
   * after the last transmission will result in a transmission as soon
   * as the radio is available.
   *
   * @note This function is expected to be invoked from the
   * application main loop.  Invocation from a FLIH may result in
   * anomalous behavior.
   *
   * @warning If this function is invoked on a beacon that has a @link
   * set_prepare_backoff prepare backoff@endlink the prepare
   * notification will not be issued for the first transmission after
   * reset.  If a prepare notification had already been emitted the
   * rescheduled transmission may occur before the expected backoff
   * period completes.
   *
   * @return negative if the beacon was not active; positive if a
   * scheduled beacon transmission was cancelled and a new one
   * scheduled.  Zero if the beacon was ready to transmit or being
   * transmitted. */
  int reset_interval ();

  /** Enable the beacon.
   *
   * Subclasses may override pre_activate_() to confirm that their
   * requirements have been met.  Any non-zero value returned from
   * that function will be returned from this function, aborting the
   * activation process.
   *
   * @return zero on successful activation, or a non-zero result if
   * the activation failed (positive only if from pre_activate_()). */
  int activate ();

  /** Disable the beacon. */
  int deactivate ();

  /** Return the number of transmissions since activation. */
  unsigned int tx_count () const
  {
    return tx_count_;
  }

  /** Helper to store a checksum immediately following a frame instance.
   *
   * This calculates a CRC-16/DNP checksum over the entire value of a
   * (packed) structure, and stores it in residue-compatible format
   * immediately following the data.
   *
   * @tparam FT a structure containing beacon data, often within a
   * Manufacturer Specific Data block that's allocated to hold the
   * structure plus the checksum.
   *
   * @param bp pointer to the prepared beacon data.  The checksum will
   * be stored immediate after object. */
  template <typename FT>
  crc_type update_crc (FT * bp)
  {
    return update_crc_(reinterpret_cast<uint8_t*>(bp), sizeof (*bp));
  }

  static const Beacon* active ()
  {
    return active_;
  }

  /** Register the notifier that signals application to process a
   * beacon event. */
  static void set_notify (notifier_type notify);

  /** Method to be invoked by main loop when a beacon-related event
   * occurs. */
  static int process_event ();

  /** Method to be invoked by main loop when the beacon has been
   * transmitted. */
  static int process_completion ();

  /** Function to maintain beacon telemetry state across resets.
   *
   * This must be invoked from the application's systemState @link
   * systemState::state_type::app_handler app_handler@endlink to
   * ensure beacon @link telemetry_state telemetry state@endlink is
   * properly managed. */
  static void telemetry_state_setup (const systemState::state_type& ss,
                                     bool is_reset,
                                     bool retained);

protected:
  /** Construct a new beacon. */
  Beacon ();

  /** Called by activate() to make sure everything needed for
   * activation is present. */
  virtual int pre_activate_ ()
  {
    return 0;
  }

  /** Function to calculate a checksum and store it after the data.
   *
   * @param sp pointer to a block of data
   *
   * @param span the number of octets at @p sp to be covered by the
   * checksum
   *
   * @return the checksum that was stored at @p sp + @p span. */
  crc_type update_crc_ (uint8_t* sp, size_t span);

  /** Function that must be overridden to fill in beacon content.
   *
   * The return value is interpreted in this way:
   * * A zero value indicates that the beacon content was not
   *   populated, e.g. because nothing is available yet.  The beacon
   *   will not transmit, but it will be immediately rescheduled as if
   *   it had.  Application or infrastructure should invoke
   *   reset_interval() whenever the conditions that are blocking
   *   transmission have been resolved.
   * * A negative value indicates that a critical error has been
   *   discovered.  The beacon will transition into @link state_e
   *   FAILED@endlink state.
   * * A positive value indicates that the beacon has been filled and
   *   transmission should succeed.  If, however, @p ad is not valid
   *   the result will be as if the function had returned `-EINVAL`
   *   and the beacon will be failed.
   *
   * @param ad an advertising data structure, with the Flags and (if
   * nonzero) TxPower data items already populated.
   *
   * @return as documented above. */
  virtual int populate_ (pabigot::ble::gap::adv_data& ad)
  {
    return -EINVAL;
  }

private:
  /** Executes the time-triggered transitions. */
  static bool alarm_callback (clock::alarm& alarm);

  /** Move active_ back onto the queue and notify if there's something
   * left to do. */
  static void requeue_active_ ();

  /** How the application is notified that there are actions to be
   * taken. */
  static notifier_type notify_;

  /** The beacon that is currently transmitting. */
  static Beacon* active_;

  /** Beacons that are ready to transmit, in order of transmission. */
  static queue_type readyq_;

  /** Retained information about system uptime and advertisement counts. */
  static telemetry_state_type telemetry_state_;

  /** Invoke the setter, with failsafe if setter is not provided. */
  static void checked_notify_ ();

  /** Reschedule the beacon at its next deadline. */
  Beacon* reschedule_ ();

  clock::alarm alarm_;

  notifier_type prepare_notify_;
  notifier_type tx_notify_;
  unsigned int tx_count_ = 0;

  unsigned int interval_ = 0;
  unsigned int min_interval_ = 0;
  unsigned int max_interval_ = 0;
  unsigned int prepare_backoff_ = 0;

  /** The time the next beacon should be configured and transmitted,
   * as set by reschedule_(). */
  unsigned int last_due_ = 0;

  /** Pointer to the next beacon in the ready queue. */
  queue_type::pointer_type next_ready_ = queue_type::unlinked_ptr();

  state_e volatile state_ = INVALID;
  uint8_t dt_flags_ = pabigot::ble::gap::FDT_LE_NON_DISCOVERABLE;
  int8_t dt_tx_power_ = 0;

  /* Although S130 will store the PDU in SD RAM, S132 appears to
   * transmit from an application-owned buffer.  Have the beacon
   * instance own the data regardless of soft device. */
  std::array<uint8_t, pabigot::ble::gap::ASR_DATA_SIZE> pdu_;

#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
  /* S132 and S140 use a handle to identify the advertising set
   * characteristics, but contrary to the implication you can only
   * define one advertising set.
   *
   * See https://devzone.nordicsemi.com/f/nordic-q-a/35200/sd_ble_gap_adv_set_configure-returning-error-4-nrf_error_no_mem */
  static uint8_t handle_;
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
};

/** Infrastructure to manage a beacon with a fixed payload.
 *
 * In simple situations the post-prefix frame content has a fixed
 * layout that corresponds a natively-aligned structure, with at most
 * some padding required at its end.  Some boilerplate can be
 * eliminated by using this class to maintain a copy of the data to be
 * transmitted.
 *
 * @tparam CONTENT_TYPE the native structure carrying the frame
 * content.  This must have a constexpr static `size_t` member named
 * `SPAN` that provides the number of octets that contain
 * transmittable data.  This may differ from `sizeof(CONTENT_TYPE)` in
 * the case where the structure has trailing padding to support
 * alignment of consecutive instances, a requirement not relevant to
 * this use.
 *
 * @tparam ID the Beacon::FRAME_TYPE value to use. */
template <typename CONTENT_TYPE,
          uint8_t ID>
class GenericBeacon : public nrfcxx::sd::Beacon
{
public:
  using content_type = CONTENT_TYPE;
  static constexpr uint8_t FRAME_TYPE = ID;
  static constexpr size_t content_span = CONTENT_TYPE::SPAN;

  /** Frame content for this beacon is the first #content_span octets
   * of a #content_type instance.
   *
   * It is assumed the layout is designed to be natively packed, with
   * padding only at its end. */
  struct frame_s
  {
    /** Common prefix for all infrastructure beacons.
     *
     * This is set from #FRAME_TYPE and #flags. */
    frame_prefix_s prefix;

    /** Where the first #content_span octets of #content is copied. */
    uint8_t content[0];
  } __attribute__((__packed__));

  /** The content that will be copied into the beacon by populate_(). */
  content_type content{};

  /** The flags that will be used by populate_(). */
  uint8_t flags = 0;

  /** Set to `true` when #content has been updated.
   *
   * When `false` (default) populate_() will return zero, inhibiting
   * transmission of the beacon while leaving it enabled. */
  bool ready = false;

protected:
  int populate_ (pabigot::ble::gap::adv_data& ad) override
  {
    static_assert(content_span <= sizeof(content_type),
                  "span overruns content");
    if (!ready) {
      return 0;
    }
    auto beacon_span = sizeof(frame_s) + content_span;
    auto bp = static_cast<frame_s *>(ad.set_ManufacturerSpecificData(COMPANY_ID, beacon_span));
    if (bp) {
      bp->prefix = {FRAME_TYPE, flags};
      memcpy(bp->content, &content, content_span);
    }
    return 1;
  }
};

/** %Beacon providing telemetry information.
 *
 * This includes estimated battery voltage, number of advertisements
 * sent, total system uptime, and duty cycle information. *
 * This beacon is configured with:
 * * min_interval_utt() set to 1 s
 * * max_interval_utt() set to 5 min
 *
 * It is not expected that the beacon interval will be reset. */
class TelemetryBeacon : public Beacon
{
public:
  /** frame_prefix_s::frame_type value for this beacon. */
  static constexpr uint8_t FRAME_TYPE = 0;

  /** Function type for calculating power source level from power voltage.
   *
   * @param pwr_mV the measured power source voltage, in millivolts
   *
   * @return the estimated power source level, in parts per ten
   * thousand. */
  typedef unsigned int (* power_level_type) (unsigned int);

  /** Bits used in frame_prefix_s::flags to refine content of the frame.
   *
   */
  enum frame_flag_e : uint8_t
  {
    /** Set to indicate that the device is being powered by mains.
     *
     * For devices that are permanently mains-powered, e.g. through a
     * USB dongle, this will always be true.  In that situation
     * #pwr_mV should always be zero, and #FT_FLAG_HAS_POWER_LEVEL
     * should be cleared.
     *
     * For devices that are battery-powered but can be charged
     * in-place this will be true when the charging connector is
     * powered.  There #pwr_mV may be provided, but cannot be trusted
     * to actually represent the true level of the battery, and in
     * that situation #FT_FLAG_HAS_POWER_LEVEL should be cleared. */
    FT_FLAG_MAINS_POWER = 0x01,

    /** Set to indicate that frame_s::pwr_mV is a measure of system
     * Vdd rather than a measure of some external power source like a
     * battery. */
    FT_FLAG_PWR_IS_Vdd = 0x02,

    /** Set to indicate that the battery is being charged. */
    FT_FLAG_CHARGING = 0x04,

    /** Set to indicate that frame_s::adv_cnt and frame_s::sec_cnt are
     * aggregates since installation (or power lost)
     *
     * If cleared these values count since last reset.
     *
     * @deprecated This feature has been removed.  Once the last
     * installation has been updated past 2018-11-09 remove the flag. */
    FT_FLAG_AGGREGATE_UPTIME = 0x08,

    /** Set to indicate that the fixed content of #frame_s is followed
     * in frame_s::optional by an 8-bit unsigned value that represents
     * the normalized level of the device power source.  E.g. 255
     * indicates 100%, 128 indicates 50.2%, 0 indicates 0% remaining.
     *
     * See comments at #FT_FLAG_MAINS_POWER. */
    FT_FLAG_HAS_POWER_LEVEL = 0x10,

    /** Set to indicate that the field formerly used for wfe_cnt
     * carries reset_count instead.
     *
     * @todo Remove this flag once all firmware that stored `wfe_cnt`
     * where frame_s::reset_cnt is now has been replaced. */
    FT_FLAG_HAS_RESET_COUNT = 0x20,

    //
    // Bits 6-7 are undefined and should be zero.
  };

  /** Frame content for this beacon. */
  struct frame_s
  {
    /** Common prefix for all infrastructure beacons. */
    frame_prefix_s prefix;

    /** Measured system power source voltage in millivolts.
     *
     * The value is intended to be compatible with the Eddystone
     * [VBATT](https://github.com/google/eddystone/blob/master/eddystone-tlm/tlm-plain.md)
     * field.  As such, it should be zero on any device that is always
     * mains-powered (#FT_FLAG_MAINS_POWER).  It should be non-zero on
     * battery-powered devices that are connected to a charging power
     * source.
     *
     * @note The value provided here may be the system Vdd
     * measurement, rather than a battery, as indicated by the value
     * of #FT_FLAG_PWR_IS_Vdd. */
    uint16_t pwr_mV;

    /** Number of beacons sent since system reset or installation.
     *
     * This value is retained across controlled resets, and is
     * compatible with the Eddystone
     * [ADV_CNT](https://github.com/google/eddystone/blob/master/eddystone-tlm/tlm-plain.md)
     * field.
     *
     * @see FT_FLAG_AGGREGATE_UPTIME */
    uint32_t adv_cnt;

    /** Time since system system reset or installation.
     *
     * This value is maintained across controlled resets, and is
     * compatible with the Eddystone
     * [SEC_CNT](https://github.com/google/eddystone/blob/master/eddystone-tlm/tlm-plain.md)
     * field.
     *
     * @note The time is measured in deciseconds.
     *
     * @see FT_FLAG_AGGREGATE_UPTIME */
    uint32_t sec_cnt;

    /** Low bits of a low-level ARM event counter.
     *
     * This provides the low bits of systemState::wfe_count(),
     * identifying activity that is hard to see in the duty cycle. */
    uint16_t reset_cnt;

    /** Relative time spent in sleep mode since reset.
     *
     * The value is interpreted as the numerator of a fraction with
     * denominator 65536, and represents a multipler of the time since
     * system reset (not power-up) to produce the time spent in a mode
     * with systemState::OM_SLEEP set.  Values in excess of 99.5% are
     * expected. */
    uint16_t sleep_n16;

    /** Relative time spent with radio on since reset.
     *
     * The value is interpreted as the numerator of a fraction with
     * denominator 65536, and represents a multipler of the time since
     * system reset (not power-up) to produce the time spent in a mode
     * with systemState::OM_RADIO set.  Values below 0.05% are
     * expected in the steady state. */
    uint16_t radio_n16;

    /** Storage for optional content, presense indicated by the
     * following flags, in this order:
     * * #FT_FLAG_HAS_POWER_LEVEL.  This flag will be added or not
     *    based on #FT_FLAG_MAINS_POWER and the availability of a
     *    non-zero #pwr_mV and a @ref power_level converter.
     */
    uint8_t optional[0];
  } __attribute__((__packed__));

  /** Constructor for beacon requires a systemState reference. */
  TelemetryBeacon (const systemState& system_state);

  /** The systemState instance from which information is taken. */
  const systemState& system_state;

  /** Provide a value for pwr_mV to be used in the next beacon.
   *
   * @param pwr_mV the power source voltage to be recorded
   *
   * @param is_vdd whether #FT_FLAG_PWR_IS_Vdd should be set.
   *
   * @note The provided value will be used once then cleared.  The
   * application is responsible for ensuring a fresh value is made
   * available before the infrastructure invokes populate_().  One way
   * to do that is to use set_prepare_backoff() to be notified of the
   * need to collect or store a new sample. */
  void update_pwr_mV (uint16_t pwr_mV,
                      bool is_vdd = false)
  {
    pwr_mV_ = pwr_mV;
    if (is_vdd) {
      mutate_flags(FT_FLAG_PWR_IS_Vdd, 0);
    } else {
      mutate_flags(0, FT_FLAG_PWR_IS_Vdd);
    }
  }

  /** Modify the content of frame_prefix_s::flags.
   *
   * @param set values from frame_flag_e that should be set in the
   * flags field.
   *
   * @param clear values from frame_flag_e that should be cleared from
   * the flags field.
   *
   * @return the resulting flags value. */
  uint8_t mutate_flags (uint8_t set,
                        uint8_t clear)
  {
    flags_ = set | (flags_ & ~clear);
    return flags_;
  }

  /** Provide a function that can estimate power source level from
   * measured voltage.
   *
   * If this is not provided #FT_FLAG_HAS_POWER_LEVEL will never be
   * included in the beacon. */
  void power_level (power_level_type fn)
  {
    power_level_ = fn;
  }

private:
  int populate_ (pabigot::ble::gap::adv_data& ad) override;
  power_level_type power_level_{};
  uint16_t pwr_mV_ = 0;
  uint8_t flags_ = 0;
};

/** %Beacon providing system state information.
 *
 * This basically exposes all the important information from
 * systemState::state_type captured at boot, along with the relative
 * heap and stack use.
 *
 * This beacon is configured with:
 * * min_interval_utt() set to 1 s
 * * max_interval_utt() set to 60 min
 *
 * It is not expected that the beacon interval will be reset, nor that
 * the content of the beacon will change after a state state reached
 * within the first few minutes after reset. */
class SystemStateBeacon : public Beacon
{
public:
  /** Value used for frame_s::frame_type. */
  static constexpr uint8_t FRAME_TYPE = 1;

  /** Frame content for this beacon */
  struct frame_s
  {
    /** Common prefix for all infrastructure beacons. */
    frame_prefix_s prefix;

    uint32_t last_pc;             ///< nrfcxx::systemState::state_type::last_pc
    uint32_t code;                ///< nrfcxx::systemState::state_type::code
    uint32_t sdfault_id;          ///< nrfcxx::systemState::state_type::sdfault_id
    uint16_t reset_count;         ///< nrfcxx::systemState::state_type::reset_count
    uint16_t reset_reas;          ///< nrfcxx::systemState::state_type::reset_reas
    uint8_t wdt_status;           ///< nrfcxx::systemState::state_type::wdt_status

    /** Heap use relative to reserved heap memory.
     *
     * The value should be divided by 128 to produce the normalized
     * use. */
    uint8_t heap_use;

    /** Maximum observed stack use relative to reserved stack memory.
     *
     * The value should be divided by 128 to produce the normalized
     * use. */
    uint8_t stack_use;
  } __attribute__((__packed__));

  /** Constructor for beacon requires a systemState reference. */
  SystemStateBeacon (const systemState& system_state);

  /** The systemState instance from which information is taken. */
  const systemState& system_state;

private:
  int populate_ (pabigot::ble::gap::adv_data& ad) override;
};

/** %Beacon providing information derived from environmental sensors.
 *
 *
 * This beacon is configured with:
 * * min_interval_utt() set to 1 s
 * * max_interval_utt() set to 60 s
 *
 * The beacon interval should be reset whenever a sensor value differs
 * from the value sent in the last reset by more than some
 * metric-specific limit. */
class EnvSensorBeacon : public nrfcxx::sd::Beacon
{
public:
  /** Value used for frame_s::frame_type. */
  static constexpr uint8_t FRAME_TYPE = 2;

  /** Bits used in frame_prefix_s::flags to identify the content
   * of the frame.
   *
   * Bits are to be read from LSB up.  If the bit is clear, the
   * corresponding element is not present in the frame.  If the bit is
   * set the corresponding element is present in the frame, following
   * all content identified by lower bits.
   *
   * Following the last optional member there will be:
   * * zero or more `int16_t` values providing thermistor measurements
   *   in cCel, with flag values as with #FT_FLAG_TEMPERATURE;
   * * a CRC-16/DNP checksum.
   *
   * If there are bits set in frame_prefix_s::flags after the highest
   * bit defined in the decoding application then all content between
   * the last identified member and the checksum should be discarded,
   * as the payload will be carrying data that is probably not
   * thermistor readings.
   *
   * @note It should not be assumed that every beacon will contain the
   * same information: if a measurement is not available, it will not
   * be present.  However, if thermistors are present the ordinal
   * position for a specific thermistor will be the same across all
   * beacons (i.e. only trailing invalid thermistor measurements will
   * be discarded). */
  enum frame_flag_e : uint8_t
  {
    /** Flag indicating that the frame contains a temperature
     * measurement.
     *
     * Details:
     * * Representation: `int16_t` LE
     * * Units: cCel
     * * Valid range: -299.99 Cel to 299.99 Cel
     *
     * Values with a magnitude at or above 30000 cCel indicate an
     * error reading.
     *
     * The carried temperature is assumed to be measured at the beacon
     * device.  */
    FT_FLAG_TEMPERATURE = 0x01,

    /** Flag indicating that the frame contains a relative humidity
     * measurement.
     *
     * Details:
     * * Representation: `uint16_t` LE
     * * Units: pptt (parts per ten thousand, or 100 * percent)
     * * Valid range: 0.00% to 100.00%
     *
     * The carried humidity is assumed to be measured at the same
     * location as @link FT_FLAG_HAS_TEMPERATURE
     * temperature@endlink. */
    FT_FLAG_HUMIDITY = 0x02,

    /** Flag indicating that the frame contains an absolute
     * atmospheric pressure measurement.
     *
     * Details:
     * * Representation: `uint24_t` LE
     * * Units: cPa
     * * Valid range: 0.0000 hPa to 1258.2911 hPa
     *
     * Sea-level atmospheric pressure ranges from 870 hPa to 1085 hPa.
     * Altitude variation is roughly 120 hPa per km.
     * US continental altitude ranges from -85 m to 4506 m.
     * Inferred range requirements: 330 hPa to 1095 hPa */
    FT_FLAG_ABS_PRESSURE = 0x04,

    /** Flag indicating that the frame contains a differential
     * pressure measurement.
     *
     * Details:
     * * Representation: `int16_t` LE
     * * Units: cPa
     * * Valid range: -299.99 Pa to 299.99 Pa
     */
    FT_FLAG_DIFF_PRESSURE = 0x08,

    /** Flag indicating that the frame contains an equivalent CO2 air
     * quality measurement.
     *
     * Details:
     * * Representation: `uint16_t` LE
     * * Units: ppm
     * * Valid range: 400 .. 32768
     */
    FT_FLAG_IAQ_ECO2 = 0x10,

    /** Flag indicating that the frame contains an equivalent Total
     * Volatile Organic Compound air quality measurement.
     *
     * Details:
     * * Representation: `uint16_t` LE
     * * Units: ppb
     * * Valid range: 0 .. 32768
     */
    FT_FLAG_IAQ_ETVOC = 0x20,

    /** Flag indicating that the frame contains a relative light
     * intensity measurement that's proportional to the log of lux,
     * but with an unspecified slope and offset.
     *
     * Details:
     * * Representation: `uint8_t`
     * * Units: unspecified
     * * Valid range: 0 (complete dark) .. 255 (intense light)
     */
    FT_FLAG_LIGHT_INTENSITY = 0x40,
  };

  /** This sensor uses a variable-content frame where element presence
   * is inferred from frame_prefix_s::flags.
   *
   * See @ref frame_flag_e for the implicit layout. */
  using frame_s = frame_prefix_s;

  /** Generic frame data holder. */
  struct instance_s
  {
    /** A buffer used to hold frame content. */
    std::array<uint8_t, MAX_MSD_LENGTH> buffer;

    /** The length of the MSD content, including terminating CRC.
     *
     * If the value is zero the frame content has not been finalized. */
    uint8_t span;
  };

  /** Thresholds for detecting significant changes in readings.
   *
   * @note Thresholds are inclusive of the alerting value.  In other
   * words, if a field is zero then readings with the same value are
   * considered to have met the threshold for change.  This allows
   * trivial configuration to disable transmission limits for
   * mains-powered devices in sparse environments. */
  struct threshold_s
  {
    /** Threshold for #FT_FLAG_TEMPERATURE and for thermistors. */
    uint16_t temperature_cCel;

    /** Threshold for #FT_FLAG_HUMIDITY */
    uint16_t humidity_pptt;

    /** Threshold for #FT_FLAG_ABS_PRESSURE */
    uint32_t abs_cPa;

    /** Threshold for #FT_FLAG_DIFF_PRESSURE */
    uint16_t diff_cPa;

    /** Threshold for #FT_FLAG_IAQ_ECO2 */
    uint16_t eCO2_ppm;

    /** Threshold for #FT_FLAG_IAQ_ETVOC */
    uint16_t eTVOC_ppb;

    /** Threshold for #FT_FLAG_LIGHT_INTENSITY */
    uint8_t light_1;
  };

  /** Assess whether the current instance differs from a previous
   * instance significantly.
   *
   * A change is diagnosed if:
   * * the instance_s::span values differ
   * * the frame_prefix_s::flags values differ
   * * #FT_FLAG_TEMPERATURE differs in out-of-range value, or by at
   *   least threshold_s temperature_dCel;
   * * #FT_FLAG_HUMIDITY differs by at least threshold_s humidity_ppt;
   * * #FT_FLAG_ABS_PRESSURE differs by at least threshold_s abs_Pa;
   * * #FT_FLAG_DIFF_PRESSURE differs by at least threshold_s diff_dPa;
   * * #FT_FLAG_IAQ_ECO2 differs by at least threshold_s eCO2_ppm;
   * * #FT_FLAG_IAQ_ETVOC differs by at least threshold_s eTVOC_ppb;
   * * #FT_FLAG_LIGHT_INTENSITY differs by at least threshold_s light_1;
   *
   * @param from a previous instance against which differences are
   * calculated.
   *
   * @param thr the thresholds for detecting differences.
   *
   * @return zero` iff all measurements are equally present/valid in
   * each instance and the differences are less than the measurement
   * threshold.  If a significant change is detected it is indicated
   * by a negative return value. */
  int significantChange (const instance_s& from,
                         const threshold_s& thr) const;

  /** Get a reference to the built-up content.
   *
   * Content is constructed using resetInstance() followed by various
   * methods like addTemperature(), and finished with
   * finalizeInstance(). */
  const instance_s& instance () const
  {
    return instance_;
  }

  EnvSensorBeacon ();

  EnvSensorBeacon& resetInstance ();
  EnvSensorBeacon& addTemperature (int16_t value_cCel);
  EnvSensorBeacon& addHumidity (uint16_t value_pptt);
  EnvSensorBeacon& addAbsPressure (unsigned int value_cPa);
  EnvSensorBeacon& addDiffPressure (int16_t value_cPa);
  EnvSensorBeacon& addECO2 (uint16_t value_ppm);
  EnvSensorBeacon& addETVOC (uint16_t value_ppb);
  EnvSensorBeacon& addLightIntensity (uint8_t value_1);
  EnvSensorBeacon& addThermistor (int16_t value_cCel);
  int finalizeInstance();

private:
  /** Add content to buffer_ if it fits.
   *
   * If adding the content would exceed the space available for
   * content, #invalid_ is set.
   *
   * Adds the new material if #invalid_ is clear.
   *
   * Returns `true` iff the new material was added. */
  bool append_ (const void* vp, size_t len);

  /** Set the provided flag.
   *
   * @warning If any bits higher than @p flag are already set in
   * frame_prefix_s::flags then a @ref FailSafeCode::API_VIOLATION is
   * diagnosed. */
  void setFlag_ (uint8_t flag)
  {
    auto fp = reinterpret_cast<frame_s *>(instance_.buffer.data());
    if (fp->flags & (~(flag - 1))) {
      nrfcxx::failsafe(nrfcxx::FailSafeCode::API_VIOLATION);
    }
    fp->flags |= flag;
  }

  /** Storage for the built-up frame content. */
  instance_s instance_;

  /** Index into buffer_ for next data to be stored.
   *
   * A zero value indicates that an append operation failed and the
   * instance is invalid.
   *
   * Initialized to a non-zero value by resetInstance(), cleared by
   * append_() on overflow.  */
  uint8_t bi_ = 0;

  int populate_ (pabigot::ble::gap::adv_data& ad) override;
};

/** %Beacon providing application identification information.
 *
 * This basically exposes all the important information from
 * systemState::state_type captured at boot, along with the relative
 * heap and stack use.
 *
 * This beacon is configured with:
 * * min_interval_utt() set to 10 s
 * * max_interval_utt() set to 6 h
 *
 * It is not expected that the beacon interval will be reset, nor that
 * the content of the beacon will change unless the firmware is
 * updated. */
class ApplicationIdBeacon : public Beacon
{
public:
  /** Value used for frame_s::frame_type. */
  static constexpr uint8_t FRAME_TYPE = 3;

  /** Frame content for this beacon */
  struct frame_s
  {
    /** Common prefix for all infrastructure beacons. */
    frame_prefix_s prefix;

    /** The value from application_crc32(). */
    uint32_t app_crc;

    /** Soft-device firmware identifier.
     *
     * This corresponds to `ble_version_t::subversion_number`
     * uniquely(?) identifying the soft device family and version.  We
     * assume this is a Nordic device, so don't bother providing the
     * company id.*/
    uint16_t sd_fwid;

    /** Bluetooth Link Layer Version parameter.
     *
     * From `ble_version_t::version_number`.
     *
     * @see https://www.bluetooth.com/specifications/assigned-numbers/link-layer */
    uint8_t llver;

    /** Total reserved stack space, in kilobytes.
     *
     * This may be combined with information from
     * SystemStateBeacon::frame_s to determine absolute stack
     * usage. */
    uint8_t stack_KiBy;

    /** Total reserved heap space, in kilobytes.
     *
     * This may be combined with information from
     * SystemStateBeacon::frame_s to determine absolute heap usage. */
    uint8_t heap_KiBy;

  } __attribute__((__packed__));

  /** Constructor for beacon requires a systemState reference. */
  ApplicationIdBeacon ();

  /** Function to be invoked once the soft device is running, to
   * populate the beacon content.
   *
   * @param system_state used to determine reserved stack and heap
   * space.
   *
   * @param fwid the soft device version collected during
   * initialization. */
  void prepare (const systemState& system_state,
                const ble_version_t& fwid);

private:
  int pre_activate_ () override;

  frame_s frame_;
  int populate_ (pabigot::ble::gap::adv_data& ad) override;
};

} // namespace sd
} // namespace nrfcxx

#endif /* NRFCXX_SD_BEACON */
