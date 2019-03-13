/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Infrastructure supporting an broadcaster application.
 * @file */

#ifndef NRFCXX_SD_BROADCASTER
#define NRFCXX_SD_BROADCASTER
#pragma once

#include <nrfcxx/core.hpp>
#include <nrfcxx/sd/beacon.hpp>
#include <nrfcxx/sensor/adc.hpp>

namespace nrfcxx {
namespace sd {

/** Infrastructure that starts and stops the soft device and provides
 * a main event loop for the application, exposed through
 * make_setter().  It also supplies the maintenance operation for both
 * owned and external @link Beacon beacons@endlink, including (in
 * preferred activation order):
 * * #sys_beacon
 * * #tlm_beacon
 * * #appid_beacon
 *
 * The owned beacons are created by the broadcaster, and their
 * configurations may be changed by the application prior to @link
 * start starting the broadcaster@endlink.  The beacons are **not**
 * started along with the broadcaster; if the application desires them
 * they should be @link Beacon::activate activated@endlink once the
 * broadcaster is running.
 *
 * Any application-specific beacons may also be activated after
 * start(). These do not need to be registered with the broadcaster.
 *
 * Once activated the notifications associated with beacon maintenance
 * are processed by the broadcaster's event loop.  The application
 * should invoke wait_for_event() in its main loop, to receive any
 * @link make_setter application-specific events@endlink that it must
 * process.
 *
 * All owned beacons are @link Beacon::deactivate deactivated@endlink
 * when the broadcaster is @link stop stopped@endlink. */
class Broadcaster
{
public:
  /* NB: Events below 0x0010 are reserved for the Broadcaster
   * internals.  Available bits below EVT_APP_BASE: 0xC80 */

  /** Event informing the application that a soft-device interrupt has
   * occured.
   *
   * Such events are not expected for broadcasters, but probably
   * should be handled anyway. */
  static constexpr event_set::event_type EVT_SD = 0x0010;

  /** Event set when the radio has turned on.
   *
   * The application may ignore this event if it is not relevant. */
  static constexpr event_set::event_type EVT_RADIO_ON = 0x0020;

  /** Event set when the radio has turned off.
   *
   * The application may ignore this event if it is not relevant. */
  static constexpr event_set::event_type EVT_RADIO_OFF = 0x0040;

  /** Event set when the broadcaster requires an updated Vdd
   * measurement.
   *
   * This event should be expected to fire shortly (nominally, 500 ms)
   * before a TelemetryBeacon is transmitted.  Inspect #tlm_beacon for
   * transmission rates.
   *
   * The application should initiate a sample using #vdd_sensor as
   * soon as feasible after the event is received.
   *
   * Alternatively the application may calculate the voltage of the
   * system power source (Vbatt, by preference) through other means
   * and provide it to #tlm_beacon via
   * TelemetryBeacon::update_pwr_mV().
   *
   * If no power source voltage has been provided by the time the
   * beacon content is formed the beacon will transmit a zero
   * value. */
  static constexpr event_set::event_type EVT_VDD_REQUIRED = 0x0100;

  /** Event set when Vdd_mV() has been updated.
   *
   * This event will be provided after the application successfully
   * initiates a @link sensor::adc::vdd sample@endlink on
   * #vdd_sensor. */
  static constexpr event_set::event_type EVT_VDD_UPDATED = 0x0200;

  /** Base event available for application use.
   *
   * The application may define and set events at or higher than this
   * bit.  For example:
   *
   *     #define EVT_SAMPLE (Broadcaster::EVT_APP_BASE << 0)
   *     #define EVT_PROCESS (Broadcaster::EVT_APP_BASE << 1)
   */
  static constexpr event_set::event_type EVT_APP_BASE = 0x1000;

  /** Function to maintain broadcaster state across resets.
   *
   * This must be invoked from the application's systemState @link
   * systemState::state_type::app_handler app_handler@endlink to
   * ensure @link Beacon::telemetry_state telemetry state@endlink is
   * properly managed. */
  static void state_setup (const systemState::state_type& ss,
                           bool is_reset,
                           bool retained);

  /** Instantiate the broadcaster instance.
   *
   * @warning Attempts to instantiate a second instance will result in
   * a FailSafeCode::RESOURCE_VIOLATION failsafe.
   *
   * @param cs reference to the application-controlled system state.
   * Note that the application must have caused state_setup() to be
   * invoked when this state was initialized. */
  Broadcaster (const systemState& cs);

  /** stop() is invoked when broadcaster is destructed. */
  ~Broadcaster ();

  /** Construct a setter for the specified bits.
   *
   * @warning FailSafeCode::API_VIOLATION will occur if @p bits
   * includes any bits reserved by the Broadcaster instance.
   *
   * @return the setter. */
  notifier_type make_setter (event_set::event_type bits) const;

  /** Start the broadcast infrastructure.
   *
   * @note This simply configures and enables the soft device.  @link
   * Beacon::activate Activating@endlink any owned beacons is the
   * responsibility of the application.
   *
   * @return Zero on success.  A positive value is a Nordic
   * soft-device error code indicating why the broadcaster could not
   * be started; a negative value indicates a failure in the
   * infrastructure startup. */
  int start ();

  /** Stop the broadcast infrastructure.
   *
   * All owned beacons are @link Beacon:deactivate deactivated@endlink
   * during this process. */
  void stop ();

  /** Sleep until an event occurs.
   *
   * This checks for pending events, and if none are available drops
   * to the lowest power mode available and waits for an event.  When
   * an event is received the set of events is captured, all internal
   * events are processed, and the remaining events are returned to
   * the caller for processing.
   *
   * @note The returned event set may be empty. */
  event_set_copy wait_for_event ();

  /** Return the most recently read system voltage.
   *
   * The value will be zero until the first sample of #vdd_sensor
   * completes.  #EVT_VDD_UPDATED is generated when the value changes.
   *
   * @note The value stored here remains set until updated by another
   * sample.  However, each new value will be used only once @link
   * TelemetryBeacon::udpate_batt_mv once@endlink in a telemetry
   * beacon.  Respect #EVT_VDD_REQUIRED.
   *
   * @return the most recently read system voltage, in millivolts.*/
  uint16_t vdd_mV () const
  {
    return vdd_mV_;
  }

  /** The owned system state beacon. */
  SystemStateBeacon sys_beacon;

  /** The owned telemetry beacon. */
  TelemetryBeacon tlm_beacon;

  /** The owned application identity beacon. */
  ApplicationIdBeacon appid_beacon;

  /** A sensor instance that collects Vdd information for
   * TelemetryBeacon.
   *
   * If this sensor is not used and the source voltage not provided
   * through some other mechanism telemetry beacons will not provide
   * source voltage.
   *
   * The application is responsible for provisioning, enabling, and
   * calibrating @ref periph::ADC, and @link ADCClient::claim
   * claiming@endlink and @link ADCClient::release releasing@endlink
   * the client in response to #EVT_VDD_REQUIRED events.
   *
   * @note #EVT_VDD_UPDATED will be issued whenever this sensor
   * completes a sample. */
  sensor::adc::vdd vdd_sensor;

  /** Get the difference between required and available soft-device
   * RAM.
   *
   * A postive value indicates that the amount of memory available is
   * that much more than is required.
   *
   * A negative value indicates that the amount of memory available
   * must be increased by the absolute value in order to allow the
   * system to run.  In this case start() will fail with
   * `NRF_ERROR_NO_MEM`.
   *
   * @note This value is set by start(). */
  int ram_delta () const
  {
    return ram_delta_;
  }

  /** Reference the Bluetooth device address used in GAP. */
  const ble_gap_addr_t& address () const
  {
    return addr_;
  }

  /** Reference the firmware version structure collected during
   * start(). */
  const ble_version_t& fwid () const
  {
    return fwid_;
  }

private:
  /** Callback registered with the internal vdd instance. */
  static void vdd_callback (uint16_t vdd_mV);

  /** Process the returned voltage data. */
  void vdd_callback_ (uint16_t vdd_mV);

  int ram_delta_ = 0;
  ble_version_t fwid_{};
  ble_gap_addr_t addr_{};
  uint16_t vdd_mV_ = 0;
};

} // namespace sd
} // namespace nrfcxx

#endif /* NRFCXX_SD_BROADCASTER */
