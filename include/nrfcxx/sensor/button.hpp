/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Button API based on contact sensor.
 * @file */

#ifndef NRFCXX_SENSOR_BUTTON_HPP
#define NRFCXX_SENSOR_BUTTON_HPP
#pragma once

#include <nrfcxx/sensor/contact.hpp>

namespace nrfcxx {
namespace sensor {

/** Class that supports button state and timing events.
 *
 * This leverages sensor::contact to detect level changes using a
 * shared hardware resource.  It assumes human-initiated state
 * changes, i.e. may discard events that occur too rapidly.
 *
 * @note The application infrastructure support for the GPIOTE IRQ
 * described in @ref contact must be provided when using Button. */
class Button
{
public:

  /** Event state indicating that the button has been released.
   *
   * @note That this value is zero, and all press-related events are
   * nonzero, is part of the formal API. */
  static constexpr unsigned int EVT_RELEASE = 0;

  /** Event state indicating that the button has been pressed.
   *
   * This is the first event received for a press event.  There is no
   * delay before this event.  It may be followed by #EVT_HOLD or
   * #EVT_RELEASE. */
  static constexpr unsigned int EVT_CLICK = 1;

  /** Event state indicating that the button is being held.
   *
   * This event is generated roughly hold_utt() ticks after #EVT_CLICK
   * if there is not an earlier #EVT_RELEASE event.
   *
   * @note This event may be inhibited if the button is released
   * shortly after hold_utt() ticks.  The correct pressed duration
   * will be present in the #EVT_RELEASE event. */
  static constexpr unsigned int EVT_HOLD = 2;

  /** Event state indicating that the button appears to be stuck.
   *
   * This event is generated roughly stuck_utt() ticks after #EVT_CLICK
   * if there is not an earlier #EVT_RELEASE event. */
  static constexpr unsigned int EVT_STUCK = 3;

  static const char* eventstr (unsigned int evt);

  /** Callback type used for notification of button events.
   *
   * The callback is provided in the constructor, and is invoked from
   * within process() under application control.
   *
   * @param evt One of #EVT_RELEASE, #EVT_CLICK, #EVT_HOLD, or
   * #EVT_STUCK.
   *
   * @param duration_utt the length of time since the button was
   * pressed, in uptime ticks. */
  using event_callback = std::function<void(unsigned int evt, uint64_t duration_utt)>;

  /** Get the most recent event state.
   *
   * @return One of #EVT_RELEASE, #EVT_CLICK, #EVT_HOLD, or
   * #EVT_STUCK. */
  unsigned int state () const;

  /** Get the most recent event state and duration since click.
   *
   * @param [out] stores zero if returning #EVT_RELEASE, otherwise
   * stores the time since the button was pressed.
   *
   * @return as with state(). */
  unsigned int state (uint64_t& duration_utt) const;

  /** Construct a button instance.
   *
   * @param psel the GPIO pin to which the button is attached.
   *
   * @param notify an event setter used to notify the application that
   * process() needs to be invoked to update the button state.
   *
   * @param callback the method by which state changes are
   * communicated to the application.  The callback is invoked only
   * while process() is active.
   *
   * @param active_low whether the button on @p psel is active low
   * (true) or high (false). */
  Button (uint8_t psel,
          const notifier_type& notify,
          event_callback callback,
          bool active_low = board::button_active_low);

  /** Duration after detected release before button is confirmed to be
   * released.
   *
   * A timer is set for this duration after the GPIO for the button
   * transitions to a released state after being in a pressed state.
   * If the GPIO transitions back to a pressed state before this
   * deadline elapses the alarm is cancelled.
   *
   * The default value is about 80 ms. */
  unsigned int debounce_utt () const
  {
    return debounce_utt_;
  }

  /** Set debounce_utt().
   *
   * @param utt the duration used to detect a button release.  Values
   * less than 256 (or one @link contact::TIMESTAMP_Hz contact
   * tick@endlink) are rejected.
   *
   * @return zero on success, a negative error code if @p utt is not
   * acceptable. */
  int debounce_utt (unsigned int utt);

  /** Duration of button hold necessary to produce EVT_HOLD.
   *
   * The default value is about 500 ms. */
  unsigned int hold_utt () const
  {
    return hold_utt_;
  }

  /** Set hold_utt().
   *
   * @param utt the duration used to identify a hold.  Values less
   * than 256 (or one @link contact::TIMESTAMP_Hz contact
   * tick@endlink) or greater than the current setting of stuck_utt()
   * are rejected.
   *
   * @return zero on success, a negative error code if @p utt is not
   * acceptable. */
  int hold_utt (unsigned int utt);

  /** Duration of button hold necessary to transition from hold to
   * stuck.
   *
   * The default value is about 20 s.*/
  unsigned int stuck_utt () const
  {
    return stuck_utt_;
  }

  /** Set stuck_utt().
   *
   * @param utt the duration used to identify a stuck button.  Values
   * less than the current setting of hold_utt() are rejected.
   *
   * @return zero on success, a negative error code if @p utt is not
   * acceptable. */
  int stuck_utt (unsigned int utt);

  /** Method to be invoked by application when button state changes.
   *
   * On receipt of the notification as provided to the constructor the
   * application should invoke this method from the main event loop.
   * During the invocation the callback provided to the constructor
   * will be invoked with events related to the button.
   *
   * @note Multiple events may be emitted during a single process()
   * invocation. */
  void process ();

private:
  static constexpr bool state_xor = board::button_active_low;

  static void level_callback_ (Button* bp);

  static bool alarm_callback (clock::alarm& alarm);

  uint64_t process_ (contact::ordinal_type eo,
                     contact::timestamp_type ts);

  notifier_type notify_;
  contact contact_;
  periph::GPIOTE::sense_listener listener_;
  event_callback callback_;

  clock::alarm alarm_;
  contact::state_type snapshot_;
  uint64_t process_utt{};
  uint64_t press_utt{};

  unsigned int debounce_utt_ = clock::uptime::from_ms(80U);
  unsigned int hold_utt_ = clock::uptime::from_ms(500U);
  unsigned int stuck_utt_ = clock::uptime::from_ms(20000U);

  uint8_t state_ = 0;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_BUTTON_HPP */
