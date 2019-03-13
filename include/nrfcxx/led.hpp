/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Core LED functionality
 *
 * @file */

#ifndef NRFCXX_LED_HPP
#define NRFCXX_LED_HPP
#pragma once

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>

namespace nrfcxx {

/** Abstractions and constants around LED capability */
namespace led {

/** Base class supporting LEDs of different types. */
class led_type
{
private:
  static led_type** const ledps_;
  static uint8_t const nleds_;

protected:
  led_type () = default;

public:
  virtual ~led_type ()
  {
    disable();
  }

  /* You can't copy, assign, move, or otherwise change the set of LEDs
   * that the system provides to you.  You can define additional LEDs
   * that aren't in the board-defined sequence. */
  led_type (const led_type&) = delete;
  led_type& operator= (const led_type&) = delete;
  led_type (const led_type&&) = delete;
  led_type& operator= (led_type&&) = delete;

  /** Indicate whether the referenced LED exists
   *
   * This should return @c false only for the stub LED instance that
   * is returned when an ordinal lookup requests an LED that is not
   * supported by the target board.
   *
   * The stub LED instance exists so the application doesn't have to
   * have error handling code. */
  virtual bool exists (void) const
  {
    return false;
  }

  /** Set the GPIO associated with the LED to enable the LED.
   *
   * The LED begins in the off state. */
  void enable (void)
  {
    off();
    enable_();
  }

  /** Disable the nrf5::GPIO associated with the LED.
   *
   * You might want to do this for power reduction. */
  void disable (void)
  {
    disable_();
  }

  /** Set the LED to a specific state.
   *
   * @param v If zero the LED is turned off; if positive the LED is
   * turned on; if negative the LED changes state (off to on or on to
   * off). */
  void set (int v)
  {
    if (0 < v) {
      on();
    } else if (0 == v) {
      off();
    } else {
      toggle();
    }
  }

  /** Turn the LED off. */
  virtual void off (void)
  { }

  /** Turn the LED on. */
  virtual void on (void)
  { }

  /** Toggle the LED state.
   *
   * @note Subclasses need override this only if toggling can be
   * performed more efficiently than by invoking is_on() and selecting
   * between off() and on(). */
  virtual void toggle (void)
  {
    if (is_on()) {
      off();
    } else {
      on();
    }
  }

  /** Read the LED.
   *
   * @return `true` iff the LED is active. */
  virtual bool is_on (void) const
  {
    return false;
  }

  /** Get the number of LEDs supported by the target board */
  static size_t count (void)
  {
    return nleds_;
  }

  /** Return a reference to an LED identified by ordinal position
   *
   * @param i the ordinal position of the desired LED
   *
   * @return a reference to an LED.  If @p i specified an LED that
   * does not exist on the target a no-effect mock LED is returned. */
  static led_type& lookup (unsigned int i)
  {
    if (i < nleds_) {
      return *(ledps_[i]);
    }
    return no_led;
  }

  static led_type no_led;

protected:
  virtual void enable_ ()
  { }

  virtual void disable_ ()
  { }
};

inline led_type& lookup (unsigned int i)
{
  return led_type::lookup(i);
}

/** A class used to manage LEDs.
 *
 * This implements the functions of led_type using a
 * gpio::generic_pin.
 *
 * @tparam active_low a bool value indicating whether the LED is lit
 * when the signal is low (@c true) or when the signal is high (@c
 * false).  Most Nordic development boards use active-low LEDs so the
 * default supports them. */
template <bool active_low = true>
class generic_led : public led_type
{
public:
  /** Create an LED instance that is bound to a generic GPIO pin.
   *
   * @param pin the GPIO pin that controls the LED.  A reference to
   * this is retained by the generic_led instance. */
  explicit generic_led (gpio::generic_pin& pin) :
    pin_{pin}
  { }

private:
  gpio::generic_pin& pin_;

  bool exists (void) const override
  {
    return true;
  }

  void enable_ (void) override
  {
    pin_.configure(gpio::PIN_CNF_WRONLY);
  }

  void disable_ (void) override
  {
    pin_.configure(gpio::PIN_CNF_PWRUP);
  }

  void off (void) override
  {
    if constexpr (active_low) {
      pin_.set();
    } else {
      pin_.clear();
    }
  }

  void on (void) override
  {
    if constexpr (active_low) {
      pin_.clear();
    } else {
      pin_.set();
    }
  }

  /** Read the LED
   *
   * @return `true` iff the LED is active. */
  bool is_on (void) const override
  {
    return pin_.is_set() ^ active_low;
  }
};

/** Class that supports background LED toggles in a repeating pattern.
 *
 * @warning This infrastructure cannot be used with an implementation
 * of gpio::generic_pin that is not safe to execute from inside the
 * uptime FLIH.
 *
 * This combines an LED instance with an alarm, allowing configuration
 * of a pattern display.  The display is decoded from a 32-bit
 * sequence, where set bits indicate the LED should be illuminated and
 * cleared bits indicate the LED should be off.  The duration for
 * display of each bit is configurable, as is the number of times the
 * pattern should be repeated. */
class Pattern
{
  static constexpr uint8_t FLAG_LOOP = 0x01;
  static constexpr uint8_t FLAG_ACTIVE = 0x02;

public:
  /* Can't set an interval less than ~ 250 us.  (Really it's that you
   * can't set an interval of zero, but one less than 2 will livelock,
   * and really anything less than 500 us will probably be
   * indistinguishable.) */
  static constexpr unsigned int MIN_INTERVAL_utt = 8U;

  /** Construct referencing a specific LED. */
  Pattern (led_type& led) :
    led{led},
    alarm_{alarm_callback, this}
  { }

  /** Provide access to the LED controlled by the pattern. */
  led_type& led;

  /** Return the currently configured pattern. */
  unsigned int pattern () const
  {
    return pattern_;
  }

  /** Provide an event setter used to tell the application when the
   * pattern has completed. */
  void set_notify_complete (notifier_type notify)
  {
    notify_complete_ = notify;
  }

  /** Return the configured number of repetitions.
   *
   * @note This is not the same as the number of repetitions remaining
   * in an active pattern.
   *
   * @return negative if configured to loop until canceled, otherwise
   * the number of repetitions to perform on each start().  Zero
   * indicates an unconfigured pattern. */
  int reps () const
  {
    return (FLAG_LOOP & flags_) ? -1 : initial_reps_;
  }

  /** Return the interval between bits of the pattern. */
  unsigned int interval_utt () const
  {
    return interval_;
  }

  /** Configure the pattern.
   *
   * @param pattern a bit pattern showing the LED sequence (bit set =
   * LED on, pattern starts with MSB).
   *
   * @param interval_utt the duration for displaying each bit in the
   * pattern.  This must be at least #MIN_INTERVAL_utt.
   *
   * @param reps the number of times the pattern should be displayed.
   * A negative value means to repeat the pattern until cancel() is
   * invoked.  The value must not exceed 255.
   *
   * @return zero on successful configuration, or a negative error
   * code. */
  int configure (unsigned int pattern,
                 unsigned int interval_utt,
                 int reps = -1);

  /** Indicates whether the pattern is currently running. */
  bool active () const
  {
    return FLAG_ACTIVE & flags_;
  }

  /** Indicates whether the pattern is configured to loop until
   * cancel() is invoked. */
  bool loops () const
  {
    return FLAG_LOOP & flags_;
  }

  /** Return the deadline of the underlying alarm.
   *
   * If the pattern has completed normally the deadline is the time at
   * which the pattern completed. */
  unsigned int deadline () const
  {
    return alarm_.deadline();
  }

  /** Set the deadline of the underlying alarm.
   *
   * @return zero on success, or a negative error code on error
   * (e.g. if invoked while the pattern is running). */
  int set_deadline (unsigned int deadline);

  /** Enable #led and start the pattern.
   *
   * The LED remains enabled until cancel() or the pattern ends
   * naturally, at which point it is disabled.
   *
   * @param delay how long to wait until the pattern starts.  If
   * negative (default) the current @link deadline alarm
   * deadline@endlink is used without change.  Non-negative values
   * delegate to clock::alarm::schedule_offset(), overriding the
   * currently configured deadline.
   *
   * @return zero on successful start, or a negative error code. */
  int start (int delay_utt = -1);

  /** Cancel an active pattern.
   *
   * #led is disabled when this is invoked.
   *
   * @note Cancelling the pattern does not cause the @link
   * set_notify_complete completion event@endlink to be generated. */
  void cancel ();

private:
  static bool alarm_callback (clock::alarm& alarm);

  void complete_ (bool notify);

  bool process_ ();

  clock::alarm alarm_;
  notifier_type notify_complete_;

  unsigned int pattern_ = 0;
  unsigned int interval_ = 0;

  uint8_t flags_ = 0;

  /** Configured number of pattern repetitions.
   *
   * When the pattern completes this value is used to re-initialize
   * #reps_. */
  uint8_t initial_reps_ = 0;

  /** Number of pattern repetitions remaining.
   *
   * Zero indicates that either #IS_LOOP is in play or that the
   * pattern machine is not executing. */
  uint8_t reps_ = 0;

  /** Index of the last completed pattern bit.
   *
   * Zero indicates the next invocation of process_() should turn off
   * the LED and disable the alarm.
   *
   * 32 indicates that the next invocation of process_() should start
   * with the first (high) bit of the pattern. */
  uint8_t idx_ = 0;
};

} // namespace led
} // namespace nrfcxx

#endif /* NRFCXX_LED_HPP */
