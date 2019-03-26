/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/** Abstraction of a dry contact monitor using nrfcxx::gpio::gpiote.
 * @file */

#ifndef NRFCXX_SENSOR_CONTACT_HPP
#define NRFCXX_SENSOR_CONTACT_HPP
#pragma once

#include <type_traits>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>

namespace nrfcxx {
namespace sensor {

namespace contact_internal {

/** Divisor to convert from uptime ticks to contact timestamp clock
 * ticks.
 *
 * This is selected so the 24-bit reduced range uptime clock can be
 * converted to a 16-bit contact timestamp clock with the same
 * representational range. */
static constexpr auto utt_per_ts = 256U;

template <typename INTEGRAL>
constexpr inline
INTEGRAL
ts_from_utt (INTEGRAL dur_utt)
{
  return dur_utt / utt_per_ts;
}

template <typename INTEGRAL>
constexpr inline
INTEGRAL
utt_from_ts (INTEGRAL dur_ts)
{
  return dur_ts * utt_per_ts;
}

} // ns contact_internals

/** State and functionality related to monitoring dry contacts.
 *
 * Instances of this class track edge transitions of a GPIO connected
 * to a dry contact such as a hall-effect switch or leak detector.
 * The total number of state changes, as well as low-resolution
 * timestamps of recent state changes, are maintained for @link
 * snapshot access@endlink by an application.
 *
 * State change events are represented as ordinals described at
 * state_type::ordinal.  Note that the system assumes the default
 * state of the contact is open; if the state is closed at the point
 * the @link contact@endlink instance is constructed a close event
 * will be synthesized.
 *
 * To use this infrastructure you will need the following in your
 * application:
 *
 *     extern "C" {
 *       void GPIOTE_IRQHandler (void)
 *       {
 *         nrfcxx::periph::GPIOTE::irq_handler();
 *       }
 *     } // extern "C"
 *
 * You will also need the following in your application main function,
 * placed after initializing the contacts:
 *
 *     NVIC_EnableIRQ(GPIOTE_IRQn);
 *
 * These are required for both edge and level contact types.
 *
 * @note Methods and members with a suffix @c bi are expected to be
 * invoked under @link mutex_type mutex@endlink. */
class contact
{
public:
  /** The number of timestamped state changes recorded. */
  static constexpr unsigned int HISTORY_LENGTH = 8;

  /** The clock rate used to record @link timestamp_type state
   * transition timestamps@endlink. */
  static constexpr unsigned int TIMESTAMP_Hz = contact_internal::ts_from_utt(clock::uptime::Frequency_Hz);

  /** Calculate a timestamp duration derived from milliseconds. */
  template <typename INTEGRAL>
  static constexpr INTEGRAL ts_from_ms (INTEGRAL dur_ms)
  {
    return contact_internal::ts_from_utt(clock::uptime::from_ms(dur_ms));
  }

  /** Calculate an uptime duration derived from timestamp ticks. */
  template <typename INTEGRAL>
  static constexpr INTEGRAL utt_from_ts (INTEGRAL dur_ts)
  {
    return contact_internal::utt_from_ts(dur_ts);
  }

  /** The type used for full-scale contact state change event times.
   *
   * See now(). */
  using timestamp_type = uint64_t;

  /** The type used for full-scale time differences between event
   * times. */
  using timediff_type = std::make_signed<uint64_t>::type;

  /** The type used for stored contact state changes.
   *
   * The value of the timestamp will be as though obtained through
   * timestamp().  At the default #TIMESTAMP_Hz of 128 the timestamp
   * wraps at the same interval as clock::uptime::now24(). */
  using reduced_timestamp_type = uint16_t;

  /** Full precision duration type for @link now timestamp@endlink
   * clock.
   *
   * @warning As with other std::chrono durations the representation
   * type is signed, so be careful if a negative duration might end up
   * in a context where it's converted to an unsigned counter
   * value. */
  using duration_type = std::chrono::duration<timediff_type, std::ratio<1, TIMESTAMP_Hz>>;

  /** Generic conversion from timestamp ticks to a
   * std::chrono::duration type.
   *
   * @tparam DurT the duration type to use.
   *
   * @param ts the difference between two timestamps. */
  template <typename DurT = duration_type>
  constexpr static DurT to_duration (timediff_type dts)
  {
    return std::chrono::duration_cast<DurT>(duration_type{dts});
  }

  /** An alias to an RAII type suitable for GPIOTE mutex
   * protection. */
  using mutex_type = periph::GPIOTE::mutex_type;

  /** The type used to store state changes event ordinals. */
  using ordinal_type = uint16_t;

  /** An aggregate capturing consistent contact state.
   *
   * When constructed through snapshot() this ensures that the @ref
   * ordinal and @link event_ts event history@endlink for a given
   * contact are internally consistent. @ref live_state may not be
   * consistent with @ref ordinal. */
  struct state_type
  {
    using notify_fn = std::function<void(ordinal_type eo,
                                         timestamp_type ts)>;

    /** Timestamps of recent contact state change events.
     *
     * Timestamps are obtained using contact::timestamp().  The array
     * is indexed by bits obtained by masking the event ordinal with
     * `(HISTORY - 1)`.
     *
     * @note The duration between events can be inferred by
     * subtracting adjacent samples, but be aware that the
     * representation limits of @ref reduced_timestamp_type obscure
     * timer overflows. */
    reduced_timestamp_type event_ts[HISTORY_LENGTH];

    /** The lower bits of a ordinal of state change events.
     *
     * The value represents the ordinal identifier of the most recent
     * state change event.  An open contact state is represented with
     * an even ordinal and a closed state by an odd ordinal.
     *
     * Zero is a valid ordinal representing the initial open state at
     * the time the contact state was initialized.  If the initial
     * state is closed, the ordinal will immediately be advanced. */
    ordinal_type ordinal;

    /** A record of the actual GPIO state at the time a contact @ref
     * snapshot is captured.
     *
     * If this value does not match the boolean value of the low bit
     * of #ordinal then the snapshot was taken at a point where a
     * pending change had not yet been recorded. */
    bool live_state;

    /** Get the recorded time of a specific change.
     *
     * @param eo the state change event ordinal for which the event
     * timestamp is desired.  The returned time is correct only if @p
     * eo is less than contact::HISTORY_LENGTH changes before @ref
     * ordinal.
     *
     * @return the time recorded for the state change identified by @p
     * eo. */
    reduced_timestamp_type time_of_change (ordinal_type eo) const;

    /** As above but returns the time of the most recent change.
     *
     * @overload */
    reduced_timestamp_type time_of_change () const;

    /** The absolute time at which the snapshot was taken.
     *
     * Values are from the range of now(). */
    timestamp_type captured_ts () const
    {
      return captured_ts_;
    }

    /** Calculate the absolute time of the @ref ordinal event.
     *
     * Correct behavior requires that the snapshot have been taken
     * within the overflow of @ref reduced_timestamp_type since the @ref
     * ordinal event occurred. */
    timestamp_type ordinal_ts () const;

    /** Get the contact state corresponding to the last recorded
     * event. */
    bool state_from_ordinal () const
    {
      return contact::state_from_ordinal(ordinal);
    }

    /** Assess the changes between a previous snapshot and this one.
     *
     * This function would normally be invoked by whatever processes
     * the notification that a state change had occurred.  Assuming
     * timely invocation this reconstructs the absolute timestamps of
     * state change events from the truncated versions stored in
     * events_ts.
     *
     * Where individual events are irrelevant, this method provides
     * the elapsed time between the ordinal event of two snapshots.
     *
     * @param prev a snapshot taken at the previous processing of
     * state changes.
     *
     * @param notify optional function to be invoked on each
     * synthesized state change.  If no function is provided the only
     * effect of calling this is to produce the elapsed time.
     *
     * @param collapse if `true` do not invoke @p notify if the
     * elapsed time since the previous event is zero.  The first and
     * last events will be emitted even if their timestamp matches the
     * preceding event.
     *
     * @return the time since the @ref ordinal of @p prev and the @ref
     * ordinal of this snapshot. */
    timestamp_type process_changes (const state_type& prev,
                                    notify_fn notify = nullptr,
                                    bool collapse = true) const;

  private:
    // contact sets captured_ts_
    friend class contact;

    /** The contact::now() timestamp when this state instance was
     * initially constructed. */
    timestamp_type captured_ts_;
  };

  /** The GPIO pin on which the contact is monitored. */
  uint8_t const psel;

  /** Signature for a contact state change notification.
   *
   * @param ordinal the value from state_type::ordinal at the time the
   * function is invoked.
   *
   * @note If the contact state changes fast enough that
   * state_type::ordinal increments multiple times, the notification
   * will occur after the last increment.
   *
   * @note Functions with this signature are expected to be invoked
   * when the GPIOTE IRQ is blocked. */
  using change_callback_bi = std::function<void(ordinal_type ordinal)>;

  /** Construct and configure a contact with callback on change.
   *
   * @param psel the GPIO pin number on which the contact will be monitored.
   *
   * @param callback function invoked when the contact state changes.
   *
   * @param pin_cnf the pin configuration to use for the GPIO
   * identified by @p psel_. */
  contact (uint8_t psel,
           change_callback_bi callback,
           uint32_t pin_cnf = gpio::PIN_CNF_RDONLY);

  /** Update the ordinal to reflect a state change.
   *
   * Increment the ordinal to reflect a state change, recording the
   * current time as the time when the change occurred.
   *
   * If @p double_tap we've been informed that there was a state
   * change, but the observed state is the same as the last one, so
   * record a second, concurrent, state change.
   *
   * @param double_tap if `true` record two changes.
   */
  void record_change_bi (bool double_tap = false);

  /** Construct a @link periph::GPIOTE::sense_listener
   * GPIOTE sense listener@endlink for contact state changes.
   *
   * This method allows contacts to use a shared hardware notification
   * of state changes:
   *
   *     auto notify = [&events](ordinal_type ordinal) {events.set(EVT_BUTTON1);};
   *     contact ordinal1{NRFCXX_BOARD_PSEL_BUTTON1, notify, pin_cnf};
   *     GPIOTE::sense_listener button1{ordinal1.sense_listener()};
   *     GPIOTE::enable_sense();
   *     button1.enable();
   *
   * @warning If the sense returns to its original state prior to the
   * GPIOTE FLIH executing no change will be detected.  This is in
   * contrast to the use of event_listener(), which will detect even
   * transient changes but at the cost of requiring a dedicated GPIOTE
   * resource.
   */
  periph::GPIOTE::sense_listener sense_listener ()
  {
    return {[this](auto sp)
        {
          gpiote_sense_bi_(sp);
        }};
  }

  /** Construct a @link periph::GPIOTE::event_listener
   * GPIOTE event listener@endlink for contact state changes.
   *
   * This method allows contacts to use hardware notifications of
   * state changes:
   *
   *     auto notify = [&events](ordinal_type ordinal) {events.set(EVT_BUTTON1);};
   *     contact ordinal1{NRFCXX_BOARD_PSEL_BUTTON1, notify, pin_cnf};
   *     auto b0ch = GPIOTE::allocate();
   *     b0ch->config_event(ordinal1.psel);
   *     GPIOTE::event_listener button1{ordinal1.event_listener(*b0ch)};
   *     button1.enable();
   *     b0ch->enable_event();
   */
  periph::GPIOTE::event_listener event_listener ()
  {
    return {[this](periph::GPIOTE& instance)
        {
          gpiote_event_bi_(instance);
        }};
  }

  /** Get the full-scale current time in the representation of
   * recorded contact events. */
  static timestamp_type now ()
  {
    return clock::uptime::now() >> 8;
  }

  /** Get the reduced-range current time in the representation of
   * recorded contact events.
   *
   * This is equivalent to but faster than fetching the low 16 bits of
   * now(). */
  static reduced_timestamp_type timestamp ()
  {
    return clock::uptime::now24() >> 8;
  }

  /** Read the contact state.
   *
   * A logic high level is considered closed and is represented by
   * `true`.
   *
   * A logic low level is considered open and is represented by
   * `false`. */
  bool live_state () const
  {
    return pinref_.read();
  }

  /** Synthesize pin state from an event ordinal. */
  static bool state_from_ordinal (ordinal_type eo)
  {
    return (1 & eo);
  }

  /** Return a consistent snapshot of the contact state.
   *
   * @note See state_type::live_state for a situation where its value
   * and the low bit of state_type::ordinal may be inconsistent. */
  state_type snapshot () const;

  /** The absolute time at which the contact initial state was
   * captured.
   *
   * Values are from the range of now(). */
  timestamp_type epoch_ts () const;

protected:
  /** Return the last confirmed contact state.
   *
   * Values match those of live_state(). */
  bool recorded_state_bi_ () const
  {
    return state_from_ordinal(state_bi_.ordinal);
  }

  void gpiote_sense_bi_ (const periph::GPIOTE::sense_status_type* sp);

  void gpiote_event_bi_ (periph::GPIOTE& instance)
  {
    bool double_tap = (live_state() == recorded_state_bi_());
    record_change_bi(double_tap);
  }

  change_callback_bi callback_;

  state_type volatile state_bi_;

  gpio::pin_reference pinref_;
};

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_CONTACT_HPP */
