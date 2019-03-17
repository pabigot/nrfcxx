/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Core clock-related functionality
 *
 * @file */
#ifndef NRFCXX_CLOCK_HPP
#define NRFCXX_CLOCK_HPP
#pragma once

#include <chrono>
#include <functional>

#include <nrfcxx/impl.hpp>
#include <nrfcxx/periph.hpp>

namespace nrfcxx {

/** Functions and classes related to clocks and time */
namespace clock {

/** Functions and data related to the high-frequency clock.
 *
 * The high-frequency clock is on when the system is in ON mode, but
 * may be sourced from either crystal oscillator or an RC
 * oscillator.
 *
 * @note This is a data type for visibility management only; all
 * members are static and it is not possible to create or manipulate
 * instances of the type. */
class hfclk
{
  hfclk () = delete;
  ~hfclk () = delete;
  hfclk (const hfclk&) = delete;
  hfclk& operator= (const hfclk&) = delete;
  hfclk (hfclk&& ) = delete;
  hfclk& operator= (hfclk&) = delete;

protected:

  /** TIMER needs to request and release HFCLK constant latency. */
  friend class periph::TIMER;

  /** Set or clear timer-based requests for constant latency wakeups.
   *
   * @see periph::TIMER::configure
   *
   * @note Although this supports nRF51 PAN 11 "HFCLK: Base current
   * with HFCLK running is too high" by ensuring a running timer sets
   * the power submode to force HFCLK active, it is independently
   * useful to reduce latency when timer interrupts occur. */
  static void constlat_request (const periph::TIMER& timer,
                                bool enable);

public:

  /** The high-frequency clock always runs at nominal 16 MHz.
   *
   * This is true for both nRF51 where system clock is at 16 MHz, and
   * for nRF52 where system clock is at 64 MHz. */
  constexpr static unsigned int Frequency_Hz = 16'000'000;

  /** Return @c true iff the high-frequency clock is being driven
   * by a crystal oscillator.
   *
   * If @c false the clock uses the RC oscillator. */
  static bool hfxt_active ()
  {
    return ((CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)
            == (nrf5::CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk));
  }

  /** Enable, disable, or query the state of application request
   * for the high-frequency crystal.
   *
   * @param on a positive value to enable the HFXT; zero to disable
   * HFXT; a negative value to return the configured state without
   * change.
   *
   * @return @c true iff the configured state of the crystal is on.
   *
   * @note This controls whether the application requires the crystal as
   * a source.  hfxt_active() can be false when the crystal is
   * configured enabled (e.g. while stabilizing), and can be true when
   * the crystal is configured disabled (e.g. when calibrating a
   * synthesized low-frequency clock).
   *
   * @warning On some nRF51 chips starting the HFXT may inhibit
   * functional writes to `nrf5::CLOCK->LFCLKSRC` until the
   * `HFCLKSTARTED` event is received.  A reasonable workaround is to
   * start the LFCLK first.  See
   * [devzone](https://devzone.nordicsemi.com/f/nordic-q-a/8458/hfclkstart-blocks-changes-to-lfclksrc/)
   * for the anomaly report to Nordic. */
  static bool hfxt_configure (int on);
};

/** Functions and data related to the low-frequency clock.
 *
 * The low-frequency clock must be explicitly enabled and may be
 * sourced from a crystal oscillator, an RC oscillator, or synthesized
 * from the high frequency crystal.
 *
 * @note This is a data type for visibility management only; all
 * members are static and it is not possible to create or manipulate
 * instances of the type. */
class lfclk
{
  lfclk () = delete;
  ~lfclk () = delete;
  lfclk (const lfclk&) = delete;
  lfclk& operator= (const lfclk&) = delete;
  lfclk (lfclk&& ) = delete;
  lfclk& operator= (lfclk&) = delete;

public:

  /** The low-frequency clock always runs at nominal 32 KiHz */
  constexpr static unsigned int Frequency_Hz = 32768U;

  /** The default RC oscillator calibration interval, in
   * quarter-seconds.
   *
   * A 4 s interval is expected to maintain 250 ppm accuracy in varying
   * temperature environments. */
  constexpr static unsigned int DefaultRCOCalInterval_qs = 4 * 4;

  /** Return @c true iff the low-frequency clock is active */
  static bool active ()
  {
    return ((CLOCK_LFCLKSTAT_STATE_Running << CLOCK_LFCLKSTAT_STATE_Pos)
            == (nrf5::CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk));
  }

  /** Enable, disable, or query the state of the low-frequency
   * clock.
   *
   * @param on a positive value to enable the clock; zero to disable it;
   * a negative value to return the configured state without change.
   *
   * @return A positive if the clock was configured on prior to this
   * call, zero if the clock was configured off, a negative value if a
   * configuration change could not be executed.
   *
   * @note The default state of the clock on power-up is disabled, but
   * initialize() will always enable it.
   *
   * @note source_configure() may be used to select the desired clock
   * source, but it will not take effect until the next time the clock
   * changes from disabled to enabled.
   *
   * @note This function will fail if calibration is being performed
   * when disabling LFCLK sourced from #CLOCK_LFCLKSRC_SRC_RC.  Retry
   * after about 16 ms. */
  static int configure (int on);

  /** Return the actual source for the low-frequency clock */
  static uint32_t source ()
  {
    return (nrf5::CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_SRC_Msk) >> CLOCK_LFCLKSTAT_SRC_Pos;
  }

  /** Enable, disable, or query the source of the low-frequency
   * crystal.
   *
   * @param src one of #CLOCK_LFCLKSRC_SRC_RC, #CLOCK_LFCLKSRC_SRC_Xtal,
   * or #CLOCK_LFCLKSRC_SRC_Synth, or a negative value to return the
   * configured source without change.
   *
   * @param cal_interval_qs the clock calibration interval when
   * configured to use #CLOCK_LFCLKSRC_SRC_RC.  Pass zero to disable
   * repeated calibration.  Ignored for other configuration options.
   *
   * @return one of #CLOCK_LFCLKSRC_SRC_RC, #CLOCK_LFCLKSRC_SRC_Xtal, or
   * #CLOCK_LFCLKSRC_SRC_Synth, or a negative value on an attempt to
   * change the clock source while active() is @c true.
   *
   * @note The power-up default source is #CLOCK_LFCLKSRC_SRC_RC, but
   * clock::initialize() may change that to a crystal if the board
   * provides one.
   *
   * @note Although passing zero disables repeated calibration when @p
   * src is #CLOCK_LFCLKSRC_SRC_RC the system will unconditionally
   * execute one calibration as soon as the clock has started with this
   * source. */
  static int source_configure (int src,
                               unsigned cal_interval_qs = DefaultRCOCalInterval_qs);
};

/** Initialize the clock system
 *
 * This configures the interrupt handler and must be invoked before
 * any other function in this namespace.  It unconditionally invokes
 * lfclk::configure() to enable the low-frequency clock.
 *
 * @param enable_hfxt Passed to hfclk::hfxt_configure() to
 * default-enable the high-frequency crystal.
 *
 * @param lfclk_src Passed to lfclk::source_configure() prior to
 * invoking lfclk::configure() to set the source for the low-frequency
 * clock.  The default is a 32 KiHz crystal if the board provides one,
 * otherwise the RC oscillator.
 *
 * @param lfrc_cal_interval_qs Passed to lfclk::source_configure()
 * prior to invoking lfclk::configure() to the calibration interval
 * when @p lfclk_src is CLOCK_LFCLKSRC_SRC_RC.
 *
 * @return 0 on the first call, a negative error code on subsequent
 * calls (which have no effect on the clock configuration).
 *
 * @note The `POWER_CLOCK` interrupt is enabled by this function.  Use
 * configure_pcirq() to release it prior to enabling soft devices.
 *
 * @note Both the LF clock configuration and the HF crystal
 * configuration may be changed after this function is called.
 * Correct behavior on reconfiguration may require that the
 * `POWER_CLOCK` interrupt handler be @link configure_pcirq
 * enabled@endlink.
 */
int initialize (bool enable_hfxt = false,
                int lfclk_src = board::default_lfclk_src(),
                unsigned int lfrc_cal_interval_qs = lfclk::DefaultRCOCalInterval_qs);

/** Query or control whether the `POWER_CLOCK` interrupt handler is enabled.
 *
 * `POWER_CLOCK_IRQHandler` plays a crucial role in maintaining a
 * calibrated RC-based low-frequency clock.  However, the application
 * is not permitted to control this interrupt during periods when a
 * soft device is enabled.
 *
 * @param on a positive value to enable `POWER_CLOCK_IRQn`; zero to
 * disable it; a negative value to return the configured state without
 * change.
 *
 * @return `true` iff the `POWER_CLOCK` interrupt is enabled. */
bool configure_pcirq (int on);

/** Support for a persistent system clock with 32 KiHz resolution.
 *
 * The functionality of this namespace is backed by infrastructure
 * tied to one of the RTCs, almost certainly
 * nrfcxx::periph::RTC::RTC1. */
class uptime
{
public:
  /** Frequency of the uptime clock as a symbolic constant. */
  constexpr static unsigned int Frequency_Hz = 32768;

  /** An RAII type for mutex access to state that might be changed
   * during the uptime clock FLIH. */
  using mutex_type = mutex_irq<static_cast<IRQn_Type>(nrf5::UPTIME_RTC.IRQn)>;

  /** Full precision duration type for uptime clock.
   *
   * @warning As with other std::chono durations the representation
   * type is signed, so be careful if a negative duration might end up
   * in a context where it's converted to an unsigned counter
   * value. */
  using duration_type = std::chrono::duration<int64_t, std::ratio<1, Frequency_Hz>>;

  /** A type containing a NUL-terminated text time representation: `HH:MM:SS.mmm` */
  using text_type = char[13];

  /** A type encoding a NUL-terminated text day+time representation: `DDDD HH:MM:SS.mmm` */
  using day_text_type = char[18];

  /** Convert an uptime duration to text format.
   *
   * @param buf where the representation will be stored.
   *
   * @param dur_utt the duration to be represented.  Values wrap at
   * multiples of 100 hours.
   *
   * @return @p buf */
  static const char* as_text (text_type buf,
                              uint64_t dur_utt);

  /** Convert an uptime duration to text format.
   *
   * Normal result is `DDD HH:MM:SS` or `DDD HH:MM:SS.mmm`, where the
   * number of days has leading zeros.  If more than 999 days are
   * required, a four-digit day field is generated.
   *
   * @param buf where the representation will be stored.
   *
   * @param dur_utt the duration to be represented.
   *
   * @param with_msec whether milliseconds are to be included in the
   * output.
   *
   * @return @p buf */
  static const char* as_day_text (day_text_type buf,
                                  uint64_t dur_utt,
                                  bool with_msec = false);

  /** Low 24 bits of the uptime counter.
   *
   * This interface has the least overhead to obtain a counter, but
   * wraps every 512 s (8 min 32 s). */
  static unsigned int now24 ()
  {
    return nrf5::UPTIME_RTC->COUNTER;
  }

  /** Calculate the tick-count between two 24-bit counter values.
   *
   * @return The value that when added to counter value @p a would
   * produce counter value @p b.
   *
   * @note @p a and @p b are assumed to be 24-bit counter values as
   * returned by now24().  The duration value returned will be in the
   * range `[0 s, 512 s)`. */
  constexpr static unsigned int delta24 (unsigned int a,
                                         unsigned int b)
  {
    /* This relies on standard semantics for unsigned subtraction to
     * handle the case where b < a. */
    return periph::RTC::counter_mask & (b - a);
  }

  /** Class supporting short-term duration measurements.
   *
   * On construction this captures the current value of now24().
   * delta() can be used to get the number of ticks since the captured
   * time.  delta_reset() is similar, but resets the capture time to
   * the current time, allowing timing of multiple steps of a
   * process.
   *
   * @see periph::TIMER::timestamp */
  class timestamp24
  {
  protected:
    unsigned int capture_utt24;

  public:
    timestamp24 () :
      capture_utt24{now24()}
    { }

    /** Return the tick corresponding to the captured time. */
    unsigned int captured () const
    {
      return capture_utt24;
    }

    /** Capture the current time. */
    void reset ()
    {
      capture_utt24 = now24();
    }

    /** Return ticks since the captured time. */
    unsigned int delta () const
    {
      auto now_utt24 = now24();
      return delta24(capture_utt24, now_utt24);
    }

    /** Return ticks between captured time and @p tick. */
    unsigned int delta (unsigned int tick) const
    {
      return delta24(capture_utt24, tick);
    }

    /** Return ticks since the captured time and reset captured
     * time to now. */
    unsigned int delta_reset ()
    {
      auto now_utt24 = now24();
      auto rv = delta24(capture_utt24, now_utt24);
      capture_utt24 = now_utt24;
      return rv;
    }
  };

  /** Full-range uptime counter.
   *
   * This interface supports durations exceeding 17 Ma (i.e. about one
   * quarter of the time since Chicxulub).
   *
   * Feel free to store the value in an `unsigned int` which will
   * reduce the maximum duration to about 32 h 24 min 32 s. */
  static uint64_t now ();

  /** Generic conversion from uptime ticks to a std::chrono::duration type. */
  template <typename DurT>
  constexpr static DurT to_duration (int64_t utt)
  {
    return std::chrono::duration_cast<DurT>(duration_type(utt));
  }

  /** Generic conversion from a std::chrono::duration type to uptime ticks. */
  template <typename DurT>
  constexpr static typename duration_type::rep from_duration (DurT dur)
  {
    return std::chrono::duration_cast<duration_type>(dur).count();
  }

  /** Convert uptime ticks to integral microseconds (rounding
   * down). */
  constexpr static int64_t to_us (int64_t utt)
  {
    return to_duration<std::chrono::microseconds>(utt).count();
  }

  /** Convert integral microseconds to uptime ticks (rounding
   * down). */
  constexpr static int64_t from_us (int64_t us)
  {
    return from_duration(std::chrono::microseconds(us));
  }

  /** Convert uptime ticks to integral milliseconds (rounding
   * down). */
  constexpr static int64_t to_ms (int64_t utt)
  {
    return to_duration<std::chrono::milliseconds>(utt).count();
  }

  /** Convert integral milliseconds to uptime ticks (rounding
   * down). */
  constexpr static int64_t from_ms (int64_t ms)
  {
    return from_duration(std::chrono::milliseconds(ms));
  }

  /** Sleep for a given number of uptime ticks.
   *
   * @param dur the number of ticks to stay asleep, absent invocations
   * of wakeup().
   *
   * @return the number of ticks remaining from the value of @p dur
   * passed in.  Normally this will be zero or negative, but it may be
   * positive if wakeup() was invoked during a FLIH.
   *
   * @note If wakeup() has been invoked since the last time sleep()
   * returned the function will return immediately.  Reinvocations
   * will not return early unless wakeup() is invoked again. */
  static int sleep (int dur);

  /** Sleep for a specified duration.
   *
   * @param dur how long to stay asleep, absent invocations of wakeup().
   *
   * @return the remaining portion of @p dur at the time of return.
   * The value will be rounded towards zero so if this is to be useful
   * the precision of @p DurT should be at least as high as
   * #duration_type.
   *
   * @see sleep(int) */
  template <typename DurT,
            typename DurT::rep = 0>
  static DurT sleep (DurT dur)
  {
    return to_duration<DurT>(sleep(from_duration(dur)));
  }

  /** Cancel any in-progress sleep().
   *
   * This function should be invoked from interrupt handlers in the
   * case where the application may be blocked in a call to sleep()
   * that should be pre-empted by application processing of an event
   * noted by the handler. */
  static void wakeup ();
};

/** User-defined literal support for uptime ticks.
 *
 * @note Availability of this UDL requires `using namespace
 * nrfcxx::clock`. */
constexpr uptime::duration_type
operator"" _utt (unsigned long long n)
{
  return uptime::duration_type(n);
}

/** Class supporting an alarm with custom callback and repeatability.
 *
 * This capability supports an unbounded number of alarms multiplexed
 * onto a capture/compare register in the clock::uptime
 * infrastructure.  Alarms are given a deadline(), which is @link
 * schedule_offset normally specified@endlink relative to the time at
 * which they are scheduled but can be @link schedule
 * absolute@endlink.
 *
 * They may have a non-zero interval() which is used to support
 * periodic alarms.
 *
 * They generally have a @link #callback_type callback@endlink which
 * is invoked from the uptime interrupt handler performs
 * alarm-specific actions, as well as controlling whether the alarm is
 * rescheduled.
 *
 * @note Several member functions are only safe to use when alarms are
 * unscheduled.  Alarms are unscheduled during their callbacks; these
 * functions (including set_interval() and set_deadline()) are safe to
 * use in that context. */
class alarm
{
public:
  /** An RAII type for mutex access blocking changes to alarm state. */
  using mutex_type = uptime::mutex_type;

  /** Constants identifying the alarm state. */
  enum state_type
  {
    /** Alarm has been constructed but has either not yet been
     * scheduled or has completed.
     *
     * Transitions to #ST_scheduled on a schedule() operation. */
    ST_unscheduled,

    /** Alarm is in the queue to execute at some point in the
     * future.
     *
     * Transitions to #ST_ready (asynchronously) or #ST_cancelled
     * (through cancel()). */
    ST_scheduled,

    /** Alarm deadline has been reached and the callback will
     * soon be invoked.
     *
     * If any alarm is in this state the uptime FLIH is executing.
     *
     * Transitions to #ST_in_callback (asynchronously) or
     * #ST_cancelled (through cancel()). */
    ST_ready,

    /** The callback is being invoked.
     *
     * If any alarm is in this state the uptime FLIH is executing.
     *
     * Transitions to #ST_scheduled or #ST_unscheduled based on
     * #callback_type return value. */
    ST_in_callback,

    /** The alarm has been cancelled.
     *
     * No transitions unless schedule() is re-invoked. */
    ST_cancelled,
  };

  /** The current state of the alarm. */
  state_type state () const
  {
    return state_;
  }

  /** Return `true` if the alarm state is one of #ST_scheduled,
   * #ST_ready, or #ST_in_callback.
   *
   * If `false` the alarm must be scheduled. */
  bool active () const
  {
    auto const s = state();
    return (ST_scheduled <= s) && (s <= ST_in_callback);
  }

  /** The signature of an alarm callback function.
   *
   * In addition to performing other operations the callback may
   * change the deadline() of the alarm and, through the return value,
   * control whether the alarm is rescheduled.  If interval() is not
   * zero the deadline() when the callback is invoked is interval()
   * ticks past the deadline that caused the callback to be invoked.
   *
   * @param alarm reference to the alarm associated with the callback.
   * This may be cast to a subclass type to access
   * callback/alarm-specific data.
   *
   * @return @c true if the alarm should be rescheduled, otherwise @c
   * false.
   *
   * @warning Callbacks are invoked from the uptime RTC FLIH, so
   * should do their thing and exit as fast as possible.  Interrupts
   * are not disabled while alarm callbacks are being processed. */
  using callback_type = std::function<bool(alarm& alarm)>;

  /* You can't move or copy these. */
  alarm (const alarm&) = delete;
  alarm& operator= (const alarm&) = delete;
  alarm (alarm&&) = delete;
  alarm& operator= (alarm&&) = delete;

  /** Standard alarm constructor.
   *
   * @param callback the function to be invoked when the alarm fires.
   *
   * @param interval an interval that is added to the previous
   * deadline() prior to invoking @p callback, to simplify
   * rescheduling repeating alarms.  The effect of a callback that
   * converts to `false` is to reschedule the alarm if and only if
   * interval() is not zero.
   *
   * @param md optional pointer stored as #metadata. */
  alarm (callback_type callback,
         unsigned int interval = 0,
         void* md = 0) :
    callback_{callback},
    interval_{interval},
    metadata{md}
  { }

  /** Overload for common case of configuring with static callback.
   *
   * The alarm is configured without an initial interval.
   *
   * @param callback as usual
   *
   * @param md as usual
   *
   * @overload */
  alarm (callback_type callback,
         void* md) :
    alarm{callback, 0, md}
  { }

  /** Factory producing an alarm with a callback that sets an event.
   *
   * This is a fairly common need.  While you can do this with:
   *
   *     alarm clock::alarm{[&events](auto&) {
   *       events.set(EVT);
   *       return true;
   *     }};
   *
   * that's pretty verbose.  Attempts to push the lambda construction
   * down into a constructor end up pulling in malloc because the
   * resulting object requires external storage either for the values
   * it captures (reference to events, value of event, return value)
   * or because it has to make copy of the function object rather than
   * construct it in place (even if the constant values are captured
   * in template constructor parameters).
   *
   * The following has the same efficiency and is a little cleaner:
   *
   *    auto alarm = alarm::for_event<EVT, true>(events);
   *
   * @tparam EVT the event that is to be set.
   *
   * @tparam reschedule the value to be returned from the synthesized
   * alarm callback.
   *
   * @param events reference to the event set to which EVT will be
   * added when the alarm fires.
   *
   * @return a new alarm instance with the required callback */
  template <event_set::event_type EVT,
            bool reschedule = false>
  static alarm for_event (event_set& events)
  {
    return {[&events](auto&){
        events.set(EVT);
        return reschedule;
      }};
  }

  /** Duration-specified alarm constructor
   *
   * @param as with the standard construtor.
   *
   * @param interval an interval specified as a C++ standard
   * duration.
   *
   * @param md optional pointer stored as #metadata. */
  template <typename DurT,
            typename DurT::rep = 0>
  constexpr alarm (callback_type callback,
                   DurT interval,
                   void* md = 0) :
  alarm(callback, uptime::from_duration(interval), md)
  { }

  /** On destruction the alarm is cancelled */
  ~alarm ()
  {
    cancel();
  }

  /** Queue the alarm to execute at its current deadline().
   *
   * @note This operational is behaviorally identical to invoking
   * schedule_offset() with the signed parameter that would produce
   * deadline().  Consequently if the signed difference between
   * current time and deadline appears negative, the alarm may fire
   * "immediately".
   *
   * @warning Invoking this function when the alarm is in states
   * #ST_scheduled, #ST_ready, or #ST_in_callback will result in
   * undefined behavior. */
  void schedule ();

  /** Queue the alarm to execute at @p offset ticks from the
   * current time.
   *
   * This atomically sets the deadline to the current time plus @p
   * offset and invokes schedule().  If the offset is negative the
   * deadline will appear to be in the past and the alarm will fire
   * immediately (possibly before this function returns, if interrupts
   * are enabled).
   *
   * @note Alarms with a signed offset from current time that is less
   * than two ticks may be delayed up to two ticks to ensure reliable
   * timer configuration.  This delay is not reflected in the alarm
   * deadline().
   *
   * @param offset the time until the alarm should go off.
   *
   * @warning This function is subject to the state() constraints of
   * schedule().
   *
   * @see schedule() */
  void schedule_offset (int offset);

  /** Schedule an alarm using a `std::chrono` duration.
   *
   * @overload */
  template <typename DurT>
  void schedule_offset (DurT offset)
  {
    return schedule_offset(uptime::from_duration(offset));
  }

  /** Cancel a potentially-scheduled alarm.
   *
   * If the alarm is found in the scheduled or ready queue its state()
   * will be updated to #ST_cancelled.
   *
   * @return The state() of the alarm prior to its being cancelled.
   *
   * @note You may cancel an alarm in any state or context, but if the
   * returned state is #ST_in_callback the cancellation will be
   * superseded by the @link callback_type callback@endlink return
   * value.  The application should either re-issue the cancel or
   * somehow communicate its desires to the callback so it does not
   * request that the alarm be rescheduled. */
  state_type cancel ();

  /** An offset added to deadline() prior to invoking the
   * callback when the alarm fires. */
  unsigned int
  interval () const
  {
    return interval_;
  }

  /** Set the interval() in uptime ticks.
   *
   * @warning Invoking this function when the alarm is in states
   * #ST_scheduled or #ST_ready will result in undefined behavior. */
  alarm& set_interval (unsigned int interval)
  {
    interval_ = interval;
    return *this;
  }

  /** Set the interval() using a given duration. */
  template <typename DurT,
            typename DurT::rep = 0>
  alarm& set_interval (DurT interval)
  {
    return set_interval(uptime::from_duration(interval));
  }

  /** The value of (the low 32 bits of) uptime::now() at which
   * the alarm should fire. */
  unsigned int
  deadline () const
  {
    return deadline_;
  }

  /** Set the deadline for the alarm to fire.
   *
   * This function is generally invoked only from within a
   * #callback_type, to change the deadline of an alarm that will be
   * rescheduled.
   *
   * It may also be used to set the deadline of an unscheduled alarm
   * prior to invoking schedule(), as in the case where alarms do not
   * automatically reschedule and the desired interval must be offset
   * from the last deadline rather than the current time as with
   * schedule_offset().
   *
   * @param deadline the counter value at which the alarm will fire,
   * nominally when the low 32 bits of uptime::now() increment to @p
   * deadline.  Although this is an absolute value, the difference
   * from the current time will be interpreted as signed value when
   * schedule() is invoked (explicitly or implicitly).
   *
   * @warning Invoking this function when the alarm is in states
   * #ST_scheduled or #ST_ready will result in undefined behavior. */
  alarm& set_deadline (unsigned int deadline)
  {
    deadline_ = deadline;
    return *this;
  }

protected:
  /** @cond DOXYGEN_EXCLUDE */
  struct alarm_queue;
  friend alarm_queue;
  friend void ::RTC1_IRQHandler();
  /** @endcond */

  /** Calculate an ordinal for the alarm using the distance from @p
   * now until its deadline. */
  int ordinal_ (unsigned int now) const noexcept
  {
    return deadline_ - now;
  }

  callback_type const callback_;
  unsigned int deadline_ = 0;
  unsigned int interval_;
  state_type volatile state_ = ST_unscheduled;
  alarm* next_ = nullptr;
  static alarm_queue queue_;

public:
  /** Pointer to arbitrary data associated with the alarm
   *
   * This exists because in modern C++ you can't generally pull the
   * trick of using `offsetof(struct, field)` to derive a pointer to a
   * structure from a pointer to a member of that structure.  Such a
   * cast is only allowed when the structure is a POD or has standard
   * layout, which excludes anything that has non-static data members
   * that are references.  As @link callback_type callbacks@endlink
   * are passed pointers to alarms, and the operation of the callback
   * may require access to sibling members, casting this pointer
   * instead provides a solution:
   *
   *     bool my_callback (clock::alarm& alarm)
   *     {
   *        auto self = static_cast<my_container*>(alarm.metadata);
   *        self->events.set(self.event);
   *        return false;
   *     }
   */
  void* const metadata;
};

/** Support for converting between time domains with the same
 * resolution but unstable clocks.
 *
 * This class is designed to support conversion from the local uptime
 * clock (32 KiHz ticks since power-up) to civil time (32 KiHz ticks
 * since the POSIX epoch 1970-01-01T00:00:00Z).  It operates on
 * synchronizations composed of matched samples of civil time and
 * local time.
 *
 * The concept is that until a civil time sample is available the
 * converted time will remain the uptime clock.  Once a
 * synchronization is available, the uptime clock will be shifted
 * based on the offset from that sample.  Once a second
 * synchronization is available the relative rate difference between
 * clocks is recorded and used to adjust the transformation.
 *
 * The goal here is to get something reasonably close to civil time on
 * the device, "good enough" to locally retain windowed sample
 * statistics (min/max/mean) that are roughly aligned with civil time
 * boundaries (e.g. 1 minute or 1 h windows).  Errors on the order of
 * a few seconds are assumed to be tolerable, so we don't get all
 * fancy with multiple sources. */
class clock_shift
{
public:
  /** Type for unsigned 32 KiHz ticks since the epoch of a time
   * system.
   *
   * This type is selected to support a 17 Ma era, and we know that
   * civil time will always be greater than local time, so we're not
   * going to fuss about the potential of integer overflow or
   * underflow. */
  using time_type = uint64_t;

  /** The signed version of #time_type.
   *
   * Used solely for the nominally unsupported case of converting a
   * time that was prior to the latest synchronization. */
  using diff_type = std::make_signed<time_type>::type;

  /** The implicit denominator of rate(). */
  static constexpr unsigned int RATE_DENOM = (1U << 16);

  /** Provide a new synchronization pair.
   *
   * @param now_ctt a timestamp in the civil time domain.  This had
   * better always be positive, and should never go backwards.
   *
   * @param now_utt the local domain time corresponding to @p now_ctt.
   * This value should never go backwards between invocations. */
  void update_sync (time_type now_ctt,
                    time_type now_utt) noexcept;

  /** Indicates whether a synchronization point has been provided.
   *
   * Until this returns `true` convert_ctt() will simply return its
   * argument. */
  bool synchronized () const noexcept
  {
    return (0 != sync_ctt_);
  }

  /** Best effort conversion from local time to civil time.
   *
   * @param now_utt the local time for which civil time is desired.
   *
   * @return the civil time corresponding to @p now_utt based on the
   * most recent synchronization data.
   *
   * @warning This system does not guarantee monotonic converted
   * timestamp.  That is, update_sync() can change the offset and rate
   * so that an invocation of `convert_ctt(t)` produced a value
   * greater than `convert_ctt(t+d)` does `d` time units later. */
  time_type convert_ctt (time_type now_utt) noexcept;

  /** The differential rate of the civil time clock relative to the
   * local clock.
   *
   * This is the numerator of a fractional value using #RATE_DENOM as
   * the implicit denominator. */
  unsigned int rate () const noexcept
  {
    return rate_;
  }

  /** The nominal offset between the civil time and local clocks at
   * the time of the last synchronization point. */
  time_type offset () const noexcept
  {
    return sync_ctt_ - sync_utt_;
  }

private:
  time_type sync_ctt_{};
  time_type sync_utt_{};
  unsigned int rate_ = RATE_DENOM;
};

} // namespace clock
} // namespace nrfcxx

#endif /* NRFCXX_CLOCK_HPP */
