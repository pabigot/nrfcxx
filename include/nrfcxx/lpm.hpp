/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Material supporting low-power-mode operations.
 * @file */

#ifndef NRFCXX_LPM_HPP
#define NRFCXX_LPM_HPP
#pragma once

#include <nrfcxx/clock.hpp>

namespace nrfcxx {

/** Infrastructure to support low-power-mode operations */
namespace lpm {

/** State machine abstraction for time-delayed transitions and error
 * captures.
 *
 * An instance of this can be used to coordinate multi-step processes
 * that are often executed in the background.  A 24-bit task-specific
 * state is retained by the instance.  Bits in the state indicate
 * whether the process has an error and whether it is blocked waiting
 * for an alarm.
 *
 * An uptime-based alarm infrastructure is also maintained, using
 * nrfcxx::event_set to notify the application when the alarm has
 * completed.  Fact-of a pending alarm is recorded in the state value,
 * allowing premature entry into the state transition manager to be
 * ignored by the manager.
 *
 * A 32-bit error code can be set in the state machine.  A cooperative
 * manager will respect the error and wait until reset() is invoked
 * before taking steps to restart the machine. */
class state_machine
{
public:
  /** Representation for both generic and specific machine state.
   *
   * The low 24 bits of this are available for specific machines to
   * use to encode their state.  The upper 8 bits are reserved for
   * infrastructure support such as @link STATE_HAS_ERROR error
   * markers@endlink and @link STATE_BLOCKING_ALARM delayed
   * transitions@endlink. */
  using state_type = unsigned int;

  /** Representation for flags returned from lpsm_capable::lpsm_process().
   *
   * Flags at and above #PF_APP_BASE are available to the
   * application. */
  using process_flags_type = unsigned int;

  /** Representation for values stored in error().
   *
   * Error values are simply negative values returned by
   * lpsm_capable::lpsm_process_(). */
  using error_type = int;

  /** Mask isolating the bits of #state_type that are available
   * to record machine state.
   *
   * Bits outside this range are reserved for shared machine
   * operations. */
  static constexpr state_type STATE_MASK = ((1U << 24) - 1);

  /** Bit set in #state_type when the machine is in an error state.
   *
   * This bit may be set in combination with any state.  When in an
   * error state the controlling code should invoke reset() to restore
   * the state.  If this bit is set invocation of
   * lpsm_capable::lpsm_process() will return zero without invoking to
   * lpsm_capable::lpsm_process_(). */
  static constexpr state_type STATE_HAS_ERROR = (1U << 24);

  /** Bit set in #state_type when an @link set_delay alarm@endlink
   * guarding entry to the configured state has not yet fired.
   *
   * A blocking alarm should be used when there is a required delay
   * before the machine can progress with the configured state.  An
   * example would be when processing #EXIT_RESET after a hardware
   * reset: the machine should take no action until the required reset
   * backoff has passed.  Invocation of lpsm_capable::lpsm_process()
   * while the alarm is still pending will return a diagnostic error.
   *
   * This bit will be set by lpsm_capable::lpsm_process() when @p
   * delay has been set to a positive value by
   * lpsm_capable::lpsm_process_(). */
  static constexpr state_type STATE_BLOCKING_ALARM = (1U << 25);

  /** Bit set in #state_type when an @link set_delay alarm@endlink
   * triggering post_event() has not yet fired.
   *
   * A fallback alarm should be used when a normal cue to process the
   * machine state may be missed, e.g. a device that normally signals
   * data ready through a GPIO signal, but where that signal may not
   * be available.  On expiration of the alarm post_event() will be
   * invoked.  Invocation of lpsm_capable::lpsm_process() while the
   * alarm is still pending will cause the alarm to be cancelled, and
   * the machine will process.  If the machine determines that it
   * cannot proceed yet, it may set another alarm to try again.
   *
   * This bit will be set by lpsm_capable::lpsm_process() when @p
   * delay has been set to a positive value by
   * lpsm_capable::lpsm_process_(). */
  static constexpr state_type STATE_FALLBACK_ALARM = (1U << 26);

  /** Bit set in #state_type when lpsm_capable::lpsm_stop() has been
   * invoked by the application to request a shutdown. */
  static constexpr state_type STATE_STOP_PENDING = (1U << 26);

  /** State when the machine is off and all resources associated with
   * it are disabled.
   *
   * Transition from this state is performed by invoking
   * lpsm_capable::lpsm_start().  The destination state is
   * #MS_ENTRY_START. */
  static constexpr state_type MS_OFF = 0x0000;

  /** State indicating the machine should begin automated
   * transitions. */
  static constexpr state_type MS_ENTRY_START = 0x0001;

  /** State indicating the machine entered processing in a state not
   * supported by the machine.
   *
   * This should be invoked in the `default` case of a switch
   * statement processed in lpsm_capable::lpsm_process_().  The effect
   * includes use of set_error() to record the unrecognized state. */
  static constexpr state_type MS_LOST = 0x0003;

  /** State available to indicate that the machine needs to executing
   * an internal reset operation. */
  static constexpr state_type MS_ENTRY_RESET = 0x0010;

  /** State available to indicate that the machine is waiting to
   * complete an internal reset operation.
   *
   * This is likely to follow #MS_ENTRY_RESET after a delay.
   * Concurrent with transition from this state #PF_RESET should be
   * returned from lpsm_capable::lpsm_process(). */
  static constexpr state_type MS_EXIT_RESET = 0x0011;

  static constexpr state_type MS_ENTRY_ERRORED = 0x0012;
  static constexpr state_type MS_ERRORED = 0x0013;
  static constexpr state_type MS_ENTRY_FAILED = 0x0014;
  static constexpr state_type MS_FAILED = 0x0015;
  static constexpr state_type MS_ENTRY_STOPPED = 0x0016;

  /** States available to guard entry to a setup state.
   *
   * For example when #MS_ENTRY_START initiates a peripheral
   * configuration that requires a 10 ms delay, #MS_ENTRY_SETUP might
   * be used as the target to initiate a second configuration step.
   *
   * Up to 15 additional instances may be defined by the application
   * to support complex setup processes that require multiple guarded
   * entry conditions. */
  static constexpr state_type MS_ENTRY_SETUP = 0x0020;

  /** States available to guard exit from a setup step.
   *
   * Similar to #MS_ENTRY_SETUP or #MS_ENTRY_SAMPLE in cases where the
   * operation is more clearly associated with the source state.
   * Pairs with #MS_ENTRY_SETUP with same low nybble. */
  static constexpr state_type MS_EXIT_SETUP = 0x0030;

  /** State indicating the machine is in a functional state from which
   * an application signal is required to make further transitions.
   *
   * Generally you'd use this if the machine does not internally
   * initiate new observations.  Up to 15 additional instances may be
   * defined by the application to support multiple idle states; these
   * are generally paired with the #MS_SAMPLE instance sharing a low
   * nybble. */
  static constexpr state_type MS_IDLE = 0x0040;

  /** States available to guard entry to a sample state.
   *
   * For example, when waiting for a signal that a sampling resource
   * is available.  Pairs with the #MS_SAMPLE instance sharing a low
   * nybble. */
  static constexpr state_type MS_ENTRY_SAMPLE = 0x0050;

  /** States available to implement a sample state.
   *
   * A machine is in a sample state when it has initiated an
   * observation.
   *
   * Up to 15 additional instances may be defined by the application
   * to support multiple sample states. */
  static constexpr state_type MS_SAMPLE = 0x0060;

  /** States available to guard exit from a sample state.
   *
   * Similar to #MS_ENTRY_SAMPLE in cases where the operation is
   * more clearly associated with the source state.  For example, when
   * waiting for a result to become available.
   *
   * Pairs with the #MS_SAMPLE instance sharing a low nybble. */
  static constexpr state_type MS_EXIT_SAMPLE = 0x0070;

  /** Base state value available for application-specific states. */
  static constexpr state_type MS_APP_BASE{0x100};

  /** lpsm_capable::lpsm_process() flag bit when the machine is turned
   * off.
   *
   * This is set in any condition when, on exit from
   * lpsm_capable::lpsm_process(), the machine is in state #MS_OFF. */
  static constexpr process_flags_type PF_OFF = 0x01;

  /** lpsm_capable::lpsm_process() flag bit when the machine is ready
   * to provide observations.
   *
   * This should be emitted once per start, on successful completion
   * of all setup steps performed by the machine.
   *
   * An application that maintains a persisted dynamic calibration
   * might use this as a cue to when the calibration should be made
   * available to the machine, or installed in the sensor. */
  static constexpr process_flags_type PF_STARTED = 0x02;

  /** lpsm_capable::lpsm_process() flag bit when the machine has
   * initiated a peripheral reset.
   *
   * For example a CCS811 found to be operating in the wrong drive
   * mode needs a clean reset before programming.  This bit indicates
   * that the reset is in progress.
   *
   * This is so the application can track such resets, which may
   * result in delays before valid observations are received or
   * discontinuous observation curves.  The process of reset
   * completion is managed by the machine. */
  static constexpr process_flags_type PF_RESET = 0x04;

  /** Bit set in non-negative lpsm_capable::lpsm_process() result when
   * the machine is in an unrecoverable failed state.
   *
   * Presence of this is sticky, and the system should mark the sensor
   * as nonfunctional. */
  static constexpr process_flags_type PF_FAILED = 0x08;

  static constexpr process_flags_type PF_STOPPED = 0x10;

  /** lpsm_capable::lpsm_process() flag bit when a new observation is
   * available. */
  static constexpr process_flags_type PF_OBSERVATION = 0x20;

  /** First lpsm_capable::lpsm_process() flag bit available for
   * application-specific result code bits.
   *
   * If multiple result codes are needed shift each one left by one
   * more bit. */
  static constexpr process_flags_type PF_APP_BASE = 0x100;

  /** @cond DOXYGEN_EXCLUDE */
  /* You can't move or copy these. */
  state_machine (const state_machine&) = delete;
  state_machine& operator= (const state_machine&) = delete;
  state_machine (state_machine&&) = delete;
  state_machine& operator= (state_machine&&) = delete;
  /** @endcond */

  /** Create a state machine that records state and supports
   * delayed transitions.
   *
   * @param notify the event setter through which the application is
   * notified of transitions. */
  state_machine (notifier_type notify) :
    notify_{notify},
    alarm_{alarm_callback_, this}
  { }

  /** Cancel any pending transitions and restore the @link
   * state state@endlink to its as-constructed value. */
  void reset();

  /** Return the current machine state excluding error/pending flags.
   *
   * This is appropriate for comparing against specific states.
   *
   * On construction the state is zero. */
  state_type state () const
  {
    return state_ & STATE_MASK;
  }

  /** Return the current machine state including error/pending flags.
   *
   * This is appropriate for testing whether invocation of
   * lpsm_capable::lpsm_process() had any machine-visible side effects
   * (e.g., setting an alarm).
   *
   * On construction the state is zero. */
  state_type full_state () const
  {
    return state_;
  }

  /** Set the machine state
   *
   * @param state the state of the machine.  In most cases the value
   * should not include any error/alarm/pending flags.
   *
   * @param post if `true` invoke post_event() immediately. */
  void set_state (state_type state,
                  bool post = false)
  {
    state_ = state;
    if (post) {
      post_event();
    }
  }

  /** `true` iff the current state is marked with
   * #STATE_HAS_ERROR.
   *
   * This method should be invoked after any call to
   * lpsm_capable::lpsm_process() to determine whether the machine has
   * recorded an error state. */
  bool has_error () const
  {
    return STATE_HAS_ERROR & state_;
  }

  /** Mark the current state with #STATE_HAS_ERROR.
   *
   * This API does not change the value stored in error(). */
  void set_error ()
  {
    state_ |= STATE_HAS_ERROR;
  }

  /** Mark the current state with #STATE_HAS_ERROR.
   *
   * @param error the error code to be stored. */
  void set_error (error_type error)
  {
    state_ |= STATE_HAS_ERROR;
    error_ = error;
  }

  /** Set the current state as #MS_LOST and store the unrecognized
   * state in error(). */
  void set_lost ();

  /** Read the current error code.
   *
   * Error values are most often negative values returned by the
   * machine-specific lpsm_capable::lpsm_process_().  The specific
   * interpretation is not generally defined, but in many cases these
   * will indicate a @link
   * periph::details::comm_error_support::error_decoded TWI/SPI
   * error@endlink.
   *
   * Error values may also be set in context with #MS_LOST, where the
   * value is the erroneous state.
   *
   * Specific machines may also indicate other problems through
   * setting an error.  Again, the interpretation is not specified.
   *
   * The error value is cleared on reset(), and set by
   * lpsm_capable::lpsm_process() through set_error(error_type). */
  error_type error () const
  {
    return error_;
  }

  /** `true` iff the current state is marked with
   * #STATE_BLOCKING_ALARM. */
  bool blocking_alarm_pending () const
  {
    return STATE_BLOCKING_ALARM & state_;
  }

  /** `true` iff the current state is marked with
   * #STATE_FALLBACK_ALARM. */
  bool fallback_alarm_pending () const
  {
    return STATE_FALLBACK_ALARM & state_;
  }

  /** Use the constructor-provided notifier to inform the application
   * that the machine needs to be serviced. */
  void post_event ()
  {
    notify_();
  }

  /** Read the @link clock::alarm::deadline deadline@endlink for
   * the most recently scheduled alarm. */
  int deadline () const
  {
    return alarm_.deadline();
  }

  /** Set an alarm to invoke post_event() after a delay.
   *
   * @param delay the duration to wait.  A positive value indicates a
   * @link STATE_BLOCKING_ALARM blocking@endlink alarm.  A negative
   * value indicates a @link STATE_FALLBACK_ALARM fallback@endlink
   * alarm.  In either case the absolute value of the delay will be
   * used as the parameter to
   * clock::alarm::schedule_offset(). */
  void set_delay (int delay_utt);

  /** Cancel any delay initiated by set_delay(). */
  void cancel_delay ();

protected:
  static bool alarm_callback_ (clock::alarm& alarm);

private:
  notifier_type notify_;
  clock::alarm alarm_;
  state_type state_ = 0;
  error_type error_ = 0;
};

/** Base (or mixin) class for anything that supports a state_machine.
 *
 * This provides ownership of the machine, access to it, and the
 * documentation and default implementation of the
 * application-accessible machine operations. */
class lpsm_capable
{
  int process_rc_ = 0;
protected:
  /** Create a state machine that records state and supports
   * delayed transitions.
   *
   * @param notify as with lpsm::state_machine. */
  lpsm_capable (notifier_type notify) :
    machine_{notify}
  { }

  state_machine machine_;

  using state_type = state_machine::state_type;
  using process_flags_type = state_machine::process_flags_type;

  /** Override to implement machine-specific operations of lpsm_process().
   *
   * The wrapper confirms that the machine is not in off, error, or
   * failed state before invoking this function.  If an error is
   * returned the wrapper puts the machine into an error state;
   * otherwise it sets up any requested delay and returns the process
   * flags to its caller.
   *
   * Applications may observe the last returned value through the
   * lpsm_process_rc() API.
   *
   * @param[out] delay set in the call to indicate that
   * state_machine::post_event() should be invoked in support of
   * either a @link state_machine::STATE_BLOCKING_ALARM
   * blocking@endlink or @link state_machine::STATE_FALLBACK_ALARM
   * fallback@endlink delay.
   *
   * @param[out] pf process flags to return to the caller.
   *
   * @return a negative value to mark the machine as being in an error
   * state.  Return non-negative if the machine was successful. */
  virtual int lpsm_process_ (int& delay,
                             process_flags_type& pf)
  {
    return -1;
  }

  /** Override to reset state that is held outside the machine. */
  virtual void lpsm_reset_ ()
  { }

public:
  virtual ~lpsm_capable () = default;

  /** Validate and prepare to initiate an LPM collection.
   *
   * This should be invoked to transfer a machine from
   * state_machine::MS_OFF.  The default implementation initiates a
   * move to state_machine::MS_ENTRY_START.
   *
   * @note Invoking this merely validates and prepares the state
   * machine for execution.  To initiate the collection lpsm_process()
   * must be invoked consequent to detecting the @link
   * lpm::state_machine::post_event need-to-process@endlink event.
   *
   * @note Subclasses may override or post-extend superclass
   * implementations.
   *
   * @return zero on success, or a negative error code if starting the
   * machine is not permitted. */
  virtual int lpsm_start ();

  /** Ask the LPM infrastructure to initiate a new sample.
   *
   * The default implementation confirms that the machine is in
   * state_machine::MS_IDLE and initiates a transition to
   * state_machine::MS_ENTRY_SAMPLE.
   *
   * @note This function is only used for machines that don't
   * automatically generate new readings through an internal interval
   * or event, such as sensor::sht21 or sensor::hts221 in one-shot
   * mode.  The declaration may be hidden on machines where it cannot
   * be used.
   *
   * @return zero on success, or a negative error code if the sample
   * could not be initiated.  An error return does not affect the
   * machine state. */
  virtual int lpsm_sample ();

  /** Make progress on an LPM collection.
   *
   * This function should be invoked when the application detects that
   * the @link lpm::state_machine::post_event need-to-process@endlink
   * signal has been set.
   *
   * @return a set of flags such as
   * lpm::state_machine::PF_OBSERVATION.  If the machine is in an
   * error state the returned value has no flags set. */
  lpm::state_machine::process_flags_type lpsm_process ();

  /** @todo figure out how machines should be stopped */
  int lpsm_stop ()
  {
    return -1;
  }

  int lpsm_reset ()
  {
    /* This is not functionally complete, but is stubbed to trace the
     * dependence between lpsm_capable and its subclasses for clearing
     * external state such as flags not maintained in the state
     * machine. */
    lpsm_reset_();
    machine_.reset();
    return 0;
  }

  /** Get the internal result code from the last invocation of
   * lpsm_process_().
   *
   * This may be useful for diagnostics, but the value has no generic
   * interpretation. */
  int lpsm_process_rc () const
  {
    return process_rc_;
  }

  /** Gain read-only access to the LPM machine state. */
  const lpm::state_machine& machine () const
  {
    return machine_;
  }

};

} // ns lpm
} // ns nrfcxx

#endif /* NRFCXX_LPM_HPP */
