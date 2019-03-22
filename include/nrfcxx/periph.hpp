/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Abstraction of Nordic device peripherals.
 * @file */

#ifndef NRFCXX_PERIPH_HPP
#define NRFCXX_PERIPH_HPP
#pragma once

#include <functional>

#include <pabigot/container.hpp>

#include <nrfcxx/impl.hpp>

namespace nrfcxx {

/** Abstractions of nRF51 peripherals */
namespace periph {

namespace {
/** @cond DOXYGEN_EXCLUDE */
static constexpr uint32_t gpiote_config_from_psel (unsigned int psel)
{
  return 0
#if (NRF52840 - 0)
    | (GPIOTE_CONFIG_PORT_Msk & (psel << GPIOTE_CONFIG_PSEL_Pos))
#endif /* NRF52840 */
    | (GPIOTE_CONFIG_PSEL_Msk & (psel << GPIOTE_CONFIG_PSEL_Pos))
    ;
}
/** @endcond */
} // ns anonymous

#ifndef NRFCXX_PERIPH_UART0_RXB_SIZE
/** The length of the software FIFO holding UART::UART0
 * received data */
#define NRFCXX_PERIPH_UART0_RXB_SIZE 8
#endif /* NRFCXX_PERIPH_UART0_RXB_SIZE */

#ifndef NRFCXX_PERIPH_UART0_TXB_SIZE
/** The length of the software FIFO holding UART::UART0
 * untransmitted data */
#define NRFCXX_PERIPH_UART0_TXB_SIZE 32
#endif /* NRFCXX_PERIPH_UART0_TXB_SIZE */

/** Wrapper around the nRF51 UART peripheral.
 *
 * This provides interrupt-driven (only) transmission and reception.
 *
 * @note This module does not configure the RTS and CTS GPIOs.  If
 * hardware flow control is enabled RTS should be configured through
 * GPIO logic high to notify connected devices that there is no
 * listener during periods when the UART is disabled.  When the UART
 * is enabled RTS will be controlled by it, based on whether there is
 * space in the peripheral receive buffer. */
class UART
{
public:
  /** The type used for transmission and reception buffers */
  using fifo_type = pabigot::container::rr_adaptor<uint8_t>;
  /** The type used for transfer sizes (unsigned) */
  using size_type = unsigned int;
  /** The type used for transfer sizes or errors (signed) */
  using ssize_type = int;

  /** Statistics on the UART operation */
  struct statistics_type
  {
    /** The number of octets transmitted */
    unsigned int tx_count;

    /** The number of octets received */
    unsigned int rx_count;

    /** The number of received octets dropped due to lack of
     * buffer space. */
    unsigned int rx_dropped;

    /** The number of break errors detected */
    unsigned int rx_break_errors;

    /** The number of frame errors detected */
    unsigned int rx_frame_errors;

    /** The number of parity errors detected */
    unsigned int rx_parity_errors;

    /** The number of overrun errors detected */
    unsigned int rx_overrun_errors;
  };

  /** Enable the UART.
   *
   * @note Use of autoenable() may cause the device to be disabled during
   * periods when there is no transmission pending.
   *
   * @param cfg_baudrate the NRF51 BAUDRATE.BAUDRATE field value from
   * the Nordic headers.  Note that the value is not numerically
   * equivalent to the baud rate.  An explicit zero value selects the
   * default baud rate.
   *
   * @param hwfc @c true iff hardware flow control should be enabled.
   * The value is ignored if either RTS or CTS pins are undefined. */
  void enable (uint32_t cfg_baudrate = UART_BAUDRATE_BAUDRATE_Baud115200,
               bool hwfc = false);

  /** Disable the UART. */
  void disable ();

  /** Indicate whether UART is enabled.
   *
   * @param live if `true` this returns the actual state of the
   * peripheral.  If `false` this disregards a disable due to
   * autoenable(). */
  bool enabled (bool live = false) const;

  /** Query or control whether the UART self-enables for output.
   *
   * By default the UART is entirely controlled by enable().  If
   * auto-enable is requested the UART will be disabled internally
   * except when there is material pending transmission.
   *
   * An explicit use invocation of enable() or disable() turns off
   * auto-enable.
   *
   * @param on positive to turn on auto-enable; zero to turn off
   * auto-enable; negative to return the current state without
   * changing it.
   *
   * @return `true` if auto-enable is turned on, otherwise `false`. */
  bool autoenable (int on);

  /** Write data to the UART.
   *
   * @param sp pointer to the source of the data
   *
   * @param count the number of octets to be written
   *
   * @return the number of octets actually written.  This may be less
   * than @p count if the transmit buffer lacks space. */
  ssize_type write (const uint8_t* sp, size_type count);

  /** Read data from the UART.
   *
   * @param dp pointer to where received octets should be written
   *
   * @param count the maximum number of octets to be read
   *
   * @return the number of octets actually read.  This may be less
   * than @p count if the receive buffer does not have the requested
   * amount of data. */
  ssize_type read (uint8_t* dp, size_type count);

  /** Get a snapshot of the UART statistics */
  statistics_type statistics () const;

  /** Event set in events() when data is received */
  static constexpr event_set::event_type EVT_RXAVAIL = 0x01;

  /** Event set in events() when there is space in the transmit buffer */
  static constexpr event_set::event_type EVT_TXAVAIL = 0x02;

  /** Event set in events() when transmission completes.
   *
   * This event does not occur when the transmit buffer is emptied,
   * but only when it remains empty after the UART has transmitted the
   * last byte. */
  static constexpr event_set::event_type EVT_TXDONE = 0x04;

  /** Event set in events() when an error is detected.
   *
   * Specifically a hardware error; dropped received data is not
   * detected through this event. */
  static constexpr event_set::event_type EVT_ERROR = 0x04;

  /** Reference the UART events.
   *
   * @see EVT_RXAVAIL
   * @see EVT_TXAVAIL
   * @see EVT_TXDONE
   * @see EVT_ERROR */
  event_set& events ();

  /** A reference to a board-specific standard UART instance */
  static UART UART0;

  /** Implementation for `UART#_IRQHandler` required by this module.
   *
   * This class requires that the `UART#_IRQHandler` process events to
   * fulfill its obligations.  At this time the handler is implicitly
   * installed for UART0.
   */
  void irq_handler ();

  /** Reference the abstraction instance for UART0. */
  static UART& instance ();

  /** Reference the nRF5 UART peripheral instance used by the
   * abstraction. */
  const nrf5::UART_Type& peripheral () const
  {
    return uart_;
  }

protected:
  UART (const nrf5::UART_Type& uart,
        fifo_type& rxb,
        fifo_type& txb,
        int rxd_pin,
        int txd_pin,
        int cts_pin = -1,
        int rts_pin = -1) :
    uart_{uart},
    rxb_{rxb},
    txb_{txb},
    rxd_pin_(rxd_pin), // narrowing conversion
    txd_pin_(txd_pin),
    cts_pin_(cts_pin),
    rts_pin_(rts_pin)
  { }

private:
  void set_enabled_bi_ (bool on,
                        bool from_autoenable);

  const nrf5::UART_Type& uart_;
  fifo_type& rxb_;
  fifo_type& txb_;
  statistics_type statistics_{};
  event_set events_{};
  uint8_t volatile flags_ = 0;
  int8_t const rxd_pin_;
  int8_t const txd_pin_;
  int8_t const cts_pin_;
  int8_t const rts_pin_;
};

/** Wrapper around the nRF51 RTC peripheral.
 *
 * @note When a soft device is used RTC0 is restricted to the soft
 * device.  RTC1 is available to applications, but is probably taken
 * by clock::uptime. */
class RTC
{
protected:
  constexpr RTC (const nrf5::RTC_Type& rtc) :
    rtc_{rtc}
  { }

public:
  /** Modulus for the 24-bit RTC counter */
  constexpr static uint32_t counter_modulus = (1U << 24);

  /** Mask for the 24-bit RTC counter */
  constexpr static uint32_t counter_mask = (counter_modulus - 1);

  /** Calculate the tick-count between two counter values.
   *
   * @return The value that when added to counter value @p a would
   * produce counter value @p b.
   *
   * @note @p a and @p b are assumed to be 24-bit counter values; if
   * bits are set outside #counter_mask the result may be
   * incorrect. */
  constexpr static unsigned int counter_delta (uint32_t a,
                                               uint32_t b)
  {
    /* This relies on standard semantics for unsigned subtraction to
     * handle the case where b < a. */
    return counter_mask & (b - a);
  }

  /** Return the underlying 24-bit counter value */
  uint32_t counter () const
  {
    return rtc_->COUNTER;
  }

  /** Reference the abstraction instance for a specific
   * peripheral instance.
   *
   * @param idx the peripheral instance desired.
   *
   * @warning If `idx` specifies a peripheral instance that does not
   * exist on the device the system will reset into failsafe mode with
   * FailSafeCode::NO_SUCH_PERIPHERAL. */
  static RTC& instance (int idx);

  /** Reference the nRF5 RTC peripheral instance used by the
   * abstraction. */
  const nrf5::RTC_Type& peripheral () const
  {
    return rtc_;
  }

  /** The number of capture/compare registers on the device */
  const unsigned int ccr_count () const
  {
    return rtc_.AUX;
  }

private:
  const nrf5::RTC_Type& rtc_;
};

/** Wrapper around the nRF5 TIMER peripheral.
 *
 * This abstraction supports using specific timers in timer mode.
 * Timer IRQ handler definitions and interrupt configuration, if
 * necessary, are the responsibility of the application, as is
 * allocation of capture/compare registers to specific roles.
 *
 * @note When a soft device is used TIMER0 is restricted to the soft
 * device.  Other TIMER instances are available to applications. */
class TIMER
{
protected:
  constexpr TIMER (const nrf5::TIMER_Type& timer) :
    timer_{timer},
    mask_{}
  { }

  /** Capture the current counter into CC @p ccidx (unvalidated). */
  void capture_ (unsigned int ccidx) const
  {
    timer_->TASKS_CAPTURE[ccidx] = 1;
  }

  /** Return the captured value in CC @p ccidx (unvalidated). */
  unsigned int captured_ (unsigned int ccidx) const
  {
    return timer_->CC[ccidx];
  }

  /** Capture and return the current counter using @p ccidx (unvalidated). */
  unsigned int counter_ (unsigned int ccidx) const
  {
    capture_(ccidx);
    return captured_(ccidx);
  }

public:
  /** Mask to create a valid CC index from a non-negative
   * integer.
   *
   * This allows up to 8 capture/compare registers.  Not all registers
   * are available on all timers. */
  constexpr static unsigned int cc_mask = 7;

  /** Verify a specific timer can support the requested bitmode.
   *
   * @note The user is responsible for ensuring that @p timer
   * references an existing nRF5x TIMER peripheral and that @p bitmode
   * is a valid setting for the timer `BITMODE` register. */
  constexpr static bool bitmode_supported (const nrf5::TIMER_Type& timer,
                                           unsigned int bitmode)
  {
#if (NRF51 - 0)
    /* nRF51 timers 1 and 2 do not support 24- and 32-bit modes */
    return ((TIMER_BITMODE_BITMODE_24Bit > bitmode)
            || (0 == timer.INSTANCE));
#else /* SERIES */
    /* All nRF52832 timers support all modes. */
    return true;
#endif /* SERIES */
  }

  /** Configure the TIMER to run as a timer.
   *
   * All interrupts are cleared and disabled, and the timer counter is
   * cleared.  Configuration is performed as specified by the
   * parameters.
   *
   * @param freq_Hz the desired frequency of the timer.  The value is
   * limited by clock::hfclk::Frequency_Hz, and the actual
   * value is the largest power-of-2 divisor of that frequency that
   * does not exceed @p freq_Hz.  Zero may be passed to deconfigure
   * the TIMER.
   *
   * @param bitmode the `NRF_TIMER_Type::BITMODE.BITMODE` setting
   * controlling the range for timer counter values.
   *
   * @param use_constlat pass @c true to ensure that
   * NRF_POWER->TASKS_CONSTLAT is in force as long as the timer is
   * configured; pass @c false if the timer can tolerate running in
   * NRF_POWER->TASKS_LOWPWR mode.  If @p freq_Hz is zero any
   * previously-specified constant latency dependency is removed.
   *
   * @return 0 if configuration is successful, or -1 if a parameter is
   * invalid.
   *
   * @note The timer must be @link start started@endlink before it is
   * useful.  Startup time may be reduced if @p use_constlat is @p
   * true. */
  int configure (unsigned int freq_Hz,
                 unsigned int bitmode = TIMER_BITMODE_BITMODE_16Bit,
                 bool use_constlat = true);

  /** Deconfigure the TIMER. */
  void deconfigure ()
  {
    (void)configure(0);
  }

  /** Get the configured clock frequency in Hz. */
  unsigned int frequency_Hz () const;

  /** Invoke TASKS_START. */
  void start ()
  {
    timer_->TASKS_START = 1;
  }

  /** Invoke TASKS_CLEAR. */
  void clear ()
  {
    timer_->TASKS_CLEAR = 1;
  }

  /** Invoke TASKS_STOP. */
  void stop ()
  {
    timer_->TASKS_STOP = 1;
  }

  /** Invoke TASKS_SHUTDOWN. */
  void shutdown ()
  {
    timer_->TASKS_SHUTDOWN = 1;
  }

  /** Capture the current counter using CC @p ccidx.
   *
   * @note This function intentionally does not return the captured
   * value, as reading it from the peripheral takes a couple clock
   * cycles which can negatively impact instrumentation accuracy.  If
   * the overhead impact on subsequent operations is not a concern,
   * use counter() to capture and return the current timer value. */
  void capture (unsigned int ccidx) const
  {
    capture_(cc_mask & ccidx);
  }

  /** Return the captured counter in CC @p ccidx. */
  unsigned int captured (unsigned int ccidx) const
  {
    return captured_(cc_mask & ccidx);
  }

  /** Read the current timer value.
   *
   * Since the timer counter is not exposed a capture/compare index
   * must be provided.  The caller is responsible for ensuring that @p
   * ccidx is not used for other purposes that would be disrupted by
   * invocations of counter().
   *
   * @return the full-precision counter value. */
  unsigned int counter (unsigned int ccidx = 0) const
  {
    return counter_(cc_mask & ccidx);
  }

  /** Calculate the count distance from @p a to @p b.
   *
   * The value is truncated based on the @p bitmode passed to
   * configure(). */
  unsigned int delta (unsigned int a,
                      unsigned int b) const
  {
    return mask_ & (b - a);
  }

  /** Class supporting (high-resolution) timing.
   *
   * On construction this captures the current value of the underlying
   * timer's counter.  delta() can be used to get the number of counts
   * since the captured counter.  delta_reset() is similar, but resets
   * the captured counter to the current time, allowing timing of
   * multiple steps of a process.
   *
   * @note The overhead of this mechanism is roughly twice the
   * overhead of separated invocations of the underlying
   * TIMER::capture() API (8 CPU clock cycles versus 4 clock cycles
   * when the captured value is not immediately accessed).
   *
   * @see clock::uptime::timestamp24 */
  class timestamp_type
  {
  protected:
    /** Reference to timer used for timestamp operations. */
    const TIMER& timer_;
    /** Validated capture/compare index used for timestamp operations. */
    unsigned int ccidx_;
    /** Captured counter value used for timestamp delta calculations. */
    unsigned int capture_;

    /** TIMER needs access to create instances of this type. */
    friend class TIMER;

    timestamp_type (const TIMER& timer,
                    unsigned int ccidx) :
      timer_{timer},
      ccidx_{cc_mask & ccidx},
      capture_{timer.counter(ccidx_)}
    { }

  public:
    /** Return the captured counter. */
    unsigned int captured () const
    {
      return capture_;
    }

    /** Capture the current time. */
    void reset ()
    {
      capture_ = timer_.counter(ccidx_);
    }

    /** Return counts since the captured value. */
    unsigned int delta () const
    {
      auto now = timer_.counter_(ccidx_);
      return timer_.delta(capture_, now);
    }

    /** Return counts between captured counter and @p counter. */
    unsigned int delta (unsigned int counter) const
    {
      return timer_.delta(capture_, counter);
    }

    /** Return counts since the captured time and reset
     * captured counter to current counter. */
    unsigned int delta_reset ()
    {
      auto now = timer_.counter_(ccidx_);
      auto rv = timer_.delta(capture_, now);
      capture_ = now;
      return rv;
    }
  };

  /** Create a timestamp instance using this timer.
   *
   * @param ccidx the capture/compare index to be used for counter
   * captures.
   *
   * @return a #timestamp_type object initialized to time activity
   * relative to the current counter value. */
  timestamp_type timestamp (unsigned int ccidx = 0) const
  {
    return timestamp_type{*this, ccidx};
  }

  /** Reference the abstraction instance for a specific
   * peripheral instance.
   *
   * @param idx the peripheral instance desired.
   *
   * @warning If `idx` specifies a peripheral instance that does not
   * exist on the device the system will reset into failsafe mode. */
  static TIMER& instance (int idx);

  /** Reference the nRF5 TIMER peripheral instance used by the
   * abstraction. */
  const nrf5::TIMER_Type& peripheral () const
  {
    return timer_;
  }

  /** The number of capture/compare registers on the device */
  const unsigned int ccr_count () const
  {
    return timer_.AUX;
  }

private:
  const nrf5::TIMER_Type& timer_;

  /* A bit mask isolating a count to the bits of the configured BITMODE */
  unsigned int mask_;
};

#ifndef NRFCXX_PERIPH_RNG_BUFFER_SIZE
/** The length of the software FIFO holding available random
 * bytes. */
#define NRFCXX_PERIPH_RNG_BUFFER_SIZE 8
#endif /* NRFCXX_PERIPH_RNG_BUFFER_SIZE */

/** Wrapper around the nRF51 RNG peripheral.
 *
 * Random bytes are generated in the background and kept in a buffer,
 * from which fill() reads values.
 *
 * @see NRFCXX_PERIPH_RNG_BUFFER_SIZE */
class RNG
{
public:
  /** Fill a block of memory with random bytes.
   *
   * @param dest pointer to area to be filled
   *
   * @param count number of bytes to be written starting at @p dest
   *
   * @return @p dest
   *
   * @note It is safe to invoke this from any context including FLIHs
   * and blocks in which interrupts are disabled. */
  static void* fill (void* dest,
                     size_t count);

  /** Generate a random value of the specified type.
   *
   * This delegates to fill() to obtain a block of random octets of
   * the required length.
   *
   * @tparam T any integral type */
  template <typename T,
            typename = std::enable_if<std::is_integral<T>::value>>
  static T generate ()
  {
    union {
      T v;
      uint8_t u8[sizeof(T)];
    } u;
    fill(u.u8, sizeof(u.u8));
    return u.v;
  }
};

/** Wrapper around the nRF5 TEMP peripheral. */
class TEMP
{
public:
  /** Read the nRF5 die temperature.
   *
   * This is not particularly useful for environment monitoring but can
   * assist in determining that the ADC needs to be recalibrated due to
   * temperature variation.
   *
   * @note This function is blocking and will take about 36 us on an
   * nRF52.  For accuracy it requires that clock::hfclk::hfxt_active()
   * be `true`, but (a) it provides an answer even without accuracy,
   * (b) the "accurate" reading is +/- 5 Cel, and (c) sample-to-sample
   * accuracy is +/- 0.25 Cel (specifications for nRF52840).  So for
   * detecting changes in excess of 5 Cel it's probably not necessary
   * to ensure HFXT is running.
   *
   * @warning This function uses a resource that is restricted by the
   * soft device, and should only be invoked when
   * systemState::softdevice_is_enabled() returns `false`.
   *
   * @return Die temperature in quarter-Celsius. */
  static int temperature ();
};

/** Class supporting GPIO task and event operations.
 *
 * The nRF51 provides a set of four channels that can detect or emit
 * GPIO pin state changes.  It also provides a general facility to
 * detect pin level.
 *
 * References to GPIOTE instances are obtained through allocate(), and
 * released through release().  Each instance can be in one of the
 * following modes:
 * * config_event() detects changes to pin input state;
 * * config_task() allows PPI use to change pin output state;
 * * config_disabled() to decouple the instance from any state change
 *   operation.
 *
 * Each instance starts disabled when @link allocate allocated@endlink
 * and is automatically disabled when @link release released@endlink.
 *
 * In addition to these instances the module provides an ability to
 * invoke callbacks when a GPIO pin transitions to a specific @link
 * enable_sense state@endlink.
 */
class GPIOTE
{
  /* Record which instances are in use.  An 8-bit map is sufficient
   * for nRF51 and nrf52832/840. */
  using registry_type = uint8_t;

public:
  /** Type for a reference to a peripheral event */
  using event_reference_type = __O uint32_t &;

  /** Type for a reference to a peripheral task */
  using task_reference_type = __IO uint32_t &;

  /** The number of GPIOTE channels available.
   *
   * The nRF51 has four; nrf52 has eight. */
  constexpr static size_t CHANNEL_COUNT = nrf5::GPIOTE.AUX;

  /** The channel index for this GPIOTE instance. */
  const uint8_t channel;

  /** Signature for a @link enable_event event
   * callback@endlink.
   *
   * @note Functions with this signature are expected to be invoked
   * when the GPIOTE IRQ is blocked.
   *
   * @param instance the GPIOTE instance on which the event was
   * triggered. */
  using event_callback_bi = std::function<void(GPIOTE& instance)>;

  /** Structure used to convey information about pin levels to
   * sense_listener callbacks. */
  struct sense_status_type
  {
    /** The global pin selector index, or a negative value to indicate
     * the end of the sense-enabled pins. */
    int8_t psel;

    /** Pin state and count of confirmed changes.
     *
     * Bit 0 reflects the state of the pin at the time the event was
     * processed.  It may have changed subseqently; if so, it is
     * guaranteed that another sense event will be generated.
     *
     * Bits 1..7 provide a count of confirmed changes since the last
     * sense change event. */
    uint8_t counter_state;

    /** Update an external aggregate state with new material.
     *
     * This returns a where bit 0 reflects the state of the pin from
     * #counter_state, and bits 1 through 31 reflect an aggregate
     * count of changes detected.
     *
     * @param previous the previous result from aggregate state, from
     * which the previous counter is extracted. */
    unsigned int aggregate_state (unsigned int previous = 0) const
    {
      return counter_state + (previous & ~1U);
    }
  };

  /** Signature for a @link enable_sense sense callback@endlink.
   *
   * @note Functions with this signature are expected to be invoked
   * when the GPIOTE IRQ is blocked.
   *
   * @note All sense callbacks are invoked any time a level change is
   * detected on any GPIO that has the SENSE feature enabled.  The
   * callbacks must determine whether pins they're interested in have
   * changed.
   *
   * The callback should do something like this:
   *
   *     while ((0 <= sp->psel)
   *            && (psel < sp->psel)) {
   *       ++sp;
   *     }
   *     if (psel == sp->psel) {
   *       // process sp->counter_state
   *     } else {
   *       // apparently psel isn't configured for sense detection
   *     }
   *
   * @param sp pointer to a sequence of records indicating the status
   * of all GPIO pins for which sense detection is enabled.  The pins
   * will be in increasing order of psel; the sequence terminates with
   * a negative psel.  See sense_status_type::counter_state for the
   * information returned. */
  using sense_callback_bi = std::function<void(const sense_status_type* sp)>;

  /* You can't copy, assign, move, or otherwise change the set of
   * GPIOTE instances that the system provides to you. */
  GPIOTE () = delete;
  GPIOTE (const GPIOTE&) = delete;
  GPIOTE& operator= (const GPIOTE&) = delete;
  GPIOTE (const GPIOTE&&) = delete;
  GPIOTE& operator= (GPIOTE&&) = delete;

  /** An RAII type for mutex access to state that must be
   * protected from GPIOTE interrupts. */
  using mutex_type = mutex_irq<GPIOTE_IRQn>;

  /** Process as though a PORT event had been received.
   *
   * This should be invoked after configuring a GPIO for sense
   * detection, to ensure that the configured sense detects a change
   * from the current level.  All enabled sense listeners will invoked
   * during the call. */
  static void synchronize_sense ();

  /** Object used to manage @link enable_event event
   * callbacks@endlink.
   *
   * This object provides a callback and argument to be invoked from
   * the GPIOTE IRQ handler in situations where an @c IN event is
   * triggered for a @link GPIOTE@endlink instance.
   *
   * Association of the listener with the instance is maintained by
   * enable() and disable() regardless of the mode of the @link
   * GPIOTE@endlink instance or whether @link GPIOTE::enable_event
   * event interrupts@endlink are enabled.  The association is
   * implicitly removed if the @link event_listener@endlink instance
   * is destructed or enable() is invoked for the same @link
   * GPIOTE@endlink instance but a different listener.
   */
  class event_listener
  {
  public:
    /** Mutex used for blocked interrupts related to the listener. */
    using mutex_type = GPIOTE::mutex_type;

    /** Construct an event listener.
     *
     * @param callback_bi_ the function to be invoked from the FLIH
     * when an event is detected.  The callback must not be null.
     *
     * @param arg_ a pointer that will provided to the callback so it can
     * access additional context.
     *
     * @p callback and @p arg are preserved regardless of whether the
     * listener is @link enable enabled@endlink. */
    event_listener (event_callback_bi callback_bi) :
      callback_bi{callback_bi},
      instance_{}
    { }

    /** Disable listener on destruction. */
    ~event_listener ()
    {
      disable();
    }

    /** Associate this listener with a specific @link GPIOTE@endlink instance.
     *
     * If the listener is associated with another instance, or @p
     * instance is associated with another listener, those
     * associations are removed before this operation creates its own
     * association.  If @p instance is already associated with this
     * listener nothing is changed.
     *
     * @param instance the @link GPIOTE@endlink instance to which the
     * listener should be associated. */
    void enable (GPIOTE& instance);

    /** Dissociate this listener from any @link GPIOTE@endlink
     * instance. */
    void disable ()
    {
      mutex_type mutex;
      disable_bi_();
    }

    /** @cond DOXYGEN_EXCLUDE */
    /* You can't copy, assign, move, or otherwise muck with event
     * listeners, since they participate in a reference maintained by
     * the @link GPIOTE@endlink module. */
    event_listener (const event_listener&) = delete;
    event_listener& operator= (const event_listener&) = delete;
    event_listener (const event_listener&&) = delete;
    event_listener& operator= (event_listener&&) = delete;
    /** @endcond */

    /** The callback invoked from the GPIOTE IRQ handler. */
    event_callback_bi const callback_bi;

  private:
    friend GPIOTE;

    /* Dissociate this listener from its GPIOTE instance, if any. */
    void disable_bi_ ()
    {
      if (instance_) {
        instance_->listener_ = nullptr;
      }
      instance_ = nullptr;
    }

    /** pointer to the @link GPIOTE@endlink instance associated
     * with this callback.
     *
     * When this pointer is not null, @c instance_->listener_ must
     * equal @c this. */
    GPIOTE* instance_;
  };

  /** Object used to manage @link enable_sense sense
   * callbacks@endlink.
   *
   * Sense events on individual pins are collapsed to a single event
   * that provides no identification of the pin(s) that triggered the
   * event.  It is thus necessary to register multiple listeners are
   * informed of all changes, and assess relevance of the changes in
   * their individual callbacks. */
  class sense_listener
  {
    struct ref_next
    {
      using pointer_type = sense_listener *;
      pointer_type& operator() (sense_listener& sl) noexcept
      {
        return sl.next_;
      }
    };

  public:
    using chain_type = pabigot::container::forward_chain<sense_listener, ref_next>;

    /** Mutex used for blocked interrupts related to the listener. */
    using mutex_type = GPIOTE::mutex_type;

    /** Construct a sense listener.
     *
     * @param callback_bi_ the function to be invoked from the FLIH when
     * an sense event is detected.  The callback must not be null.
     *
     * @p callback and @p arg are preserved regardless of whether the
     * listener is @link enable enabled@endlink. */
    sense_listener (sense_callback_bi callback_bi) :
      callback_bi{callback_bi}
    { }

    /** Destruct a sense listener.
     *
     * Listeners are automatically @link disable disabled@endlink when
     * they are destroyed. */
    ~sense_listener ()
    {
      disable();
    }

    /** @cond DOXYGEN_EXCLUDE */
    /* You can't copy, assign, move, or otherwise muck with sense
     * listeners, since they participate in a linked list maintained
     * by the @link GPIOTE@endlink module. */
    sense_listener (const sense_listener&) = delete;
    sense_listener& operator= (const sense_listener&) = delete;
    sense_listener (const sense_listener&&) = delete;
    sense_listener& operator= (sense_listener&&) = delete;
    /** @endcond */

    /** Enable @link config_event event processing@endlink for
     * this instance.
     *
     * This links the instance into the set of instances for which
     * events are processed by irq_handler().
     *
     * @note Although the enabled state remains regardless of the
     * instance configured mode, it has an effect only when configured
     * for @link config_event event monitoring@endlink.
     *
     * @warning Undefined behavior is produced if this operation is
     * invoked by a sense listener callback. */
    void enable ();

    /** Disable @link config_event event processing@endlink for
     * this instance.
     *
     * This unlinks the instance from the set of instances for which
     * events are processed by irq_handler().
     *
     * @warning Undefined behavior is produced if this operation is
     * invoked by a sense listener callback, except when disabling the
     * invoking listener. */
    void disable ();

    /** The callback invoked from the GPIOTE IRQ handler. */
    sense_callback_bi const callback_bi;

  protected:
    friend GPIOTE;

    /** Pointer used to link this listener into the enabled sense listeners. */
    chain_type::pointer_type next_ = chain_type::unlinked_ptr();
  };

  /** Allocate a GPIOTE instance.
   *
   * @return a pointer to an available instance, or a null pointer if
   * all instances are currently in use within the application. */
  static GPIOTE* allocate ();

  /** Release the instance.
   *
   * This should be invoked at the point where the instance is no
   * longer needed.  It provides an equivalent to this sequence:
   *
   *     gpiote.config_disabled()
   *     gpiote.disable_event()
   *     gpiote_listener.disable()
   *
   * as well as making the resource available for subsequent
   * invocations of allocate().
   */
  void release ();

  /** Configure the instance to monitor edge events.
   *
   * This function only configures the GPIOTE instance to detect edge
   * events.  The event must also be enabled through enable_event()
   * and a @link event_listener event listener@endlink must be
   * provided.
   *
   * See enable_event() for an example of using this method in
   * context.
   *
   * @note It is safe to re-invoke this operation with different
   * arguments while events are enabled, if there is a need to change
   * the source of the event.
   *
   * @param psel the GPIO global pin index on which edge events will
   * be detected.
   *
   * @param polarity optional configuration for the type of edge
   * event.  By default both rising and falling edges are detected.
   */
  void config_event (unsigned int psel,
                     unsigned int polarity = GPIOTE_CONFIG_POLARITY_Toggle)
  {
    nrf5::GPIOTE->CONFIG[channel] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
      | gpiote_config_from_psel(psel)
      | (GPIOTE_CONFIG_POLARITY_Msk & (polarity << GPIOTE_CONFIG_POLARITY_Pos));
  }

  /** Configure the instance to change pin level.
   *
   * This function only configures the GPIOTE instance to trigger
   * state changes when events are detected.  It has no effect on
   * @link enable_event event handling@endlink nor does it provide a
   * mechanism to trigger events.
   *
   * @param psel the GPIO global pin index that will change state.
   *
   * @param polarity the state change to be executed when an event is
   * detected.
   */
  void config_task (unsigned int psel,
                    unsigned int polarity = GPIOTE_CONFIG_POLARITY_Toggle)
  {
    nrf5::GPIOTE->CONFIG[channel] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
      | gpiote_config_from_psel(psel)
      | (GPIOTE_CONFIG_POLARITY_Msk & (polarity << GPIOTE_CONFIG_POLARITY_Pos));
  }

  /** Deconfigure the instance.
   *
   * This function atomically configures the GPIOTE instance to
   * disable both event monitoring and automatic state changes.  It
   * has no effect on @link enable_event event handling@endlink.
   */
  void config_disabled ()
  {
    nrf5::GPIOTE->CONFIG[channel] = 0;
  }

  /** Enable event callbacks for this instance.
   *
   * This only enables the instance within the peripheral module
   * instance.  The @link irq_handler IRQ@endlink must separately be
   * configured and enabled, and an event listener must be @link
   * event_listener::enable enabled@endlink.
   *
   * Example:
   *
   *     using namespace nrfcxx::gpio;
   *
   *     // Configure the GPIO as an input with pull.  SENSE is not needed.
   *     GPIO->PIN_CNF[NRFCXX_BOARD_PSEL_BUTTON0] = PIN_CNF_ACTIVE_LOW & ~GPIO_PIN_CNF_SENSE_Msk;
   *
   *     // Allocate and configure a channel to generate events from the configured GPIO
   *     auto b0ch = GPIOTE::allocate();
   *     b0ch->config_event(NRFCXX_BOARD_PSEL_BUTTON0);
   *
   *     // Create and enable an event listener tied to the GPIOTE channel
   *     GPIOTE::event_listener button0{[](auto& r, void*) { events.set(EVT_EDGE_BUTTON0); }};
   *     button0.enable(b0ch);
   *
   *     // Start listening to events on the GPIOTE channel
   *     b0ch->enable_event();
   *
   * @note The requirements of @ref irq_handler must also be satisfied.
   *
   * @return 0 on success, or a negative error code (e.g. when the
   * instance is already enabled or ).
   *
   * @see disable_event()
   */
  void enable_event ()
  {
    nrf5::GPIOTE->EVENTS_IN[channel] = 0;
    nrf5::GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << (GPIOTE_INTENSET_IN0_Pos + channel));
  }

  /** Disable event callbacks for this instance.
   *
   * @return 0 on success, or a negative error code (e.g. when the
   * instance is not @link config_event configured for
   * events@endlink).
   *
   * @see enable_event()
   */
  void disable_event ()
  {
    nrf5::GPIOTE->INTENCLR = (GPIOTE_INTENCLR_IN0_Clear << (GPIOTE_INTENCLR_IN0_Pos + channel));
  }

  /** Reference the channel-specific EVENTS_IN register. */
  event_reference_type EVENTS_IN ();

  /** Reference the channel-specific TASKS_OUT register. */
  task_reference_type TASKS_OUT ();

#if !(NRF51 - 0)
  /** Reference the channel-specific TASKS_SET register.
   *
   * @note Only available on nRF52 MCUs. */
  task_reference_type TASKS_SET ();

  /** Reference the channel-specific TASKS_CLR register.
   *
   * @note Only available on nRF52 MCUs. */
  task_reference_type TASKS_CLR ();
#endif /* !NRF51 */

  /** Enable sense callbacks.
   *
   * This facility allows detection of a generic event that is
   * triggered whenever a GPIO pin configured for input changes state
   * to a level specified in its @link gpio::pin_config SENSE@endlink
   * field.  When detected any registered @link sense_listener sense
   * listeners@endlink will be invoked.
   *
   * Example:
   *
   *     using namespace nrfcxx::gpio;
   *
   *     // Configure the GPIO as an input with pull and sense.
   *     GPIO->PIN_CNF[NRFCXX_BOARD_PSEL_BUTTON1] = PIN_CNF_ACTIVE_LOW;
   *
   *     // Create and enable a sense listener
   *     volatile unsigned int b1_state;
   *     GPIOTE::sense_listener button1{[&](auto sp) {
   *       while ((0 <= sp->psel)
   *              && (sp->psel < NRFCXX_BOARD_PSEL_BUTTON1)) {
   *         ++sp;
   *       }
   *       if (NRFCXX_BOARD_PSEL_BUTTON1 == sp->psel) {
   *         b1_state = sp->aggregate_state(b1_state);
   *         if (1 < b1_state->counter_state) {
   *           events.set(EVT_LEVEL_BUTTON1);
   *         }
   *       }
   *     }};
   *     button1.enable();
   *
   *     // Start listening to sense events (only need this once)
   *     GPIOTE::enable_sense();
   *     GPIOTE::synchronize_sense();
   *
   * @note The requirements of @ref irq_handler must also be satisfied.
   *
   * @see sense_listener
   * @see gpio::PIN_CNF_ACTIVE_LOW
   * @see gpio::PIN_CNF_ACTIVE_HIGH
   */
  static void enable_sense ();

  /** Stop monitoring @link enable_sense sense
   * events@endlink. */
  static void disable_sense ()
  {
    nrf5::GPIOTE->INTENCLR = (GPIOTE_INTENCLR_PORT_Clear << GPIOTE_INTENCLR_PORT_Pos);
  }

  /** Implementation for `GPIOTE_IRQHandler` required by this module.
   *
   * This module requires that the `GPIOTE_IRQHandler` process events
   * to fulfill its obligations.  To reduce code space this handler is
   * not normally installed.  Applications that require @ref GPIOTE
   * must install it:
   *
   *     extern "C" {
   *       void GPIOTE_IRQHandler (void) { nrfcxx::periph::GPIOTE::irq_handler(); }
   *     }
   *
   * When invoked this function will process and clear all GPIOTE
   * events, regardless of whether instances in this module are
   * configured for those events.
   *
   * You will also need the following in your application main function:
   *
   *     NVIC_EnableIRQ(GPIOTE_IRQn);
   */
  static void irq_handler (void);

private:
  /* The constructor is only invoked through the static initializer
   * for instances_. */
  constexpr GPIOTE (uint8_t channel_) :
    channel{channel_},
    listener_{}
  { }

  void config_disabled_bi_ ()
  {
    auto listener = listener_;
    if (listener) {
      listener->disable_bi_();
    }
  }

  event_listener * volatile listener_;

  static sense_listener::chain_type sense_chain_;
  static GPIOTE instances_[CHANNEL_COUNT];
  static registry_type in_use_;
}; // class GPIOTE

/** Resource allocation manager for PPI module.
 *
 * Used to avoid the need for compile-time assignment of channels or
 * channel groups to functions.  Also provides an abstraction that
 * will ultimately support the supervisor calls required for soft
 * devices. */
class PPI
{
public:
  /** Type for a reference to a peripheral event */
  using event_reference_type = __O uint32_t &;

  /** Type for a reference to a peripheral task */
  using task_reference_type = __IO uint32_t &;

  /** Allocate a PPI channel for application use.
   *
   * @return the PPI channel index, or a negative error code. */
  static int request ();

  /** Release a previously allocated PPI channel.
   *
   * @return 0 on success, or a negative error code. */
  static int release (int ppi);

  /** Allocate a PPI channel group for application use.
   *
   * @return the PPI channel group index, or a negative error code. */
  static int group_request ();

  /** Release a previously allocated PPI channel group.
   *
   * @return 0 on success, or a negative error code. */
  static int group_release (int ppi);

  /** Configure a PPI channel
   *
   * @param ppidx an PPI channel index as returned from request()
   *
   * @param eep the event endpoint of the connection
   *
   * @param tep the task endpoint of the connection
   *
   * @return 0 on success, or a nonzero error code */
  static uint32_t configure (int ppidx,
                             event_reference_type eep,
                             task_reference_type tep)
  {
    return configure_(ppidx, eep, tep);
  }

  /** Abstraction to enable a set of PPI channels */
  static uint32_t CHENSET (uint32_t v)
  {
    return CHENSET_(v);
  }

  /** Abstraction to disable a set of PPI channels */
  static uint32_t CHENCLR (uint32_t v)
  {
    return CHENCLR_(v);
  }

  /** Abstraction to read the set of enabled PPI channels */
  static uint32_t CHEN ()
  {
    return CHEN_();
  }

protected:
  static uint32_t CHENSET_ (uint32_t v)
  {
    nrf5::PPI->CHENSET = v;
    return 0;
  }

  static uint32_t CHENCLR_ (uint32_t v)
  {
    nrf5::PPI->CHENCLR = v;
    return 0;
  }

  static uint32_t CHEN_ ()
  {
    return nrf5::PPI->CHEN;
  }

  static uint32_t configure_ (int ppidx,
                              event_reference_type eep,
                              task_reference_type tep)
  {
    nrf5::PPI->CH[ppidx].EEP = reinterpret_cast<uintptr_t>(&eep);
    nrf5::PPI->CH[ppidx].TEP = reinterpret_cast<uintptr_t>(&tep);
    return 0;
  }
};

/** Class supporting GPIO instrumentation triggered by peripheral events.
 *
 * Instances of this link a GPIO identified by PSEL with an event
 * register.  When enabled each event triggers a polarity change in
 * the GPIO.
 *
 * @note As long as the instance exists one GPIOTE channel and one PPI
 * channel will be assigned to it.  If allocation fails during
 * construction the bool value of the instance will be `false`. */
class instr_psel_gpiote
{
public:

  /** Construct the instance.
   *
   * @param psel the GPIO pin selector for the line that should be toggled on events.
   *
   * @param eep the event end point that triggers changes in line state.
   *
   * @param enable if `true` (default) the instance is automatically
   * @link enable enabled@endlink when it is constructed. */
  instr_psel_gpiote (unsigned int psel,
                     PPI::event_reference_type eep,
                     bool enable = true) :
    gpiote_{GPIOTE::allocate()},
    eep_{eep},
    psel_(psel),
    ppi_idx_(PPI::request())
  {
    if (gpiote_ && (0 > ppi_idx_)) {
      gpiote_->release();
      gpiote_ = nullptr;
    } else if ((!gpiote_) && (0 <= ppi_idx_)) {
      PPI::release(ppi_idx_);
      ppi_idx_ = -1;
    } else if (enable) {
      this->enable();
    }
  }

  instr_psel_gpiote () = delete;
  instr_psel_gpiote (const instr_psel_gpiote&) = delete;
  instr_psel_gpiote& operator= (const instr_psel_gpiote&) = delete;
  instr_psel_gpiote (instr_psel_gpiote&&) = delete;
  instr_psel_gpiote& operator= (instr_psel_gpiote&&) = delete;

  ~instr_psel_gpiote ()
  {
    if (0 <= ppi_idx_) {
      PPI::CHENCLR(1U << ppi_idx_);
      PPI::release(ppi_idx_);
      gpiote_->release();
    }
  }

  /** Implicit cast to bool, true if the instrumentation has
   * its required instances. */
  operator bool () const
  {
    return gpiote_;
  }

  /** Enable the link from the event to the GPIO */
  void enable ()
  {
    if (gpiote_) {
      gpiote_->config_task(psel_);
      PPI::configure(ppi_idx_, eep_, gpiote_->TASKS_OUT());
      PPI::CHENSET(1U << ppi_idx_);
    }
  }

  /** Disable the link from the event to the GPIO */
  void disable ()
  {
    if (gpiote_) {
      PPI::CHENCLR(1U << ppi_idx_);
      gpiote_->config_disabled();
    }
  }

private:
  GPIOTE* gpiote_;
  PPI::event_reference_type eep_;
  uint16_t psel_;
  int8_t ppi_idx_;
};

namespace details {

/** Abstracted support for error returns.
 *
 * This is designed around the needs of #TWI but applies to SPI as
 * well for a couple operations that can fail with #ERR_INVALID. */
class comm_error_support
{
public:

  /** The type used for transfer sizes (unsigned) */
  using size_type = unsigned int;

  /** The type used for transfer sizes (non-negative) or errors
   * (negative).
   *
   * @see error_type*/
  using ssize_type = int;

  /** The type used to encode TWI peripheral errors.
   *
   * TWI errors are indicated returned as negative #ssize_type values
   * from read() and write().  Details of the error are obtained by
   * converting this value to an #error_type value using
   * error_decoded().
   *
   * The resulting code comprises bits from the TWI `ERRORSRC`
   * register along with #ERR_TIMEOUT, #ERR_CLEAR, #ERR_INVALID, and
   * #ERR_UNKNOWN.
   *
   * @see error_decoded */
  using error_type = unsigned int;

  /** NRF_TWI_Type::ERRORSRC bit indicating incomplete
   * reception at start. */
  constexpr static unsigned int ERR_OVERRUN = 0x001;

  /** NRF_TWI_Type::ERRORSRC bit indicating NACK received
   * during address transmission. */
  constexpr static unsigned int ERR_ANACK = 0x002;

  /** NRF_TWI_Type::ERRORSRC bit indicating NACK received
   * during data transmission. */
  constexpr static unsigned int ERR_DNACK = 0x004;

  /** Bit set in an error code when the TWI bus transaction
   * timed out. */
  constexpr static unsigned int ERR_TIMEOUT = 0x100;

  /** Bit set in an error code when the TWI bus could not be
   * cleared. */
  constexpr static unsigned int ERR_CLEAR = 0x200;

  /** Bit set in an error code to indicate that the bus was not
   * properly configured or a parameter was invalid. */
  constexpr static unsigned int ERR_INVALID = 0x400;

  /** Bit set in an error code to indicate a checksum error.
   *
   * Generally this error will appear only in results from methods
   * supporting higher-level operations on devices that can detect
   * data errors. */
  constexpr static unsigned int ERR_CHECKSUM = 0x800;

  /** Bit set in an error code to indicate an undescribable
   * error. */
  constexpr static unsigned int ERR_UNKNOWN = 0x1000;

  /** Extract an encoded error value from an API return value.
   *
   * @param rc a result code, which is negative if it represents an
   * error.
   *
   * @return Zero if @p rc does not represent an error, otherwise the
   * corresponding #error_type value. */
  constexpr static error_type
  error_decoded (ssize_type rc)
  {
    return (0 <= rc) ? 0 : (static_cast<unsigned int>(-rc) - 1);
  }

  /** Pack an error value into a negative return value.
   *
   * @return Zero if @p ec is zero, otherwise the encoded error
   * value. */
  constexpr static ssize_type
  error_encoded (error_type ec)
  {
    return ec ? -(1 + ec) : 0;
  }
};

/** RAII instance that ensures the an instance is enabled within a
 * scope.
 *
 * If constructed when the peripheral is not enabled the peripheral
 * will be enabled.  If this action fails the enabler instance will
 * evaluate to false, indicating the peripheral is not functional.  If
 * it succeeds, the peripheral will be disabled when the instance is
 * destructed.
 *
 * If the peripheral is enabled prior to constructing the enabler its
 * enabled state will not be changed when the enabler is
 * destructed. */
template <typename PERIPH>
class scoped_enabler
{
  using ssize_type = typename PERIPH::ssize_type;

  scoped_enabler (const scoped_enabler&) = delete;
  scoped_enabler& operator= (const scoped_enabler&) = delete;
  scoped_enabler (scoped_enabler&&) = delete;
  scoped_enabler& operator= (scoped_enabler&&) = delete;

public:
  ~scoped_enabler () noexcept
  {
    if (0 < result_) {
      periph.disable();
    }
  }

  /** Boolean value is true iff construction completed with an enabled device. */
  operator bool() const
  {
    return (0 <= result_);
  }

  /** The result of attempting to enable the peripheral.
   *
   * @return 0 if peripheral was already enabled on construction,
   * positive if construction caused the peripheral to be enabled,
   * or a negative @link error_decoded encoded error@endlink. */
  ssize_type result () const
  {
    return result_;
  }

private:
  friend PERIPH;

  scoped_enabler (PERIPH& periph) :
    periph{periph}
  {
    if (!periph.enabled()) {
      result_ = periph.enable();
      if (0 <= result_) {
        result_ = 1;
      }
    }
  }

  PERIPH& periph;
  ssize_type result_ = 0;
};

} // ns details

/** Wrapper around the nRF51 TWI peripheral.
 *
 * This is what the rest of the world calls I2C.
 *
 * @warning TWI0 and SPI0 share an interrupt and peripheral address
 * space and cannot be simultaneously enabled.  Similarly for TWI1,
 * SPI1, and SPIS1. */
class TWI : public details::comm_error_support
{
public:
  /** The minimum allowed per-byte bus timeout, in microseconds */
  constexpr static unsigned int minimum_timeout_us = 100;

  /** Reference the underlying peripheral. */
  const nrf5::TWI_Type& peripheral() const
  {
    return twi_;
  }

  /** Return @c true iff the device is enabled. */
  bool enabled () const
  {
    return ((TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos)
            == (TWI_ENABLE_ENABLE_Msk & twi_->ENABLE));
  }

  /** The scoped enabler uses a TWI instance. */
  using scoped_enabler = details::scoped_enabler<TWI>;

  /** Construct an RAII object that controls whether the instance is enabled.
   *
   * Example:
   *
   *     // twi may or may not be enabled
   *     if (auto enabler = twi.scoped_enable()) {
   *       // twi is enabled, use it
   *     } else {
   *       // enable() failed, result in enabler.result()
   *     }
   *     // twi is disabled if it was enabled by previous statement
   *     // twi remains enabled if it was enabled before previous statement
   */
  scoped_enabler scoped_enable () noexcept
  {
    return {*this};
  }

  /** Configure the TWI bus.
   *
   * @param psel_scl the GPIO pin number to be used for the SCL
   * (clock) signal.
   *
   * @param psel_sdc the GPIO pin number to be used for the SDA (data)
   * signal.
   *
   * @param frequency the bus frequency, being one of
   * #TWI_FREQUENCY_FREQUENCY_K100, #TWI_FREQUENCY_FREQUENCY_K250,
   * #TWI_FREQUENCY_FREQUENCY_K400, or another value to be assigned
   * directly to the FREQUENCY register of the TWI peripheral.
   *
   * @param timeout_us the maximum time to wait for completion of each
   * byte in the transaction.  Values below #minimum_timeout_us are
   * clamped to that value.
   *
   * @return Zero if successfully configured, otherwise a negative
   * @link error_encoded encoded error@endlink.
   *
   * @note There is no clear relationship between @p frequency and the
   * numeric frequency in Hz, nor does this API validate the
   * parameter.  Use the Nordic defined constants. */
  ssize_type bus_configure (int psel_scl,
                            int psel_sda,
                            uint32_t frequency,
                            unsigned int timeout_us);

  /** Enable the TWI peripheral.
   *
   * @return 0 on success or negative @link error_encoded encoded
   * error@endlink.
   *
   * @see bus_configure */
  ssize_type enable ()
  {
    return error_encoded(set_enabled_(true));
  }

  /** Disable the TWI peripheral. */
  void disable ()
  {
    set_enabled_(false);
  }

  /** Read a block of data from an I2C device.
   *
   * @param addr the 7-bit I2C address of the device
   *
   * @param buf where the received data should be stored
   *
   * @param count the number of bytes to be read
   *
   * @return @p count successfully read bytes, or a negative encoded
   * error. */
  ssize_type read (unsigned int addr,
                   uint8_t* buf,
                   size_type count);

  /** Write a block of data to an I2C device.
   *
   * @param addr the 7-bit I2C address of the device
   *
   * @param buf the data to be written
   *
   * @param count the number of bytes to write.  A value of zero is
   * permitted; this can be used as a check that a device is
   * responsive at @p addr.
   *
   * @return @p count successfully written bytes, or a negative @link
   * error_encoded encoded error@endlink. */
  ssize_type write (unsigned int addr,
                    const uint8_t* buf,
                    size_type count);

  /** Write to then read from an I2C device.
   *
   * @param addr the 7-bit I2C address of the device.
   *
   * @param sbuf the data to be written.
   *
   * @param scount the number of bytes to write.  If zero the write
   * phase is skipped.
   *
   * @param dbuf where the data to be read should be stored.  The
   * object may overlap @p sbuf.
   *
   * @param dcount the number of bytes to read
   *
   * @return @p count successfully read bytes, or a negative @link
   * error_encoded encoded error@endlink. */
  ssize_type write_read (unsigned int addr,
                         const uint8_t* sbuf,
                         size_type scount,
                         uint8_t* dbuf,
                         size_type dcount)
  {
    int rv = 0;
    if (0 < scount) {
      rv = write(addr, sbuf, scount);
    }
    if (0 <= rv) {
      rv = read(addr, dbuf, dcount);
    }
    return rv;
  }

  /** Send the general call RESET command.
   *
   * For devices that support it (such as the Sensirion SHT31) this
   * provides a software mechanism to reset the device.
   *
   * @return as with write(). */
  ssize_type general_call_reset ()
  {
    uint8_t cmd = 6;
    return write(0, &cmd, sizeof(cmd));
  }

  /** Probe for a device at the provided address.
   *
   * This uses a generic mechanism (a start condition with no command)
   * to determine whether a device is present on the bus.
   *
   * @note A successful probe will be resolved by assessing a bus
   * timeout, as determined by the `timeout_us` parameter to
   * bus_configure().  As such this call will generally take
   * `timeout_us` microseconds to complete.
   *
   * @param addr the 7-bit I2C address of the device
   *
   * @return zero if it appears that there is a device at the address,
   * or a negative encoded error. */
  int check_addr (unsigned int addr) noexcept;

  /** Reference the abstraction instance for a specific
   * peripheral instance.
   *
   * @param idx the peripheral instance desired.
   *
   * @warning If `idx` specifies a peripheral instance that does not
   * exist on the device the system will reset into failsafe mode. */
  static TWI& instance (int idx);

protected:

  constexpr TWI (const nrf5::TWI_Type& twi) :
    twi_{twi},
    configuration_{}
  { }

  struct configuration_type
  {
    /** Maximum duration to wait for a single-byte transaction to
     * complete, in us. */
    unsigned int timeout = 0;

    /** Encoded value for FREQUENCY register,
     * e.g. TWI_FREQUENCY_FREQUENCY_K400. */
    uint32_t frequency = 0;

    /** Pin selector for SCL, or -1 if not known. */
    int8_t psel_scl = -1;

    /** Pin selector for SDA, or -1 if not known. */
    int8_t psel_sda = -1;

    /** PPI index used for PAN 36 workaround, if enabled. */
    int8_t ppidx = -1;
  };

  error_type set_enabled_ (bool enabled);

  /** Reset the I2C bus to idle state if a secondary device is holding
   * it active.
   *
   * The bus is enabled if and only if the return value is zero (no
   * error, bus left in idle state: SCL and SDA not pulled low).
   *
   * @return zero, or #ERR_CLEAR */
  error_type clear_bus_ ();

  /** Invoked when a read or write operation detects a bus error.
   *
   * The error cause(s) are read and returned as a bit set.  If no TWI
   * error is detected, #ERR_UNKNOWN is set.  An attempt is made to
   * restore the bus to a cleared state; failure to do so is also
   * indicated in the return value. */
  error_type clear_error_ ();

  /** Power cycle the peripheral and re-initialize it.
   *
   * Returns zero if the reset was successful, otherwise an error
   * code. */
  error_type reset_periph_ ();

  const nrf5::TWI_Type& twi_;
  configuration_type configuration_;
};

/** Wrapper around the nRF51 SPI peripheral.
 *
 * @note This abstraction handles only configuration and input/output.
 * Control of the chip-select is not incorporated.
 *
 * @warning TWI0 and SPI0 share an interrupt and peripheral address
 * space and cannot be simultaneously enabled.  Similarly for TWI1,
 * SPI1, and SPIS1. */
class SPI : public details::comm_error_support
{
public:

  /** The type used for transfer sizes (unsigned) */
  using size_type = unsigned int;

  /** The type used for transfer sizes (non-negative) or errors
   * (negative).
   *
   * @see error_type*/
  using ssize_type = int;

  /** Reference the underlying peripheral. */
  const nrf5::SPI_Type& peripheral() const
  {
    return spi_;
  }

  /** Return @c true iff the device is enabled. */
  bool enabled () const
  {
    return ((SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos)
            == (SPI_ENABLE_ENABLE_Msk & spi_->ENABLE));
  }

  /** The scoped enabler uses a SPI instance. */
  using scoped_enabler = details::scoped_enabler<SPI>;

  /** Construct an RAII object that controls whether the instance is
   * enabled.
   *
   * Example:
   *
   *     // spi may or may not be enabled
   *     if (auto enabler = spi.scoped_enable()) {
   *       // spi is enabled, use it
   *     } else {
   *       // enable() failed, result in enabler.result()
   *     }
   *     // spi is disabled if it was enabled by previous statement
   *     // spi remains enabled if it was enabled before previous statement
   */
  scoped_enabler scoped_enable () noexcept
  {
    return {*this};
  }

  /** Configure the SPI bus.
   *
   * @param psel_scl the GPIO pin number to be used for the SCL
   * (clock) signal.
   *
   * @param psel_mosi the GPIO pin number to be used for the MOSI
   * signal.
   *
   * @param psel_miso the GPIO pin number to be used for the MISO
   * signal.  Pass -1 to disable MISO, as required by some write-only
   * devices.
   *
   * @param frequency the bus frequency, being one of
   * #SPI_FREQUENCY_FREQUENCY_K125, #SPI_FREQUENCY_FREQUENCY_M8,
   * another value defined value allowed for the FREQUENCY register of
   * the SPI peripheral. See frequency_from_Hz().
   *
   * @param config the value desired for the CONFIG register of the
   * SPI peripheral.  See config_from_mode().
   *
   * @return Zero if successfully configured, otherwise a negative
   * @link error_encoded encoded error@endlink. */
  ssize_type bus_configure (int psel_sck,
                            int psel_mosi,
                            int psel_miso,
                            uint32_t frequency,
                            uint32_t config);

  /** Calculate the appropriate frequency assignment for a
   * requested rate.
   *
   * Supported frequencies for nRF51 are 125kHz * `2^n` for `n`
   * between 0 (125 kHz) through 6 (8 MHz) inclusive.  Take largest
   * value that does not exceed the requested frequency, or the
   * smallest allowed value, whichever is greater.
   *
   * @param freq_Hz requested clock speed in Hz
   *
   * @return value to be stored in the FREQUENCY register of the SPI
   * peripheral. */
  static constexpr uint32_t frequency_from_Hz (unsigned int freq_Hz)
  {
    unsigned int multiplier = 125000;
    uint32_t rv = SPI_FREQUENCY_FREQUENCY_K125;
    while (true) {
      multiplier *= 2;
      if ((multiplier > freq_Hz)
          || (SPI_FREQUENCY_FREQUENCY_M8 == rv)) {
        break;
      }
      rv <<= 1;
    }
    return rv;
  }

  /** Determine the value for the `CONFIG` register.
   *
   * @param mode the standard 2-bit %SPI mode where CPOL is the upper
   * bit and CPHA is the lower bit.  Only the low two bits are used.
   *
   * @param lsb_first `true` if the device should shift out the least
   * significant bit first.  Defaults to MSB first.
   *
   * @return the corresponding CONFIG value. */
  static constexpr uint32_t config_from_mode (uint8_t mode,
                                              bool lsb_first = false)
  {

    return ((lsb_first?1:0) << SPI_CONFIG_ORDER_Pos)
      | ((mode & 3) << SPI_CONFIG_CPHA_Pos);
  }

  /** Enable the SPI peripheral.
   *
   * @return 0 on success or negative @link error_encoded encoded
   * error@endlink.
   *
   * @see bus_configure */
  ssize_type enable ()
  {
    return set_enabled_(true);
  }

  /** Disable the SPI peripheral. */
  void disable ()
  {
    set_enabled_(false);
  }

  /** Transmit and/or receive data over the SPI bus.
   *
   * This routine transmits @p tx_len octets from @p tx_data, storing
   * the octets received in response into @p rx_data.  It then
   * transmits @p rx_len dummy bytes, appending the resulting response
   * into @p rx_data.
   *
   * @param tx_data the data to be transmitted (generally, a command).
   * The pointer may be null only if @p tx_len is zero.
   *
   * @param tx_len the number of bytes to transmit as the command.
   * The value may be zero if this call reads data resulting from a
   * previous command.
   *
   * @param rx_len the number of additional bytes expected in
   * response, exclusive of the synchronous responses to bytes
   * transmitted from @p tx_data.  This data is elicited by
   * transmitting the requested number of `tx_dummy` values.
   *
   * @param rx_data where to store the responses received during the
   * transmit and receive phases.  A null pointer may be passed if the
   * incoming data is not of interest.  If the pointer is not null,
   * the space available must be at least @p tx_len + @p rx_len.
   *
   * @return the total number of bytes stored in @p rx_data (or that
   * would have been stored if @p rx_data were not null), or -1 if an
   * error occcured. */
  ssize_type tx_rx (const uint8_t* tx_data,
                    size_type tx_len,
                    size_type rx_len,
                    uint8_t* rx_data,
                    uint8_t tx_dummy = 0);

  /** Reference the abstraction instance for a specific
   * peripheral instance.
   *
   * @param idx the peripheral instance desired.
   *
   * @warning If `idx` specifies a peripheral instance that does not
   * exist on the device the system will reset into failsafe mode. */
  static SPI& instance (int idx);

protected:

  constexpr SPI (const nrf5::SPI_Type& spi) :
    spi_{spi},
    configuration_{}
  { }

  struct configuration_type
  {
    uint32_t frequency = 0;
    uint32_t config = 0;
    int8_t psel_sck = -1;
    int8_t psel_mosi = -1;
    int8_t psel_miso = -1;
    int8_t ppidx = -1;
  };

  error_type set_enabled_ (bool enabled);

  const nrf5::SPI_Type& spi_;
  configuration_type configuration_;
};

// Forward declaration
class ADCClient;

/** Wrapper around the %ADC or SAADC peripheral.
 *
 * The peripheral that provides A2D capabilities is completely
 * incompatible between nRF51 and nRF52.  This class extends a
 * series-specific base with basic common functionality.  To use it
 * consult the series-specific class that this extends:
 * * For nRF51 see nrf5::series::ADC_Peripheral
 * * For nRF52 see nrf5::series::SAADC_Peripheral
 *
 * All interaction with this is performed using an ADCClient subclass,
 * which may support single or multiple-channel collections.
 *
 * This module requires that the @link ADCSeriesVariant_IRQHandler
 * series-specific IRQ handler@endlink process events to fulfill its
 * obligations.  To reduce code space @link irq_handler this
 * handler@endlink is not normally installed.  Applications that
 * require @ref ADC must install it by providing this definition in an
 * application implementation source file:
 *
 *     extern "C" {
 *       void ADCSeriesVariant_IRQHandler (void) {
 *         nrfcxx::periph::ADC::irq_handler();
 *       }
 *     }
 *
 * and by issuing these commands to enable the peripheral:
 *
 *     nvic_SetPriority(nrfcxx::periph::ADC::INSTANCE.IRQn, IRQ_PRIORITY_APP_HIGH); // or APP_LOW
 *     nvic_EnableIRQ(nrfcxx::periph::ADC::INSTANCE.IRQn);
 *
 * In addition the ADCClient::calibrate function should be invoked
 * prior to first use and whenever the processor temperature changes
 * by more than 10 Cel.
 */
class ADC : protected nrf5::series::ADC_Variant
{
public:
  /** The underlying variant traits class. */
  using peripheral = nrf5::series::ADC_Variant;

  /** Reference to the variant-specific instance characteristics. */
  static constexpr auto& INSTANCE{
#if (NRF51 - 0)
    nrf5::ADC
#elif (NRF52832 - 0) || (NRF52840 - 0)
    nrf5::SAADC
#endif
  };

  /** Expose the series-specific function.
   *
   * @see nrf5::series::ADC_Peripheral::busy
   * @see nrf5::series::SAADC_Peripheral::busy */
  using peripheral::busy;

  /** Genericized states of the ADC. */
  enum class state_type : uint8_t
  {
    /** ADC is unclaimed.
     *
     * Transitions to @link state_type::claimed claimed@endlink on
     * ADCClient::claim(). */
    available,

    /** ADC is owned by a client but is not started.
     *
     * In this state the ADC's configuration may be safely changed.
     *
     * Transitions:
     * * to @link state_type::available available@endlink on
     *   ADCClient::release();
     * * to @link state_type::calibrating calibrating@endlink on
     *   ADCClient::calibrate();
     * * to @link state_type::starting starting@endlink on
     *   ADCClient::sample(). */
    claimed,

    /** ADC is being calibrated via ADCClient::calibrate()
     *
     * Transitions to @link state_type::stopping stopping@endlink on
     * receipt of `CALIBRATEDONE`. */
    calibrating,

    /** ADC is in the process of starting via ADCClient::sample()
     *
     * Transitions to @link state_type::sampling sampling@endlink on
     * receipt of `STARTED`. */
    starting,

    /** ADC is collecting a sample via ADCClient::sample()
     *
     * Transitions to @link state_type::stopping stopping@endlink when
     * the sample has been completed. */
    sampling,

    /** ADC is in the process of stopping in the final stages of
     * ADCClient::calibrate() or ADCClient::sample().
     *
     * Transitions to @link state_type::claimed claimed@endlink on
     * receipt of `STOPPED`. */
    stopping,
  };

  /** Implementation for `ADCSeriesVariant_IRQHandler` required by
   * this module. */
  static void irq_handler ();

  /** Return the current ADC state. */
  static uint8_t state ()
  {
    return static_cast<uint8_t>(state_);
  }

private:
  friend class ADCClient;

  /* Validate authorization to become owner and update state. */
  static int claim_bi_ (ADCClient* client);

  /* Validate then release ownership. */
  static int release_bi_ (ADCClient* client);

  /* Attempt to transition into calibrating state. */
  static int try_calibrate_bi_ (ADCClient* client,
                                const notifier_type& notify);

  /* Attempt to transition into sampling state. */
  static int try_sample_bi_ (ADCClient* client,
                             const notifier_type& notify);

  /* Notify application of completion of calibration or sampling.
   *
   * Operation callbacks are invoked for both sampling and
   * calibration, but ADCClient notification callbacks are only
   * invoked when sampling completes.
   *
   * @param calibrating `true` iff the operation that completed was
   * calibration.*/
  static void complete_notify_bi_ (bool calibrating);

  /** The ADCClient instance that controls the ADC peripheral.
   *
   * If this is a null pointer the ADC is available to be @link
   * ADCClient::claim claimed@endlink.  The pointer value is changed
   * through the ADCClient::claim() and ADCClient::release()
   * methods. */
  static ADCClient* volatile owner_;

  /** The callback associated with the current ADCClient::sample() or
   * ADCClient::calibrate() operation. */
  static notifier_type notify_callback_;

  static state_type volatile state_;
};

/** Base class for a client of @ref ADC.
 *
 * This class supports shared use of the %ADC peripheral.  To make use
 * of it, applications must provide infrastructure, configuration, and
 * calibration as documented at ADC::irq_handler.
 *
 * Much of the complexity of handling one-shot ADC collections may be
 * delegated to sensor::adc::lpsm_wrapper.
 *
 * Available implementations include:
 * * nrfcxx::sensor::adc::vdd to measure the device voltage;
 * * nrfcxx::sensor::adc::voltage_divider to measure an external
 *   voltage or variable-resistance sensor;
 * * nrfcxx::sensor::adc::ntcThermistor to measure temperature with an
 *   NTC thermistor. */
class ADCClient
{
public:
  /** Type for client-specific @link flags_ flags@endlink. */
  using flags_type = unsigned int;

  /** Signature for application notification of completed @link
   * ADCClient::queue queue operation@endlink.
   *
   * @warning Queue result callbacks are invoked with @ref mutex_type
   * held, commonly from within the ADC FLIH.
   *
   * @param rc the result from the queued ADCClient::calibrate() or
   * ADCClient::sample() request executed when the client reaches the
   * front of the queue. */
  using queued_callback_type = std::function<void(int rc)>;

protected:
  /** Defined values for flags. */
  enum flags_e : flags_type
  {
    /** Bit set in flags_ when the client is queued to invoke sample(). */
    FL_QUEUED_SAMPLE = 0x0001,

    /** Bit set in flags_ when the client is queued to invoke calibrate(). */
    FL_QUEUED_CALIBRATE = 0x0002,

    /** Mask to isolate queue state flags(). */
    FL_QUEUED_Msk = FL_QUEUED_SAMPLE | FL_QUEUED_CALIBRATE,

    /** Base for subclass use of flags_. */
    FL_SUBCLASS_BASE = 0x0010,
  };

private:
  struct ref_next
  {
    using pointer_type = ADCClient*;
    pointer_type& operator() (ADCClient& cl) noexcept
    {
      return cl.next_;
    }
  };

  using queue_type = pabigot::container::forward_chain<ADCClient, ref_next>;
  static queue_type queue_;

  queue_type::pointer_type next_ = queue_type::unlinked_ptr();
  queued_callback_type queued_callback_{};
  notifier_type notify_callback_{};

public:
  using peripheral = ADC::peripheral;

  /** Mutex required to inhibit ADC interrupts. */
  using mutex_type = peripheral::mutex_type;

  /** @cond DOXYGEN_EXCLUDE */
  /* You can default construct ADCClient instances, but you cannot
   * copy, assign or move them, since they can be referenced by the
   * infrastructure. */
  ADCClient () = default;
  ADCClient (const ADCClient&) = delete;
  ADCClient (const ADCClient&&) = delete;
  ADCClient& operator= (const ADCClient&) = delete;
  ADCClient& operator= (ADCClient&&) = delete;
  /** @endcond */

  /** Release the peripheral when the client is destructed. */
  virtual ~ADCClient ()
  {
    release();
  }

  /** Attempt to claim exclusive use of the ADC peripheral.
   *
   * @return zero on success, or a negative error code. */
  int claim ()
  {
    mutex_type mutex;
    return ADC::claim_bi_(this);
  }

  /** Release the ADC peripheral.
   *
   * @return zero on success, or a negative error code. */
  int release ()
  {
    mutex_type mutex;
    return ADC::release_bi_(this);
  }

  /** Calibrate the ADC.
   *
   * This is a no-op on nRF51 devices which do not support/require
   * calibration.
   *
   * On nRF52 devices calibrate() should be invoked prior to first use
   * and whenever the processor temperature changes by more than 10
   * Cel.  Calibration is performed by first invoking configure_bi(),
   * which is expected to set up SAADC channel zero: acquisition time
   * and oversampling configurations affect the calibration.
   *
   * @param notify a callback to be invoked when calibration
   * completes, either immediately or sometime in the future.  The
   * callback is not invoked if this function returns an error.
   *
   * @return zero if calibration completes immediately, positive if
   * the calibration has been initiated, or a negative error code. */
  int calibrate (const notifier_type& notify = {})
  {
    mutex_type mutex;
    return ADC::try_calibrate_bi_(this, notify);
  }

  /** Overridable function to set up for an ADC measurement.
   *
   * This might be used to enable current flow to a sensor that
   * produces an analog output, such as a thermistor or photo
   * transistor/resistor.  In many cases there will be some delay
   * before the sensor produces an accurate output; that delay is to
   * be returned from this function.
   *
   * Override of this method should be paired with an override of
   * sample_teardown_().
   *
   * The base class implementation does nothing and returns zero.
   *
   * @note It is the responsibility of the application to ensure this
   * is invoked prior to any invocation of sample() or calibrate(),
   * including through queue(), and that the component
   * sample_teardown() is invoked after the ADC operations complete.
   * This responsibility may be fulfilled through @ref
   * sensor::adc::lpsm_wrapper.
   *
   * @return a delay in uptime ticks to wait before proceeding with
   * the sample operation.  If zero is returned the sample request may
   * be initiated immediately after this function returns.  A negative
   * return may be used to indicate that sampling cannot be
   * performed. */
  virtual int sample_setup ()
  {
    return 0;
  }

  /** Reverse the effects of sample_setup_().
   *
   * For example, turn off the power flow to the analog sensor. */
  virtual void sample_teardown ()
  { }

  /** Trigger a client-specific collection.
   *
   * @note To initiate a sample when @ref mutex_type is already held
   * use sample_bi_().
   *
   * @param notify a callback to be invoked when collection completes.
   * The callback is not invoked if this function returns an error.
   * The callback is invoked after client-specific post-collection
   * processing via ADCClient::complete_bi_().
   *
   * @return positive if sampling has been initiated, or a negative
   * error code. */
  int sample (const notifier_type& notify = {})
  {
    mutex_type mutex;
    return sample_bi_(notify);
  }

  /** Queue an operation to be initiated as soon as the %ADC becomes
   * available.
   *
   * This capability allows multiple ADC clients to be scheduled
   * simultaneously (e.g. at a fixed interval) while not requiring the
   * application to manage the claim, initiate, release notifications
   * directly.
   *
   * Invoking this method causes this client to be added to a FIFO
   * queue of operations.  As soon as the ADC is available---which may
   * be during the invocation of this method---the claim() method is
   * invoked and, if successful, either sample() or calibrate() is
   * invoked.  The result of these two operations may be provided to
   * the application through @p qnotify.  When the operation is
   * complete release() is invoked and the next queued operation
   * processed.
   *
   * @note A client can appear within the queue only once.  Attempts
   * to queue the client while it is in the queue, or while it is
   * running after reaching the head of the queue, will be rejected.
   * A client that has to perform multiple samples may delay release()
   * by invoking sample_bi_() from within complete_bi_().
   *
   * @warning Both @p qnotify and @p notify are invoked with @ref
   * mutex_type held, generally from the ADC FLIH.
   *
   * @param notify the notification callback to be passed to sample()
   * or calibrate().
   *
   * @param qnotify function invoked when the requested operation is
   * initiated, to provide the status of the initiation attempt to the
   * application.  Since the operation is invoked only when it is
   * known that the ADC is available the implicit claim() and the
   * sample()/calibrate() operation will both succeed and parameter
   * should always indicate success.
   *
   * @param calibrate if `true` the queued operation invokes
   * calibrate(); if `false` (default) the queued operation invokes
   * sample().
   *
   * @return zero on success, or a negative error code if the
   * operation could not be queued. */
  int queue (const notifier_type& notify,
             const queued_callback_type& qnotify = {},
             bool calibrate = false);

  /** Overload when no notifications are required.
   * @overload */
  int queue (bool calibrate = false)
  {
    return queue({}, {}, calibrate);
  }

protected:
  friend class ADC;

  /** Flags for use by the core infrastructure and client specifications.
   *
   * Bits below #FL_SUBCLASS_BASE are not to be touched by subclasses.
   *
   * All manipulations of this field must be performed @link
   * mutex_type under mutex@endlink. */
  flags_type flags_ = 0;

  /** Implements sample() given that the mutex is already held. */
  int sample_bi_ (const notifier_type& notify)
  {
    return ADC::try_sample_bi_(this, notify);
  }

  static void process_queue_bi_ ();
  void complete_queue_bi_ ();

  /** Method to be invoked to configure the ADC for the next collection.
   *
   * Subclasses should use this to set up the configuration registers
   * in the available ADC peripheral and provide a place in memory
   * into which normalized ADC results will be written.
   *
   * This method is invoked from calibrate() and for each sample().
   *
   * @note For SAADC clients this should ensure that
   * SAADC->CH[0].CONFIG is set up with the desired acquisition time.
   *
   * @return zero on success, or a negative error code.  Non-zero
   * values are returned through sample() or calibrate(). */
  virtual int configure_bi_ ()
  {
    return 0;
  }

  /** Method invoked when the collection completes.
   *
   * Subclasses may implement this to support per-client callbacks
   * that provide post-processed results from the collection.  Classes
   * may also update state and invoke sample_bi_().  For @link queue
   * queued collections@endlink if the state remains @link
   * ADC::state_type::claimed claimed@endlink on completion of this
   * call the client releases the %ADC and processes the remainder of
   * the queue.
   *
   * This method is invoked prior to any per-sample callback installed
   * by the client. */
  virtual void complete_bi_ ()
  { }

  /** Function used to reconfigure ADC for next sample within the
   * collection.
   *
   * This is used to simulate the nRF52's multi-channel ADC on the
   * nRF51 by allowing a client to provide a new `CONFIG` register
   * value that selects another channel.
   *
   * If not overridden only single-channel collections will work on
   * nRF51.
   *
   * @param ci the channel index next to be collected.
   *
   * @return a non-negative value to be used in the nrf5::ADC `CONFIG`
   * register, or a negative value to indicate that collection has
   * completed. */
  virtual int nrf51_next_bi_ (size_t ci)
  {
    return -1;
  }
};

#if (NRF52840 - 0) || (NRFCXX_DOXYGEN - 0)

/** Wrapper around the nRF52 %QSPI peripheral.
 *
 * There is only one %QSPI peripheral, but it may be associated with
 * multiple flash devices each of which is controlled through an
 * instance of this class.  At most one instance may be in use (@link
 * claim claimed@endlink) at any time.
 *
 * The implementation supports deep power-down mode and 24-bit
 * addressing. */
class QSPI
{
public:
  /** An RAII type for mutex access to state that must be
   * protected from %QSPI interrupts. */
  using mutex_type = mutex_irq<QSPI_IRQn>;

  /** Unsigned type specifying flash memory addresses. */
  using offset_type = unsigned int;

  /** Unsigned type holding size of transfers. */
  using size_type = unsigned int;

  /** Signed type holding transfer sizes or error codes. */
  using ssize_type = int;

  /** Bit flag for status register write-in-progress bit. */
  static constexpr uint8_t SR_WIP = 0x01;
  /** Bit flag for status register write-enable-latch bit. */
  static constexpr uint8_t SR_WEL = 0x02;
  /** Bit flag for status register protected area block bit 0. */
  static constexpr uint8_t SR_BP0 = 0x04;
  /** Bit flag for status register protected area block bit 1. */
  static constexpr uint8_t SR_BP1 = 0x08;
  /** Bit flag for status register protected area block bit 2. */
  static constexpr uint8_t SR_BP2 = 0x10;
  /** Bit flag for status register protected area block bit 3. */
  static constexpr uint8_t SR_BP3 = 0x20;
  /** Bit flag for status register quad-enabled bit. */
  static constexpr uint8_t SR_QE = 0x40;
  /** Bit flag for status register write protect bit.
   *
   * Don't set this unless you're sure you'll never want to
   * reconfigure the status register again. */
  static constexpr uint8_t SR_SWRD = 0x80;

  /** Convert a duration expressed in microseconds to the units of the
   * DPMDUR register.
   *
   * DPMDUR expresses durations in multiples of 256 * 62.5 ns, or 16
   * us.  This function rounds up to nearest integral duration that is
   * not less than the provided value.
   *
   * @param dur_us the duration of a deep power-down mode transition,
   * in microseconds.
   *
   * @return that same duration in the representation of `DPMDUR`.
   */
  static constexpr unsigned int convert_us_dpmdur (unsigned int dur_us)
  {
    return (16U * dur_us + 255U) / 256U;
  }

  /** Configuration parameters for %QSPI peripheral.
   *
   * This structure default-initializes to the nRF52840 power-up
   * default values, except for DPMDUR which uses a shorter maximum
   * delay. */
  struct configuration_type
  {
    /** The values for the IFCONFIG0 and IFCONFIG1 registers.
     *
     * @note If IFCONFIG0.READOC or IFCONFIG0.WRITEOC indicates that
     * IO2 and IO3 are required for read or write operations
     * activate() will attempt to ensure the SR.QE field in the device
     * is set before it returns.
     *
     * @note Do not set IFCONFIG1.DPMEN to Enter here.  The
     * implementation assumes the configured value is Exit. */
    uint32_t ifconfig[2] = {0, 0x40480};

    /** The value for the ADDRCONF register. */
    uint32_t addrconf = 0xB7;

    /** The value for the XIPOFFSET register. */
    uint32_t xipoffset = 0;

    /** Pin selectors for IO0 through IO3 */
    int8_t psel_io[4] = {-1, -1, -1, -1};

    /** Pin selector for SCK. */
    int8_t psel_sck = -1;

    /** Pin selector for CSn. */
    int8_t psel_csn = -1;

    /** The value of the DPMDUR.ENTER field.
     *
     * @see convert_us_dpmdur */
    uint8_t enter_dpmdur = 0xFFu;

    /** The value of the DPMDUR.EXIT field.
     *
     * @see convert_us_dpmdur */
    uint8_t exit_dpmdur = 0xFFu;

    /** Set to use high drive for the control and IO pins.
     *
     * If cleared standard drive will be used. */
    bool drive_high = false;
  };

  /** @cond DOXYGEN_EXCLUDE */
  /* You can't copy, assign, move, or otherwise change the set of
   * QSPI instances that the system provides to you. */
  QSPI () = delete;
  QSPI (const QSPI&) = delete;
  QSPI& operator= (const QSPI&) = delete;
  QSPI (const QSPI&&) = delete;
  QSPI& operator= (QSPI&&) = delete;
  /** @endcond */

  /** Construct a new QSPI interface.
   *
   * @warning An attempt to create an instance with an invalid
   * configuration will produce FailsafeCode::API_VIOLATION.
   *
   * @warning An attempt to create an instance when the %QSPI
   * peripheral is associated with another instance will produce
   * FailsafeCode::API_VIOLATION.
   *
   * @warning The provided configuration must be persistent and
   * unmodified for the lifespan of the instance.
   *
   * @param configuration how to talk to the device.  A reference to
   * the passed object is retained by the constructed instance. */
  QSPI (const configuration_type& configuration);

  /** Reference the %QSPI configuration provided on construction. */
  const configuration_type& configuration () const
  {
    return configuration_;
  }

  /** Return the most recent cached flash device status register value.
   *
   * @note The value may be not be fresh. */
  uint8_t latest_sr () const
  {
    return (QSPI_STATUS_SREG_Msk & nrf5::QSPI->STATUS) >> QSPI_STATUS_SREG_Pos;
  }

  /** Read the current device status register value.
   *
   * @retval non-negative the device SR value.
   * @retval -EINVAL if the instance does not have control of the
   * %QSPI peripheral.
   * @retval -EBUSY if the instance is not in a state supporting new
   * commands. */
  int read_sr () const;

  /** Determine whether the %QSPI peripheral is available for new commands.
   *
   * This checks that the STATUS.READY field is marked READY, and the
   * STATUS.DPM field is marked Disabled. */
  bool is_qspi_available () const
  {
    return (((QSPI_STATUS_READY_READY << QSPI_STATUS_READY_Pos)
             | (QSPI_STATUS_DPM_Disabled << QSPI_STATUS_DPM_Pos))
            == ((QSPI_STATUS_READY_Msk | QSPI_STATUS_DPM_Msk)
                & nrf5::QSPI->STATUS));
  }

  /** Test a status register value for completion of all write
   * activities.
   *
   * Although write() is (currently) synchronous, erase() is
   * asynchronous.  Use this function to test whether a requested
   * operation has completed.
   *
   * @param sr the status register value to check.  If the passed
   * value is negative, `false` will be returned, allowing defined
   * behavior when passing the unchecked return value of read_sr().
   *
   * @return `true` iff @p sr is a valid status for which #SR_WIP and
   * #SR_WEL are cleared. */
  bool is_write_complete (int sr) const
  {
    bool rv = false;
    if (0 <= sr) {
      rv = (0 == ((periph::QSPI::SR_WIP | periph::QSPI::SR_WEL) & sr));
    }
    return rv;
  }

  /** Returns the peripheral flag value indicating whether the device
   * is in deep power-down mode.
   *
   * @note The state of this flag is not valid unless the device is
   * currently @link activate active@endlink.
   *
   * @see is_qspi_available() */
  bool is_powerdown () const
  {
    return (QSPI_STATUS_DPM_Msk & nrf5::QSPI->STATUS);
  }

  /** Returns the isolated peripheral flag value indicating whether
   * the device is ready to receive new tasks or mode transisions.
   *
   * @see is_qspi_available() */
  bool is_ready () const
  {
    return (QSPI_STATUS_READY_Msk & nrf5::QSPI->STATUS);
  }

  /** Read data from the device at a specified address.
   *
   * @param addr the offset within the flash device from which data
   * will be read.
   *
   * @param dest the RAM location into which data will be stored.
   *
   * @param count the number of bytes to read from the device.
   *
   * @retval count if the read succeeds.
   * @retval -EINVAL if this instance doesn't own the peripheral.
   * @retval -EBADF if the instance is not activated..
   * @retval -EBUSY if this instance is actively using the peripheral. */
  ssize_t read (offset_type addr,
                void* dest,
                size_type count);

  /** Write data to the device at a specified address.
   *
   * @note It is the caller's responsibility to ensure that the flash
   * area that will be written is adequately erased.
   *
   * @warning At this time this call is synchronous: it will not
   * return until the write has completed, which may take some
   * significant amount of time (roughly 1.5 ms per KiBy at 32 MHz).
   * The call will not return until the device has completed the write
   * (is_write_complete() will succeed).
   *
   * @param addr the offset within the flash device at which data will
   * be written.
   *
   * @param dest the RAM location from which data will be transferred.
   *
   * @param count the number of bytes to write to the device.
   *
   * @retval count if the write succeeds.
   * @retval -EINVAL if this instance doesn't own the peripheral.
   * @retval -EBADF if the instance is not activated..
   * @retval -EBUSY if this instance is actively using the peripheral. */
  ssize_t write (offset_type addr,
                 const void* src,
                 size_type count);

  /** Flag value denoting intent to erase a single 4 KiBy sector.
   *
   * Passed to the @p type argument of erase().  The corresponding
   * @p addr argument must be aligned to a 4 KiBy boundary. */
  static constexpr auto ERASE_4_KB = static_cast<uint8_t>(QSPI_ERASE_LEN_LEN_4KB) << QSPI_ERASE_LEN_LEN_Pos;

  /** Flag value denoting intent to erase a single 64 KiBy block.
   *
   * Passed to the @p type argument of erase().  The corresponding
   * @p addr argument must be aligned to a 64 KiBy boundary. */
  static constexpr auto ERASE_64_KB = static_cast<uint8_t>(QSPI_ERASE_LEN_LEN_64KB) << QSPI_ERASE_LEN_LEN_Pos;

  /** Flag value denoting intent to erase the entire chip.
   *
   * Passed to the @p type argument of erase().  The corresponding
   * @p addr argument must be zero. */
  static constexpr auto ERASE_CHIP = static_cast<uint8_t>(QSPI_ERASE_LEN_LEN_All) << QSPI_ERASE_LEN_LEN_Pos;

  /** Initiate an erase of a single flash storage item.
   *
   * @note Erase takes significant amounts of time, possibly measured
   * in seconds.  This call returns as soon as the erase has request
   * has been initiated.  Use is_write_complete() to detect when the
   * erase completes.
   *
   * @param type one of #ERASE_4_KB, #ERASE_64_KB, or #ERASE_CHIP.
   * Other values are rejected.
   *
   * @param addr an offset within the flash consistent with the value
   * passed to @p type.
   *
   * @retval 0 on successful initiation
   * @retval -EINVAL if this instance doesn't own the peripheral, or
   * @p addr is inconsistent with @p type.
   * @retval -EBADF if the instance is not activated..
   * @retval -EBUSY if this instance is actively using the peripheral. */
  int erase (uint8_t type,
             offset_type addr = -1);

  /** Return the JEDEC ID cached on the first activation of the
   * instance.
   *
   * This is a standard unsigned integer embedding the JEDEC
   * manufacturer ID in bits 16..23, and a manufacturer-specific
   * device id in bits 0..15. */
  unsigned int jedec_id () const
  {
    return jedec_id_;
  }

  /** Get a pointer to the instance that owns the %QSPI peripheral. */
  static QSPI* owner ()
  {
    return owner_;
  }

  /** Test whether this instance is already activated. */
  bool is_activated () const
  {
    return activated_;
  }

  /** Test whether this instance has control of the %QSPI peripheral. */
  bool is_owner () const
  {
    return this == owner_;
  }

  /** Attempt to claim use of the %QSPI peripheral.
   *
   * This configures the peripheral to support this device, and
   * enables it.  It does not #activate it.
   *
   * There is no power consumption penalty for having %QSPI configured
   * and enabled, only for having it @link activate activated@endlink.
   *
   * @retval Zero on success
   * @retval -EBUSY if another instance owns the peripheral */
  int claim ();

  /** Release the %QSPI peripheral if held by this instance.
   *
   * This @link deactivate deactivates@endlink and disables the %QSPI
   * peripheral, making it available for subsequent claim() attempts.
   *
   * @retval Zero on success
   * @retval -EINVAL if this instance doesn't own the peripheral
   * @retval -EBUSY if this instance is actively using the
   * peripheral.
   */
  int release ();

  /** Activate the %QSPI peripheral to interact with this device.
   *
   * Activation ensures the device is not in deep powerdown mode, that
   * the device is configured for #SR_QE if the peripheral requires
   * quad I/O signals, and that the %QSPI `STATUS` register is current.
   * On the first activation the @link jedec_id JEDEC ID@endlink is
   * collected. */
  int activate ();

  /** Deactivate the %QSPI peripheral.
   *
   * Deactivation includes putting the controlled device in to deep
   * power-down mode (if supported), and deactivating the %QSPI
   * peripheral.  The peripheral remains enabled until release() is
   * invoked. */
  int deactivate ();

private:
  /* The QSPI instance that has control of the %QSPI peripheral. */
  static QSPI* owner_;

  const configuration_type& configuration_;
  unsigned int jedec_id_ = 0;
  bool activated_ = false;
};

#endif /* NRF52840 */

} // namespace periph
} // namespace nrfcxx

#endif /* NRFCXX_PERIPH_HPP */
