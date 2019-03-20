/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Primary header for nrfcxx interface dependencies.
 *
 * Including this module introduces a dependency on CMSIS headers and
 * Nordic device peripheral structures.  It does not import the Nordic
 * device bitfield macro definitions as with <nrfcxx/impl.hpp>.
 *
 * The ::nrfcxx namespace defines nrfcxx::peripheral instances for all
 * peripherals on the supported product, such as `CLOCK`, `TWI0`, or
 * `SAADC`.  Some peripherals are common and compatible between
 * products; others are only available on a specific product.  Note
 * that these objects appear in the documentation regardless of series
 * and availability on a specific product.
 *
 * @anchor nrfcxx_mutex Mutex support classes:
 * * @link nrfcxx::primask@endlink
 * * @link nrfcxx::null_mutex@endlink
 * * @link nrfcxx::nvic_BlockIRQ@endlink and @link nrfcxx::mutex_irq@endlink
 *
 * @file */

#ifndef NRFCXX_CORE_HPP
#define NRFCXX_CORE_HPP
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include <pabigot/container.hpp>

#include <nrfcxx/version.hpp>

/** @cond DOXYGEN_EXCLUDE */
extern "C" {
extern char __HeapBase;       ///< symbol at start of heap region
extern char __HeapLimit;      ///< symbol at end of heap region
extern char __StackLimit;     ///< symbol at bottom of stack region
extern char __StackTop;       ///< symbol at top of stack region
}
/** @endcond */

#ifndef NRFCXX_FAKED
/** Macro defined to an nRF5 series number for host-based testing.
 *
 * When defined to a non-zero value this causes the nRF5 register,
 * bitfield, and CMSIS headers to be included even when not
 * cross-compiling.  This allows references to structures and
 * constants in header files even though the execution environment
 * won't provide them.
 *
 * In that situation #NRFCXX_CROSS_COMPILING should be a preprocessor
 * false (i.e. 0). */
#define NRFCXX_FAKED 0
#endif /* NRFCXX_FAKED */

#if (NRF51 - 0) || (NRFCXX_DOXYGEN - 0) || (NRFCXX_FAKED - 0)
/** Nordic nRF5 series.
 *
 * Derived from the externally provided device identifier (`NRF51`,
 * `NRF52832`, etc) this is used in preprocessor directives when
 * series-specific API is required.  The value is either 51 or 52. */
#define NRF_SERIES 51
#elif (NRF52832 - 0) || (NRF52840 - 0)
#define NRF_SERIES 52
#elif (NRF52810 - 0)
/* NRF52810 has a different set of peripherals.  Add that when
 * we have hardware that needs it. */
#error NRF52810 not currently supported
#else /* NRF device */
#error Unrecognized NRF5 device
#endif /* NRF device */

/** Primary namespace for nrfcxx functionality */
namespace nrfcxx {

/** Namespace holding board-specific configuration data.
 *
 * Most material is put into this namespace through the board-specific
 * <nrfcxx/board.hpp> header. */
namespace board {

/** Perform board-specific initialization.
 *
 * This function should be invoked at the start of main() to ensure
 * the basic functionality expected of all boards is available.
 *
 * Operations performed by the default implementation include:
 * * clock::initialize() is invoked passing @p enable_hfxt.  This
 *   ensures the clock::uptime infrastructure is present.
 * * Other boards may need to provide other actions.
 *
 * @return zero in almost all cases.  If a negative value is returned
 * some applications may be able to enter a failsafe mode. */
int initialize (bool enable_hfxt = false); // weak implemented in src/core.cc

} // ns board

/** Namespace holding support for bare nRF5 peripheral instances.
 *
 * Nordic nRF5 peripherals are defined in the vendor headers with
 * structure layouts (`NRF_TWI_Type`) with instances at specified
 * addresses (`NRF_TWI0_BASE`) and IRQ numbers (`TWI0_IRQn`).  Some
 * peripherals have multiple instances.  The content of the
 * structures, addresses, and instance is specific to the product
 * (nRF51822 vs nRF52832, etc.).
 *
 * This namespace provides types and constant values allowing generic
 * reference to peripherals across the product line.  Use of
 * `constexpr` summaries simplifies changing the product or peripheral
 * instance in a single location while leaving code references to the
 * peripheral both optimized (accesses the peripheral registers
 * directly) and generic.
 *
 * For a peripheral like `TWI0` the namespace would provide:
 * * `TWI_Type` as a specialization of nrf::peripheral providing
 *   product-specific `NRF_TWI_Type` structure access along with base
 *   address, IRQ information, instance ordinal, and other relevant
 *   information via nrf::peripheral::AUX.
 * * Instances of `TWI_Type` for `TWI0`, `TWI1`, etc.
 * * Correctly typed shadow instances like `TWIM0`, `TWIS0`, `SPIM0`
 *   when the same peripheral address is shared among multiple
 *   peripherals.
 *
 * Code can then generically access the instance registers like
 * `TWI0->ADDRESS` without need for conditional compilation directives
 * or other overt distinction between products in the application or
 * abstraction code. */
namespace nrf5 {

/** Namespace holding series-specific implementations that
 * support the genericized API of nrfcxx. */
namespace series {

#if (NRFCXX_DOXYGEN - 0)
/** Identifier for series-specific @link
 * nrfcxx::periph::ADC::irq_handler ADC IRQ entrypoint@endlink.
 *
 * For nRF51 this is `ADC_IRQHandler`.  For nRF52 this is
 * `SAADC_IRQHandler.` The define is provided by the series-specific
 * `core.hpp` header. */
#define ADCSeriesVariant_IRQHandler [SA]ADC_IRQHandler
#endif /* NRFCXX_DOXYGEN */

} // ns series

/** Capture information about an nRF5 peripheral instance.
 *
 * All instances of this clas, provided by the series-specific
 * extension included through <nrfcxx/core.hpp>, are `static
 * constexpr` so with standard optimization there is no data object
 * taking up space and requiring memory access to get the peripheral
 * address.  An exception is for material in nrfcxx::periph where an
 * abstraction object must capture the underlying peripheral instance
 * by reference.  The instances in different modules are at different
 * addresses, but have identical content, so detecting whether a
 * specific peripheral instance is used should be done by comparing
 * the #BASE field.
 *
 * @tparam S the type describing the peripheral structure,
 * e.g. `NRF_GPIO_Type`. */
template <typename S>
struct peripheral {
  /** The structure describing the instance content, such as
   * `NRF_GPIO_Type` or `NRF_UART_Type`. */
  using Type = S;

  /** Flag value for #IRQn indicating that the peripheral does
   * not have an assign interrupt vector entry. */
  static constexpr int8_t NO_IRQ = -128;

  /** Flag value for #INSTANCE indicating that the peripheral
   * does not have enumerated instances (e.g. `CLOCK`, as compared to
   * `TIMER0`). */
  static constexpr uint8_t NO_INSTANCE = 255;

  /** Create an object referencing a peripheral instance.
   *
   * @param base initializes #BASE
   * @param irqn initializes #IRQn */
  constexpr explicit peripheral (uintptr_t base,
                                 int8_t irqn = NO_IRQ,
                                 uint8_t instance = NO_INSTANCE,
                                 uint8_t aux = 0) :
    BASE{base},
    IRQn{irqn},
    INSTANCE{instance},
    AUX{aux}
  { }

  /** Get a type-correct pointer to the peripheral
   * structure. */
  Type* instance() const
  {
    return reinterpret_cast<Type*>(BASE);
  }

  /** Allow dereferencing the instance as though it were a
   * pointer to the peripheral structure. */
  Type* operator->() const
  {
    return instance();
  }

  /** The address of the instance. */
  const uintptr_t BASE;

  /** The interrupt number associated with the instance, or negative
   * if the instance has no dedicated interrupt.
   *
   * @note The Nordic headers define `IRQn_Type` as an enumeration
   * with default underlying type in a header that we can't see until
   * we've declared this structure, so we can't declare this with the
   * standard type unless we want to do horrifying dependency
   * avoidance tricks, which we don't.  A consequence is that, to
   * satisfy C++'s type finickiness, CMSIS standard functions like
   * `NVIC_EnableIRQ()` must be wrapped with customized versions like
   * nvic_EnableIRQ() to perform the necessary cast operation. */
  const int8_t IRQn;

  /** The peripheral instance, for peripherals like `TIMER`
   * that have multiple instances.
   *
   * #NO_INSTANCE is used for peripherals that have no instances, such
   * as `CLOCK`. */
  const uint8_t INSTANCE;

  /** Auxiliary information relevant to the specific peripheral
   * and type.
   *
   * * `GPIO` instances use this field to specify the number of pins
   *   supported by the instance.
   * * `GPIOTE` and `SAADC` instances specify the number of channels
   *   available.
   *
   * * `PPI` instances specify the number of programmable
   *   interconnections.
   * * `TIMER` and `RTC` instances use this field to specify the
   *    number of CC registers supported by the peripheral
   *    instance.
   *
   * * For other peripherals the value is zero. */
  const uint8_t AUX;
};

/** A traits type identifying GPIO peripheral instances.
 *
 * The following members are to be defined:
 * * `nrf5::GPIO_Type instance` is the peripheral instance;
 * * `int begin_psel` is the lowest ordinal GPIO pin falling within the instance;
 * * `int end_psel` is the lowest ordinal GPIO pin falling above the instance.
 *
 * @note The instance `GPIO` is always an alias for `P0`.
 *
 * @tparam I the peripheral<GPIO_Type>::INSTANCE value used to
 * identify an instance. */
template <int I>
struct GPIO_Instance
{ };

} // nrf5
} // nrfcxx

#ifndef NRFCXX_CROSS_COMPILING
/** Macro defined to preprocessor true when cross-compiling.
 *
 * This is defined to preprocessor false when building on a host for
 * non-embedded testing of implementation. */
#define NRFCXX_CROSS_COMPILING 1
#endif /* NRFCXX_CROSS_COMPILING */

#if (NRFCXX_CROSS_COMPILING - 0)
#if (51 == NRF_SERIES)
#include <nrfcxx/nrf51/core.hpp>
#elif (52 == NRF_SERIES)
#include <nrfcxx/nrf52/core.hpp>
#endif // NRF_SERIES
#elif (NRFCXX_FAKED - 0)
#include <nrfcxx/faked/core.hpp>
#else
using IRQn_Type = int;
#endif /* NRFCXX_CROSS_COMPILING */

namespace nrfcxx {

/** Delay for exactly the specified duration.
 *
 * The implementation is series-specific. */
inline void __attribute__((__gnu_inline__,__always_inline__))
delay_us (unsigned int number_of_us)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  if (number_of_us) {
    nrf5::series::delay_cycles(nrf5::series::CLOCK_MHz * number_of_us - nrf5::series::DELAY_US_OVERHEAD_cyc);
  }
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Sleep for the specified duration.
 *
 * @note This delegates to clock::uptime::sleep().  If the uptime
 * clock is not running this function may not return.  Durations at or
 * above PT18H12M16S may misbehave.
 *
 * @note If interrupts occur during the sleep this function will
 * return to sleep until the requested duration passes.  If early exit
 * is desired use clock::uptime::sleep().
 *
 * @param dur_ms the time to sleep, in milliseconds. */
void sleep_ms (unsigned int dur_ms);

/** RAII class that performs no mutex operations.
 *
 * This is used as the default value for template parameters that
 * identify the mutex required to protect an operation in cases where
 * the operation may not need protection. */
class null_mutex
{
public:
  null_mutex ()
  { }

  null_mutex (const null_mutex&) = delete;
  null_mutex& operator= (const null_mutex&) = delete;
  null_mutex (null_mutex&& ) = delete;
  null_mutex& operator= (null_mutex&) = delete;
};

/** RAII class to block exceptions.
 *
 * The PRIMASK configuration is recorded and then disabled.  When the
 * instance is destructed the recorded PRIMASK configuration is
 * restored.
 *
 * Note that this class is safe to use in contexts where interrupts
 * are already disabled: they will not be re-enabled when the object
 * is destructed. */
class primask
{
public:
  primask () :
    in_mask_{}
  {
#if (NRFCXX_CROSS_COMPILING - 0)
    __ASM volatile ("mrs\t%0, primask\n\t"
                    "cpsid\ti"
                    : "=r" (in_mask_)
                    :
                    : "memory");
#endif /* NRFCXX_CROSS_COMPILING */
  }

  ~primask ()
  {
#if (NRFCXX_CROSS_COMPILING - 0)
    __ASM volatile ("msr\tprimask, %0"
                    :
                    : "r" (in_mask_)
                    : "memory");
#endif /* NRFCXX_CROSS_COMPILING */
  }

  primask (const primask&) = delete;
  primask& operator= (const primask&) = delete;
  primask (primask&& ) = delete;
  primask& operator= (primask&) = delete;

private:
  uint32_t in_mask_;
};

/** Wrapper around `NVIC_EnableIRQ` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_EnableIRQ (int irqn)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_EnableIRQ(static_cast<IRQn_Type>(irqn));
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Wrapper around `NVIC_DisableIRQ` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_DisableIRQ (int irqn)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_DisableIRQ(static_cast<IRQn_Type>(irqn));
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Wrapper around `NVIC_GetPending` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_GetPendingIRQ (int irqn)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_GetPendingIRQ(static_cast<IRQn_Type>(irqn));
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Wrapper around `NVIC_SetPending` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_SetPendingIRQ (int irqn)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_SetPendingIRQ(static_cast<IRQn_Type>(irqn));
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Wrapper around `NVIC_ClearPending` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_ClearPendingIRQ (int irqn)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(irqn));
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Wrapper around `NVIC_SetPriority` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_SetPriority (int irqn, uint32_t priority)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_SetPriority(static_cast<IRQn_Type>(irqn), priority);
#endif /* NRFCXX_CROSS_COMPILING */
}

/** Wrapper around `NVIC_GetPriority` to work around issues
 * with peripheral::IRQn. */
static inline void nvic_GetPriority (int irqn)
{
#if (NRFCXX_CROSS_COMPILING - 0)
  NVIC_GetPriority(static_cast<IRQn_Type>(irqn));
#endif /* NRFCXX_CROSS_COMPILING */
}

/** RAII class to block a peripheral interrupt.
 *
 * The NVIC configuration for the specified IRQ is recorded and the
 * interrupt disabled when the instance is constructed.  When the
 * instance is destructed the interrupt is enabled if it was enabled
 * when constructed.
 *
 * This allows safe manipulation of state that is shared between the
 * application and the interrupt handler, assuming that instances of
 * this class are created (for a specific IRQ) only when not invoked
 * from within (recursively) the corresponding handler.
 *
 * Note that if the code invoked from within the blocked region calls
 * functions that independently block the same interrupt, the
 * interrupt will not be prematurely re-enabled. */
class nvic_BlockIRQ
{
public:
  explicit nvic_BlockIRQ (IRQn_Type irqn) :
    in_mask_{}
  {
#if (NRFCXX_CROSS_COMPILING - 0)
    uint32_t mask = 1U << (0x1F & (unsigned int)irqn);
    in_mask_ = mask & NVIC->ISER[0];
    NVIC->ICER[0] = mask;
#endif /* NRFCXX_CROSS_COMPILING */
  }

#if (NRFCXX_CROSS_COMPILING - 0)
  /** Variant required to work around lack of IRQn_Type when
   * declaring nrfcxx::peripheral. */
  explicit nvic_BlockIRQ (int irqn) :
    nvic_BlockIRQ{static_cast<IRQn_Type>(irqn)}
  { }
#endif /* NRFCXX_CROSS_COMPILING */

  ~nvic_BlockIRQ ()
  {
#if (NRFCXX_CROSS_COMPILING - 0)
    NVIC->ISER[0] = in_mask_;
#endif /* NRFCXX_CROSS_COMPILING */
  }

  nvic_BlockIRQ (const nvic_BlockIRQ&) = delete;
  nvic_BlockIRQ& operator= (const nvic_BlockIRQ&) = delete;
  nvic_BlockIRQ (nvic_BlockIRQ&& ) = delete;
  nvic_BlockIRQ& operator= (nvic_BlockIRQ&) = delete;

private:
  uint32_t in_mask_;
};

/** nvic_BlockIRQ as a template type.
 *
 * This supports IRQ mutex in contexts where the mutex type has a
 * nullary constructor.
 *
 * @tparam IRQn the integer IRQ number that is to be blocked by the
 * mutex instance. */
template <IRQn_Type IRQn>
class mutex_irq : public nvic_BlockIRQ
{
public:
  mutex_irq () :
    nvic_BlockIRQ(IRQn)
  { }
};

/** Type used to hold a notifier.
 *
 * This type is used when an operation must record a callback to be
 * invoked when it completes, and that callback does not transfer any
 * information outside of the fact of its being invoked.
 *
 * Values compatible with this are nullary static functions and
 * lambdas with no arguments.  A common source for such a value is
 * event_set::make_setter(). */
using notifier_type = std::function<void()>;

/* Forward declaration */
class event_set_copy;

/** A record of events that occur asynchonously.
 *
 * This class supports recording up to 32 distinct types of event for
 * the application to process at its convenience.
 *
 * The order that the events occurred is not retained, nor is the fact
 * of duplicate events.
 *
 * Example:
 *
 *     // main loop
 *     while (true) {
 *       nrfcxx::event_set::cev();
 *       auto pending = my_events.copy_and_clear();
 *       if (pending.fetch_and_clear(EVT_EVT0)) {
 *         // ...
 *       }
 *       // ...
 *       __WFE(); // sleep until next IRQ-driven event
 *     }
 *
 */
class event_set
{
public:
  /** The type used to represent a (set of) event(s).
   *
   * Each event is an independent bit in the word representation. */
  using event_type = unsigned int;

  /** Construct with an optional initial set of events */
  constexpr explicit event_set (event_type events = 0U) :
    events_{events}
  { }

  event_set (const event_set&) = delete;
  event_set& operator= (const event_set&) = delete;
  event_set (event_set&&) = delete;
  event_set& operator= (event_set&&) = delete;

  /** Implicit cast to bool, true if there are events pending. */
  operator bool() const
  {
    return events_;
  }

  /** Return the current event set. */
  event_type fetch () const
  {
    return events_;
  }

  /** Safely clear the MCU event flag.
   *
   * This function should be invoked prior to fetch_and_clear() for
   * event sets that record fact-of interrupt occurrence.  Doing so
   * ensures that `__WFE()` invoked after processing events will not
   * suspend if an interrupt occurred while or after the event records
   * were fetched, but will suspend if no interrupts occurred after
   * invoking cev(). */
  static void cev ()
  {
#if (NRFCXX_CROSS_COMPILING - 0)
    __SEV();
    __WFE();
#endif /* NRFCXX_CROSS_COMPILING */
  }

  /** Atomically return the current event set and clear the
   * event record. */
  event_type fetch_and_clear ()
  {
    primask mutex;
    event_type rv = events_;
    events_ = {};
    return rv;
  }

  /** Create a non-mutex copy then clear the current event set.
   *
   * The resulting object can be used to process individual events
   * without incuring mutex costs. */
  event_set_copy copy_and_clear ();

  /** Atomically record one or more events.
   *
   * This sets the processor event register after the event set has
   * been updating, ensuring a subsequent `__WFE()` will not block
   * even if the event was not set in an interrupt handler. */
  void set (event_type events)
  {
    primask mutex;
    events_ |= events;
#if (NRFCXX_CROSS_COMPILING - 0)
    __SEV();
#endif /* NRFCXX_CROSS_COMPILING */
  }

  /** Reset the event set to a specific state. */
  void reset (event_type events = 0U)
  {
    primask mutex;
    events_ = events;
  }

  /** Return an invokable object that records events in this set.
   *
   * Example:
   *
   *     auto setter = events.make_setter(EVT_ALARM);
   *     // ...
   *     if (alarmed) {
   *       setter();
   *     }
   *
   * @param events the events that should be set by the returned
   * notifier.
   *
   * @warning There is no link from an @link event_set@endlink to a
   * @link setter setter@endlink that references it.  If an event set
   * instance has a limited lifetime then before the set is destroyed
   * any infrastructure that has recorded a setter for it should be
   * updated to clear that setter. */
  notifier_type make_setter (event_type events)
  {
    return [this,events]
      {
        this->set(events);
      };
  }

private:
  volatile event_type events_;
};

/** A helper class for processing snapshot nrfcxx::event_set
 * values.
 *
 * Generally obtained through event_set::copy_and_clear(), and used to
 * test for individual events without incurring mutex overhead. */
class event_set_copy
{
public:
  /** The type used to represent a (set of) event(s).
   *
   * Each event is an independent bit in the word representation. */
  using event_type = event_set::event_type;

  /** Capture events and reset the original set */
  event_set_copy (event_set& events) :
    events_{events.fetch_and_clear()}
  { }

  /** Create an empty event set. */
  event_set_copy () = default;

  /** Implicit cast to bool, true if there are events pending. */
  operator bool() const
  {
    return events_;
  }

  /** Return @c true iff no events remain */
  bool empty () const
  {
    return !events_;
  }

  /** Return the bitmask of remaining events */
  event_type events () const
  {
    return events_;
  }

  /** Add a new event to the set */
  void set (event_type events)
  {
    events_ |= events;
  }

  /** Return @c true iff @p evt was set.
   *
   * @p evt is cleared from the remaining events. */
  bool test_and_clear (event_type evt)
  {
    bool rv = (events_ & evt);
    events_ &= ~evt;
    return rv;
  }

private:
  event_type events_ = {};
};

inline
event_set_copy
event_set::copy_and_clear ()
{
  return event_set_copy(*this);
}

/** Enumerated constants used in failsafe() calls.
 *
 * The values of codes listed here are public API and shall not
 * change. */
enum class FailSafeCode : unsigned int
{
  /** Base for system-assigned fail-safe codes. */
  SYSTEM_BASE = 0xbad00000,

  /** Application attempted to retrieve a non-existent
   * peripheral instance. */
  NO_SUCH_PERIPHERAL = SYSTEM_BASE + 1,

  /** Exceeded allocated space of a @link utility::memory_pool
   * memory pool@endlink. */
  MEMORY_POOL = SYSTEM_BASE + 2,

  /** Allocated @link ::_nrfcxx_sbrk_error too much@endlink
   * from system heap. */
  HEAP_OVERRUN = SYSTEM_BASE + 3,

  /** @link validate_stack_pointer Detected@endlink stack
   * pointer into heap memory. */
  STACK_OVERFLOW = SYSTEM_BASE + 4,

  /** Default code for violation of a @link
   * nrfcxx::utility::Persist persisted memory@endlink region.
   *
   * A candidate recovery method is to perform a factory reset of
   * persisted configuration data. */
  PERSIST_VIOLATION = SYSTEM_BASE + 5,

  /** Unspecified internal error. */
  INTERNAL_ERROR = SYSTEM_BASE + 6,

  /** Application failed to perform all steps required to run. */
  INCOMPLETE_SETUP = SYSTEM_BASE + 7,

  /** Application attempted to allocate more resources than available.
   *
   * An example would be creating multiple instances of a class for
   * which a single instance is allowed, or logging a failure in @link
   * periph::PPI::request PPI allocation@endlink. */
  RESOURCE_VIOLATION = SYSTEM_BASE + 8,

  /** Application tried something that isn't allowed. */
  API_VIOLATION = SYSTEM_BASE + 9,

  /** Board setup failed in some critical way.
   *
   * This is used for board-specific setup that identifies and
   * configures crucial resources without which the system is not
   * operationally viable.  An example might be allocation of the TWI
   * controller used to configure and manipulate an IO extender.
   *
   * If this failure occurs the only part of the system that can be
   * assumed to be functional is the Nordic MCU itself. */
  BOARD_INIT_FAILURE = SYSTEM_BASE + 10,

  /** Application left the main loop.
   *
   * Use this when something totally unexpected happens and the
   * temporary fix is to break out of the event loop and exit
   * main(). */
  EVENT_LOOP_TERMINATED = SYSTEM_BASE + 11,

  /** Base for application-assigned fail-safe codes.
   *
   * Type-correct application code values can be obtained by adding to
   * this value, as with:
   *
   *     FailSafeCode mycode{FailSafeCode::APPLICATION_BASE + 2}
   */
  APPLICATION_BASE = 0xbad10000,
};
static inline
FailSafeCode operator+ (const FailSafeCode& lhs,
                        unsigned int incr)
{
  return static_cast<FailSafeCode>(static_cast<unsigned int>(lhs) + incr);
}

/** Record a critical system failure and reset the system.
 *
 * This API should be used in lieu of systemState::reset() in
 * situations where normal operation followed a path that led to an
 * unrecoverable fatal error, such as running out of memory during
 * system initialization.  The failure is marked by @link
 * state_type::RESET_REAS_FAILSAFE RESET_REAS_FAILSAFE@endlink being
 * set in @link state_type::reset_reas state().reset_reas@endlink.
 *
 * Applications should react to this failure by either automatically
 * resetting to a default configuration or by providing a minimal
 * capability allowing the fact of the failure to be detected and
 * configuration to mitigate it.
 *
 * @warning This function may be invoked by the infrastructure
 * regardless of whether a systemState instance has been installed,
 * preventing the fact of the failure from being discovered.  In
 * that situation the device will likely continually reset and may
 * not provide a mechanism to recover.  Any operational firmware
 * should install a system state instance and be prepared to process
 * a failsafe reset. */
[[noreturn]]
inline __attribute__((__always_inline__))
void failsafe (FailSafeCode code);

[[noreturn]]
inline __attribute__((__always_inline__))
void failsafe (unsigned int code);

/** A class supporting watchdog configuration and cross-reset
 * retention of state.
 *
 * Applications may encounter situations where they can no longer
 * function and need to restart.  These situations may be detectable
 * by program code and @link systemState::reset initiated
 * intentionally@endlink; result from a @link
 * systemState::watchdogInit watchdog reset@endlink detecting failure
 * to execute some critical operation in a timely manner; or when an
 * unrecoverable system failure causes a #failsafe restart.
 *
 * Information about the reset is available from the `POWER` module,
 * but for completeness must be accompanied by information persisted
 * from the session that reset.  This class supports that state
 * transfer.
 *
 * Example:
 *
 *     static __attribute__((__section__(".noinit.systemState")))
 *       nrfcxx::systemState::state_type state;
 *     nrfcxx::systemState cs{state}; // constructor invoked before main()
 *     int main ()
 *     {
 *       if (nrfcxx::systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
 *         // bring system up in fail-safe mode
 *       }
 */
class systemState
{
  /** Type used as basis of operational mode bitmask. */
  using om_type = uint8_t;

public:
  /** Bits used to build up an operational mode. */
  enum om_enum : om_type {
    /** Operational mode flag indicating that CPU is turned off.
     *
     * In #operationalModeText this flag is represented in the first
     * character, which is `A` for active (cleared) and `S` for sleeping
     * (set).  Use WFE() to automatically update this part of the
     * mode. */
    OM_SLEEP = 0x01,

    /** Operational mode flag indicating that high-frequency
     * clock is running.
     *
     * In #operationalModeText this flag is represented in the second
     * character, which is `-` for disabled (cleared) and `H` for
     * enabled (set).  Control of this flag is managed by
     * nrfcxx::clock.
     *
     * @note Although nrfcxx::clock will set and clear this flag, when
     * soft devices are used they are responsible for managing HFCLK and
     * this flag will generally remain off even when the clock is
     * enabled. */
    OM_HFCLK = 0x02,

    /** Operational mode flag indicating that 2.4 GHz radio is
     * running.
     *
     * In #operationalModeText this flag is represented in the third
     * character, which is `-` for disabled (cleared) and `R` for
     * enabled (set).  Control of this flag is the responsibility of the
     * application's use of `RADIO_IRQHandler`. */
    OM_RADIO = 0x04,

    /** The number of distinct operational modes supported.
     *
     * @note Code depends on this being a power of two. */
    NUM_OPERATIONAL_MODES = (OM_RADIO << 1),
  };

  /** Query or control whether the soft device is expected to be
   * enabled.
   *
   * Applications that enable a soft device should invoke this to tell
   * the infrastructure.  Operations that use resources that are
   * restricted by the soft device should use this to determine
   * whether the SoC Library API or native interaction should be used.
   *
   * @param on a positive value to record the softdevice as being
   * active; zero to record the softdevice as being inactive; a
   * negative value to return the current state without changing it.
   *
   * @return `true` iff the softdevice is recorded as being active
   * after any changes made by the call. */
  static bool softdevice_is_enabled (int on = -1);

  /** Update the operational mode.
   *
   * @param om_clear bits in the operational mode that are to be
   * turned off.  Pass zero to leave all currently set bits set.
   *
   * @param om_set bits in the operational mode that are to be turned
   * on.  Pass zero to leave all currently cleared bits clear.
   *
   * @return the number of uptime clock ticks since the last change to
   * the operational mode.
   *
   * @note Because of the resolution of the time tracking variables
   * this function must be invoked at least once every 512 s to ensure
   * that tick counter wraps are measured properly.
   *
   * @see OM_SLEEP
   * @see OM_HFCLK
   * @see OM_RADIO
   */
  static unsigned int updateOperationalMode (unsigned int om_clear = 0,
                                             unsigned int om_set = 0);

  /** Text representations of each operational mode. */
  static const char* const operationalModeText[NUM_OPERATIONAL_MODES];

  /** RAII instance that configures the operational mode to enter
   * #OM_SLEEP when constructed, and to leave #OM_SLEEP when
   * destructed.
   *
   * This is used in WFE() and can be used when the operational
   * implementation of sleeping is not `__WFE()`, e.g. when it's
   * `sd_app_evt_wait()`.
   *
   * @see make_scoped_sleeper() */
  class scoped_sleeper
  {
    scoped_sleeper (const scoped_sleeper&) = delete;
    scoped_sleeper& operator= (const scoped_sleeper&) = delete;
    scoped_sleeper (scoped_sleeper&&) = delete;
    scoped_sleeper& operator= (scoped_sleeper&&) = delete;

  public:
    ~scoped_sleeper () noexcept
    {
      ++systemState::wfe_count_;
      if (track_om) {
        updateOperationalMode(OM_SLEEP, 0);
      }
    }

    /** `true` iff time-in-mode is being tracked by this instance. */
    const bool track_om;

  private:
    friend systemState;

    scoped_sleeper (bool track_om) :
      track_om{track_om}
    {
      if (track_om) {
        updateOperationalMode(0, OM_SLEEP);
      }
    }
  };

  /** Construct an RAII object to @link scoped_sleeper track sleep periods@endlink.
   *
   * Example:
   *
   *     {
   *       auto sleeper = systemState::make_scoped_sleeper;
   *       sd_app_evt_wait();
   *     }
   */
  static scoped_sleeper make_scoped_sleeper () noexcept
  {
    return {statep_};
  }

  /** Issue a `__WFE` instruction that records time spent
   * sleeping.
   *
   * This invokes updateOperationalMode() before and after the `__WFE`
   * passing #OM_SLEEP appropriately to track the sleep time.
   */
  static void WFE ();

  /** The raw data supporting cross-reset state transfer.
   *
   * An instance of this should be declared in an uninitialized data
   * section then used as the parameter when constructing a single
   * #systemState instance.  The raw state object should not be
   * accessed directly by application code; instead use @link
   * systemState::state cs.state()@endlink.
   */
  struct state_type {
    /** Constant encoding a revision number marking the last change to
     * the layout or interpretation of this structure.
     *
     * This value should be updated whenever structure content is
     * changed in a way that would make a program inspecting an
     * instance from a previous execution to misinterpret it. */
    static constexpr uint32_t DECL_MAGIC = 0x20181013U;

    /** A magic number that is used to determine the state is valid.
     *
     * The value is specific to the #systemState instance used to wrap
     * the state data, allowing it to be derived from material such as
     * a compile-time constant encoding an image version or from a
     * program checksum.  The application-provided base magic number
     * is convolved with the size of this structure and @ref
     * DECL_MAGIC to increase the likelihood that changes to the
     * content of this structure will invalidate state on a system
     * reset. */
    uint32_t magic;

    /** Time of last change to operational mode.
     *
     * This is the value of clock::uptime::now24() the last time
     * systemState::updateOperationalMode() was invoked. */
    unsigned int om_updated;

    /** Cumulative time spent in each operational mode since
     * restart. */
    uint64_t om_total[NUM_OPERATIONAL_MODES];

    /** Duration of the last system session, in uptime clock
     * ticks.
     *
     * This is calculated on restart by summing the individual
     * components of #om_total from the previous session.
     *
     * @note This time does not incorporate any time waiting for a
     * running watchdog to expire when systemState::reset() or other
     * system reset causes have occurred. */
    uint64_t last_uptime;

    /** Total of #last_uptime values since the state was reset.
     *
     * This should be zero only when #reset_count is zero.  On reset,
     * the value stored in #last_uptime is added to the retained value
     * from the previous session. */
    uint64_t total_uptime;

    /** The program counter at which the reset was initiated.
     *
     * This may be zero if the reset cause did not provide a program
     * counter.  It should be correct if #RESET_REAS_PROGRAMMATIC,
     * #RESET_REAS_SDFAULT, #RESET_REAS_WDTBARKED, or
     * #RESET_REAS_FAILSAFE are set in #reset_reas. */
    uint32_t last_pc;

    /** Data recording the code passed to reset() in the
     * previous session.
     *
     * If #RESET_REAS_FAILSAFE is set in #reset_reas this field holds
     * the value passed to failsafe() in the previous session.
     *
     * If #RESET_REAS_PROGRAMMATIC is set in #reset_reas this field
     * holds the value passed to reset() in the previous session.
     *
     * If #RESET_REAS_SDFAULT is set in #reset_reas this field holds
     * the value passed to `info` in sd_fault_handler() in the
     * previous session.  This may be zero.
     *
     * Otherwise this field should be zero. */
    unsigned int code;

    /** Data recording the `id` of a soft-device fault.
     *
     * If #RESET_REAS_SDFAULT is set in #reset_reas this field holds
     * the value passed to `id` in sd_fault_handler() in the previous
     * session.
     *
     * Otherwise this field should be zero. */
    unsigned int sdfault_id;

    /** @cond DOXYGEN_EXCLUDE */
    /* Cache for data to be copied to code during initialization when
     * #RESET_REAS_PROGRAMMATIC, #RESET_REAS_FAILSAFE, or
     * #RESET_REAS_SDFAULT is set in #reset_reas_.
     *
     * This is cleared after initialization completes. */
    unsigned int code_;
    /** @endcond */

    /** The number of times the system has been restarted with
     * cross-reset state successfully transferred.
     *
     * The value after a power-up reset should be zero. */
    uint16_t reset_count;

    /** Base type underlying bits stored in #reset_reas. */
    using reset_reas_type = uint16_t;

    /** Defined bits in #reset_reas.
     *
     * Carries the defined bits from the nRF5 `NRF_POWER->RESETREAS`
     * register packed into an 8-bit value:
     * * Bits 3..0 carry bits 3..0 of `RESETREAS`, providing `RESETPIN`,
     *   `DOG`, `SREQ`, and `LOCKUP` reasons.
     * * Bits 8..4 carry bits 20..16 of `RESETREAS`, providnig `OFF`,
     *   `LPCOMP`, `DIF`, `NFC` (nRF52), and `VBUS` (nRF52840)
     *   reasons.
     *
     * Other bits provide additional information:
     * * Bit 9 is reserved and should be zero.
     * * Bit 10 is #RESET_REAS_INTERNAL.  It should never be visibly set.
     * * Bit 11 is #RESET_REAS_PROGRAMMATIC.  The other bits may
     *   include `DOG` or `SREQ`.
     * * Bit 12 is #RESET_REAS_FAILSAFE.  The other bits may
     *   include `DOG` or `SREQ`.
     * * Bit 13 is #RESET_REAS_WDTBARKED.  The other bits may
     *   include `DOG` or `SREQ`.
     * * Bit 14 is #RESET_REAS_SDFAULT.  The other bits may
     *   include `DOG` or `SREQ`.
     * * Bit 15 is #RESET_REAS_CONTROLLED.  Presence of this bit
     *   indicates that all bits are valid; absence that only the bits
     *   covered by #RESET_REAS_HARDWARE_Msk are valid. */
    enum reset_reas_enum : reset_reas_type
    {
      /** Bit mask for #reset_reas to indicate reset due to the reset
       * pin.
       *
       * This reason will probably never co-occur with
       * #RESET_REAS_CONTROLLED. */
      RESET_REAS_RESETPIN = 0x0001,

      /** Bit mask for #reset_reas to indicate reset due to
       * watchdog. */
      RESET_REAS_DOG = 0x0002,

      /** Bit mask for #reset_reas to indicate reset due to
       * AIRCR.SYSRESETRQ. */
      RESET_REAS_SREQ = 0x004,

      /** Bit mask for #reset_reas to indicate reset due to CPU
       * lock-up. */
      RESET_REAS_LOCKUP = 0x0008,

      /** Bit mask for #reset_reas to indicate wakeup from system OFF
       * mode due to DETECT from the GPIO peripheral. */
      RESET_REAS_OFF = 0x0010,

      /** Bit mask for #reset_reas to indicate wakeup from system OFF
       * mode due to ANADETECT from the LPCOMP peripheral. */
      RESET_REAS_LPCOMP = 0x0020,

      /** Bit mask for #reset_reas to indicate wakeup from system OFF
       * mode due to debug interface mode. */
      RESET_REAS_DIF = 0x0040,

      /** Bit mask for #reset_reas to indicate wakeup from system OFF
       * mode due to NFC field detection.
       *
       * This is available on the nRF52 only, and is provided by the
       * POWER peripheral. */
      RESET_REAS_NFC = 0x0080,

      /** Bit mask for #reset_reas to indicate wakeup from system OFF
       * mode due to VBUS becoming valid.
       *
       * This is available on the nRF52840 only, and is provided by
       * the POWER peripheral. */
      RESET_REAS_VBUS = 0x0100,

      /** Bit mask isolating the bits of #reset_reas that are set from
       * hardware.
       *
       * If #RESET_REAS_CONTROLLED is not set, only the bits isolated by
       * this mask will be defined. */
      RESET_REAS_HARDWARE_Msk = 0x01FF,

      /* Bits below are synthesized; bits above are from nRF5 MCUs.
       * The following bits are available: 9 @ 0x0200. */

      /** Bit mask for #reset_reas to indicate framework-requested
       * reset. */
      RESET_REAS_INTERNAL = 0x0400,

      /** Bit mask for #reset_reas to indicate an
       * application-requested reset.
       *
       * Resets due to invocation of reset() are flagged with this
       * bit.  The #code field provides additional information. */
      RESET_REAS_PROGRAMMATIC = 0x0800,

      /** Bit mask for #reset_reas to indicate a fatal system error
       * from which a reset will not recover.
       *
       * Resets due to invocation of failsafe() are flagged with this
       * bit.  The #code field provides additional information.
       *
       * @see failsafe() */
      RESET_REAS_FAILSAFE = 0x1000,

      /** Bit mask for #reset_reas to indicate a captured watchdog
       * reset.
       *
       * Resets due to a watchdog that reached WDT_IRQHandler() are
       * flagged with this bit. */
      RESET_REAS_WDTBARKED = 0x2000,

      /** Bit mask for #reset_reas to indicate a soft-device fault.
       *
       * Resets due to invocation of sd_fault_handler() are flagged
       * with this bit.  The #code and #sdfault_id fields provide
       * additional information. */
      RESET_REAS_SDFAULT = 0x4000,

      /** Bit mask for #reset_reas to indicate that the reset was
       * controlled.
       *
       * This bit is set if the lowest level reset function was
       * invoked to initiate or wait out the reset.  Absence of the
       * bit on startup indicates that any state content that should
       * have been updated during the reset process may be invalid. */
      RESET_REAS_CONTROLLED = 0x8000,
    };

    /** Captured data from `NRF_POWER->RESETREAS`.
     *
     * @see reset_reas_enum  */
    reset_reas_type reset_reas;

    /** @cond DOXYGEN_EXCLUDE */
    /** Cache for bits to be added to #reset_reas during
     * initialization.
     *
     * Content from this is added to the bits of #reset_reas
     * calculated from `NRF_POWER->RESETREAS` when the system
     * restarts. */
    uint16_t reset_reas_;
    /** @endcond */


    /** Bits identifying watchdog channels that caused a system
     * reset.
     *
     * If non-zero the value contains the value of
     * `NRF_WDT->REQSTATUS` captured in the watchdog interrupt handler
     * just before reset occurred.  Set bits identify the reloads that
     * were not made in time to inhibit the watchdog.
     *
     * If zero the system was reset for reasons that did not include a
     * watchdog timeout, or was due to systemState::reset()
     * (systemState::failsafe()) being invoked when the watchdog was
     * active in which case #RESET_REAS_PROGRAMMATIC
     * (#RESET_REAS_FAILSAFE) should take precedence.
     *
     * @note If this field is zero while `DOG` is set in #reset_reas
     * and #RESET_REAS_PROGRAMMATIC is cleared then a watchdog reset
     * occured at a time when the watchdog interrupt handler was
     * blocked from recording the unloaded channels. */
    uint8_t wdt_status;

    /** Bits identifying the current operational mode.
     *
     * This encodes the current set of capabilities that distinguish operational modes.
     *
     * @see systemState::updateOperationalMode()
     * @see systemState::OM_SLEEP
     * @see systemState::OM_HFCLK
     * @see systemState::OM_RADIO
     */
    uint8_t om_value;
  };

  /** Type of a function allowing application state maintenance.
   *
   * This function is invoked twice:
   * * As the last step of the constructor, to allow
   *   application-specific retained state to be updated based on
   *   history from the previous invocation;
   * * As the last step before initiating or waiting out a controlled
   *   reset.
   *
   * Implementations should reference internally-held or global
   * retained application state.
   *
   * A template for filling in state is:
   *
   *     if (is_reset) {
   *       // Shutting down.  Save anything that has to be saved
   *       // that isn't maintained in the structure supported by this
   *       // function.  If anything is done here, the condition on
   *       // retained should include the test for RESET_REAS_CONTROLLED.
   *     } else if (retained
   *                && (systemState::state_type::RESET_REAS_CONTROLLED & ss.reset_reas)) {
   *       // Starting up and previous shutdown was controlled. Update
   *       // from retained state.
   *     } else {
   *       // Starting up, everything was lost.
   *     }
   *
   * @param sp pointer to the initialized system state
   *
   * @param is_reset `false` on the invocation from the constructor,
   * `true` on the invocation during a controlled reset.
   *
   * @param was_retained valid only when @p is_reset is `false`, this
   * indicates whether the constructor found a valid state object. */
  using app_handler_type = void (*) (const state_type& ss,
                                     bool is_reset,
                                     bool was_retained);

  /** Set up a wrapper around a cross-reset state object.
   *
   * This verifies that the retained state is compatible with `magic`;
   * if not it is cleared.  Information available from the reset is
   * copied into the state object.
   *
   * @warning Invoking this will clear `NRF_POWER->RESETREAS`.  The
   * pre-clear content can be reconstructed from @link
   * state_type::reset_reas state().reset_reas@endlink.
   *
   * @param state reference to a @link systemState::state_type
   * state_type@endlink instance.
   *
   * @param magic the application constant used to derive
   * state_type::magic, which indicates that the state has been
   * properly initialized.  A non-random reasonable value would be the
   * timestamp expressed as decimal YYYYMMDDHH, e.g. 2018101310.
   *
   * @param app_handler optional application handler used to retain
   * application state. */
  systemState (state_type& state,
               uint32_t magic,
               app_handler_type app_handler = nullptr);

  /** Access a read-only instance of the state referenced by
   * this object. */
  const state_type& state () const
  {
    return state_;
  }

  /** Frequency at which the watchdog clock runs. */
  static constexpr unsigned int WATCHDOG_Hz = 32768U;

  /** Pseudo-channel used to support extended watchdog channels.
   *
   * Use this in the @p channel_mask parameter to watchdogInit() to
   * enable the feature. */
  static constexpr auto WATCHDOG_MASK_EXTENDED = (1U << 8);

  /** Configure the watchdog infrastructure.
   *
   * Set up the nRF5 watchdog peripheral and add hooks so that if it
   * fires state will be persisted across the reset.
   *
   * If the watchdog is already running when this function is invoked
   * and the requested configuration matches the existing
   * configuration then the active channels will be fed and the
   * existing session continued to be used.  If the configuration is
   * different from the running configuration state_type::reset_reas
   * will be preserved and the system will wait out the previous
   * watchdog.
   *
   * After configuration watchdogFeed() must be invoked for all
   * channels enabled in @p channel_mask with less than @p
   * delay_32KiHz intervals or the system will reset.
   *
   * If #WATCHDOG_MASK_EXTENDED is present in @p channel_mask then the
   * @ref watchdog_extended_channel infrastructure is enabled, and
   * watchdogCheckExtended() must also be invoked within @p
   * delay_32KiHz.
   *
   * @param delay_32KiHz the value to write to `NRF_WDT->CRV`
   *
   * @param channel_mask a bit mask identifying which watchdog
   * channels are required to complete a reload.  The low 8 bits
   * correspond to the eight available physical watchdog channels.  If
   * #WATCHDOG_MASK_EXTENDED is included then channel 7 is not
   * available and attempting to enable it will produce
   * FailsafeCode::API_VIOLATION.
   *
   * @param run_in_sleep if `true` the watchdog will continue to run
   * while the CPU is inactive.
   *
   * @param run_in_debug if `true` the watchdog will continue to run
   * while the CPU is halted through a debug interface.
   *
   * @return Zero if the watchdog is successfully started, or a
   * negative code if the watchdog is not running and could not be
   * started. */
  int watchdogInit (unsigned int delay_32KiHz,
                    unsigned int channel_mask = 1,
                    bool run_in_sleep = true,
                    bool run_in_debug = false) const;

  /** Check whether the watchdog infrastructure is active.
   *
   * @note If you want to avoid the overhead of a function call in
   * code that includes <nrfcxx/impl.hpp> this is functionally
   * equivalent to reading `NRF_WDT->RUNSTATUS` */
  bool watchdogActive () const;

  /** Reload a specific channel in the watchdog.
   *
   * @param channel the channel to be reloaded.  Allowed values range
   * from 0 through 7.  To be useful the corresponding bit should have
   * been set in `channels` when watchdogInit() was invoked.
   *
   * @note @parblock If you want to avoid the overhead of a function
   * call in code that includes <nrfcxx/impl.hpp> this is functionally
   * equivalent to a bounds-checked execution of:
   *
   *     NRF_WDT->RR[channel] = WDT_RR_RR_Reload;
   *
   * @endparblock */
  void watchdogFeed (unsigned int channel) const;

  /** As with watchdogFeed() but feeds multiple channels.
   *
   * @param channel_mask a bit mask where a set bit indicates a
   * channel index to clear. */
  void watchdogFeedMulti (unsigned int channel_mask) const;

  /** Check that the extended watchdog channels are current.
   *
   * Applications should invoke this within the watchdog peripheral
   * delay to confirm that all extended watchdogs are current.  If so,
   * the underlying hardware channel common to those channels is fed.
   *
   * @return `true` iff all channels are fed.  If this returns
   * `false`, the system will watchdog reset within the current
   * delay. */
  bool watchdogCheckExtended ();

  /** Record some information and reset the system.
   *
   * If watchdogInit() has not been invoked---or if `bypass_watchdog`
   * is true---this delegates to `NVIC_SystemReset()`, which should
   * produce an immediate reset.  However, if the watchdog had been
   * enabled it remains enabled at the start of the next session,
   * which means unless feedings resume in a timely fashion the device
   * may reset unexpectedly.
   *
   * If watchdogInit() has been invoked and `bypass_watchdog` is false
   * this disables interrupts and goes to sleep in a loop, allowing
   * the reset to occur due to the watchdog timeout so when the system
   * starts up the watchdog is disabled and can be reprogrammed.  In
   * this situation the time before the reset takes effect is
   * determined by the watchdog delay.
   *
   * @param code a value to be stored in state_type::code after the
   * reset completes.
   *
   * @param bypass_watchdog if `true` the reset will take effect
   * immediately even if doing so will leave a running watchdog.
   */
  [[noreturn]]
  __attribute__((__always_inline__))
  void reset (unsigned int code,
              bool bypass_watchdog = false)
  {
    reset_(current_pc(), code, bypass_watchdog);
  }

  /** Return the size of the reserved stack region. */
  unsigned int stack_reserved () const;

  /** Return the size of stack region that has been used since startup.
   *
   * This delegates to stack_infer_highwater().  If the value exceeds
   * stack_reserved() there is a greater chance of stack and heap
   * colliding. */
  unsigned int stack_used () const;

  /** Return the size of the reserved heap region. */
  unsigned int heap_reserved() const;

  /** Return the current total heap allocation.
   *
   * This delegates to _nrfcxx_heap_used().  As the block allocator
   * underlying that capability does not release memory it is an
   * accurate reflection of maximum use since startup. */
  unsigned int heap_used () const;

  /** Provide information on the total time spent in various
   * operations states since reset.
   *
   * @param[out] sleep_utt the sum of the durations of modes where
   * #OM_SLEEP was set.
   *
   * @param[out] radio_utt the sum of the durations of modes where
   * #OM_RADIO was set.
   *
   * @return total_utt the sum of the durations of all modes. */
  uint64_t operationalModeBreakdown (uint64_t &sleep_utt,
                                     uint64_t &radio_utt) const;

  /** A function suitable for use as the `fault_handler`
   * argument to `sd_softdevice_enable()`.
   *
   * When invoked this causes a system reset where the next session
   * will include state_type::RESET_REAS_SDFAULT in
   * state_type::reset_reas, the `id` parameter will be in
   * state_type::sdfault_id, the `pc` parameter will be in
   * state_type::last_pc, and the `info` parameter will be in
   * state_type::code.
   *
   * @param id the SD-specific fault identifier.  After reset this
   * value is available from state_type::sdfault_id.
   *
   * @param pc the program counter of the instruction that triggered
   * the fault.  After reset this will be in state_type::last_pc.
   *
   * @param info additional fault-specific information.  After reset
   * this will be in state_type::code.
   */
  static void sd_fault_handler (uint32_t id,
                                uint32_t pc,
                                uint32_t info);

  /** Return the address of the instruction after the call.
   *
   * This is used to provide the location of the reset operation when
   * reset() is invoked.  It may be useful in other situations. */
  static uint32_t current_pc ();

  /** Return the aggregate time since the retained state was reset.
   *
   * This fetches clock::uptime::now() and adds @link
   * system_state::total_uptime total_uptime@endlink to it if a @ref
   * system_state is registered. */
  static uint64_t total_now ();

  /** Return the number of times WFE() has returned.
   *
   * In common use the application uses @ref event_set operations to
   * return to active mode.  This is accomplished by using WFE() in a
   * loop, and executing the application event loop only if
   * application events are set.  Wakeups from ARM events may occur
   * much more frequently, and although #OM_SLEEP is adjusted on each
   * ARM event wakeup the presence of rapidly firing ARM events may be
   * hidden.
   *
   * Tracking the rate at which this counter increments can reveal
   * these wakeups.
   *
   * @note This is a 32-bit counter, and is expected to wrap. */
  static unsigned int wfe_count ()
  {
    return wfe_count_;
  }

  /** Read the nRF5 die temperature.
   *
   * This is not particularly useful for environment monitoring but can
   * assist in determining that the ADC needs to be recalibrated due to
   * temperature variation.
   *
   * This function will use periph::TEMP::temperature() unless a @link
   * softdevice_is_enabled softdevice is enabled@endlink, in which
   * case it will use the SoC library `sd_temp_get()`.
   *
   * @return Die temperature in centi-Celsius. */
  static int die_cCel ()
  {
    return 25 * die_temperature_();
  }

  /** @cond DOXYGEN_EXCLUDE */
  /** Implementation for `WDT_IRQHandler`.
   *
   * The core module provides the definition of this handler, which
   * invokes this function after determining the exception return
   * address.
   *
   * @note This function is public only so the prolog code that
   * determines the stack pointer can jump to it.  Don't you be
   * messing with it.
   *
   * @param sp the value of the stack pointer at the start of the
   * `WDT_IRQHandler` vector.
   */
  static void wdt_irqhandler (void* sp);
  /** @endcond */

  /** Enter system off mode.
   *
   * @param preserve bits from state_type::reset_reas for which the
   * failures from the previous session should be preserved as though
   * they occurred in this session.  In decreasing priority this
   * supports state_type::RESET_REAS_FAILSAFE,
   * state_type::RESET_REAS_SDFAULT,
   * state_type::RESET_REAS_PROGRAMMATIC.
   *
   * @param button_psel optional PSEL for a button that will be
   * programmed to issue a DETECT signal that will wake the system
   * from off mode.  Pass a negative value to disable wakeups. */
  [[noreturn]]
  static void systemOff (unsigned int preserve,
                         int button_psel = -1);

protected:

  /** @cond DOXYGEN_EXCLUDE */
  /* Implement the reset operation. */
  [[noreturn]]
  static void reset_ (uint32_t pc,
                      unsigned int code,
                      bool bypass_watchdog = false);

  friend void failsafe (unsigned int code);
  friend void failsafe (FailSafeCode code);

  /** Function to be invoked on a controlled reset (or system off).
   *
   * Disable interrupts, update operational mode, preserve reasons
   * from previous session, invoke reset callbacks. */
  static void controlledResetPrep_ (unsigned int preserve);

  /* Implement the failsafe operation. */
  [[noreturn]]
  static void failsafe_ (uint32_t pc,
                         unsigned int code);

  /* Force or wait out the reset. */
  [[noreturn]]
  static void controlledReset_ (unsigned int preserve,
                                bool bypass_watchdog);
  /** @endcond */

  /** Pointer to the #state_ field of the first #systemState instance
   * to be constructed.
   *
   * This is used by static member functions. */
  static state_type* statep_;

  /** Pointer to the application state handler for the first
   * #systemState instance to be constructed. */
  static app_handler_type app_handler_;

private:
  /* Core provides weak definition that delegates to TEMP; sd provides
   * definition that selects between SoC library and TEMP.
   *
   * Value returned is temperature in quarter-Celsius. */
  static int die_temperature_ ();

  state_type& state_;
  const uint32_t magic_;
  static unsigned int flags_;
  static unsigned int wfe_count_;
};

/** Support for extended watchdog channels.
 *
 * In some applications there may be a need to confirm occurrence of
 * events that have a long interval between occurrences; for example,
 * a sensor that only reports every 5 minutes in a system that normally
 * samples at 1 Hz.  This class can be used to multiplex multiple
 * watchdogs, each with its own interval, onto a single hardware
 * watchdog channel.
 *
 * Unlike dedicated watchdog channels new extended watchdog instances
 * may be constructed after systemState::watchdogInit() is invoked (in
 * fact, generally they should).  For technical reasons the instances
 * can also be destructed, which will remove them from the watchdog
 * chain, but generally they should persist as long as the application
 * is running.
 *
 * The application must invoke systemState::watchdogCheckExtended()
 * within the delay of the hardware watchdog to check that all
 * extended channels are still conformant to their required
 * intervals. */
class watchdog_extended_channel
{
  struct ref_next {
    using pointer_type = watchdog_extended_channel *;
    pointer_type& operator() (watchdog_extended_channel& wec) noexcept
    {
      return wec.next_;
    }
  };

  using chain_type = pabigot::container::forward_chain<watchdog_extended_channel, ref_next>;

public:
  /** The underlying WDT channel used to support extended watchdog
   * channels. */
  static constexpr uint8_t WATCHDOG_CHANNEL_COMMON = 7;

  /** Construct and register an extended watchdog with the specified
   * interval. */
  watchdog_extended_channel (unsigned int interval_utt);

  // On destruction the channel will be removed from the chain.
  ~watchdog_extended_channel ();

  /** Feed the extended watchdog. */
  void feed ();

  watchdog_extended_channel (const watchdog_extended_channel&) = delete;
  watchdog_extended_channel& operator= (const watchdog_extended_channel&) = delete;
  watchdog_extended_channel (watchdog_extended_channel&& ) = delete;
  watchdog_extended_channel& operator= (watchdog_extended_channel&) = delete;

private:
  friend class systemState;

  bool check (unsigned int now) const
  {
    return interval_utt > (now - last_fed_);
  }

  static chain_type chain_;
  static bool failed_;

  chain_type::pointer_type next_ = chain_type::unlinked_ptr();
  unsigned int last_fed_;
  unsigned int const interval_utt;
};

void failsafe (FailSafeCode code)
{
  systemState::failsafe_(systemState::current_pc(), static_cast<unsigned int>(code));
}

void failsafe (unsigned int code)
{
  systemState::failsafe_(systemState::current_pc(), code);
}

/** Determine how much of the reserved stack space remains.
 *
 * Return the signed difference between the current stack pointer
 * against the bottom of the reserved stack space.  Negative values
 * indicate the stack has gone below the expected lower limit.
 *
 * @note Going below the lower limit of the configured space is not
 * necessarily fatal, though with the default ::_sbrk implementation
 * it is possible that doing so will overrun heap-allocated memory.
 *
 * The reserved system stack space is 1 KiBy or 256 words.  The
 * application and soft-device modules can increase this by declaring
 * objects that will force that space to be expanded, as with:
 *
 *     __attribute__((__section__(".stack.extension")))
 *     volatile uint32_t extend_by_512_words[512];
 */
inline int __attribute__((__gnu_inline__,__always_inline__))
stack_space_remaining ()
{
  return (int32_t)(__get_MSP() - static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__StackLimit)));
}

/** Force a @link FailSafeCode::STACK_OVERFLOW system
 * failure@endlink if the stack is beyond its configured maximum
 * depth.
 *
 * @see stack_space_remaining */
inline void __attribute__((__gnu_inline__,__always_inline__))
validate_stack_pointer ()
{
  if (0 > stack_space_remaining()) {
    failsafe(FailSafeCode::STACK_OVERFLOW);
  }
}

/** Fill all space between the top of the heap and the current stack
 * pointer with the provided value.
 *
 * @return the number of bytes between the top of the heap and the
 * start of the stack. */
unsigned int
stack_fill_unused (unsigned int marker);

/** Check the space between the top of the heap and the current
 * stack pointer for matches to the provided marker.
 *
 * @return the number of bytes between the top of heap and the first
 * word that did not match the marker. */
unsigned int
stack_infer_highwater (unsigned int marker);

/** Calculate a checksum of the application content.
 *
 * This covers all application code and, optionally, read-only data.
 * It can be used to identify what image is running on the board,
 * assuming a record of that checksum is retained.  It should be
 * equivalent to:
 *
 *     arm-none-eabi-objcopy -Obinary app.elf app.bin
 *     crc32 app.bin
 *
 * See `scripts/appcrc` in the nrfcxx source code.
 *
 * @note If the application includes the value of the pre-defined
 * `__TIME__` or `__DATE__` macros the checksum will change across
 * builds.
 *
 * @param exclude_data allows exclusion of the material that
 * initializes RAM data.  If this is `true` you need to add `-R .data`
 * to the `objcopy` line to get the correct comparison checksum.
 */
uint32_t application_crc32 (bool exclude_data = false);

/** Namespace for abstractions of various sensors. */
namespace sensor {
} // ns sensor

/** Namespace for abstractions that aren't handled in another namespace. */
namespace misc {
} // ns misc

} // ns nrfcxx

#endif /* NRFCXX_CORE_HPP */
