/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Core GPIO functionality.
 *
 * @file */
#ifndef NRFCXX_GPIO_HPP
#define NRFCXX_GPIO_HPP
#pragma once

#include <nrfcxx/impl.hpp>

namespace nrfcxx {

/** Abstractions and constants around GPIO capability */
namespace gpio {

/** GPIO pin configuration for input only.
 *
 * The value allows any non-default configuration for SENSE, DRIVER,
 * or PULL to be added without masking off bits. */
constexpr uint32_t PIN_CNF_RDONLY = ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                     | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                     | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                     | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos));

/** GPIO pin configuration for output only.
 *
 * The value allows any non-default configuration for SENSE, DRIVER,
 * or PULL to be added without masking off bits. */
constexpr uint32_t PIN_CNF_WRONLY = ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                     | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                     | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                     | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

/** GPIO pin configuration for input and output.
 *
 * The value allows any non-default configuration for SENSE, DRIVER,
 * or PULL to be added without masking off bits. */
constexpr uint32_t PIN_CNF_RDWR = ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                   | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                   | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                   | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

/** GPIO pin configuration at power-up.
 *
 * Use this to set the pin to what might be its lowest-power state.
 * Note that the proper configuration for true low-power operation may
 * depend on what's connected to the pin (e.g. current sink or voltage
 * source).
 *
 * @warning The integer value of this configuration is **not**
 * zero. */
constexpr uint32_t PIN_CNF_PWRUP = ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                    | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                    | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                    | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos));

/** Addition to no-pull GPIO pin configuration to pull up. */
constexpr uint32_t PIN_CNF_PULLUP = (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

/** Addition to no-pull GPIO pin configuration to pull down. */
constexpr uint32_t PIN_CNF_PULLDOWN = (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

/** GPIO pin configuration to detect active-low signals without
 * pulling to inactive.
 *
 * Use this with @link gpiote@endlink instances. */
constexpr uint32_t PIN_CNF_ACTIVE_LOW_NOPULL = (PIN_CNF_RDONLY
                                                | (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos));

/** GPIO pin configuration to detect active-low signals with
 * pull to inactive.
 *
 * Use this with @link gpiote@endlink instances. */
constexpr uint32_t PIN_CNF_ACTIVE_LOW = (PIN_CNF_ACTIVE_LOW_NOPULL | PIN_CNF_PULLUP);

/** GPIO pin configuration to detect active-high signals
 * without pulling to inactive.
 *
 * Use this with @link gpiote@endlink instances. */
constexpr uint32_t PIN_CNF_ACTIVE_HIGH_NOPULL = (PIN_CNF_RDONLY
                                                 | (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos));

/** GPIO pin configuration to detect active-low signals with
 * pull to inactive.
 *
 * Use this with @link gpiote@endlink instances. */
constexpr uint32_t PIN_CNF_ACTIVE_HIGH = (PIN_CNF_ACTIVE_HIGH_NOPULL | PIN_CNF_PULLDOWN);

/** Determine the GPIO instance associated with a given pin selector.
 *
 * @return the instance number starting from zero, or negative if the
 * selector is not supported on the device. */
static constexpr int instance_for_psel (int psel)
{
  if ((nrf5::GPIO_Instance<0>::begin_psel <= psel)
      && (psel < nrf5::GPIO_Instance<0>::end_psel)) {
    return 0;
  }
#if (NRF52840 - 0)
  if ((nrf5::GPIO_Instance<1>::begin_psel <= psel)
      && (psel < nrf5::GPIO_Instance<1>::end_psel)) {
    return 1;
  }
#endif
  return -1;
}

/** Instrumentation through toggling GPIOs
 *
 * This feature provides a relatively non-intrusive way to add
 * compile-time optional instrumentation to a system.  File-local
 * instances of this class are defined, with #NRFCXX_GPIO_PSEL_SCOPEn
 * or a non-disabled GPIO pin selection like #NRFCXX_GPIO_PSEL_SCOPE0.
 * Prior to use the instances are enabled()d, then within the code the
 * assert(), deassert(), set(), and toggle() methods may be invoked to
 * signal context-specific state changes.
 *
 * @see make_scoped_instr()
 *
 * @tparam PSEL the GPIO pin number to use for the instrumentation.
 * The value is a global ordinal with the peripheral instance
 * represented by a multiple of 32 added to the GPIO pin within that
 * instance. E.g. `P1.23` is identified by PSEL `(32 + 23)`.  Use `-1`
 * to disable the instrumentation at compile-time.  Attempts to
 * reference a pin on a non-existent GPIO peripheral will fail at
 * compile time with obscure messages about non-existent members. */
template <int PSEL>
class instr_psel
{
  /** This works for nRF51, nRF52832, nRF52810, and nRF52840.  If a
   * new product adds a third GPIO instance without filling in the 16
   * missing PSELs on the 52840's second GPIO instance it won't
   * work.
   *
   * If PSEL is -1 this maps to instance 0 and other conditions result
   * in disabled instrumentation. */
  using GPIO_Instance = nrf5::GPIO_Instance<PSEL / 32>;

public:
  using this_type = instr_psel<PSEL>;

  /** Index for the PSEL within all GPIO peripherals on the device. */
  constexpr static int global_psel = PSEL;

  /** Index for the PSEL within its GPIO instance. */
  constexpr static uint8_t psel = (PSEL - GPIO_Instance::begin_psel);

  /** The GPIO instance ordinal. */
  constexpr static uint8_t gpio_instance = GPIO_Instance::peripheral.INSTANCE;

  /** The GPIO instance that provides access to #PSEL. */
  constexpr static nrf5::GPIO_Type gpio = GPIO_Instance::peripheral;

  /** Valid GPIOs are non-negative values that are among the
   * pins supported by the instance. */
  constexpr static bool psel_valid = (0 <= PSEL) && (psel < gpio.AUX);

  /** The bit mask corresponding to PSEL */
  constexpr static uint32_t bit = (psel_valid ? (1U << psel) : 0);
  /* No-op constructor and destructor, as these objects are generally
   * statically allocated in an anonymous namespace.  This means the
   * application must enable() them explicitly. */
  instr_psel ()
  { }

  ~instr_psel ()
  { }

  /* No copying or moving these things */
  instr_psel (const instr_psel&) = delete;
  instr_psel& operator= (const instr_psel&) = delete;
  instr_psel (instr_psel&&) = delete;
  instr_psel& operator= (instr_psel&&) = delete;

  /** Enable the instrumentation functionality.
   *
   * This must be invoked prior to using the object.
   *
   * @param set control whether initial pin condition is high (@c
   * true) or low (@c false). */
  void enable (bool set = false) const
  {
    if (psel_valid) {
      if (set) {
        gpio->OUTSET = bit;
      } else {
        gpio->OUTCLR = bit;
      }
      gpio->PIN_CNF[psel] = PIN_CNF_WRONLY;
    }
  }

  /** Disable the instrumentation functionality.
   *
   * Potentially useful for reducing power consumption or if the GPIO
   * can be used for other purposes. */
  void disable () const
  {
    if (psel_valid) {
      gpio->PIN_CNF[psel] = PIN_CNF_PWRUP;
    }
  }

  /** Set the corresponding GPIO to logic 1. */
  void assert () const
  {
    if (psel_valid) {
      gpio->OUTSET = bit;
    }
  }

  /** Set the corresponding GPIO to logic 0. */
  void deassert () const
  {
    if (psel_valid) {
      gpio->OUTCLR = bit;
    }
  }

  /** Set the corresponding GPIO to a specific state.
   *
   * @param asserted @c true iff the GPIO should be set to logic 1. */
  void set (bool asserted) const
  {
    if (psel_valid) {
      if (asserted) {
        gpio->OUTSET = bit;
      } else {
        gpio->OUTCLR = bit;
      }
    }
  }

  /** Invert the state of the corresponding GPIO. */
  void toggle () const
  {
    if (psel_valid) {
      if (gpio->OUT & bit) {
        gpio->OUTCLR = bit;
      } else {
        gpio->OUTSET = bit;
      }
    }
  }

};

/** RAII class for scoped instrumentation.
 *
 * When constructed the referenced scope instance is enabled and set
 * to the specified initial state.  On destruction the instance is
 * cleared and disabled.
 *
 * @tparam INSTR_PSEL the class of the instrumentation instance. */
template <typename INSTR_PSEL>
class instr_psel_scoped
{
public:
  using instr_psel_type = INSTR_PSEL;

  constexpr instr_psel_scoped (const instr_psel_type& instance_,
                               bool start_set) :
    instance{instance_}
  {
    instance.enable(start_set);
  }

  ~instr_psel_scoped ()
  {
    instance.deassert();
    instance.disable();
  }

  /* You can't copy or assign scoped instances but you can
   * move-construct them. */
  instr_psel_scoped (const instr_psel_scoped&) = delete;
  instr_psel_scoped& operator= (const instr_psel_scoped&) = delete;
  instr_psel_scoped (instr_psel_scoped&&) = default;
  instr_psel_scoped& operator= (instr_psel_scoped&&) = delete;

  /** Forward to instr_psel::assert(). */
  void assert () const
  {
    instance.assert();
  }

  /** Forward to instr_psel::deassert(). */
  void deassert () const
  {
    instance.deassert();
  }

  /** Forward to instr_psel::set(). */
  void set (bool asserted) const
  {
    instance.set(asserted);
  }

  /** Forward to instr_psel::toggle(). */
  void toggle () const
  {
    instance.toggle();
  }

  const instr_psel_type& instance;
};

/** Create an RAII-style object that instruments a code block.
 *
 * Use this in cases where there's no need for cross-block
 * instrumentation, especially in cases where there's no convenient
 * code site where instrumentation instances can be @link
 * instr_psel::enable enabled@endlink.
 *
 *     {
 *         auto scope = nrfcxx::gpio::make_scoped_instr(scope1);
 *         // region marked by scope1 high
 *     }
 *
 * @tparam INSTR_PSEL the type of the instrumentation instance.
 * Generally this will be inferred.
 *
 * @param start_set whether the instrumentation signal should be set
 * on construction (default @c true) or left unset (@c false).  Note
 * that the default here is not the same as in instr_psel::enable().
 */
template <typename INSTR_PSEL>
instr_psel_scoped<INSTR_PSEL>
make_scoped_instr (const INSTR_PSEL& instance,
                   bool start_set = true)
{
  return {instance, start_set};
}

/** Generalized reference to a GPIO pin using a global psel ordinal.
 *
 * This allows using psel values like 42 at runtime to identify the
 * 10th GPIO on the second GPIO peripheral.
 *
 * @warning An attempt to create an instance for a psel that does not
 * exist on the device will produce FailsafeCode::API_VIOLATION. */
class pin_reference
{
public:
  /** The pin selector index across all GPIO instances. */
  uint8_t const global_psel;

  /** The pin selector index within #peripheral. */
  uint8_t const local_psel;

  /** A mask isolating the #local_psel bit within the value space of
   * #peripheral. */
  uint32_t const local_bit;

  /** Reference to a valid GPIO peripheral wrapper.
   *
   * This is the GPIO peripheral to which the pin belongs. */
  const nrf5::GPIO_Type &peripheral;

  /** Construct the instance for the given global ordinal GPIO pin.
   *
   * @warning An attempt to create an instance for a psel that does
   * not exist on the device will produce
   * FailSsafeCode::NO_SUCH_PERIPHERAL.
   *
   * @param psel a valid pin index from 0 to nrf5::GPIO_PSEL_COUNT.
   *
   * @return a GPIO pin reference. */
  static pin_reference create (int psel);

  /** Set the pin to drive the output high. */
  void set () const
  {
    peripheral->OUTSET = local_bit;
  }

  /** Set the pin to drive the output low. */
  void clear () const
  {
    peripheral->OUTCLR = local_bit;
  }

  /** Toggle the pin drive state. */
  void toggle () const
  {
    if (is_set()) {
      clear();
    } else {
      set();
    }
  }

  /** Return the `PIN_CNF` entry for the pin, or zero if the
   * reference is invalid. */
  uint32_t configuration () const
  {
    return peripheral->PIN_CNF[local_psel];
  }

  /** Set the `PIN_CNF` entry for the pin. */
  void configure (uint32_t pin_cnf) const
  {
    peripheral->PIN_CNF[local_psel] = pin_cnf;
  }

  /** Return the input signal observed at the pin, or zero if the
   * reference is invalid. */
  bool read () const
  {
    return local_bit & peripheral->IN;
  }

  /** Return `true` iff the pin is valid and is configured to drive
   * the output high. */
  bool is_set () const
  {
    return (local_bit & peripheral->OUT);
  }

private:
  pin_reference (const nrf5::GPIO_Type& peripheral,
                 int psel,
                 int begin_psel);
};

/** Class supporting a generic GPIO pin interface.
 *
 * The pin_reference class requires a valid GPIO pin, making it
 * unsuitable for applications where a signal may not be connected on
 * the board (e.g. where `RESETn` is pulled high in hardware).
 *
 * This class provides a generic API for pins, and does nothing in the
 * base class.  Subclasses can be implemented that delegate to a
 * pin_reference instance, or to an external GPIO extender, allowing a
 * common API in re-usable code without assumptions about hardware
 * configuration. */
class generic_pin
{
public:
  generic_pin () = default;
  virtual ~generic_pin () = default;

  /** Indicate whether the pin is functional.
   *
   * Base class returns `false`.  Subclasses should override to return
   * `true` in cases where invoking the other methods provides or
   * affects pin state. */
  virtual bool valid () const
  {
    return false;
  }

  /** Set the pin to drive the output high. */
  virtual void set ()
  { }

  /** Set the pin to drive the output low. */
  virtual void clear ()
  { }

  /** Toggle the pin drive state. */
  virtual void toggle ()
  { }

  /** Set the `PIN_CNF` entry for the pin if the reference is #valid.
   *
   * This API should always use the standard bit encoding of DIR,
   * INPUT, PULL, DRIVE, and SENSE of Nordic nRF5 GPIO peripherals,
   * even if the underlying implementation is a non-Nordic system.
   *
   * Subclasses should provide access to any contained instance of a
   * non-Nordic pin reference if the providing implementation requires
   * configuration that is not supported in the Nordic GPIO bit
   * encoding.
   *
   * @param pin_cnf a description of the desired pin configuration.
   * This should  */
  virtual void configure (unsigned int pin_cnf)
  { }

  /** Return implementation-specific information about the pin configuration.
   *
   * Values should be as with @p pin_cnf in configure(). */
  virtual unsigned int configuration () const
  {
    return 0;
  }

  /** Return the input signal observed at the pin, or zero if the
   * reference is invalid. */
  virtual bool read () const
  {
    return false;
  }

  /** Return `true` iff the pin is valid and is configured to drive
   * the output high. */
  virtual bool is_set () const
  {
    return false;
  }
};

/** Extension of generic_pin using an owned pin_reference. */
class gpio_pin : public generic_pin
{
public:
  /** Construct the instance.
   *
   * @param psel as with pin_reference.  Note that this *must* be a
   * valid pin selector on the device. */
  gpio_pin (int psel) :
    pr_{pin_reference::create(psel)}
  { }

  /** Access the underlying pin_reference instance. */
  const pin_reference& implementation () const
  {
    return pr_;
  }

  bool valid () const override
  {
    return true;
  }

  void set () override
  {
    pr_.set();
  }

  void clear () override
  {
    pr_.clear();
  }

  void toggle () override
  {
    pr_.toggle();
  }

  unsigned int configuration () const override
  {
    return pr_.configuration();
  }

  void configure (unsigned int pin_cnf) override
  {
    pr_.configure(pin_cnf);
  }

  bool read () const override
  {
    return pr_.read();
  }

  bool is_set () const override
  {
    return pr_.is_set();
  }

private:
  pin_reference pr_;
};

/** Wrapper supporting GPIO control of output signals by explicit or
 * scoped assertion.
 *
 * This references an externally defined @ref generic_pin that must
 * remain valid for the lifespan of the signal wrapper.
 *
 * @tparam ACTIVE_HIGH if `true` the pin asserts with high voltage,
 * and deasserts with a low voltage.  If `false` (default) the pin
 * asserts with a low voltage, and deasserts with a high voltage. */
template <bool ACTIVE_HIGH = false>
class active_signal
{
public:
  static constexpr bool active_high = ACTIVE_HIGH;
  using this_type = active_signal<active_high>;

  generic_pin& pin;

  /** RAII instance used to assert the signal within a scope.
   *
   * Construct with make_scoped(). */
  struct scoped_assert
  {
    ~scoped_assert ()
    {
      al_.deassert();
    }

  private:
    friend this_type;

    scoped_assert (const this_type& al) :
      al_{al}
    {
      al_.assert();
    }

    const this_type& al_;
  };

  /** Construct an RAII object that asserts the signal while it exists. */
  scoped_assert make_scoped () const
  {
    return scoped_assert{*this};
  }

  /** Construct the wrapper for the active signal on a given pin.
   *
   * @param psel the GPIO PSEL index for the signal. */
  active_signal (generic_pin& pin) :
    pin{pin}
  {
    deassert();
  }

  /** Indicate whether the signal is configured with a valid pin reference. */
  bool valid () const
  {
    return true;
  }

  /** Indicate whether the signal is currently asserted. */
  bool asserted () const
  {
    return active_high == pin.is_set();
  }

  /** Clear the associated GPIO to assert the active-low signal. */
  void assert () const
  {
    return active_high ? pin.set() : pin.clear();
  }

  /** Set the associated GPIO to deassert the active-low signal. */
  void deassert () const
  {
    return active_high ? pin.clear() : pin.set();
  }

  /** Configure the associated GPIO to control the signal output.
   *
   * @param aux any additional flags (e.g. DRIVE) that would augment
   * #PIN_CNF_WRONLY to produce the correct configuration for the
   * pin. */
  void enable (unsigned int aux = 0) const
  {
    pin.configure(PIN_CNF_WRONLY | aux);
  }

  /** Configure the associated GPIO to its power-up (non-controlling) state. */
  void disable () const
  {
    pin.configure(PIN_CNF_PWRUP);
  }
};

/** Alias type used for CSn, RESETn, and other active low output signals. */
using active_low = active_signal<false>;

/** Helper to build up a GPIO `PIN_CNF`.
 *
 * @param dir one of:
 * * `GPIO_PIN_CNF_DIR_Input` (default)
 * * `GPIO_PIN_CNF_DIR_Output`
 *
 * @param input one of:
 * * `GPIO_PIN_CNF_INPUT_Connect`
 * * `GPIO_PIN_CNF_INPUT_Disconnect` (default)
 *
 * @param pull one of:
 * * `GPIO_PIN_CNF_PULL_Disabled` (default)
 * * `GPIO_PIN_CNF_PULL_Pulldown`
 * * `GPIO_PIN_CNF_PULL_Pullup`
 *
 * @param drive one of:
 * * `GPIO_PIN_CNF_DRIVE_S0S1` (default)
 * * `GPIO_PIN_CNF_DRIVE_H0S1`
 * * `GPIO_PIN_CNF_DRIVE_S0H1`
 * * `GPIO_PIN_CNF_DRIVE_H0S1`
 * * `GPIO_PIN_CNF_DRIVE_D0S1`
 * * `GPIO_PIN_CNF_DRIVE_S0D1`
 * * `GPIO_PIN_CNF_DRIVE_H0D1`
 *
 * @param sense one of:
 * * `GPIO_PIN_CNF_SENSE_Disabled` (default)
 * * `GPIO_PIN_CNF_SENSE_High`
 * * `GPIO_PIN_CNF_SENSE_Low`
 *
 * @see PIN_CNF_ACTIVE_HIGH
 * @see PIN_CNF_ACTIVE_HIGH_NOPULL
 * @see PIN_CNF_ACTIVE_LOW
 * @see PIN_CNF_ACTIVE_LOW_NOPULL
 * @see PIN_CNF_PWRUP
 * @see PIN_CNF_RDONLY
 * @see PIN_CNF_RDWR
 * @see PIN_CNF_WRONLY
 */
inline
constexpr
uint32_t
pin_config (unsigned int dir = GPIO_PIN_CNF_DIR_Input,
            unsigned int input = GPIO_PIN_CNF_INPUT_Disconnect,
            unsigned int pull = GPIO_PIN_CNF_PULL_Disabled,
            unsigned int drive = GPIO_PIN_CNF_DRIVE_S0S1,
            unsigned int sense = GPIO_PIN_CNF_SENSE_Disabled)
{
  return (GPIO_PIN_CNF_DIR_Msk & (dir << GPIO_PIN_CNF_DIR_Pos))
    | (GPIO_PIN_CNF_INPUT_Msk & (input << GPIO_PIN_CNF_INPUT_Pos))
    | (GPIO_PIN_CNF_PULL_Msk & (input << GPIO_PIN_CNF_PULL_Pos))
    | (GPIO_PIN_CNF_DRIVE_Msk & (input << GPIO_PIN_CNF_DRIVE_Pos))
    | (GPIO_PIN_CNF_SENSE_Msk & (input << GPIO_PIN_CNF_SENSE_Pos));
}

/** Function to clear the SENSE field of a GPIO pin configuration.
 *
 * @tparam mutex_type the @link nrfcxx_mutex mutex type@endlink
 * required to ensure the pin configuration is not changed while the
 * clear operation is in progress.
 *
 * @see pin_config()
 */
template <typename mutex_type = null_mutex>
void
clear_sense (unsigned int psel)
{
  mutex_type mutex;

  const uint32_t pin_cnf = nrf5::GPIO->PIN_CNF[psel];
  nrf5::GPIO->PIN_CNF[psel] = pin_cnf & ~GPIO_PIN_CNF_SENSE_Msk;
}

/** Function to set the SENSE field of a GPIO pin configuration
 * to detect a change in the input signal.
 *
 * This function reads the IN signal of a GPIO and configures its
 * SENSE field to detect a change from that state.  The function
 * ensures that the IN signal state after pin configuration is
 * complete matches the original state.
 *
 * The as-configured state of the pin is specified in the low bit of
 * the return value, while bits 1 and higher count the number of times
 * the state changed during the configuration.  For example, the value
 * 3 indicates that the final observed state of the pin was set; on
 * completion the GPIO was configured to emit a SENSE event on a
 * falling edge (transition to clear), and that at least one state
 * change occurred while setting the pin configuration.
 *
 * The caller is responsible for determining what, if anything, should
 * be recorded about state changes observed during configuration.
 *
 * @note This function must be invoked in an approriate @link
 * nrfcxx_mutex mutex type@endlink context to ensure the pin
 * configuration is not changed and DETECT events are not processed
 * while the configuration operation is in progress.  If not invoked
 * within a mutex region periph::GPIOTE::mutex_type might be
 * appropriate.
 *
 * @param psel the pin index for which sense is to be updated
 *
 * @param assume_change if `false` and the sense matches the input
 * signal the return value will indicate that no changes occurred.  If
 * `true` and the sense matches the input signal the return value will
 * indicate that two changes occurred.  If the sense does not match
 * the input signal at least one change will occur.
 *
 * @return an unsigned integer value as described above.
 */
unsigned int
update_sense_bi (unsigned int psel,
                 bool assume_change = false);

} // namespace gpio
} // namespace nrfcxx

#endif /* NRFCXX_GPIO_HPP */
