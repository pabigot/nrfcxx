/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Support for Semtex SX1509B.
 *
 * @file. */

#ifndef NRFCXX_MISC_SX1509B_HPP
#define NRFCXX_MISC_SX1509B_HPP
#pragma once

#include <nrfcxx/periph.hpp>
#include <nrfcxx/gpio.hpp>

namespace nrfcxx {
namespace misc {

/** Abstraction around the Semtex SX1509B Level-Shifting GPIO Extender.
 *
 * @note At this time only basic GPIO and LED driver functionality is
 * supported.
 *
 * @see https://www.semtech.com/products/smart-sensing/io-expanders/sx1509b
 */
class sx1509b
{
public:
  /** The pieces you need to talk to the device. */
  struct iface_config_type
  {
    /** Reference to TWI device used to communicate with sensor. */
    periph::TWI& twi;

    /** GPIO pin selector used for RESETn signal. */
    int8_t resetn_psel = -1;

    /** The I2C address used to communicate with the device.
     *
     * @note This is assigned by the infrastructure based on the @p
     * address parameter to the sensor constructor.  The application
     * should not set it. */
    uint8_t address;

    /** @cond DOXYGEN_EXCLUDE */
    iface_config_type (const iface_config_type&) = delete;
    iface_config_type& operator= (const iface_config_type&) = delete;
    iface_config_type (iface_config_type&&) = delete;
    iface_config_type& operator= (iface_config_type&&) = delete;
    /** @endcond */
  };

  /** Access the interface configuration for the device. */
  const iface_config_type& iface_config () const
  {
    return iface_config_;
  }

  /** Structure for SX1509 registers that are controlled.
   *
   * This should be maintained to match the SX1509 in all controlled
   * values.  A default-initialized instance corresponds to the power-up
   * configuration.
   *
   * @note The SX1509 uses big-endian storage.  Values accessed through
   * this structure are expected to be in host byte order. */
  struct gpio_regs_type
  {
    uint16_t input_disable = 0U;     // 0x00
    uint16_t long_slew = 0U;         // 0x02
    uint16_t low_drive = 0U;         // 0x04
    uint16_t pull_up = 0U;           // 0x06
    uint16_t pull_down = 0U;         // 0x08
    uint16_t open_drain = 0U;        // 0x0A
    uint16_t polarity = 0U;          // 0x0C
    uint16_t dir = -1;               // 0x0E
    uint16_t data = -1;              // 0x10
    // Subsequent registers deal with input, which is not supported on
    // the Thingy:52 nor currently via this driver.
  };

  /** Access the internal cache of GPIO registers. */
  const gpio_regs_type& gpio_cache () const noexcept
  {
    return gpio_cache_;
  }

  /** Access the internal cache of LEDDriverEnable. */
  uint16_t led_driver_cache () const
  {
    return led_driver_cache_;
  }

  /** Access the internal cache of Clock. */
  uint8_t clock_cache () const
  {
    return clock_cache_;
  }

  /** Access the internal cache of Misc. */
  uint8_t misc_cache () const
  {
    return misc_cache_;
  }

  /** Read all locally cached registers from the device.
   *
   * This may be used if you believe gpio_cache() or other local
   * storage is not accurate.
   *
   * @warning Be aware that the gpio_regs_type::data carries the input
   * signal, not the output setting, if gpio_regs_type::input_disable
   * is clear.  This may result in confusing values if the signal
   * connected to the pin is pulled in the opposite direction.
   *
   * @return non-negative on success, or a negative error code. */
  int reload_cache () noexcept;

  /** Return the GPIO pin configuration for the given IOX pin.
   *
   * The value is compatible with Nordic GPIOs, and is derived from
   * the internal cache of the SX1509B register set.
   *
   * @param psel the IOX pin index, in the range 0..15.
   *
   * @return the Nordig GPIO pin configuration inferred from the
   * SX1509B register cache. */
  unsigned int configuration (unsigned int psel) const noexcept;

  /** Change the GPIO pin configuration for the given IOX pin.
   *
   * This translates the standard Nordic GPIO pin flags into SX1509B
   * register mutations through this process:
   * * If the pin had been configured via configure_as_leds() its
   *   LEDDriverEnable bit will be cleared;
   * * If an initial value is provided, the corresponding bit in Data
   *   will be updated if necessary;
   * * INPUT controls InputDisable;
   * * DIR controls Dir;
   * * PULL controls PullUp and PullDown;
   * * SENSE is not used;
   * * DRIVE is not used;
   * * OpenDrain is cleared
   *
   * @param psel the IOX pin index, in the range 0..15.
   *
   * @param pin_cnf the Nordic GPIO pin configuration to emulate.
   *
   * @param initial if zero ensure the Data bit is cleared before
   * configuration; if positive ensure the Data is set before
   * configuration; if negative the Data register is not affected by
   * configuration.
   *
   * @return a non-negative value on success, or a negative error
   * code. */
  int configure (unsigned int psel,
                 unsigned int pin_cnf,
                 int initial = -1) noexcept;

  /** Configure multiple pins at once.
   *
   * Pins where the corresponding bit is not set in @p out_set or @p
   * out_clear are configured as inputs.
   *
   * @note This function is intended to be used immediately after
   * reset.  The only registers configured are: PullUp, PullDown, Dir,
   * Data.
   *
   * @param out_set bits identifying pins that are to be configured as
   * output, normal drive, initially set.
   *
   * @param out_clear bits identifying pins that are to be configured
   * as output, normal drive, initially clear.  Any bit set here and
   * in @p out_set will be treated as if only in @p out_set.
   *
   * @param pull_down bits identifying pins that are to be configured as
   * pull-down.  Both input and output pins will be configured.
   *
   * @param pull_up bits identifying pins that are to be configured as
   * pull-up.  Both input and output pins will be configured.  Any bit
   * set here and in @p pull_down will be treated as if only in @p
   * pull_down.
   *
   * @return non-negative on success, or a negative error code. */
  int multiconfigure (uint16_t out_set,
                      uint16_t out_clear,
                      uint16_t pull_down,
                      uint16_t pull_up = 0) noexcept;

  /** Basic structure for LED configuration. */
  struct led_type
  {
    /** ON time register.
     *
     * If zero, the LED follows the Data register.  Set to a non-zero
     * value for single-shot and blink mode. */
    uint8_t ton = 0;

    /** ON intensity register. */
    uint8_t ion = -1;

    /** OFF time/intensity register. */
    uint8_t off = 0;
  };

  /** Extended configuration for fade-capable pins.
   *
   * These are pins 4 through 7, and 12 through 15. */
  struct led_pwm_type : public led_type
  {
    /** Fade in register. */
    uint8_t trise = 0;

    /** Fade out register. */
    uint8_t tfall = 0;
  };

  static constexpr auto RegClock_FREQ_Pos = 5U;
  static constexpr uint16_t RegClock_FREQ_Msk = 0x03 << RegClock_FREQ_Pos;
  static constexpr uint16_t RegClock_FREQ_Off = 0;
  static constexpr uint16_t RegClock_FREQ_External = 1;
  static constexpr uint16_t RegClock_FREQ_2MHz = 2;

  static constexpr auto RegClock_OSCIO_Pos = 4U;
  static constexpr uint16_t RegClock_OSCIO_Msk = 0x01 << RegClock_OSCIO_Pos;
  static constexpr uint16_t RegClock_OSCIO_Input = 0;
  static constexpr uint16_t RegClock_OSCIO_Output = 1;

  static constexpr auto RegClock_OUTFREQ_Pos = 0U;
  static constexpr uint16_t RegClock_OUTFREQ_Msk = 0x0F << RegClock_OUTFREQ_Pos;
  static constexpr uint16_t RegClock_OUTFREQ_Low = 0;
  static constexpr uint16_t RegClock_OUTFREQ_High = 0x0F;

  static constexpr auto RegMisc_LEDB_Pos = 7U;
  static constexpr uint16_t RegMisc_LEDB_Msk = 1U << RegMisc_LEDB_Pos;
  static constexpr uint16_t RegMisc_LEDB_Linear = 0;
  static constexpr uint16_t RegMisc_LEDB_Logarithmic = 1;

  static constexpr auto RegMisc_LEDFREQ_Pos = 4U;
  static constexpr uint16_t RegMisc_LEDFREQ_Msk = 0x07U << RegMisc_LEDFREQ_Pos;
  static constexpr uint16_t RegMisc_LEDFREQ_Off = 0;

  static constexpr auto RegMisc_LEDA_Pos = 3U;
  static constexpr uint16_t RegMisc_LEDA_Msk = 1U << RegMisc_LEDA_Pos;
  static constexpr uint16_t RegMisc_LEDA_Linear = 0;
  static constexpr uint16_t RegMisc_LEDA_Logarithmic = 1;

  static constexpr auto RegMisc_NRESET_Pos = 2U;
  static constexpr uint16_t RegMisc_NRESET_Msk = 1U << RegMisc_NRESET_Pos;
  static constexpr uint16_t RegMisc_NRESET_POR = 0;
  static constexpr uint16_t RegMisc_NRESET_ResetCounters = 1;

  static constexpr auto RegMisc_AUTOINC_Pos = 1U;
  static constexpr uint16_t RegMisc_AUTOINC_Msk = 1U << RegMisc_AUTOINC_Pos;
  static constexpr uint16_t RegMisc_AUTOINC_On = 0;
  static constexpr uint16_t RegMisc_AUTOINC_Off = 1;

  static constexpr auto RegMisc_AUTOCLR_Pos = 0U;
  static constexpr uint16_t RegMisc_AUTOCLR_Msk = 1U << RegMisc_AUTOCLR_Pos;
  static constexpr uint16_t RegMisc_AUTOCLR_On = 0;
  static constexpr uint16_t RegMisc_AUTOCLR_Off = 1;

  static constexpr auto RegTOnX_TOn_Pos = 0U;
  static constexpr uint16_t RegTOnX_TOn_Msk = 0x1F << RegTOnX_TOn_Pos;
  static constexpr auto RegOffX_TOff_Pos = 3U;
  static constexpr uint16_t RegOffX_TOff_Msk = 0x1F << RegOffX_TOff_Pos;
  static constexpr auto RegOffX_IOff_Pos = 0U;
  static constexpr uint16_t RegOffX_IOff_Msk = 0x07 << RegOffX_IOff_Pos;

  /** Instantiate the device.
   *
   * @param ifc reference to an externally owned struct providing the
   * resources required to communicate with the device.
   *
   * @param addr the device address as controlled by the A0 and A1
   * pins, value in the range 0 to 3. */
  sx1509b (iface_config_type& ifc,
           unsigned int addr = 0) noexcept;

  /** Reset the device to its power-on state via RESETn.
   *
   * This works as long as misc() is not configured to use the RESETn
   * signal to synchronize PWM counters.
   *
   * @return a non-negative duration in ms to wait until the device
   * will have completed reset, or a negative error code. */
  int hw_reset () noexcept;

  /** Reset the device to its power-on state via software command.
   *
   * @return a non-negative duration in ms to wait until the device
   * will have completed reset, or a negative error code. */
  int sw_reset () noexcept;

  /** Read the Clock register from the SX1509B.
   *
   * @note The @link clock_cached cached@endlink value will *not* be
   * updated. */
  int clock () const noexcept;

  /** Set the Clock register in the SX1509B.
   *
   * On success the @link clock_cached cached@endlink value will also
   * be updated. */
  int clock (uint8_t value) noexcept;

  /** Read the Misc register from the SX1509B.
   *
   * @note The @link misc_cached cached@endlink value will *not* be
   * updated. */
  int misc () const noexcept;

  /** Set the Misc register in the SX1509B.
   *
   * On success the @link clock_cached cached@endlink value will also
   * be updated. */
  int misc (uint8_t value) noexcept;

  /** Configure a set of pins for LED operations.
   *
   * @param leds bit mask identifying pins that should be enabled the LED driver.
   *
   * @param source_current if `true` the SX1509B will be configured to
   * drive the LED pins as normal outputs.  If `false` OpenDrain will
   * be enabled for the LED pins.
   *
   * @return a non-negative value on success, or a negative error
   * code. */
  int configure_as_leds (uint16_t leds,
                         bool source_current = false) noexcept;

  /** Read the LED configuration for the given pin.
   *
   * @param psel the IOX pin index.  The pin need not be configured
   * for LED use.
   *
   * @param cfg where to store the non-fade configuration to use.
   *
   * @return a non-negative value on success, or a negative error
   * code. */
  int led_configuration (unsigned int psel,
                         led_type& cfg) const noexcept;

  /** Set the LED configuration for the given pin.
   *
   * @param psel the IOX pin index.  The pin need not be configured
   * for LED use.
   *
   * @param cfg where to store the fade-capable configuration to use.
   * If the pin does not support fade the call will return an error.
   *
   * @return a non-negative value on success, or a negative error
   * code. */
  int led_configuration (unsigned int psel,
                         led_pwm_type& cfg) const noexcept;

  /** Set the LED configuration for the given pin.
   *
   * @note If this is used to configure a LED that supports fade the
   * rise and fall times will not be affected.
   *
   * @param psel the IOX pin index.  The pin need not be configured
   * for LED use.
   *
   * @param cfg the non-fade configuration to use.
   *
   * @return a non-negative value on success, or a negative error
   * code. */
  int led_configure (unsigned int psel,
                     const led_type& cfg) const noexcept;

  /** Set the LED configuration for the given pin.
   *
   * @param psel the IOX pin index.  The pin need not be configured
   * for LED use.
   *
   * @param cfg the fade configuration to use.  If the pin does not
   * support fade the call will return an error.
   *
   * @return a non-negative value on success, or a negative error
   * code. */
  int led_configure (unsigned int psel,
                     const led_pwm_type& cfg) const noexcept;

  /** Multi-signal set, clear, and toggle API.
   *
   * If a bit is set in both @p set and @p clear its current value is
   * toggled.  If no bits are set in @p set or @p clear the current IOX
   * output configuration is returned from cache.
   *
   * @param set bits identifying IOX outputs that should be high.
   *
   * @param clear bits identifying IOX outputs that should be low.
   *
   * @return a non-negative value provides the current setting for all
   * IOX outputs.  A negative value indicates a failure to configure the
   * outputs. */
  int output_sct (uint16_t set,
                  uint16_t clear) noexcept;

private:
  void reset_cache_ () noexcept;

  iface_config_type& iface_config_;
  gpio::pin_reference resetn_;
  gpio_regs_type gpio_cache_{};
  uint16_t led_driver_cache_ = 0;
  uint8_t clock_cache_ = 0;
  uint8_t misc_cache_ = 0;
};

} // ns misc
} // namespace
#endif /* NRFCXX_MISC_SX1509B_HPP */
