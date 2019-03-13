/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Target-specific capabilities for Nordic Thingy:52 aka PCA20020
 *
 * These are items that require access to data structures that are not
 * available at the time <nrfcxx/board.hpp> is processed.
 *
 * @file */

#ifndef NRFCXX_BOARD_THINGY52_HPP
#define NRFCXX_BOARD_THINGY52_HPP
#pragma once

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/lpm.hpp>
#include <nrfcxx/misc/lipomon.hpp>
#include <nrfcxx/misc/sx1509b.hpp>

namespace nrfcxx {
namespace board {

/** Access the SX1509B instance.
 *
 * Certain assumptions are made about how the pins on this device will
 * be used.  All pins are configured in board::initialize() to the
 * reference implementation defaults.  LEDs function as expected, but
 * do not support the SX1509B LED driver functionality.
 *
 * Frequency information in the Clock and Misc registers is assumed to
 * be under the complete control of enable_led_driver() and
 * disable_led_driver().
 *
 * @warning This function must not be invoked before
 * board::initialize(). */
misc::sx1509b& iox () noexcept;

/** Estimated battery level from measured voltage.
 *
 * @note This is meaningful only when the there is no external power
 * source.  When V_USB is non-zero the measured battery voltage is
 * high even when the actual capacity is low, regardless of whether
 * the battery is charging or not. */
unsigned int battery_level_pptt (unsigned int batt_mV);

/** Configure to display the battery level.
 *
 * This configures the lightwell red or green LEDs to display an
 * estimate of battery level:
 * * Levels at or above 50% display as green, with dimmest display at
 *   50% and brightest at 100%.
 * * Levels below 50% display as red, with dimmest display at 0% and
 *   brightest at 49.99%.
 *
 * The configuration assumes the LED driver is enables, and sets the
 * duration of the display is roughly 1 s.  The positive return value
 * is the bit mask, which should be used as:
 *
 *     rc = board::led_setup_battery_display(batt_mV);
 *     if (0 <= rc) {
 *        board::iox().output_sct(0, rc); // enable the LED
 *        // wait long enough for the driver to start
 *        board::iox().output_sct(rc, 0); // clear the LED
 *     }
 */
int led_setup_battery_display (unsigned int batt_mV);

/** Enable the SX1509B LED driver functionality.
 *
 * This configures a 2 MHz Clock divided to 256 MHz for the LED
 * driver.
 *
 * @param iox_mask optional mask specifying a subset of LED pins to be
 * configured for LED driver control. If zero is passed all LED pins
 * will be configured.
 *
 * @param linear optionally use linear intensity interpretation.
 * Default `false` produces logarithmic intensity, which suits human
 * vision better for a graded sequence. */
int enable_led_driver (unsigned int iox_mask = 0,
                       bool linear = false) noexcept;

/** Enable the SX1509B LED driver functionality.
 *
 * This disables the 2 MHz Clock and the LED clock.  All six LED pins
 * are configured as normal outputs, set. */
int disable_led_driver () noexcept;

/** Extension of gpio::generic_pin using the SX1509B IO extender.
 *
 * @note Changing the state of any pin requires about 128 us and the
 * use of the I2C interface.  At this time changing the configuration
 * of a pin is not supported. */
class iox_pin : public gpio::generic_pin
{
public:
  /** Construct the instance.
   *
   * @param psel as with pin_reference.  Note that this *must* be a
   * valid pin selector on the device. */
  iox_pin (int psel) :
    bit(1U << psel),
    psel(psel)
  {
    if (psel != (0x0F & psel)) {
      failsafe(FailSafeCode::NO_SUCH_PERIPHERAL);
    }
  }

  /** The bit associated with the pin on the IO extender. */
  const uint16_t bit;

  /** The pin index on the IO extender, from 0 through 15 inclusive . */
  const uint8_t psel;

  bool valid () const override
  {
    return true;
  }

  void set () override
  {
    iox().output_sct(bit, 0);
  }

  void clear () override
  {
    iox().output_sct(0, bit);
  }

  void toggle () override
  {
    iox().output_sct(bit, bit);
  }

  unsigned int configuration () const override;

  void configure (unsigned int pin_cnf) override;

  bool read () const override;

  bool is_set () const override
  {
    return bit & iox().gpio_cache().data;
  }
};

/** Support for the power source interface. */
class power_monitor : public misc::lipo_monitor {
  using super = misc::lipo_monitor;
public:

  /** Construct an instance.
   *
   * @param notify as with misc::lipo_monitor::lipo_monitor() */
  power_monitor (notifier_type notify);
};

} // ns board
} // ns nrfcxx
#endif /* NRFCXX_BOARD_HPP */
