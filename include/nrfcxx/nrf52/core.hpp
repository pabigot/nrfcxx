/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** API specific to the nRF52 series supporting <nrfcxx/core.hpp>.
 *
 * @file */

#ifndef NRFCXX_NRF52_CORE_HPP
#define NRFCXX_NRF52_CORE_HPP
#pragma once

#if (NRF52832 - 0)
#include "core832.hpp"
#elif (NRF52840 - 0)
#include "core840.hpp"
#else /* NRF52 */
#error Unsupported NRF52 MCU
#endif /* NRF52 */

namespace nrfcxx {
namespace nrf5 {
namespace series {

/** CPU clock speed in MHz. */
static constexpr unsigned int CLOCK_MHz = 64;

/** Overhead setting up the call to delay_cycles() in the standard
 * delay_us() implementation.
 *
 * The value must be strictly less than CLOCK_MHz.
 *
 * This value has been estimated from biased measurement, and is
 * conservative (smaller than the actual overhead).  Note that the
 * resolution of the used value depends on the number of cycles per
 * delay_cycles() iteration: at the nRF52 value of 3 there is no
 * difference between 12 and 14.  12 is chosen because the measured
 * overhead for single-pass loop with a non-zero overhead is about 225
 * ns, with the overhead of the scope operation probably around 32 ns.
 * 12 is 187.5 ns which is close to the difference without exceeding
 * it. */
static constexpr unsigned int DELAY_US_OVERHEAD_cyc = 12;

/** Loop to delay for a requested number of cycles. */
void delay_cycles (unsigned int cycles); // see nrf52/delay_cycles.S

/** Enable the nRESET functionality.
 *
 * Nordic dev boards have a nice button labelled "RESET", but it only
 * resets if you build `system_nr52.c` with `CONFIG_GPIO_AS_PINRESET`
 * defined.  Having done that, it's sticky until UICR is cleared and
 * won't go away if you rebuild with that undefined.  Horrible API.
 *
 * Instead use this function to enable the reset pin functionality.
 * To disable it, wait until we have UICR support or use:
 *
 *     nrfjprog -f NRF52 --eraseuicr
 *
 * @note Generally only one pin is supported for nRESET.  On nRF52832
 * it's 21; on nRF52840 it's 18.  The function picks the one it thinks
 * is right.
 *
 * @warning This function will move to the UICR module once that's
 * designed and implemented. */
void enable_pinreset ();

} // ns series
} // ns nrf5
} // ns nrfcxx

#endif /* NRFCXX_NRF52_HPP */
