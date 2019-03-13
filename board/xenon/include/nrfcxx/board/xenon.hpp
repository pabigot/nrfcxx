/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2019 Peter A. Bigot */

/** Target-specific capabilities for Particle Xenon
 *
 * These are items that require access to data structures that are not
 * available at the time <nrfcxx/board.hpp> is processed.
 *
 * @file */

#ifndef NRFCXX_BOARD_XENON_HPP
#define NRFCXX_BOARD_XENON_HPP

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/lpm.hpp>
#include <nrfcxx/misc/lipomon.hpp>

namespace nrfcxx {
namespace board {

/** Estimated battery level from measured voltage.
 *
 * @note This is meaningful only when the there is no external power
 * source.  When V_USB is non-zero the measured battery voltage is
 * high even when the actual capacity is low, regardless of whether
 * the battery is charging or not. */
unsigned int battery_level_pptt (unsigned int batt_mV);

/** Support for the power source interface. */
class power_monitor : public misc::lipo_monitor {
  using super = misc::lipo_monitor;
public:

  /** Construct an instance.
   *
   * @param notify as with misc::lipo_monitor::lipo_monitor() */
  power_monitor (notifier_type notify);
};

/** Query or control use of external antenna.
 *
 * The Xenon has a SPDT switch that selects between the PCB antenna
 * and the uFL socket.  On power-up the PCB antenna is used.  This
 * function may be used to switch between the PCB and uFL antennas.
 *
 * @param on positive to use the external antenna, zero to use the PCB
 * antenna, negative to query current source without changing it.
 *
 * @return true if the external antenna is being used, false if the
 * PCB antenna is being used. */
bool external_antenna (int on);

/** Access the QSPI instance for the on-board MX25L32. */
periph::QSPI& mx25l32 () noexcept;

} // ns board
} // ns nrfcxx

#endif /* NRFCXX_BOARD_HPP */
