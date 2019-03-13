// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Test application that does nothing.
 *
 * When preparing a new board you might want to use this with
 * `WITH_BOARD_INITIALIZE` and `WITH_SLEEP` both disabled, just to
 * confirm that the toolchain works and you can produce a
 * minimally-sized binary.
 *
 * Otherwise this is used for assessing the power consumption of a
 * particular board under various circumstances, by default when it's
 * been initialized and is sleeping. */

#include <nrfcxx/clock.hpp>
#include <nrfcxx/periph.hpp>

#define WITH_BOARD_INITIALIZE 1
#define WITH_TIMER 0
#define WITH_UART 0
#define WITH_SLEEP 1

int
main (void)
{
#if (WITH_BOARD_INITIALIZE - 0)
  nrfcxx::board::initialize();

#if (WITH_TIMER - 0)
  auto& timer1 = nrfcxx::periph::TIMER::instance(1);
  timer1.configure(nrfcxx::clock::hfclk::Frequency_Hz);
  timer1.start();
#endif // WITH_TIMER

#if (WITH_UART - 0)
  auto& uart = nrfcxx::periph::UART::instance();
  uart.enable();
#endif // WITH_UART

#endif // WITH_BOARD_INITIALIZE

  /* By default spin with the CPU enabled.  Optionally enter low-power
   * mode (requires source code change).
   *
   * **NOTE** This application is built without libnosys support,
   * which means if you're modifying it to test basic features you
   * don't have cstdio and other capabilities available. */
  while (true) {
#if (WITH_SLEEP - 0)
    static volatile unsigned int wakeups;
    __WFE();
    ++wakeups;
#endif
  }
}
