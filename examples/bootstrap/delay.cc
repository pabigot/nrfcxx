// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Test core delay function.
 *
 * Use this with a logic analyzer to confirm that delays are accurate
 * for the corresponding hardware. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/clock.hpp>

// Optionally configure the clock before testing the time.  If the
// system clock is not using a stabilized high-frequency crystal
// source the durations will be inaccurate.  Even when it is, errors
// less than 100 ppm should be allowed.
// #define WITHOUT_CLOCK 1

static const
nrfcxx::gpio::instr_psel<NRFCXX_BOARD_PSEL_SCOPE0> scope;

int
main (void)
{
  const unsigned int dur_us[] = {
    0, 1, 2, 5, 8, 16, 32,
    64, 100, 128, 250, 256, 500, 512, 1000, 1024,
    10'000, 100'000, 1'000'000,
  };
  auto dp = dur_us;
  auto const dpe = dp + sizeof(dur_us)/sizeof(*dur_us);

#if !(WITHOUT_CLOCK - 0)
  // This is one case where we do not invoke board::initialize().
  nrfcxx::clock::initialize();
#endif /* WITHOUT_CLOCK */

  scope.enable(false);
  scope.assert();
  scope.deassert();

  // Nominal 10 ms delay to differentiate the trigger from the delays.
  nrfcxx::delay_us(10'000);

  while (dp < dpe) {
    __disable_irq();
    scope.assert();
    nrfcxx::delay_us(*dp);
    scope.deassert();
    __enable_irq();
    ++dp;
  }
}
