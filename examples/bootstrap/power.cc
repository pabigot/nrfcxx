// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

/** Cycles through various power configurations. */

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/periph.hpp>

#define DWELL_s 5

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);
  if (cisstdio()) {
    periph::UART::instance().autoenable(1);
  }

  cputs("Active default");
  delay_us(DWELL_s * 1000000U);

  cputs("Sleep default");
  sleep_ms(DWELL_s * 1000);

  cputs("Off");
  sleep_ms(10);

  systemState::systemOff(0, NRFCXX_BOARD_PSEL_BUTTON0);

  while (true) {
    systemState::WFE();
  }
}
