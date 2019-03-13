// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Test application for board UART operation without involving
 * libc. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>

int
main (void)
{
  using namespace nrfcxx;

  auto& uart = periph::UART::instance();

  auto& led = led::lookup(0);
  led.enable();
  uart.enable();
  led.on();
  unsigned int ctr = 0;
  while (true) {
    const uint8_t ch = (0x3f & ++ctr) ? 'a' : '\n';
    uart.write(&ch, sizeof(ch));
    delay_us(10000);
    if (0 == (0x3f & ctr)) {
      led.toggle();
    }
  }

  return 0;
}
