// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Light up to 5 LEDs as a binary representation of a counter
 * incrementing at 10 Hz. */

#include <nrfcxx/led.hpp>

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  auto& led0 = led::lookup(0);
  auto& led1 = led::lookup(1);
  auto& led2 = led::lookup(2);
  auto& led3 = led::lookup(3);
  auto& led4 = led::lookup(4);
  led0.enable();
  led1.enable();
  led2.enable();
  led3.enable();
  led4.enable();
  unsigned int ctr = 0;
  while (1) {
    led0.set(1 & ctr);
    led1.set(2 & ctr);
    led2.set(4 & ctr);
    led3.set(8 & ctr);
    led4.set(16 & ctr);
    delay_us(100000);
    ++ctr;
  }
}
