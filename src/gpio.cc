// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2018 Peter A. Bigot

#include <cstdio>

#include <nrfcxx/gpio.hpp>

namespace nrfcxx {
namespace gpio {

unsigned int
update_sense_bi (unsigned int psel,
                 bool assume_change)
{
  bool before;
  bool after;
  auto pinref = gpio::pin_reference::create(psel);
  unsigned int changes = 0;
  do {
    uint32_t pin_cnf = pinref.configuration();
    after = pinref.read();
    /* after is non-zero iff the pin read high.
     * Low is detected by SENSE = 3.
     * High is detected by SENSE = 2.
     *
     * If after is equal to the low bit of SENSE then the
     * configuration is correct.  Fast-exit indicating whether zero or
     * (synthesized) two changes are diagnosed. */
    bool detect_low = (1U << GPIO_PIN_CNF_SENSE_Pos) & pin_cnf;
    if (after == detect_low) {
      if (assume_change) {
        changes = 2;
      }
      break;
    }
    /* We need to change the expected sense, then verify that it
     * remains correct. */
    pin_cnf &= ~GPIO_PIN_CNF_SENSE_Msk;
    do {
      before = after;
      /* Trickiness: before is non-zero iff the pin read high.
       * Low is detected by SENSE = 3.
       * High is detected by SENSE = 2.
       * If we're low (!before) we want high: 2 = 3 ^ 1
       * If we're high (!!before) we want low: 3 = 3 ^ 0
       */
      unsigned int sense = GPIO_PIN_CNF_SENSE_Low ^ !before;
      pinref.configure(pin_cnf | (sense << GPIO_PIN_CNF_SENSE_Pos));
      after = pinref.read();
      ++changes;
      // Loop if the pin state changed during the reconfig
    } while (before != after);
  } while (0);
  return after | (changes << 1);
}

pin_reference
pin_reference::create (int psel)
{
  switch (instance_for_psel(psel)) {
    default:
      failsafe(FailSafeCode::NO_SUCH_PERIPHERAL);
    case 0: {
      using GPIOI = nrf5::GPIO_Instance<0>;
      return pin_reference(GPIOI::peripheral, psel, GPIOI::begin_psel);
    }
#if (NRF52840 - 0)
    case 1: {
      using GPIOI = nrf5::GPIO_Instance<1>;
      return pin_reference(GPIOI::peripheral, psel, GPIOI::begin_psel);
    }
#endif
  }
}

} // namespace gpio
} // namespace nrfcxx
