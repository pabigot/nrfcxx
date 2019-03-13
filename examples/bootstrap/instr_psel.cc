// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Toggles all available SCOPE pins at rates corresponding to bit positions.
 *
 * This can be used with a logic analyzer to confirm that a device is
 * wired up to the scope pins in the desired order. */

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>

using namespace nrfcxx::gpio;

#ifndef NRFCXX_BOARD_PSEL_SCOPE0
#define NRFCXX_BOARD_PSEL_SCOPE0 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE0 */
#ifndef NRFCXX_BOARD_PSEL_SCOPE1
#define NRFCXX_BOARD_PSEL_SCOPE1 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE1 */
#ifndef NRFCXX_BOARD_PSEL_SCOPE2
#define NRFCXX_BOARD_PSEL_SCOPE2 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE2 */
#ifndef NRFCXX_BOARD_PSEL_SCOPE3
#define NRFCXX_BOARD_PSEL_SCOPE3 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE3 */
#ifndef NRFCXX_BOARD_PSEL_SCOPE4
#define NRFCXX_BOARD_PSEL_SCOPE4 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE4 */
#ifndef NRFCXX_BOARD_PSEL_SCOPE5
#define NRFCXX_BOARD_PSEL_SCOPE5 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE5 */
#ifndef NRFCXX_BOARD_PSEL_SCOPE6
#define NRFCXX_BOARD_PSEL_SCOPE6 -1
#endif /* NRFCXX_BOARD_PSEL_SCOPE6 */

namespace {
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE0> scope0;
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE1> scope1;
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE2> scope2;
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE3> scope3;
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE4> scope4;
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE5> scope5;
const instr_psel<NRFCXX_BOARD_PSEL_SCOPE6> scope6;
};

int
main (void)
{
  auto& led = nrfcxx::led::led_type::lookup(0);

  led.enable();

  scope1.enable();
  scope2.enable();
  scope3.enable();
  scope4.enable();
  scope5.enable();
  scope6.enable();

  unsigned int ctr = 0;
  while (true) {
    auto scope = make_scoped_instr(scope0);
    scope1.set(ctr & 0x01);
    scope2.set(ctr & 0x02);
    scope3.set(ctr & 0x04);
    scope4.set(ctr & 0x08);
    scope5.set(ctr & 0x10);
    scope6.set(ctr & 0x20);
    led.set(ctr & 0x40);
    ++ctr;
  }
  return 0;
}
