// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Simulate uncoordinated contacts with short level durations.
 *
 * This was designed to use as a test infrastructure for the Nordic
 * level sense detection code in Zephyr.  It didn't really work
 * because the generated edges remained digital with larger distances
 * between edges than were required to produce the desired test
 * behavior. */

#include <nrfcxx/periph.hpp>
#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <cstdio>

namespace {

static constexpr unsigned int MIN_utt = nrfcxx::clock:: uptime::from_ms(1);
static constexpr unsigned int PER_SCALE = 2;

class scope_type {
public:
  scope_type (unsigned int pin);
  nrfcxx::clock::alarm alarm;
  nrfcxx::gpio::gpio_pin pin;
  unsigned int ctr = 0;
};

bool scope_callback (nrfcxx::clock::alarm &alarm)
{
  using namespace nrfcxx;

  scope_type* stp = static_cast<scope_type*>(alarm.metadata);
  stp->pin.toggle();
  alarm.set_deadline(alarm.deadline() + MIN_utt + PER_SCALE * periph::RNG::generate<uint8_t>());
  stp->ctr += 1;
  return true;
}

scope_type::scope_type (unsigned int psel) :
  alarm{scope_callback, 0, this},
  pin(psel)
{
  using namespace nrfcxx;
  pin.configure(gpio::PIN_CNF_WRONLY);
  alarm.schedule();
}

} // anonymous

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);
  scope_type sled[] = {
    {NRFCXX_BOARD_PSEL_SCOPE0},
#if 0
    {NRFCXX_BOARD_PSEL_SCOPE1},
    {NRFCXX_BOARD_PSEL_SCOPE2},
    {NRFCXX_BOARD_PSEL_SCOPE3},
    {NRFCXX_BOARD_PSEL_SCOPE4},
#endif
  };

  printf("Signals on");
  for (auto i = 1U; i < sizeof(sled)/sizeof(*sled); ++i) {
    printf(" %u", sled[i].pin.implementation().global_psel);
  }

  while (true) {
    printf("\nCtr");
    for (auto i = 1U; i < sizeof(sled)/sizeof(*sled); ++i) {
      printf(" %u", sled[i].ctr);
    }
    sleep_ms(1000);
  }

  return 0;
}
