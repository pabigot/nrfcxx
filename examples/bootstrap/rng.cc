// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

/** Demonstrate and experiment with random number generation API. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/periph.hpp>

int
main (void)
{
  using namespace nrfcxx;
  unsigned int ctr = 0;

  board::initialize();

  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);
  while (1) {
    auto rv = periph::RNG::generate<unsigned int>();
    printf("%08x", rv);
    if (0 == (0x07 & ++ctr)) {
      putchar('\n');
      using namespace std::literals;
      using clock::uptime;
      uptime::sleep(1s);
    }
  }

  return 0;
}
