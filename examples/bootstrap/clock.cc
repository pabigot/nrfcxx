// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Demonstrate and test clocks and timers.
 *
 * Configures low frequency and high frequency clocks.  Demonstrates
 * uptime duration management, and high-resolution timer
 * capabilities. */

#include <nrfcxx/core.hpp>
#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <cstdio>

int
main (void)
{
  using namespace nrfcxx;
  using namespace nrfcxx::clock;

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  printf("System clock: %u Hz\n", hfclk::Frequency_Hz);
  initialize(true);

  printf("HFC %d LFC %d src %lu\n", hfclk::hfxt_active(),
         lfclk::active(), lfclk::source());

  lfclk::configure(1);
  printf("HFC %d LFC %d src %lu\n", hfclk::hfxt_active(),
         lfclk::active(), lfclk::source());

  {
    using namespace std::literals;
    uptime::timestamp24 ts;
    uptime::timestamp24 ts0 = ts;

    printf("d0 %u\n", ts.delta_reset());
    printf("1 ms = %u utt\n", static_cast<unsigned int>(uptime::from_duration(1ms)));
    printf("d1 %u\n", ts.delta_reset());
    auto tick = 1_utt;
    printf("1 utt = %u ns\n", static_cast<unsigned int>(uptime::to_duration<std::chrono::nanoseconds>(tick.count()).count()));
    printf("d2 %u\n", ts.delta_reset());
    printf("dt %u\n", ts0.delta(ts.captured()));
  }

  {
    using namespace nrfcxx::periph;
    TIMER& timer0{TIMER::instance(0)};

    timer0.configure(hfclk::Frequency_Hz);
    timer0.start();

    printf("hrt %u Hz\n", timer0.frequency_Hz());

    auto ts = timer0.timestamp();
    auto ts0 = ts;

    printf("hrt0 %u\n", ts.delta());
    printf("hrt1 %u\n", ts.delta_reset());
    printf("hrt2 %u\n", ts.delta_reset());
    printf("hrt %u\n", ts0.delta(ts.captured()));

    timer0.clear();
    timer0.capture(0);
    timer0.capture(1);
    unsigned int c0 = timer0.captured(0);
    printf("hrt0 %u %u\n",
           timer0.delta(0, c0),
           timer0.delta(c0, timer0.captured(1)));

    timer0.deconfigure();
  }

  auto last_src = lfclk::source();
  printf("HFC %d LFC %d src %lu\n", hfclk::hfxt_active(),
         lfclk::active(), last_src);

  auto& led0 = led::lookup(0);
  led0.enable();
  while (true) {
    delay_us(500'000);
    led0.toggle();
    auto cur_src = lfclk::source();
    if (cur_src != last_src) {
      printf("LFCLK src changed %lu to %lu\n", last_src, cur_src);
      last_src = cur_src;
    }
  }
  return 0;
}
