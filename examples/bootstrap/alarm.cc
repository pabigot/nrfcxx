// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Test of high data-rate alarms.
 *
 * Each alarm repeats at a fixed interval between 20 Hz and 2 Hz,
 * toggling an LED in its callback.  In the foreground there's a
 * busy-wait for 1 s; the effect of the alarm overhead is visible in
 * the clock difference between timestamps. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto& led0 = led::lookup(0);
  auto& led1 = led::lookup(1);
  auto& led2 = led::lookup(2);
  auto& led3 = led::lookup(3);

  led0.enable();
  led1.enable();
  led2.enable();
  led3.enable();
  clock::alarm alarm0{[&led0](auto& alarm)
      {
        led0.toggle();
        return true;
      }};
  clock::alarm alarm1{[&led1](auto& alarm)
      {
        led1.toggle();
        return true;
      }};
  clock::alarm alarm2{[&led2](auto& alarm)
      {
        led2.toggle();
        return true;
      }};
  clock::alarm alarm3{[&led3](auto& alarm)
      {
        led3.toggle();
        return true;
      }};

  unsigned int now = clock::uptime::now();

  using namespace std::literals;
  alarm0.set_interval(50ms).set_deadline(now).schedule();
  alarm1.set_interval(70ms).set_deadline(now).schedule();
  alarm2.set_interval(90ms).set_deadline(now).schedule();
  alarm3.set_interval(500ms).set_deadline(now).schedule();

  while (true) {
    delay_us(1'000'000);
    unsigned int now = clock::uptime::now();
    printf("Up %u\n", now);
  }
  return 0;
}
