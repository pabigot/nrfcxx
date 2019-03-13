// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Demonstrate the background LED pattern infrastructure. */

#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/console/cstdio.hpp>

int
main (void)
{
  using namespace nrfcxx;
  using clock::uptime;

  board::initialize();

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  led::Pattern lp0{led::lookup(0)};
  led::Pattern lp1{led::lookup(1)};
  led::Pattern lp2{led::lookup(2)};
  led::Pattern lp3{led::lookup(3)};
  led::Pattern lp4{led::lookup(4)};

  event_set events;

  constexpr event_set::event_type EVT_C0 = 0x01;
  lp0.set_notify_complete(events.make_setter(EVT_C0));

  constexpr event_set::event_type EVT_C1 = 0x02;
  lp1.set_notify_complete(events.make_setter(EVT_C1));

  constexpr event_set::event_type EVT_C2 = 0x04;
  lp2.set_notify_complete(events.make_setter(EVT_C2));

  constexpr event_set::event_type EVT_C3 = 0x08;
  lp3.set_notify_complete(events.make_setter(EVT_C3));

  constexpr event_set::event_type EVT_C4 = 0x10;
  lp4.set_notify_complete(events.make_setter(EVT_C4));

  int rc = lp0.configure(0xA0C0E0F0, 4096, 3);
  printf("configure got %d, pattern %08X, interval %u, reps %d, loops %d\n", rc, lp0.pattern(), lp0.interval_utt(), lp0.reps(), lp0.loops());
  lp0.set_deadline(uptime::Frequency_Hz);

  lp1.configure(0xAAAACCCC, 2048, 5);
  lp1.set_deadline(uptime::Frequency_Hz);

  lp2.configure(0xAAAAAAAA, 1024, 10);
  lp2.set_deadline(uptime::Frequency_Hz);

  lp3.configure(0x8CAE1357, 8192, 2);
  lp3.set_deadline(uptime::Frequency_Hz);

  lp4.configure(~lp3.pattern(), lp3.interval_utt(), lp3.reps());
  lp4.set_deadline(uptime::Frequency_Hz);

  rc = lp0.start() + lp1.start() + lp2.start() + lp3.start() + lp4.start();
  printf("start at %u got %d\n", static_cast<unsigned int>(uptime::now()), rc);

  unsigned int patno{};
  while (true) {
    event_set::cev();
    auto pending = events.copy_and_clear();
    if (pending) {
      uptime::text_type buf;
      printf("%s evt %x\n", uptime::as_text(buf, uptime::now()), pending.events());
    }

    if (pending.test_and_clear(EVT_C0)) {
      cputs("P0 complete");
    }
    if (pending.test_and_clear(EVT_C1)) {
      cputs("P1 complete");
    }
    if (pending.test_and_clear(EVT_C2)) {
      cputs("P2 complete");
    }
    if (pending.test_and_clear(EVT_C3)) {
      cputs("P3 complete");
      if (0 == patno++) {
        cputs("Starting next pattern");
        auto deadline = lp3.deadline() + uptime::Frequency_Hz;
        lp0.configure(0x88888888, 2048, 5);
        lp0.set_deadline(deadline);
        lp1.configure(lp0.pattern() >> 1, lp0.interval_utt(), lp0.reps());
        lp1.set_deadline(deadline);
        lp2.configure(lp1.pattern() >> 1, lp1.interval_utt(), lp1.reps());
        lp2.set_deadline(deadline);
        lp3.configure(lp2.pattern() >> 1, lp2.interval_utt(), lp2.reps());
        lp3.set_deadline(deadline);
        lp0.start();
        lp1.start();
        lp2.start();
        lp3.start();
      }
    }
    systemState::WFE();
  }
}
