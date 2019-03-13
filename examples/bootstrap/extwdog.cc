// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Confirms functionality of extended watchdog infrastructure. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/clock.hpp>
#include <cstdio>
#include <cstring>
#include <cinttypes>

namespace {

/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

__attribute__((__section__(".noinit.test_passed")))
bool test_passed;

nrfcxx::systemState cs{core_state, 2019031107U};
} // ns anonymous

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\n" __FILE__ " " __DATE__ " " __TIME__ " : %08lx\n",
         application_crc32());
  printf("tp %d\n", test_passed);

  /* Display information about the cause of the restart. */
  {
    clock::uptime::text_type buf;
    printf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           clock::uptime::as_text(buf, cs.state().last_uptime));
    printf("system total uptime %s\n",
           clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    puts("***UNCONTROLLED RESET");
    test_passed = false;
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    printf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    printf("- due to program code %u %s watchdog\n",
           cs.state().code, barked ? "with" : "without");
  } else if (barked) {
    printf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
    if (test_passed) {
      puts("Test passed.");
      sleep_ms(10);
      systemState::systemOff(0, -1);
    }
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    printf("- due to direct system reset\n");
  }
  test_passed = false;

  constexpr auto WATCHDOG_CHANNEL_FAST = 0U;
  int rc = cs.watchdogInit(cs.WATCHDOG_Hz / 2,
                           cs.WATCHDOG_MASK_EXTENDED
                           | (1U << WATCHDOG_CHANNEL_FAST));
  printf("Enabling watchdog at 4 Hz got %d\n", rc);

  /* Watchdog already running, wait for reset */
  while (0 > rc) {
    systemState::WFE();
  }

  puts("No extended for 3 s");
  auto ts = clock::uptime::timestamp24{};
  clock::uptime::text_type as_text;

  /* If no extended channels are set, the check should be trivially satisfied. */
  while ((3 * clock::uptime::Frequency_Hz) >= ts.delta()) {
    cs.watchdogFeed(WATCHDOG_CHANNEL_FAST);
    auto wce = cs.watchdogCheckExtended();
    printf("%s WCE is%s satisfied\n", clock::uptime::as_text(as_text, ts.delta()),
           wce ? "" : " not");
    sleep_ms(200);
  }

  /* Set up a channel for 5 s, then don't feed it for 3 s. */
  {
    watchdog_extended_channel wec{5 * cs.WATCHDOG_Hz};
    puts("Extended at 5 s for 3 s w/o feed");
    ts.reset();
    while ((3 * clock::uptime::Frequency_Hz) >= ts.delta()) {
      cs.watchdogFeed(WATCHDOG_CHANNEL_FAST);
      auto wce = cs.watchdogCheckExtended();
      printf("%s WCE is%s satisfied\n", clock::uptime::as_text(as_text, ts.delta()),
             wce ? "" : " not");
      sleep_ms(200);
    }
  }

  /* Extended was unfed for 3 s; should be destroyed, run for another
   * 3 s to see. */
  puts("Extended destructed, going for 3 s");
  ts.reset();
  while ((3 * clock::uptime::Frequency_Hz) >= ts.delta()) {
    cs.watchdogFeed(WATCHDOG_CHANNEL_FAST);
    auto wce = cs.watchdogCheckExtended();
    printf("%s WCE is%s satisfied\n", clock::uptime::as_text(as_text, ts.delta()),
           wce ? "" : " not");
    sleep_ms(200);
  }

  /* Create a new extended at 2 s and run it out to reset. */
  {
    watchdog_extended_channel wec{2 * cs.WATCHDOG_Hz};
    puts("Extended at 2 s for 3 s w/o feed should reset");
    ts.reset();
    while ((3 * clock::uptime::Frequency_Hz) >= ts.delta()) {
      cs.watchdogFeed(WATCHDOG_CHANNEL_FAST);
      auto wce = cs.watchdogCheckExtended();
      test_passed = !wce;
      printf("%s WCE is%s satisfied\n", clock::uptime::as_text(as_text, ts.delta()),
             wce ? "" : " not");
      sleep_ms(200);
    }
  }
  puts("Failed");

  return 0;
}
