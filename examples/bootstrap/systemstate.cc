// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

/** A complete and documented example of tracking telemetry and
 * diagnostic information across system restarts.
 *
 * However, some of what it collects is already better done by the
 * framework system state infrastructure. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/clock.hpp>
#include <cstdio>
#include <cstring>
#include <cinttypes>

#define MAGIC 0x12345678

extern "C" {
  extern char __stext;          ///< symbol at start of application text
  extern char __etext;          ///< symbol at end of application text
  extern char __data_start__;   ///< symbol at start of application rodata
  extern char __data_end__;     ///< symbol at end of application rodata
}

/** Application-specific state. */
struct app_state_s {
  /** Total uptime in previous sessions.
   *
   * To get instantaneous total uptime this must be added to
   * uptime::now(). */
  uint64_t total_uptime;
};

namespace {
/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

__attribute__((__section__(".noinit.app_state")))
app_state_s app_state;

void
app_state_handler (const nrfcxx::systemState::state_type& ss,
                   bool is_reset,
                   bool retained)
{
  using nrfcxx::systemState;

  if (is_reset) {
    // Cache any state not held in app_state
  } else if (retained
             && (systemState::state_type::RESET_REAS_CONTROLLED & ss.reset_reas)) {
    /* When flashing over JTAG retained will be true because the magic
     * number will be right, but the previous session did not have a
     * controlled reset so anything that would have been done on the
     * reset wasn't done.  This is an argument for keeping the live
     * values in the app_state structure so no on-reset actions are
     * required. */
  } else if (retained) {
    /* Here we had an uncontrolled reset, but because the state we
     * care about doesn't depend on an on-reset action we're still
     * good.  This is why systemState::total_now() remains monotonic
     * non-decreasing across uncontrolled resets. */
    app_state.total_uptime += ss.last_uptime;
  } else {
    // Uncontrolled or unretained reset.  Clear everything.
    memset(&app_state, 0, sizeof(app_state));
  }
}

nrfcxx::systemState cs{core_state, MAGIC, app_state_handler};

} // ns anonymous

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\n" __FILE__ " " __DATE__ " " __TIME__ " : %08lx\n",
         application_crc32());

  /* Display information about the cause of the restart. */
  {
    clock::uptime::text_type buf;
    printf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           clock::uptime::as_text(buf, cs.state().last_uptime));
    printf("app total uptime %s\n",
           clock::uptime::as_text(buf, app_state.total_uptime));
    printf("system total uptime %s\n",
           clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    puts("***UNCONTROLLED RESET");
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    printf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    printf("- due to program code %u %s watchdog\n",
           cs.state().code, barked ? "with" : "without");
  } else if (barked) {
    printf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    printf("- due to direct system reset\n");
  }

  auto reset_count = cs.state().reset_count;

  /* Every other reset program the watchdog for channels 0 and 1. */
  if (1 & reset_count) {
    sleep_ms(10);
    int rc = cs.watchdogInit(3 * 32768, 3);
    printf("Enabling watchdog at 3 s got %d\n", rc);
  } else {
    /* Indicate whether the watchdog was left running in the previous session. */
    printf("Watchdog left as-is: %s\n",
           NRF_WDT->RUNSTATUS ? "running" : "off");
  }

  switch (reset_count & 0x03) {
    case 0:                     // no watchdog
    case 1:                     // watchdog
      if (4 & reset_count) {
        puts("force watchdog due to not feeding channel 1 (rr mask 2)");
        delay_us(10000);        // flush uart
        while (true) {
          cs.watchdogFeed(0);
        }
      } else if (8 & reset_count) {
        puts("Executing system failsafe");
        delay_us(10000);        // flush uart
        failsafe(reset_count);
      } else {
        puts("Executing system reset");
        delay_us(10000);        // flush uart
        cs.reset(reset_count);
      }
      puts("shouldn't get here");
      break;

    case 2:                     // no watchdog
    case 3:                     // watchdog
      puts("Executing NVIC reset");
      delay_us(10000);          // flush uart
      NVIC_SystemReset();
      while (true) {
        puts("shouldn't get here...");
      }
      break;
  }

  return 0;
}
