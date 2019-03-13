// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrates detecting, preserving, and recovering from failsafe
 * resets.
 *
 * On initial powerup this configures a persisted system state, then
 * induces a failsafe reset.  In its next life the application detects
 * the failsafe, displays a failure code, then goes into system off
 * mode until BTN0 is pressed.
 *
 * On wakeup this repeats, unless BTN1 was being held when the reset
 * occured, in which case the failure is cleared and the application
 * goes on to produce another failsafe reset.
 *
 * NOTE: If the board is connected to a debugger the behavior may not
 * be what you expect.
 *
 * The application also displays the RAM configuration relative to
 * preserving RAM in system off mode, since system off is supposed to
 * safe power by turning off RAM that is outside the noinit
 * section. */

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>

namespace {

/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

nrfcxx::systemState cs{core_state, 2018100912U};
} // ns anonymous

int
main (void)
{
  using namespace nrfcxx;
  using nrfcxx::clock::uptime;

  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Determine whether B1 is being held */
  bool b1_pressed = false;

#ifdef NRFCXX_BOARD_PSEL_BUTTON1
  auto btn = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_BUTTON1);
  btn.configure(gpio::PIN_CNF_RDONLY);
  b1_pressed = board::button_active_low ^ btn.read();
  btn.configure(gpio::PIN_CNF_PWRUP);
#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */

  /* Display information about the cause of the restart. */
  {
    uptime::text_type buf;
    printf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           uptime::as_text(buf, cs.state().last_uptime));
    bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
    if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
      // Any state copied from the previous instance is incomplete or
      // invalid.
      puts("***UNCONTROLLED RESET");
    }
    if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
      if (barked && (cs.state().magic == cs.state().code)) {
        puts("- due to putting previous watchdog to sleep");
      } else {
        printf("- due to program code %u %s watchdog\n",
               cs.state().code, barked ? "with" : "without");
      }
    } else if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
      printf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
      printf("Displaying failure information; b1 %spressed\n",  b1_pressed ? "" : "un");

      /* This simply displays a fast flash for 5 s then terminates.
       * More complex indications might try to present the failure
       * code as a blink sequence. */
      led::Pattern flasher{led::lookup(0)};
      flasher.configure(0xAAAAAAAA, uptime::Frequency_Hz / 32, 5);
      volatile bool done = false;
      flasher.set_notify_complete([&done]()
                                  {
                                    done = true;
                                  });
      flasher.start();
      while (!done) {
        systemState::WFE();
      }
      if (b1_pressed) {
        puts("Failure cleared.");
      } else {
        puts("Failure preserved. Press B0 to wakeup.  Hold B1 during B0 to clear.\nSystem turning off.");
        delay_us(100000);
        systemState::systemOff(systemState::state_type::RESET_REAS_FAILSAFE, NRFCXX_BOARD_PSEL_BUTTON0);
      }
    } else if (barked) {
      printf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
    } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
      printf("- due to direct system reset\n");
    } else if (systemState::state_type::RESET_REAS_OFF & cs.state().reset_reas) {
      printf("- due to DETECT wakeup from off mode\n");
    }
  }

#if (NRF51 - 0)
  printf("RAMON: %08lx %08lx; RAMSTAT %08lx\n",
         nrf5::POWER->RAMON, nrf5::POWER->RAMONB, nrf5::POWER->RAMSTATUS);
#else
  {
    unsigned int limit = 8;
#if (NRF52840 - 0)
    limit += 1;
#endif
    printf("RAM:");
    for (auto i = 0U; limit > i; ++i) {
      printf(" %08lx", nrf5::POWER->RAM[i].POWER);
      if (3 == (0x03 & i)) {
        printf("\n    ");
      }
    }
    putchar('\n');
  }
#endif

  delay_us(1000 * 1000);
  puts("Inducing failsafe\n");
  delay_us(10000);
  failsafe(FailSafeCode::INTERNAL_ERROR);
}
