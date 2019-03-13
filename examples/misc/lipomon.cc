// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Basic demonstration of the lipomon infrastructure.
 *
 * On boards that support it this displays the battery and charging
 * state at 5 s intervals.  It also detects charging state events, and
 * reflects them in LED status.
 *
 * Enabling `WITH_MAXPOWER` increases the current drain of the loop,
 * allowing faster checking of automatic transition from charged to
 * charging when on mains. */

#define WITH_MAXPOWER 0

#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/utility.hpp>
#include <nrfcxx/sensor/button.hpp>

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
#include <nrfcxx/board/thingy52.hpp>
#elif (NRFCXX_BOARD_IS_XENON - 0)
#include <nrfcxx/board/xenon.hpp>
#endif

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

namespace {

constexpr nrfcxx::event_set::event_type EVT_ALARM = 0x01;
constexpr nrfcxx::event_set::event_type EVT_BUTTON = 0x02;
constexpr nrfcxx::event_set::event_type EVT_POWERMON = 0x04;
constexpr nrfcxx::event_set::event_type EVT_SHOWBATT = 0x08;
nrfcxx::event_set events;

/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

nrfcxx::systemState cs{core_state, 2018101520};

void
button_callback (unsigned int evt,
                 uint64_t duration_utt)
{
  using nrfcxx::sensor::Button;
  using nrfcxx::clock::uptime;

  uptime::text_type buf;
  cprintf("BTN %s ", uptime::as_text(buf, uptime::now()));
  cprintf("%s for %s\n", Button::eventstr(evt), uptime::as_text(buf, duration_utt));
  if (Button::EVT_CLICK == evt) {
    events.set(EVT_SHOWBATT);
  }
}

} // ns anonymous

extern "C" {
void GPIOTE_IRQHandler (void)
{
  nrfcxx::periph::GPIOTE::irq_handler();
}

void ADCSeriesVariant_IRQHandler (void)
{
  nrfcxx::periph::ADC::irq_handler();
}
} // extern C

int
main (void)
{
  using namespace nrfcxx;
  using namespace pabigot;

  board::initialize();

  auto& led_active = led::lookup(5);
  led_active.on();

  cputs("\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Display information about the cause of the restart. */
  {
    clock::uptime::text_type buf;
    cprintf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
            cs.state().reset_count, cs.state().reset_reas,
            cs.state().last_pc,
            clock::uptime::as_text(buf, cs.state().last_uptime));
    cprintf("total uptime %s\n",
            clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    cputs("***UNCONTROLLED RESET\n");
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    cprintf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    if (barked && (cs.state().magic == cs.state().code)) {
      cputs("- to put previous watchdog to sleep");
    } else {
      cprintf("- due to program code %u %s watchdog\n",
              cs.state().code, barked ? "with" : "without");
      delay_us(10000);
      systemState::systemOff(0, -1);
    }
  } else if (barked) {
    cprintf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    cprintf("- due to direct system reset\n");
  }

#define WATCHDOG_DELAY_32KiHz (10 * 32768)
#define WATCHDOG_CHANNEL_ALARM 0

  cs.watchdogInit(WATCHDOG_DELAY_32KiHz,
                  (1 << WATCHDOG_CHANNEL_ALARM));

  int rc;

  board::power_monitor powermon{events.make_setter(EVT_POWERMON)};
  rc = powermon.lpsm_start();
  cprintf("Powermon start got %d\n", rc);

  sensor::Button button{NRFCXX_BOARD_PSEL_BUTTON0, events.make_setter(EVT_BUTTON), button_callback};

  NVIC_EnableIRQ(GPIOTE_IRQn);
  periph::GPIOTE::enable_sense();

  auto alarm = clock::alarm::for_event<EVT_ALARM, true>(events);
  alarm.set_interval(5 * clock::uptime::Frequency_Hz)
    .set_deadline(0)
    .schedule();

  auto& ledr = led::lookup(0);
  auto& ledg = led::lookup(1);
  auto& ledb = led::lookup(2);
  ledr.enable();
  ledg.enable();
  ledb.enable();

  bool sample_batt = true;
  do {
    using clock::uptime;
    using lpm::state_machine;
    uptime::text_type as_text;

    event_set::cev();
    auto pending = events.copy_and_clear();
    auto now = uptime::now();

    if (pending.test_and_clear(EVT_ALARM)) {
      static const char* const ps_str[] = {
        "Unknown",
        "OnBattery",
        "Charging",
        "Charged",
      };
      cprintf("%s alarm ; %s ; live %sVCHG %sCHARGING %lx\n", uptime::as_text(as_text, now),
              ps_str[powermon.power_source()],
              powermon.vchg_detected_live() ? "" : "no ",
              powermon.charging_live() ? "" : "not ",
              nrf5::GPIO->IN);
      rc = powermon.lpsm_sample();
      if (0 != rc) {
        cprintf("!!BATT sample %d, %x\n", rc, powermon.machine().state());
      }
      cs.watchdogFeed(WATCHDOG_CHANNEL_ALARM);
    }
    if (pending.test_and_clear(EVT_BUTTON)) {
      button.process();
    }
    if (pending.test_and_clear(EVT_POWERMON)) {
      if (auto pf{powermon.lpsm_process()}) {
        using lpm::state_machine;
        if (state_machine::PF_STARTED & pf) {
          cputs("POWER started\n");
        }
        if (board::power_monitor::PF_MAINS & pf) {
          cputs("POWER on external");
        }
        if (board::power_monitor::PF_LIPO & pf) {
          cputs("POWER on battery");
          ledr.off();
          ledg.off();
        }
        if (board::power_monitor::PF_CHARGING & pf) {
          cputs("POWER battery charging");
          ledr.on();
          ledg.off();
        }
        if (board::power_monitor::PF_CHARGED & pf) {
          cputs("POWER battery charged");
          ledr.off();
          ledg.on();
        }
        if (board::power_monitor::PF_CALIBRATED & pf) {
          puts("POWER calibration completed");
        }
        if (state_machine::PF_OBSERVATION & pf) {
          cprintf("%s BATT %d mV\n", uptime::as_text(as_text, now), powermon.batt_mV());
        }
        /* We want a battery sample on startup, but we can't do it
         * until all power monitor startup activities complete.
         * That'll put us in PF_LIPO, PF_CHARGING, or PF_CHARGED but
         * we can't tell which state.  We could check for IDLE
         * explicitly, or just rely on the one built in to
         * lpsm_sample() which is a bit more robust. */
        if (sample_batt) {
          rc = powermon.lpsm_sample();
          if (0 <= rc) {
            sample_batt = false;
          }
        }
      } else if (powermon.machine().has_error()) {
        cprintf("POWER machine error %d\n", powermon.machine().error());
      }
    }
    if (pending.test_and_clear(EVT_SHOWBATT)) {
      /* Generated on button press; intent is to display the battery
       * status through led patterns for a short duration. */
    }
#if (WITH_MAXPOWER - 0)
    ledr.on();
    ledg.on();
    ledb.on();
#else
    if (!events) {
      led_active.off();
      while (!events) {
        systemState::WFE();
      }
      led_active.on();
    }
    (void)ledb;
#endif
  } while (true);

  cprintf("completed\n");

  return 0;
}
