// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrate API for LPS22HB pressure sensor. */

#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/core.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/lps22hb.hpp>
#include <nrfcxx/utility.hpp>

namespace {

/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

nrfcxx::systemState cs{core_state, 2018101520};

} // ns anonymous

extern "C" {
void GPIOTE_IRQHandler (void)
{
  nrfcxx::periph::GPIOTE::irq_handler();
}
} // extern "C"

int
main (void)
{
  using namespace nrfcxx;
  using namespace pabigot;

  board::initialize();

  puts("\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Display information about the cause of the restart. */
  {
    clock::uptime::text_type buf;
    printf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           clock::uptime::as_text(buf, cs.state().last_uptime));
    printf("total uptime %s\n",
           clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    puts("***UNCONTROLLED RESET\n");
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    printf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    if (barked && (cs.state().magic == cs.state().code)) {
      puts("- to put previous watchdog to sleep");
    } else {
      printf("- due to program code %u %s watchdog\n",
             cs.state().code, barked ? "with" : "without");
    }
  } else if (barked) {
    printf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    printf("- due to direct system reset\n");
  }

  int rc;

  NVIC_EnableIRQ(GPIOTE_IRQn);
  periph::GPIOTE::enable_sense();

  event_set events;
  constexpr event_set::event_type EVT_LPS22HB = 0x01;
  constexpr event_set::event_type EVT_ALARM = 0x02;

  sensor::lps22hb::iface_config_type ifc{board::twi(), NRFCXX_BOARD_PSEL_LPS_INT};
  sensor::lps22hb lps22hb{events.make_setter(EVT_LPS22HB), ifc};

  printf("LPS22HB with DRDY on %u, I2C addr %02x\n",
         NRFCXX_BOARD_PSEL_LPS_INT,
         lps22hb.iface_config().address);

  auto alarm = clock::alarm::for_event<EVT_ALARM, true>(events);
  alarm.set_interval(5 * clock::uptime::Frequency_Hz);

  //lps22hb.odr(lps22hb.ODR_10_Hz);
  lps22hb.odr(lps22hb.ODR_OneShot);
  rc = lps22hb.lpsm_start();
  printf("lps22hb start %u: %d, drdy %d\n", lps22hb.odr(), rc, lps22hb.drdy_asserted());
  if (lps22hb.ODR_OneShot == lps22hb.odr()) {
    alarm
      .set_deadline(0)
      .schedule();
  }

  do {
    using clock::uptime;
    using lpm::state_machine;
    uptime::text_type as_text;

    event_set::cev();
    auto pending = events.copy_and_clear();
    auto now = uptime::now();

    if (pending.test_and_clear(EVT_LPS22HB)) {
      if (auto pf = lps22hb.lpsm_process()) {
        if (state_machine::PF_OBSERVATION & pf) {
          auto& obs = lps22hb.observations();
          printf("%s LPS22HB %d cCel ; %u cPa ; drdy %u\n", uptime::as_text(as_text, now),
                 obs.temperature_cCel, obs.pressure_cPa, lps22hb.drdy_asserted());
        }
        if (state_machine::PF_STARTED & pf) {
          puts("Started");
        }
      } else if (lps22hb.machine().has_error()) {
        printf("LPS22HB error %d\n", lps22hb.machine().error());
      }
    }
    if (pending.test_and_clear(EVT_ALARM)) {
      if (true) {
        rc = lps22hb.lpsm_sample();
      } else {
        /* Measure the sample time.  This only works in ODR_OneShot. */
        rc = lps22hb.lpsm_sample();
        uptime::timestamp24 ts;
        int prc = lps22hb.lpsm_process();
        while ((0 <= rc)
               && (!lps22hb.drdy_asserted())
               && (uptime::Frequency_Hz > ts.delta())) {
        }
        auto dt = ts.delta();
        printf("sample %d %d took %u utt = %u ms ; state %x/%d %d; evt %x\n",
               rc, prc, dt, static_cast<unsigned int>(uptime::to_ms(dt)),
               lps22hb.machine().state(), lps22hb.drdy_asserted(), lps22hb.status(),
               events.fetch());
      }
    }

    while (!events) {
      systemState::WFE();
    }
  } while (true);

  printf("completed\n");

  return 0;
}
