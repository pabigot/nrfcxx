// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrate API for HTS221 temperature/humidity sensor. */

#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/core.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/hts221.hpp>
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
  constexpr event_set::event_type EVT_HTS221 = 0x01;
  constexpr event_set::event_type EVT_ALARM = 0x02;

  sensor::hts221::iface_config_type ifc{board::twi(), NRFCXX_BOARD_PSEL_HTS_INT};
  sensor::hts221 hts221{events.make_setter(EVT_HTS221), ifc};

  auto alarm = clock::alarm::for_event<EVT_ALARM, true>(events);
  alarm.set_interval(5 * clock::uptime::Frequency_Hz);

  // hts221.odr(hts221.ODR_7_Hz);
  hts221.odr(hts221.ODR_OneShot);
  rc = hts221.lpsm_start();
  printf("hts221 start %u: %d\n", hts221.odr(), rc);
  if (hts221.ODR_OneShot == hts221.odr()) {
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

    if (pending.test_and_clear(EVT_HTS221)) {
      if (auto pf = hts221.lpsm_process()) {
        if (state_machine::PF_OBSERVATION & pf) {
          auto& obs = hts221.observations();
          printf("%s HTS221 %d cCel ; %u pptt\n", uptime::as_text(as_text, uptime::now()),
                 obs.temperature_cCel, obs.humidity_pptt);
        }
        if (state_machine::PF_STARTED & pf) {
          auto& cal = hts221.calibration();
          printf("Started: cal %d %d cCel ; %u %u pptt\n",
                 cal.cal_cCel[0], cal.cal_cCel[1],
                 cal.cal_pptt[0], cal.cal_pptt[1]);
        }
      }
      if (hts221.machine().has_error()) {
        printf("HTS221 error %d\n", hts221.machine().error());
      }
    }
    if (pending.test_and_clear(EVT_ALARM)) {
      if (true) {
        rc = hts221.lpsm_sample();
      } else {
        /* Measure the sample time.  This only works in ODR_OneShot. */
        rc = hts221.lpsm_sample();
        uptime::timestamp24 ts{};
        int prc = hts221.lpsm_process();
        while ((0 <= rc)
               && (!hts221.drdy_asserted())
               && (uptime::Frequency_Hz > ts.delta())) {
        }
        auto dt = ts.delta();
        printf("sample %d %d took %u utt = %u ms ; state %x/%d %d; evt %x\n",
               rc, prc, dt, static_cast<unsigned int>(uptime::to_ms(dt)),
               hts221.machine().state(), hts221.drdy_asserted(), hts221.status(),
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
