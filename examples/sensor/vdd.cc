// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/* Demonstration of ADC capabilities using Vdd measurement.
 *
 * This supports both direct commands and queued ADC operations.
 *
 * When using queued operations a calibration is performed before each
 * sample.  This is primarily a demonstration to confirm the SAADC
 * peripheral has only 9 bits of relative accuracy, in that even when
 * constantly calibrated measured values will vary within about 0.2%
 * on each sample.  The rejected theory is that the initial
 * calibration was poor. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/adc.hpp>

#define USE_QUEUE 1

namespace {

#define EVT_ALARM 0x01
#define EVT_CALIBRATE 0x02
#define EVT_CALIBRATED 0x04
#define EVT_SAMPLE 0x10
#define EVT_READY 0x20
nrfcxx::event_set events;

} // anonymous namespace

extern "C" {
void ADCSeriesVariant_IRQHandler (void)
{
  nrfcxx::periph::ADC::irq_handler();
}

void GPIOTE_IRQHandler (void) {
  nrfcxx::periph::GPIOTE::irq_handler();
}
} // extern "C"

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  using clock::uptime;

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto alarm = clock::alarm::for_event<EVT_ALARM, true>(events);

  using namespace std::literals;
  alarm
    .set_interval(1s)
    .set_deadline(alarm.interval())
    .schedule();

#if 0 && !(NRF51 - 0)
  periph::instr_psel_gpiote ig_done{NRFCXX_BOARD_PSEL_SCOPE0, nrf5::SAADC->EVENTS_DONE};
  periph::instr_psel_gpiote ig_resultdone{NRFCXX_BOARD_PSEL_SCOPE1, nrf5::SAADC->EVENTS_RESULTDONE};
  periph::instr_psel_gpiote ig_end{NRFCXX_BOARD_PSEL_SCOPE2, nrf5::SAADC->EVENTS_END};
  periph::instr_psel_gpiote ig_stopped{NRFCXX_BOARD_PSEL_SCOPE3, nrf5::SAADC->EVENTS_STOPPED};
  periph::instr_psel_gpiote ig_caldone{NRFCXX_BOARD_PSEL_SCOPE4, nrf5::SAADC->EVENTS_CALIBRATEDONE};
  periph::instr_psel_gpiote ig_started{NRFCXX_BOARD_PSEL_SCOPE5, nrf5::SAADC->EVENTS_STARTED};
#endif

  sensor::adc::vdd vdd{[](auto vdd_mV)
      {
        events.set(EVT_READY);
      }};
  int rc = 0;
#if (USE_QUEUE - 0)
  events.set(EVT_CALIBRATE);
#else // USE_QUEUE
  rc = vdd.claim();
  printf("claim got %d\n", rc);
  if (0 == rc) {
    auto st = periph::ADC::state();
    rc = vdd.calibrate(events.make_setter(EVT_CALIBRATED));
    printf("calibrate from %x got %d\n", st, rc);
  }
#endif //USE_QUEUE

  int cal_rc = 0;
  while (0 <= rc) {
    event_set::cev();

    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      uptime::text_type buf;
      if (pending.test_and_clear(EVT_ALARM)) {
#if (USE_QUEUE - 0)
        pending.set(EVT_CALIBRATE);
#else
        pending.set(EVT_SAMPLE);
#endif // USE_QUEUE
      }
      if (pending.test_and_clear(EVT_CALIBRATED)) {
        printf("%s : ADC calibrated, %d\n", uptime::as_text(buf, uptime::now()), cal_rc);
#if (USE_QUEUE - 0)
        if (0 <= cal_rc) {
          pending.set(EVT_SAMPLE);
        }
#endif // USE_QUEUE
      }
      if (pending.test_and_clear(EVT_CALIBRATE)) {
#if (USE_QUEUE - 0)
        cal_rc = 99;
        rc = vdd.queue(events.make_setter(EVT_CALIBRATED),
                       [&cal_rc](auto rc) {
                         cal_rc = rc;
                         if (0 > rc) {
                           events.set(EVT_CALIBRATED);
                         }
                       },
                       true);
        printf("%s : Calibration initiated: %d\n", uptime::as_text(buf, uptime::now()), rc);
#endif // USE_QUEUE
      }
      if (pending.test_and_clear(EVT_SAMPLE)) {
#if (USE_QUEUE - 0)
        rc = vdd.queue();
        printf("%s : Queue got %d .. ", uptime::as_text(buf, uptime::now()), rc);
#else // USE_QUEUE
        rc = vdd.sample();
        printf("%s : Sample got %d .. ", uptime::as_text(buf, uptime::now()), rc);
#endif // USE_QUEUE
      }
      if (pending.test_and_clear(EVT_READY)) {
        printf("Vdd %u mV\n", vdd.vdd_mV());
      }
    }
    systemState::WFE();
  }
  printf("Exit rc %d\n", rc);
}
