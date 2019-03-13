// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/* Demonstration of ADD voltage divider API. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/adc.hpp>

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
#include <nrfcxx/board/thingy52.hpp>
#endif /* NRFCXX_BOARD_IS_THINGY52 */

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

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
  board::iox().configure(NRFCXX_BOARD_IOX_BAT_MON_EN, gpio::PIN_CNF_WRONLY, 1);
#endif /* NRFCXX_BOARD_IS_THINGY52 */

  sensor::adc::voltage_divider vbatt{NRFCXX_BOARD_BATTERY_R1,
      NRFCXX_BOARD_BATTERY_R2,
      NRFCXX_BOARD_BATTERY_AIN};
  int rc = 0;
  events.set(EVT_CALIBRATE);

  int cal_rc = 0;
  while (0 <= rc) {
    event_set::cev();

    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      uptime::text_type buf;
      if (pending.test_and_clear(EVT_ALARM)) {
        pending.set(EVT_CALIBRATE);
      }
      if (pending.test_and_clear(EVT_CALIBRATED)) {
        printf("%s : ADC calibrated, %d\n", uptime::as_text(buf, uptime::now()), cal_rc);
        if (0 <= cal_rc) {
          pending.set(EVT_SAMPLE);
        }
      }
      if (pending.test_and_clear(EVT_CALIBRATE)) {
        cal_rc = 99;
        auto after_queued = [&cal_rc](auto rc)
          {
            /* If the queue operation failed calibration failed post
             * the event we're waiting for so we can diagnose it. */
            cal_rc = rc;
            if (0 > rc) {
              events.set(EVT_CALIBRATED);
            }
          };
        rc = vbatt.queue(events.make_setter(EVT_CALIBRATED), after_queued, true);
        printf("%s : Calibration initiated: %d\n", uptime::as_text(buf, uptime::now()), rc);
      }
      if (pending.test_and_clear(EVT_SAMPLE)) {
        rc = vbatt.queue(events.make_setter(EVT_READY));
        printf("%s : Queue got %d .. ", uptime::as_text(buf, uptime::now()), rc);
      }
      if (pending.test_and_clear(EVT_READY)) {
        printf("Vbatt %u a16 ; %u mV\n", vbatt.sample_adc16(), vbatt.input_mV());
      }
    }
    systemState::WFE();
  }
  printf("Exit rc %d\n", rc);
}
