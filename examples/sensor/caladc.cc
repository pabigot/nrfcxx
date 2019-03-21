// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Demonstrate and test use of the ADC calibration state machine. */

#include <cstdio>

#include <nrfcxx/core.hpp>
#include <nrfcxx/sensor/adc.hpp>

#define APP_EVT 0x0001

extern "C" {
void ADCSeriesVariant_IRQHandler (void)
{
  nrfcxx::periph::ADC::irq_handler();
}
} // extern "C"

int
main (void)
{
  using namespace nrfcxx;
  using clock::uptime;

  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  event_set events;
  /* For test purposes check every 3 s and recalibrate on any
   * change. */
  sensor::adc::calibrator lpsm{events.make_setter(APP_EVT), 3, 1};
#if (NRF51 - 0)
  lpsm.resolution(ADC_CONFIG_RES_10bit);
#else
  lpsm.resolution(SAADC_RESOLUTION_VAL_14bit, SAADC_OVERSAMPLE_OVERSAMPLE_Over4x);
#endif
  printf("Calibration configuration: %lu oversample, %lu res, %08lx config\n",
         lpsm.oversample(), lpsm.resolution(), lpsm.config());
  printf("Temp %d %d\n", lpsm.latest_cCel(), lpsm.calibrated_cCel());

  auto rc = lpsm.lpsm_start();
  printf("Start got %d\n", rc);

  while (true) {
    uptime::text_type as_text;
    event_set::cev();
    auto pending = events.copy_and_clear();
    unsigned int now = uptime::now();

    if (pending.test_and_clear(APP_EVT)) {
      auto& machine = lpsm.machine();
      if (auto pf = lpsm.lpsm_process()) {
        printf("%s pf %x ; temp %d %d\n", uptime::as_text(as_text, now),
               pf, lpsm.latest_cCel(), lpsm.calibrated_cCel());
      } else if (machine.has_error()) {
        printf("%s error pf %d %x\n", uptime::as_text(as_text, now),
               lpsm.lpsm_process_rc(), machine.error());
      }
    }
    systemState::WFE();
  }
}
