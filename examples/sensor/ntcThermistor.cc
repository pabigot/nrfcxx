// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Demonstrate API for NTC thermistors.
 *
 * Four ADC inputs are read and their readings converted to
 * temperatures using the thermistor API. */

#include <nrfcxx/clock.hpp>
#include <nrfcxx/sensor/adc.hpp>
#include <nrfcxx/gpio.hpp>
#include <cstdio>

/* Optionally control the current source for the thermistor.
 *
 * Cursory experimentation suggests this works adequately.  Rise and
 * fall times are less than 0.3 us, while the measurement takes about
 * 1.8 ms.  Logic Pro shows about 2.859 measuring a Vdd pin and 2.840V
 * on the GPIO output.  So there is a voltage drop for GPIO output,
 * but it's very small.  Might not need a transistor to supply power
 * for just one or two thermistors. */
#define THERM_PSEL_SUPPLY NRFCXX_BOARD_PSEL_SCOPEn

#ifndef THERM_AIN
/* AIN4 is P0.03 on PCA10028 and P0.28 on PCA10040, same
 * header position on the DK layout */
#define THERM_AIN 4
#endif /* THERM_AIN */

namespace {
#define EVT_CALIBRATED 0x01
#define EVT_SAMPLE 0x02
#define EVT_READY 0x04

nrfcxx::event_set events;
} // anonymous namespace


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

  board::initialize();

  using supply_psel = gpio::instr_psel<THERM_PSEL_SUPPLY>;
  const supply_psel supply;

  supply.enable();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  printf("Thermistor at AIN %u\n", THERM_AIN);
  if (supply_psel::psel_valid) {
    printf("Thermistor supply controlled by P%u.%u\n",
           supply_psel::gpio_instance, supply_psel::psel);
  } else {
    printf("Thermistor supply external\n");
  }

  auto alarm = clock::alarm::for_event<EVT_SAMPLE, true>(events);

  using namespace std::literals;

  alarm.set_interval(1s).set_deadline(0);

  volatile uint16_t adc16[8];
  sensor::adc::ntcThermistor therm{{THERM_AIN, 1 + THERM_AIN, 2 + THERM_AIN, THERM_AIN},
      adc16,
      &sensor::adc::steinhartHart::adafruit372hvac};

  int rc = therm.claim();
  printf("claim got %d\n", rc);
  if (0 == rc) {
    auto st = periph::ADC::state();
    rc = therm.calibrate(events.make_setter(EVT_CALIBRATED));
    printf("calibrate from %x got %d\n", st, rc);
  }

  using namespace nrfcxx;

#if 0 && !(NRF51 - 0)
  periph::instr_psel_gpiote ig_done{NRFCXX_BOARD_PSEL_SCOPE0, nrf5::SAADC->EVENTS_DONE};
  periph::instr_psel_gpiote ig_resultdone{NRFCXX_BOARD_PSEL_SCOPE1, nrf5::SAADC->EVENTS_RESULTDONE};
  periph::instr_psel_gpiote ig_end{NRFCXX_BOARD_PSEL_SCOPE2, nrf5::SAADC->EVENTS_END};
  periph::instr_psel_gpiote ig_stopped{NRFCXX_BOARD_PSEL_SCOPE3, nrf5::SAADC->EVENTS_STOPPED};
  periph::instr_psel_gpiote ig_caldone{NRFCXX_BOARD_PSEL_SCOPE4, nrf5::SAADC->EVENTS_CALIBRATEDONE};
  periph::instr_psel_gpiote ig_started{NRFCXX_BOARD_PSEL_SCOPE5, nrf5::SAADC->EVENTS_STARTED};
#endif

  while (true) {
    using clock::uptime;

    event_set::cev();
    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      uptime::text_type buf;
      if (pending.test_and_clear(EVT_CALIBRATED)) {
        alarm.schedule();
        printf("%s : ADC calibrated\n", uptime::as_text(buf, uptime::now()));
      }
      if (pending.test_and_clear(EVT_SAMPLE)) {
        supply.assert();
        rc = therm.sample(events.make_setter(EVT_READY));
        printf("%s : Sample got %d\n", uptime::as_text(buf, uptime::now()), rc);
      }
      if (pending.test_and_clear(EVT_READY)) {
        supply.deassert();
        const auto shp = therm.steinhartHart;
        printf("%s : Completed %u:\n", uptime::as_text(buf, uptime::now()), therm.count());
        for (unsigned int ci = 0; ci < therm.count(); ++ci) {
          auto adc16 = therm.sample_adc16(ci);
          printf("%u : %u <= %u <= %u adc16 ; temp %d cCel\n",
                 ci, shp->short_adc16, adc16, shp->open_adc16,
                 therm.temperature_cCel(ci));
        }
      }
    }
    systemState::WFE();
  }

}
