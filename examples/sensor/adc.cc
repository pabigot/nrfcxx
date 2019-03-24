// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Playground for ADCClient customizations.
 *
 * In the current version this shows using a voltage divider
 * configuration to produce an estimate of relative ambient light
 * using a photo transistor, specifically an HW5P-1 or
 * ALS-PT243-3C/L177.  The log of the deduced resistance is converted
 * to a unitless estimate of intensity, which should be proportional
 * to the log of the illuminance in lux. */

#include <climits>
#include <cmath>
#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/misc/regulator.hpp>
#include <nrfcxx/sensor/adc.hpp>

namespace {

#define EVT_CALADC 0x01
#define EVT_ALARM 0x02
#define EVT_PROCESS 0x04
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

  auto vdd_gp = gpio::gpio_pin{32 + 3};
  auto vdd = misc::gpio_controlled_voltage{gpio::active_signal<true>{vdd_gp}};
  printf("vdd %d on %u\n", vdd.enabled(), vdd_gp.implementation().global_psel);

  using namespace std::literals;
  alarm
    .set_interval(1s)
    .set_deadline(alarm.interval())
    .schedule();

  sensor::adc::light_intensity client{1};
  client.set_regulator(&vdd, nrfcxx::clock::uptime::from_ms(1));

  // Calibrate on any die temperature change, checking at 5 s interval
  sensor::adc::calibrator caladc{events.make_setter(EVT_CALADC), 5, 1};
#if !(NRF51 - 0)
  caladc.resolution(SAADC_RESOLUTION_VAL_14bit);
#endif
  int rc = caladc.lpsm_start();
  printf("ADC calibrator start got %d: %d %d\n", rc, caladc.latest_cCel(), caladc.calibrated_cCel());

  sensor::adc::lpsm_wrapper lpmclient{events.make_setter(EVT_PROCESS), client};
  rc = lpmclient.lpsm_start();
  printf("Client start got %d\n", rc);
  events.set(EVT_ALARM);

  while (0 <= rc) {
    uptime::text_type buf;
    event_set::cev();

    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      if (pending.test_and_clear(EVT_CALADC)) {
        auto& machine = caladc.machine();
        if (auto pf = caladc.lpsm_process()) {
          if (caladc.PF_CALIBRATED & pf) {
            printf("%s ADC calibrated at %d\n", uptime::as_text(buf, uptime::now()),
                   caladc.calibrated_cCel());
            lpmclient.lpsm_calibrated(true);
          }
        } else if (machine.has_error()) {
          printf("%s: caladc error %d in %x\n", uptime::as_text(buf, uptime::now()),
                 caladc.lpsm_process_rc(), machine.state());
        }
      }
      if (pending.test_and_clear(EVT_PROCESS)) {
        auto& machine = lpmclient.machine();
        if (auto pf = lpmclient.lpsm_process()) {
          using lpm::state_machine;
          if (state_machine::PF_STARTED & pf) {
            printf("%s : client started\n", uptime::as_text(buf, uptime::now()));
          }
          if (lpmclient.PF_CALIBRATED & pf) {
            printf("%s: client calibrated\n", uptime::as_text(buf, uptime::now()));
          }
          if (state_machine::PF_OBSERVATION & pf) {
            auto r_Ohm = client.sample_Ohm();
            unsigned int lg = 100 * log(r_Ohm);
            printf("%s: sample %u adc16 ; %u mV ; %u Ohm ; %u lg; %u inten\n",
                   uptime::as_text(buf, uptime::now()),
                   client.sample_adc16(),
                   client.sample_mV(),
                   r_Ohm, lg, client.intensity());
          }
        } else if (machine.has_error()) {
          printf("%s: client error %d in %x\n", uptime::as_text(buf, uptime::now()),
                 lpmclient.lpsm_process_rc(), machine.state());
        }
      }
      if (pending.test_and_clear(EVT_ALARM)) {
        rc = lpmclient.lpsm_sample();
        if (0 != rc) {
          printf("%s: sample got %d\n", uptime::as_text(buf, uptime::now()), rc);
        }
      }
    }
    systemState::WFE();
  }
  printf("Exit rc %d\n", rc);
}
