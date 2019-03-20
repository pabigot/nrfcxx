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

#define EVT_ALARM 0x01
#define EVT_PROCESS 0x02
nrfcxx::event_set events;

class Client : public nrfcxx::sensor::adc::voltage_divider
{
  using super = nrfcxx::sensor::adc::voltage_divider;

public:
  Client (unsigned int r1_Ohm,
          unsigned int r2_Ohm,
          uint8_t ain,
          nrfcxx::misc::controlled_voltage* regulator = nullptr) :
    super{r1_Ohm, r2_Ohm, ain}
  {
    set_regulator(regulator);
  }

  uint8_t intensity () const;

private:

  int sample_setup () override
  {
    /* The HW5P-1 datasheet indicates a 2 us reaction time, but that's
     * at 10 V, 5 mA, a 100 Ohm lower resistor, and unspecified
     * illuminance.  LogicPro testing digital and analog channels
     * to determine time to analog stability shows:
     *
     * Standard drive: logic high dur 108 us, > 500 us max 800 us
     * High drive: no visible difference (which seems odd)
     *
     * So supply power to the transistor for at least 800 us before
     * the output voltage stabilizes with a 10 kOhm lower resistor and
     * dark conditions; it's much faster in bright light. */
    int rc = super::sample_setup();
    auto rv = nrfcxx::clock::uptime::from_ms(1);
    if (rv < rc) {
      rv = rc;
    }
    return rv;
  }
};

uint8_t
Client::intensity () const
{
  /* At 10 kOhm reference resistor the maximum measured resistance
   * would be about 650 MOhm ignoring tolerance.
   *
   * The lowest non-zero resistance is about 20 Ohm; 100 Ohm is
   * maximum brightness.
   *
   * The no-light condition is 4 GOhm, but the lowest non-zero
   * voltage produces about 100 MOhm (maximum darkness).
   *
   * We want a log representation of the potential range where low
   * is dark and high is bright; 8 bits of resolution are
   * sufficient.  So:
   *
   * 255 for values less than 100 Ohm
   * 0 for values exceeding 100 MOhm.
   * The range [ln(10^2), (10^9)] maps reversed to [1, 251). */
  static constexpr auto BRIGHT_Ohm = 100U;
  static constexpr auto BRIGHT_lg = 4.60517018598809136803;
  static constexpr auto DARK_Ohm = 100'000'000U;
  static constexpr auto DARK_lg = 18.42068074395236547214;
  unsigned int light_Ohm = sample_Ohm(0, true);
  if (BRIGHT_Ohm >= light_Ohm) {
    return 255;
  }
  if (DARK_Ohm <= light_Ohm) {
    return 0;
  }
  auto light_lg = log(light_Ohm);
  return 1U + 250U * (DARK_lg - light_lg) / (DARK_lg - BRIGHT_lg);
}

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
  auto vdd = misc::gpio_controlled_voltage{gpio::active_signal<false>{vdd_gp}};
  printf("vdd %d on %u\n", vdd.enabled(), vdd_gp.implementation().global_psel);

  using namespace std::literals;
  alarm
    .set_interval(1s)
    .set_deadline(alarm.interval())
    .schedule();

  Client client{0, 10000, 1, &vdd};

  sensor::adc::lpsm_wrapper lpmclient{events.make_setter(EVT_PROCESS), client};
  int rc = lpmclient.lpsm_start();
  printf("Client start got %d\n", rc);
  events.set(EVT_ALARM);

  while (0 <= rc) {
    uptime::text_type buf;
    event_set::cev();

    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      if (pending.test_and_clear(EVT_PROCESS)) {
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
        } else if (lpmclient.machine().has_error()) {
          printf("%s: client error %d in %x\n", uptime::as_text(buf, uptime::now()),
                 lpmclient.lpsm_process_rc(), lpmclient.machine().state());
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
