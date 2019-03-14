// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Demonstrate API for SHT31 sensors. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/sht31.hpp>
#include <nrfcxx/sensor/utils.hpp>
#include <nrfcxx/utility.hpp>

#define EVT_SAMPLE 0x01
#define EVT_SHT31  0x02

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  auto& active_led = led::led_type::lookup(0);
  auto& error_led = led::led_type::lookup(1);
  auto& sampling_led = led::led_type::lookup(2);
  active_led.enable();
  active_led.on();
  error_led.enable();
  sampling_led.enable();

  using clock::uptime;
  using periph::TWI;

  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  printf("SHT31 on I2C at SCL %d SDA %d\n",
         NRFCXX_BOARD_PSEL_TWI0_SCL,
         NRFCXX_BOARD_PSEL_TWI0_SDA);

  auto& twi = TWI::instance(0);
  int rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                             NRFCXX_BOARD_PSEL_TWI0_SDA,
                             (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                             200);
  printf("I2C configure: %d\n", rc);

  if (auto enabler = twi.scoped_enable()) {
    rc = twi.general_call_reset();
    printf("General Call Reset got %d\n", rc);
  }

  nrfcxx::event_set events;

  sensor::sht31::iface_config_type ifc{twi};
  sensor::sht31 sht31{events.make_setter(EVT_SHT31), ifc};

  rc = sht31.status();
  printf("status %d: %04x\n", rc, rc);

  rc = sht31.reset();
  printf("reset: %d\n", rc);

  auto& obs = sht31.observations();

  rc = sht31.status();
  printf("status %d: %04x\n", rc, rc);
  rc = sht31.trigger();
  printf("trigger: %d; %d %d\n", rc, obs.temperature_cCel, obs.humidity_pptt);
  if (0 < rc) {
    sleep_ms(rc);
    rc = sht31.fetch();
    printf("fetch %d; %d %d\n", rc, obs.temperature_cCel, obs.humidity_pptt);
    rc = sht31.status();
    printf("status %d: %04x\n", rc, rc);
  }

  /* Configure the periodic sample alarm.  Don't start it until LPSM
   * indicates the startup sequence has completed. */
  auto alarm_sample = clock::alarm::for_event<EVT_SAMPLE, true>(events);
  alarm_sample
    .set_deadline(0)
    .set_interval(5 * uptime::Frequency_Hz);

  const auto& machine = sht31.machine();

  rc = sht31.lpsm_start();
  printf("lpm start %d\n", rc);

  bool errored = false;
  while (true) {
    uptime::text_type as_text;

    event_set::cev();
    auto pending = events.copy_and_clear();

    if (pending.test_and_clear(EVT_SAMPLE)) {
      rc = sht31.lpsm_sample();
      if (0 > rc) {
        errored = true;
      } else {
        sampling_led.on();
      }
    }
    if (pending.test_and_clear(EVT_SHT31) && (!errored)) {
      using lpm::state_machine;
      if (auto pf = sht31.lpsm_process()) {
        if (state_machine::PF_STARTED & pf) {
          alarm_sample.schedule();
        }
        if (state_machine::PF_OBSERVATION & pf) {
          sampling_led.off();
          printf("%s: %+5d cCel ; %4u [pptt_RH]\n",
                 uptime::as_text(as_text, uptime::now()),
                 obs.temperature_cCel, obs.humidity_pptt);
        }
      } else if (machine.has_error()) {
        printf("SHT31 error %d\n", machine.error());
        errored = true;
      }
    }
    if (errored) {
      printf("%s: state %x error %d\n",
             uptime::as_text(as_text, uptime::now()),
             machine.state(), machine.error());
    }
    if (errored) {
      error_led.on();
      alarm_sample.cancel();
      sht31.lpsm_stop();
    }
    active_led.off();
    do {
      systemState::WFE();
    } while (!events);
    active_led.on();
  }

  return 0;
}
