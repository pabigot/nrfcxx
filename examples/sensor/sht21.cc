// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Demonstrate API for SHT21, HTU21D, and other compatible sensors. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/sht21.hpp>
#include <nrfcxx/sensor/utils.hpp>
#include <nrfcxx/utility.hpp>

namespace {
  nrfcxx::event_set events;
#define EVT_SAMPLE 0x01
#define EVT_SHT21  0x02
}

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

  using sensor::temperature_cK_cCel;
  using clock::uptime;
  using periph::TWI;

  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  printf("SHT21 on I2C at SCL %d SDA %d\n",
         NRFCXX_BOARD_PSEL_TWI0_SCL,
         NRFCXX_BOARD_PSEL_TWI0_SDA);

  auto& twi = TWI::instance(0);
  int rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                             NRFCXX_BOARD_PSEL_TWI0_SDA,
                             (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                             200);
  printf("configure: %d\n", rc);

  sensor::sht21::iface_config_type ifc{twi};
  sensor::sht21 sht21{events.make_setter(EVT_SHT21), ifc};

  rc = ifc.twi.enable();
  printf("enable: %d\n", rc);
  rc = sht21.reset();
  printf("reset: %d\n", rc);
  uptime::sleep(uptime::from_ms(sht21.RESET_DELAY_ms));

  printf("configure: %d\n", sht21.configure(-1));
  rc = sht21.configure(sht21.CONFIG_RES_H12T14);
  printf("configure: %d = 0x%x\n", rc, rc);
  printf("configure: %d\n", sht21.configure(-1));

  sensor::sht21::eic_type eic;
  rc = sht21.read_eic(eic);
  printf("read eic: %d\n", rc);
  if (0 == rc) {
    utility::display_data(eic, sizeof(eic), 0);
  }
  ifc.twi.disable();

  uptime::text_type as_text;

  int t_cK = 0;
  int rh_pptt = 0;
  unsigned int t_utt = 0;
  unsigned int rh_utt = 0;
  printf("before %d\n", ifc.twi.enabled());
  ifc.twi.enable();
  do {
    uptime::timestamp24 ts;
    if (0 == sht21.trigger_temperature()) {
      do {
        t_cK = sht21.temperature_cK();
      } while (sht21.NOT_READY == t_cK);
      t_utt = ts.delta();
    }
    ts.reset();
    if (0 == sht21.trigger_humidity()) {
      do {
        rh_pptt = sht21.humidity_pptt();
      } while (sht21.NOT_READY == rh_pptt);
      rh_utt = ts.delta();
    }
  } while (false);
  printf("after %d\n", ifc.twi.enabled());
  ifc.twi.disable();
  printf("%s: %5u cK %+5d cCel (%u us) ; %4u [pptt_RH] (%u us)\n",
         uptime::as_text(as_text, uptime::now()),
         t_cK,
         temperature_cK_cCel(t_cK),
         (unsigned int) uptime::to_us(t_utt),
         rh_pptt, (unsigned int) uptime::to_us(rh_utt));

  auto alarm_sample = clock::alarm::for_event<EVT_SAMPLE, true>(events);
  alarm_sample
    .set_interval(5 * uptime::Frequency_Hz)
    .set_deadline(alarm_sample.interval())
    .schedule();

  const auto& machine = sht21.machine();

  rc = sht21.lpsm_start();
  printf("lpm start %d\n", rc);

  bool errored = false;
  while (true) {
    event_set::cev();
    auto pending = events.copy_and_clear();

    if (pending.test_and_clear(EVT_SAMPLE)) {
      rc = sht21.lpsm_sample();
      if (0 > rc) {
        errored = true;
      } else {
        sampling_led.on();
      }
    }
    if (pending.test_and_clear(EVT_SHT21) && (!errored)) {
      using lpm::state_machine;
      if (auto pf = sht21.lpsm_process()) {
        if (state_machine::PF_OBSERVATION & pf) {
          sampling_led.off();
          auto t_cK = sht21.observations().temperature_cK;
          auto rh_pptt = sht21.observations().humidity_pptt;
          printf("%s: %5u cK %+5d cCel ; %4u [pptt_RH]\n",
                 uptime::as_text(as_text, uptime::now()),
                 t_cK,
                 temperature_cK_cCel(t_cK),
                 rh_pptt);
        }
      }
    }
    if (errored || machine.has_error()) {
      printf("%s: machine %x error %d rc %d\n",
             uptime::as_text(as_text, uptime::now()),
             machine.state(), machine.error(), rc);
    }
    if (errored) {
      error_led.on();
      alarm_sample.cancel();
      sht21.lpsm_stop();
    }
    active_led.off();
    do {
      systemState::WFE();
    } while (!events);
    active_led.on();
  }

  return 0;
}
