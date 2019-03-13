// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/core.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/bme280.hpp>

#ifndef WITH_SPI
#define WITH_SPI 0
#endif // WITH_SPI

int
main (void)
{
  using namespace nrfcxx;
  using clock::uptime;

  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto& active_led = nrfcxx::led::led_type::lookup(0);
  auto& error_led = nrfcxx::led::led_type::lookup(1);
  auto& sampling_led = nrfcxx::led::led_type::lookup(2);
  active_led.enable();
  active_led.on();
  error_led.enable();
  sampling_led.enable();

  event_set events;
  constexpr event_set::event_type EVT_SAMPLE = 0x01;
  constexpr event_set::event_type EVT_BME280 = 0x02;

  int rc;
#if (WITH_SPI - 0)
  printf("SPI1: SCK %d, MOSI %d, MISO %d, CSn %d\n",
         NRFCXX_BOARD_PSEL_SPI1_SCK,
         NRFCXX_BOARD_PSEL_SPI1_MOSI,
         NRFCXX_BOARD_PSEL_SPI1_MISO,
         NRFCXX_BOARD_PSEL_SPI1_CSN);

  auto& spi = periph::SPI::instance(1);
  rc = spi.bus_configure(NRFCXX_BOARD_PSEL_SPI1_SCK,
                         NRFCXX_BOARD_PSEL_SPI1_MOSI,
                         NRFCXX_BOARD_PSEL_SPI1_MISO,
                         SPI_FREQUENCY_FREQUENCY_M8,
                         0);
  sensor::bme280 bme280{events.make_setter(EVT_BME280), spi, NRFCXX_BOARD_PSEL_SPI1_CSN};
  spi.enable();
#else
  printf("BME280 on I2C at SCL %d SDA %d\n",
         NRFCXX_BOARD_PSEL_TWI0_SCL,
         NRFCXX_BOARD_PSEL_TWI0_SDA);

  auto& twi = periph::TWI::instance(0);

  rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                         NRFCXX_BOARD_PSEL_TWI0_SDA,
                         (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                         200);
  sensor::bme280 bme280{events.make_setter(EVT_BME280), twi, true};
#endif
  printf("bus configure: %d\n", rc);

  rc = bme280.initialize();
  printf("init got %d\n", rc);
  if (0 != rc) {
    return -1;
  }

  sensor::bme280::observations_type obs{};

  rc = bme280.sample();
  printf("sample got %d\n", rc);
  uptime::sleep(bme280.SAMPLE_DELAY_utt);
  rc = bme280.fetch(obs);
  printf("fetch got %d ; %d cCel ; %u cPa ; %u pptt RH\n",
         rc, obs.temperature_cCel, obs.pressure_cPa, obs.humidity_pptt);

#if (WITH_SPI - 0)
  spi.disable();
#else // WITH_SPI
  twi.disable();
#endif // WITH_SPI

  auto alarm_sample = clock::alarm::for_event<EVT_SAMPLE, true>(events);
  alarm_sample
    .set_interval(5 * uptime::Frequency_Hz)
    .set_deadline(alarm_sample.interval())
    .schedule();

  const auto& machine = bme280.machine();

  rc = bme280.lpsm_start();
  printf("lpm start: %d %d\n", rc, machine.state());

  bool errored = false;
  while (true) {
    using lpm::state_machine;
    uptime::text_type as_text;

    nrfcxx::event_set::cev();
    auto pending = events.copy_and_clear();
    if (pending.test_and_clear(EVT_SAMPLE)) {
      rc = bme280.lpsm_sample();
      if (0 > rc) {
        errored = true;
      } else {
        sampling_led.on();
      }
    }
    if (pending.test_and_clear(EVT_BME280) && (!errored)) {
      if (auto pf = bme280.lpsm_process()) {
        if (state_machine::PF_OBSERVATION & pf) {
          sampling_led.off();
          auto &obs = bme280.observations();
          printf("%s: %d cCel ; %u cPa ; %u pptt RH\n",
                 uptime::as_text(as_text, uptime::now()),
                 obs.temperature_cCel, obs.pressure_cPa, obs.humidity_pptt);
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
      bme280.lpsm_stop();
    }
    active_led.off();
    do {
      systemState::WFE();
    } while (!events);
    active_led.on();
  }

  return 0;
}
