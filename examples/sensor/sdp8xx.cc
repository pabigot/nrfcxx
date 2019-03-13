// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrate API for Sensirion SDP-8xx differential pressure sensor. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/sdp8xx.hpp>
#include <nrfcxx/sensor/utils.hpp>
#include <nrfcxx/utility.hpp>

namespace {
  nrfcxx::event_set events;
#define EVT_SAMPLE 0x01
#define EVT_SDP8xx 0x02
}

static constexpr uint8_t ADDRESS = 0x25;
static constexpr uint16_t CMD_TRIG_DP_POLL = 0x362F;
static constexpr uint16_t CMD_SOFT_RESET = 0x0006;
static constexpr uint16_t CMD_READ_PI = 0x367C;
static constexpr uint16_t CMD_READ = 0x3615;

using crc_type = uint8_t;

crc_type
crc (const uint8_t* dp,
     size_t count)
{
  constexpr uint16_t poly = 0x131;
  constexpr crc_type sobit = 0x80;
  crc_type crc = 0xFF;
  while (0 != count--) {
    crc ^= *dp++;
    for (unsigned int bi = 0; bi < 8; ++bi) {
      crc = (crc << 1) ^ ((sobit & crc) ? poly : 0U);
    }
  }
  return crc;
}

int
main (void)
{
  using namespace nrfcxx;
  using clock::uptime;

  board::initialize();

  auto& active_led = led::led_type::lookup(0);
  auto& error_led = led::led_type::lookup(1);
  auto& sampling_led = led::led_type::lookup(2);
  active_led.enable();
  active_led.on();
  error_led.enable();
  sampling_led.enable();

  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  printf("SDP810 on I2C at SCL %d SDA %d\n",
         NRFCXX_BOARD_PSEL_TWI0_SCL,
         NRFCXX_BOARD_PSEL_TWI0_SDA);

  auto& twi = periph::TWI::instance(0);
  int rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                             NRFCXX_BOARD_PSEL_TWI0_SDA,
                             (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                             200);
  printf("configure: %d\n", rc);

  sensor::sdp8xx::iface_config_type ifc{twi};
  sensor::sdp8xx sdp8xx{events.make_setter(EVT_SDP8xx), ifc};
  rc = ifc.twi.enable();
  {
    uint32_t product = 0;
    uint64_t serial = 0;
    rc = sdp8xx.read_device_info(product, serial);
    printf("read got %d: %08" PRIx32 " %08" PRIx32 "%08" PRIx32 "\n", rc, product,
           static_cast<uint32_t>(serial >> 32),
           static_cast<uint32_t>(serial));
  }
  ifc.twi.disable();

  auto alarm_sample = clock::alarm::for_event<EVT_SAMPLE, true>(events);
  alarm_sample
    .set_interval(5 * uptime::Frequency_Hz)
    .set_deadline(alarm_sample.interval())
    .schedule();
  events.set(EVT_SDP8xx);

  const auto& machine = sdp8xx.machine();

  uptime::text_type as_text;

  rc = sdp8xx.lpsm_start();
  printf("lpm start: %d\n", rc);
  bool errored = false;
  while (true) {
    event_set::cev();
    auto pending = events.copy_and_clear();
    if (pending.test_and_clear(EVT_SAMPLE)) {
      rc = sdp8xx.lpsm_sample();
      if (0 > rc) {
        errored = true;
      } else {
        sampling_led.on();
      }
    }
    if (pending.test_and_clear(EVT_SDP8xx) && (!errored)) {
      using lpm::state_machine;
      if (auto pf = sdp8xx.lpsm_process()) {
        if (state_machine::PF_OBSERVATION & pf) {
          sampling_led.off();
          if (!alarm_sample.active()) {
            alarm_sample.schedule();
          }
          printf("%s: %+5d cPa ; %+5d cCel\n",
                 uptime::as_text(as_text, uptime::now()),
                 sdp8xx.observations().diffpres_cPa,
                 sdp8xx.observations().temperature_cCel);
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
      sdp8xx.lpsm_stop();
    }
    if (!events) {
      active_led.off();
      do {
        systemState::WFE();
      } while (!events);
      active_led.on();
    }
  }

  return 0;
}
