// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Test several on-board Thingy:52 peripherals:
 * * HTS221 temperature and humidity
 * * Power monitor and LiPo status
 * * LEDs.
 */

#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/utility.hpp>
#include <nrfcxx/board/thingy52.hpp>
#include <nrfcxx/sensor/button.hpp>

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

#define WITH_HTS221 1

#if (WITH_HTS221 - 0)
#include <nrfcxx/sensor/hts221.hpp>
#endif // WITH_HTS221

namespace {

constexpr nrfcxx::event_set::event_type EVT_ALARM = 0x01;
constexpr nrfcxx::event_set::event_type EVT_BUTTON = 0x02;
constexpr nrfcxx::event_set::event_type EVT_POWERMON = 0x04;
constexpr nrfcxx::event_set::event_type EVT_SHOWBATT = 0x08;
constexpr nrfcxx::event_set::event_type EVT_HTS221 = 0x100;
nrfcxx::event_set events;

/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

nrfcxx::systemState cs{core_state, 2018101520};

void
button_callback (unsigned int evt,
                 uint64_t duration_utt)
{
  using nrfcxx::sensor::Button;
  using nrfcxx::clock::uptime;

  uptime::text_type buf;
  cprintf("BTN %s ", uptime::as_text(buf, uptime::now()));
  cprintf("%s for %s\n", Button::eventstr(evt), uptime::as_text(buf, duration_utt));
  if (Button::EVT_CLICK == evt) {
    events.set(EVT_SHOWBATT);
  }
}

void
show_iox_registers ()
{
  using namespace nrfcxx;

  auto& gpio_cache = board::iox().gpio_cache();
  auto sp = &gpio_cache.input_disable;
  cputs("SX1509 registers:");
  while (sp <= &gpio_cache.data) {
    cprintf("%02x: %04x\n", 2 * (sp - &gpio_cache.input_disable), *sp);
    ++sp;
  }

  cputs("PIN_CNF:");
  for (auto psel = 0U; psel < 16; ++psel) {
    cprintf(" %08x", board::iox().configuration(psel));
    if (0x3 == (0x3 & psel)) {
      putchar('\n');
    }
  }
}

void
show_all_iox ()
{
  using nrfcxx::misc::sx1509b;

  auto& iox = nrfcxx::board::iox();
  auto& twi = iox.iface_config().twi;
  auto addr = iox.iface_config().address;

  uint8_t buf[128];
  int rc;
  if (auto enabler = twi.scoped_enable()) {
    uint8_t reg = 0;
    rc = twi.write_read(addr, &reg, 1, buf, sizeof(buf));
  } else {
    rc = enabler.result();
  }
  if (0 > rc) {
    printf("fetch failed: %d\n", rc);
    return;
  }
  const uint8_t* sp = buf;
  static const char* const gpio[] = {
    "2InputDisable",
    "2LongSlew",
    "2LowDrive",
    "2PullUp",
    "2PullDown",
    "2OpenDrain",
    "2Polarity",
    "2Dir",
    "2Data",
    "2InterruptMask",
    "2SenseHigh",
    "2SenseLow",
    "2InterruptSource",
    "2EventStatus",
    "2LevelShifter",
    "1Clock",
    "1Misc",
    "2LEDDriverEnable",
    "1DebounceConfig",
    "2DebounceEnable",
    "1KeyConfig1",
    "1KeyConfig2",
    "1KeyData1",
    "1KeyData2",
    "BALED0",
    "BALED1",
    "BALED2",
    "BALED3",
    "FALED4",
    "FALED5",
    "FALED6",
    "FALED7",
    "BBLED0",
    "BBLED1",
    "BBLED2",
    "BBLED3",
    "FBLED4",
    "FBLED5",
    "FBLED6",
    "FBLED7",
    "2HighInput",
  };
  auto const sps = sp;
  auto rpp = gpio;
  auto const rppe = rpp + sizeof(gpio) / sizeof(*gpio);
  while (rpp < rppe) {
    auto const type = **rpp;
    auto const name = 1 + *rpp;
    ++rpp;
    printf("%02x: %20s ", sp - sps, name);
    switch (type) {
      case '1':
        printf("%02x", *sp++);
        break;
      case '2':
        {
          uint16_t v;
          memmove(&v, sp, 2);
          sp += 2;
          printf("%04x", pabigot::byteorder::host_x_be(v));
        }
        break;
      case 'B':
      case 'F':
        printf("%u %u %u:%u", sp[0], sp[1],
               (sx1509b::RegOffX_TOff_Msk & sp[2]) >> sx1509b::RegOffX_TOff_Pos,
               (sx1509b::RegOffX_IOff_Msk & sp[2]) >> sx1509b::RegOffX_IOff_Pos);
        sp += 3;
        if ('F' == type) {
          printf(" %u %u", sp[0], sp[1]);
          sp += 2;
        }
        break;
    }
    putchar('\n');
  }
}

} // ns anonymous

extern "C" {
void GPIOTE_IRQHandler (void)
{
  nrfcxx::periph::GPIOTE::irq_handler();
}

void ADCSeriesVariant_IRQHandler (void)
{
  nrfcxx::periph::ADC::irq_handler();
}
} // extern C

int
main (void)
{
  using namespace nrfcxx;
  using namespace pabigot;

  board::initialize();

  auto& led_active = led::lookup(5);
  led_active.on();

  cputs("\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Display information about the cause of the restart. */
  {
    clock::uptime::text_type buf;
    cprintf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
            cs.state().reset_count, cs.state().reset_reas,
            cs.state().last_pc,
            clock::uptime::as_text(buf, cs.state().last_uptime));
    cprintf("total uptime %s\n",
            clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    cputs("***UNCONTROLLED RESET\n");
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    cprintf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    if (barked && (cs.state().magic == cs.state().code)) {
      cputs("- to put previous watchdog to sleep");
    } else {
      cprintf("- due to program code %u %s watchdog\n",
              cs.state().code, barked ? "with" : "without");
      delay_us(10000);
      systemState::systemOff(0, -1);
    }
  } else if (barked) {
    cprintf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    cprintf("- due to direct system reset\n");
  }

#define WATCHDOG_DELAY_32KiHz (10 * 32768)
#define WATCHDOG_CHANNEL_ALARM 0

  cs.watchdogInit(WATCHDOG_DELAY_32KiHz,
                  (1 << WATCHDOG_CHANNEL_ALARM));

  int rc;

  if (true) {
    show_iox_registers();
  }

  board::power_monitor powermon{events.make_setter(EVT_POWERMON)};
  rc = powermon.lpsm_start();
  cprintf("Powermon start got %d\n", rc);

  sensor::Button button{NRFCXX_BOARD_PSEL_BUTTON0, events.make_setter(EVT_BUTTON), button_callback};

  NVIC_EnableIRQ(GPIOTE_IRQn);
  periph::GPIOTE::enable_sense();

  auto alarm = clock::alarm::for_event<EVT_ALARM, true>(events);
  alarm
    .set_interval(5 * clock::uptime::Frequency_Hz)
    .set_deadline(0)
    .schedule();

  board::iox_pin ioext{NRFCXX_BOARD_IOX_EXT0};
  cprintf("IOX bit %x\n", ioext.bit);

  auto& ledr = led::lookup(0);
  auto& ledg = led::lookup(1);
  auto& ledb = led::lookup(2);
  ledr.enable();
  ledg.enable();

  if (true) {
    for (auto batt_mV : {
      4200,
      3990,
      3950,
      3850,
      3750,
      3650,
      3550,
      3200,
      3100,
      2900,
        }) {
      printf("%u mV = %u pptt\n", batt_mV, board::battery_level_pptt(batt_mV));
    }
  }
  if (false) {
    // Bypass LED driver, confirm basic LED operation
    ledb.enable();
    ledb.on();
  } else if (false) {
    // Confirm basic LED driver operation on one LED
    using misc::sx1509b;
    show_all_iox();
    constexpr uint8_t led_psel = NRFCXX_BOARD_IOX_LIGHTWELL_B;
    constexpr uint16_t led_bits = 1U << led_psel;
    rc = board::enable_led_driver(led_bits);
    printf("as led %d\n", rc);
    sx1509b::led_pwm_type cfg{};
    cfg.ion = 127;
    for (auto ion = 15U; ion < 256U; ion += 16) {
      rc = board::iox().output_sct(led_bits, 0);
      cfg.ion = ion;
      rc = board::iox().led_configure(led_psel, cfg);
      printf("LED cfg %d: %u %u %u:%u %u %u\n",
             rc, cfg.ton, cfg.ion,
             (sx1509b::RegOffX_TOff_Msk & cfg.off) >> sx1509b::RegOffX_TOff_Pos,
             (sx1509b::RegOffX_IOff_Msk & cfg.off) >> sx1509b::RegOffX_IOff_Pos,
             cfg.trise, cfg.tfall);
      rc = board::iox().output_sct(0, led_bits);
      printf("On got %d\n", rc);
      cs.watchdogFeed(WATCHDOG_CHANNEL_ALARM);
      sleep_ms(1000);
    }
    rc = board::iox().output_sct(led_bits, 0);
  } else {
    // Full LED driver operation
    rc = board::enable_led_driver();
    printf("LED driver enable got %d\n", rc);
  }

  cprintf("USB %x, CHARGE %x\n", (1U << NRFCXX_BOARD_PSEL_USB_DETECT), (1U << NRFCXX_BOARD_PSEL_BAT_CHG_STAT));

#if (WITH_HTS221 - 0)
  sensor::hts221::iface_config_type ifc{board::twi(), NRFCXX_BOARD_PSEL_HTS_INT};
  sensor::hts221 hts221{events.make_setter(EVT_HTS221), ifc};
  if (true) {
    // hts221.odr(hts221.ODR_7_Hz);
    hts221.odr(hts221.ODR_OneShot);
    rc = hts221.lpsm_start();
    cprintf("hts221 start %d: %d\n", hts221.odr(), rc);
  }
#else
  (void)EVT_HTS221;
#endif // WITH_HTS221
  bool sample_batt = true;
  do {
    using clock::uptime;
    using lpm::state_machine;
    uptime::text_type as_text;

    event_set::cev();
    auto pending = events.copy_and_clear();
    auto now = uptime::now();

    if (pending.test_and_clear(EVT_ALARM)) {
      static const char* const ps_str[] = {
        "Unknown",
        "OnBattery",
        "Charging",
        "Charged",
      };
      cprintf("%s alarm ; %s ; live %sVCHG %sCHARGING %lx\n", uptime::as_text(as_text, now),
              ps_str[powermon.power_source()],
              powermon.vchg_detected_live() ? "" : "no ",
              powermon.charging_live() ? "" : "not ",
              nrf5::GPIO->IN);
      rc = powermon.lpsm_sample();
      if (0 != rc) {
        cprintf("!!BATT sample %d, %x\n", rc, powermon.machine().state());
      }
#if (WITH_HTS221 - 0)
      rc = hts221.lpsm_sample();
      if (0 != rc) {
        cprintf("!!HTS221 sample %d, %x\n", rc, hts221.machine().state());
      }
#endif // WITH_HTS221
      cs.watchdogFeed(WATCHDOG_CHANNEL_ALARM);
    }
    if (pending.test_and_clear(EVT_BUTTON)) {
      button.process();
    }
    if (pending.test_and_clear(EVT_POWERMON)) {
      if (auto pf = powermon.lpsm_process()) {
        using lpm::state_machine;
        if (state_machine::PF_STARTED & pf) {
          cputs("POWER started\n");
        }
        if (board::power_monitor::PF_MAINS & pf) {
          cputs("POWER on external");
        }
        if (board::power_monitor::PF_LIPO & pf) {
          cputs("POWER on battery");
          ledr.off();
          ledg.off();
        }
        if (board::power_monitor::PF_CHARGING & pf) {
          cputs("POWER battery charging");
          ledr.on();
          ledg.off();
        }
        if (board::power_monitor::PF_CHARGED & pf) {
          cputs("POWER battery charged");
          ledr.off();
          ledg.on();
        }
        if (state_machine::PF_OBSERVATION & pf) {
          cprintf("%s BATT %d mV\n", uptime::as_text(as_text, now), powermon.batt_mV());
        }
        /* We want a battery sample on startup, but we can't do it all
         * startup activities stop.  That'll be on one of PF_LIPO,
         * PF_CHARGING, or PF_CHARGED but we can't tell which.  We
         * could check for IDLE explicitly, or just rely on the one
         * built in to lpsm_sample() which is a bit more robust. */
        if (sample_batt) {
          rc = powermon.lpsm_sample();
          if (0 <= rc) {
            sample_batt = false;
          }
        }
      } else if (powermon.machine().has_error()) {
        cprintf("POWER machine error %d\n", powermon.machine().error());
      }
    }
#if (WITH_HTS221 - 0)
    if (pending.test_and_clear(EVT_HTS221)) {
      if (auto pf = hts221.lpsm_process()) {
        if (state_machine::PF_OBSERVATION & pf) {
          auto& obs = hts221.observations();
          cprintf("%s HTS221 %d cCel ; %u pptt\n", uptime::as_text(as_text, now),
                  obs.temperature_cCel, obs.humidity_pptt);
        }
        if (state_machine::PF_STARTED & pf) {
          auto& cal = hts221.calibration();
          cprintf("HTS221 started: cal %d %d cCel to %d %d\n\t; %u %u pptt to %d %d; drdy %d\n",
                  cal.cal_cCel[0], cal.cal_cCel[1], cal.t0_out, cal.td_out,
                  cal.cal_pptt[0], cal.cal_pptt[1], cal.h0_out, cal.hd_out,
                  hts221.drdy_asserted());
        }
      }
      if (hts221.machine().has_error()) {
        cprintf("HTS221 error %d\n", hts221.machine().error());
      }
    }
#endif // WITH_HTS221
    if (pending.test_and_clear(EVT_SHOWBATT)) {
      auto batt_mV = powermon.batt_mV();
      auto& iox = board::iox();
      uint16_t led_bit = 0;
      rc = board::led_setup_battery_display(batt_mV);
      if (0 < rc) {
        led_bit = rc;
      }
      if (0 <= rc) {
        rc = iox.output_sct(0, led_bit);
      }
      // The LED has to be enabled long enough for the cycle to start.
      sleep_ms(1);
      if (0 <= rc) {
        rc = iox.output_sct(led_bit, 0);
      }
    }
    if (!events) {
      led_active.off();
      ioext.clear();
      while (!events) {
        systemState::WFE();
      }
      ioext.set();
      led_active.on();
    }
  } while (true);

  cprintf("completed\n");

  return 0;
}
