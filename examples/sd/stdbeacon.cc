// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Demonstration of the standard telemetry, system, and application
 * beacons. */

#include <nrfcxx/clock.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/sd/broadcaster.hpp>
#include <nrfcxx/sensor/adc.hpp>
#include <nrfcxx/sensor/button.hpp>
#include "nrf_sdm.h"

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

#define INSTR_PSEL_ACTIVE NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_RADIO NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn

#define WATCHDOG_DELAY_32KiHz (6 * 32768)
#define WATCHDOG_CHANNEL_ALARM 0

#define APP_EVT_ALARM    (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 0)
#define APP_EVT_BUTTON0  (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 1)
#define APP_EVT_ADC_CALIBRATED (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 2)
#define APP_EVT_TX_TLM (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 3)
#define APP_EVT_TX_SYS (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 4)

namespace {

/* Extend stack space by 1 KiBy to account for application
 * processing. */
__attribute__((__section__(".stack.extension")))
volatile uint32_t app_stack_extra[256];

nrfcxx::gpio::instr_psel<INSTR_PSEL_ACTIVE> instr_active;
nrfcxx::gpio::instr_psel<INSTR_PSEL_RADIO> instr_radio;
nrfcxx::gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;

static __attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

void
app_state_setup (const nrfcxx::systemState::state_type& ss,
                 bool is_reset,
                 bool retained)
{
  nrfcxx::sd::Broadcaster::state_setup(ss, is_reset, retained);
}

nrfcxx::systemState cs{core_state, 2018100121U, app_state_setup};

} // anonymous

extern "C" {

void ADCSeriesVariant_IRQHandler (void)
{
  nrfcxx::periph::ADC::irq_handler();
}

void GPIOTE_IRQHandler (void)
{
  nrfcxx::periph::GPIOTE::irq_handler();
}
}

int
main (void)
{
  using namespace nrfcxx;
  using nrfcxx::clock::uptime;

  /* Start the low-frequency clock infrastructure, but not the HF
   * crystal.  Release control of the POWER_CLOCK interrupt handler:
   * the soft-device uses this. */
  board::initialize(false);
  clock::configure_pcirq(false);

  instr_active.enable();
  instr_radio.enable();
  instr_aux.enable();

  auto& active_led = led::led_type::lookup(0);
  //active_led.enable();
  active_led.on();

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Display information about the cause of the restart. */
  {
    uptime::text_type buf;
    cprintf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           uptime::as_text(buf, cs.state().last_uptime));
    bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
    if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
      // Any state copied from the previous instance is incomplete or
      // invalid.
      cputs("***UNCONTROLLED RESET");
    }
    if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
      cprintf("- due to program code %u %s watchdog\n",
              cs.state().code, barked ? "with" : "without");
    } else if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
      cprintf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
      led::Pattern flasher{active_led};
      flasher.configure(0xAAAAAAAA, uptime::Frequency_Hz / 32, 5);
      volatile bool done = false;
      flasher.set_notify_complete([&done]()
                                  {
                                    done = true;
                                  });
      flasher.start();
      while (!done) {
        systemState::WFE();
      }
      systemState::systemOff(systemState::state_type::RESET_REAS_FAILSAFE, NRFCXX_BOARD_PSEL_BUTTON0);
    } else if (barked) {
      cprintf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
    } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
      cprintf("- due to direct system reset\n");
    } else if (systemState::state_type::RESET_REAS_OFF & cs.state().reset_reas) {
      cprintf("- due to DETECT wakeup from off mode\n");
    }
  }

  if (0 > cs.watchdogInit(WATCHDOG_DELAY_32KiHz,
                          (1 << WATCHDOG_CHANNEL_ALARM))) {
    cputs("Unable to start watchdog\n");
    delay_us(10000);
    cs.reset(cs.state().magic);
  }

#if (NRF52832 - 0) || (NRF52840 - 0)
    {
      using nrfcxx::nrf5::UICR;
      cprintf("PSELRESET %ld %ld\n", UICR->PSELRESET[0], UICR->PSELRESET[1]);
      if (0 > (int)UICR->PSELRESET[0]) {
        cputs("Configuring RESET pin");
        nrf5::series::enable_pinreset();
        uptime::sleep(uptime::from_ms(10));
        cs.reset(UICR->PSELRESET[0]);
      }
    }
#endif

  int rc;

  sd::Broadcaster broadcaster{cs};

  broadcaster.tlm_beacon.set_tx_notify(broadcaster.make_setter(APP_EVT_TX_TLM));
  led::Pattern tlm_led{led::lookup(1)};
  tlm_led.configure((1U << 31), uptime::Frequency_Hz / 1024, 1);
  broadcaster.sys_beacon.set_tx_notify(broadcaster.make_setter(APP_EVT_TX_SYS));
  led::Pattern sys_led{led::lookup(2)};
  sys_led.configure((1U << 31), uptime::Frequency_Hz / 1024, 1);

  /* Enable the ADC infrastructure */
  nvic_SetPriority(periph::ADC::INSTANCE.IRQn, IRQ_PRIORITY_APP_HIGH);
  nvic_EnableIRQ(periph::ADC::INSTANCE.IRQn);
  rc = broadcaster.vdd_sensor.claim();
  cprintf("Vdd claim got %d\n", rc);
  if (0 == rc) {
    auto set_calibrated = broadcaster.make_setter(APP_EVT_ADC_CALIBRATED);
    rc = broadcaster.vdd_sensor.calibrate([set_calibrated](){set_calibrated();});
    cprintf("Vdd calibrate got %d\n", rc);
  }

  rc = broadcaster.start();
  if (0 != rc) {
    if (0 < rc) {
      cprintf("Start soft-device failed: %x\n", rc);
    } else {
      cprintf("Start infrastructure failed: %d\n", rc);
    }
    return 1;
  }
  {
    auto bp = broadcaster.address().addr;
    cprintf("Broadcaster %02x %02x%02x%02x%02x%02x%02x start got %d\n",
           broadcaster.address().addr_type,
           bp[5], bp[4], bp[3], bp[2], bp[1], bp[0], rc);
    rc = broadcaster.tlm_beacon.activate();
    cprintf("TLM beacon start got %d\n", rc);
    rc = broadcaster.sys_beacon.activate();
    cprintf("SYS beacon start got %d\n", rc);
    rc = broadcaster.appid_beacon.activate();
    cprintf("APPID beacon start got %d\n", rc);
  }


#if (0 <= (NRFCXX_BOARD_PSEL_BUTTON0 - 0))
  sd::TelemetryBeacon& tlm_beacon{broadcaster.tlm_beacon};
  auto btn_callback = [&tlm_beacon](auto evt, auto dur)
    {
      auto now = uptime::now();
      if (sensor::Button::EVT_CLICK == evt) {
        uptime::text_type buf;
        cprintf("%s Reset beacon interval from %u\n\n", uptime::as_text(buf, now), tlm_beacon.interval_utt());
        tlm_beacon.reset_interval();
      } else if (sensor::Button::EVT_HOLD == evt) {
        const auto &ts = sd::Beacon::telemetry_state();
        cprintf("***Reset: tu %u, ac %u + %u\n",
                static_cast<unsigned int>(ts.total_uptime + now),
                ts.previous_adv, ts.current_adv);
        delay_us(10000);
        cs.reset(23);
      }
    };
  sensor::Button btn{NRFCXX_BOARD_PSEL_BUTTON0, broadcaster.make_setter(APP_EVT_BUTTON0), btn_callback};
#endif /* NRFCXX_BOARD_PSEL_BUTTON0 */

  NVIC_EnableIRQ(GPIOTE_IRQn);

  /* Wake up every 5 s to display a summary of what's happened.
   *
   * In a real application this alarm would be used to collect sensor
   * observations.  Note that the watchdog duration is determined by
   * the frequency of this alarm, which feeds the watchdog. */
  auto set_alarm = broadcaster.make_setter(APP_EVT_ALARM);
  clock::alarm alarm{[&set_alarm](auto& alarm)
      {
        instr_aux.assert();
        set_alarm();
        instr_aux.deassert();
        return true;
      }};
  {
    using namespace std::literals;
    alarm.set_interval(5s).set_deadline(0).schedule();
  }

  unsigned int nsde = 0;
  while (true) {
    event_set_copy pending;
    active_led.off();
    instr_active.deassert();
    /* For this application we don't need to know that non-application
     * events were processed. */
    do {
      pending = broadcaster.wait_for_event();
      /* Everything we care about radio state changes is handled by
       * Broadcaster.  Don't break out of the tight loop for them. */
      pending.test_and_clear(sd::Broadcaster::EVT_RADIO_ON | sd::Broadcaster::EVT_RADIO_OFF);
    } while (pending.empty());
    instr_active.assert();
    active_led.on();

    if (true) {
      uptime::text_type buf;
      unsigned int now = uptime::now();
      cprintf("%s evt %x\n", uptime::as_text(buf, now), pending.events());
    }
    if (pending.test_and_clear(sd::Broadcaster::EVT_SD)) {
      /** Storage for soft-device BLE events.
       *
       * * GATT_MTU_SIZE_DEFAULT is 23
       * * S130 does not support changing ATT_MTU size
       * * Buffer should be aligned to BLE_EVTS_PTR_ALIGNMENT (4).
       * * Implicit is that the buffer must be sized to carry an event
       *   structure plus a full GATT message.
       */
#ifndef GATT_MTU_SIZE_DEFAULT
#define GATT_MTU_SIZE_DEFAULT 23
#endif // GATT_MTU_SIZE_DEFAULT
      union {
        uint32_t u32[(sizeof(ble_evt_t) + GATT_MTU_SIZE_DEFAULT + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
        uint8_t u8[1];
      } ble_evt_buffer_union;
      uint16_t constexpr ble_evt_buffer_length = sizeof(ble_evt_buffer_union);
      uint8_t* const ble_evt_buffer = ble_evt_buffer_union.u8;
      unsigned int err;

      /* A broadcast-only device, especially one that doesn't support
       * scan responses, will probably never see a soft-device
       * event. */
      cputs("SD event\n");
      do {
        uint32_t evt_id;
        err = sd_evt_get(&evt_id);
        cprintf("evt_get got %x, evt_id %lx\n", err, evt_id);
        ++nsde;
      } while (NRF_SUCCESS == err);
      if (NRF_ERROR_NOT_FOUND != err) {
        cprintf("evt_get returned unexpected error: %x\n", err);
        return 1;
      }

      do {
        uint16_t len = ble_evt_buffer_length;
        err = sd_ble_evt_get(ble_evt_buffer, &len);
        cprintf("ble_evt_get got %x, len %u\n", err, len);
        ++nsde;
      } while (NRF_SUCCESS == err);
      if (NRF_ERROR_NOT_FOUND != err) {
        cprintf("evt_get returned unexpected error: %x\n", err);
        return 1;
      }
    }
    // Process completed calibration before attempting to take sample
    if (pending.test_and_clear(APP_EVT_ADC_CALIBRATED)) {
      //cputs("ADC calibrated");
      broadcaster.vdd_sensor.release();
    }
    if (pending.test_and_clear(sd::Broadcaster::EVT_VDD_REQUIRED)) {
      int rc = broadcaster.vdd_sensor.claim();
      if (0 == rc) {
        rc = broadcaster.vdd_sensor.sample();
      }
      cprintf("VDD sample got %d\n", rc);
    }
    if (pending.test_and_clear(sd::Broadcaster::EVT_VDD_UPDATED)) {
      int rc = broadcaster.vdd_sensor.release();
      cprintf("VDD got %u, release %d\n", broadcaster.vdd_mV(), rc);
    }
    if (pending.test_and_clear(APP_EVT_TX_TLM)) {
      tlm_led.start(0);
      cputs("TLM transmitted");
    }
    if (pending.test_and_clear(APP_EVT_TX_SYS)) {
      sys_led.start(0);
      cputs("SYS transmitted");
    }
#if (0 <= (NRFCXX_BOARD_PSEL_BUTTON0 - 0))
    if (pending.test_and_clear(APP_EVT_BUTTON0)) {
      btn.process();
    }
#endif /* NRFCXX_BOARD_PSEL_BUTTON0 */
    if (pending.test_and_clear(APP_EVT_ALARM)) {
      cs.watchdogFeed(WATCHDOG_CHANNEL_ALARM);
      uptime::text_type buf;
      uint64_t sleep;
      uint64_t radio;
      unsigned int scale = 1000000;
      auto total = cs.operationalModeBreakdown(sleep, radio);
      cprintf("%s main loop, %u SD events, hu %u / %u, su %u / %u\n", uptime::as_text(buf, uptime::now()), nsde,
             cs.heap_used(), cs.heap_reserved(),
             cs.stack_used(), cs.stack_reserved());
      if (0 < total) {
        static unsigned int last_wfe;
        unsigned int wfe{cs.wfe_count()};
        cprintf("  wake %u ppm; radio %u ppm; dwfe %u\n",
                scale - static_cast<unsigned int>((total - 1U + scale * sleep) / total),
                static_cast<unsigned int>((total - 1U + scale * radio) / total),
                wfe - last_wfe);
        last_wfe = wfe;
      }
    }
  }

  return 0;
}
