// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Basic Beacon application.
 *
 * Transmit a beacon that has a fixed header and a count of
 * transmissions.  The beacon has a backoff transmission schedule
 * ranging from 1 s to 30 s between transmissions.
 *
 * The application wakes every 10 s to display statistics on memory
 * use, duty cycle, and wakeups.
 *
 * This verifies the basic Beacon infrastructure, and is simple enough
 * to confirm that desired power profiles are satisfied. */

#include <cinttypes>
#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/sd/beacon.hpp>
#include <nrfcxx/sd/broadcaster.hpp>

#include "nrf_sdm.h"
#include "ble.h"

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

#define INSTR_PSEL_ACTIVE NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_RADIO NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn

#define APP_INTERVAL_s 10U
#define APP_EVT_ALARM (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 0)
#define APP_EVT_TX_BEACON (nrfcxx::sd::Broadcaster::EVT_APP_BASE << 1)

#define WATCHDOG_DELAY_32KiHz ((1 + APP_INTERVAL_s) * 32768)
#define WATCHDOG_CHANNEL_ALARM 0

namespace {
/* Extend stack space by 1 KiBy to account for application
 * processing. */
__attribute__((__section__(".stack.extension")))
volatile uint32_t sd_stack_addition[512];

nrfcxx::gpio::instr_psel<INSTR_PSEL_ACTIVE> instr_active;
nrfcxx::gpio::instr_psel<INSTR_PSEL_RADIO> instr_radio;
nrfcxx::gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;

__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

void
app_state_setup (const nrfcxx::systemState::state_type& ss,
                 bool is_reset,
                 bool retained)
{
  nrfcxx::sd::Broadcaster::state_setup(ss, is_reset, retained);
}

nrfcxx::systemState cs{core_state, 2019021608U, app_state_setup};

/** Content is stored in big-endian order to make it easier to confirm
 * when displayed in raw capture tools. */
struct content_type
{
  uint32_t tag;
  uint32_t counter;
  static constexpr size_t SPAN = sizeof(tag) + sizeof(counter);
};

class TestBeacon : public nrfcxx::sd::GenericBeacon<content_type, 0xFF> {
  using super = nrfcxx::sd::GenericBeacon<content_type, 0xFF>;
public:

  /** Beacon that carries a tag and a counter.
   *
   * @param tag a 32-bit value that appears in the first four octets
   * of the MSD payload, in big-endian form.
   *
   * @param min_s initial minimum transmission interval, in seconds
   *
   * @param max_s initial maximum transmission interval, in seconds
   */
  TestBeacon (uint32_t tag = 0,
              unsigned int min_s = 1,
              unsigned int max_s = 30)
  {
    constexpr auto UTT_Hz = nrfcxx::clock::uptime::Frequency_Hz;
    set_interval(min_s * UTT_Hz, max_s * UTT_Hz);
    content.tag = pabigot::byteorder::host_x_be(tag);
    ready = true;
  }

  uint32_t counter = 0;

private:
  int populate_ (pabigot::ble::gap::adv_data& ad) override
  {
    content.counter = pabigot::byteorder::host_x_be(++counter);
    return super::populate_(ad);
  }
};

} // ns anonymous

int
main (void)
{
  using namespace nrfcxx;
  using nrfcxx::clock::uptime;
  int rc;

  /* Start the low-frequency clock infrastructure, but not the HF
   * crystal.  Release control of the POWER_CLOCK interrupt handler:
   * the soft-device uses this. */
  board::initialize(false);
  clock::configure_pcirq(false);

  auto& active_led = led::led_type::lookup(0);
  active_led.enable();
  active_led.on();

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);
  nrfcxx::periph::UART::instance().autoenable(1);

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

  if (0 > cs.watchdogInit(WATCHDOG_DELAY_32KiHz,
                          (1 << WATCHDOG_CHANNEL_ALARM))) {
    cputs("Unable to start watchdog\n");
    delay_us(10000);
    cs.reset(cs.state().magic);
  }

  sd::Broadcaster broadcaster{cs};
  auto setter = broadcaster.make_setter(APP_EVT_TX_BEACON);
  TestBeacon test_bcn{0xbbb01eee};
  test_bcn.set_tx_notify(setter);

  rc = test_bcn.activate();
  cprintf("test_bcn activate: %d\n", rc);

  rc = broadcaster.start();
  cprintf("broadcaster start: %d\n", rc);

  /* Wake up every so often to display a summary of what's happened.
   *
   * In a real application this alarm would be used to collect sensor
   * observations.  Note that the watchdog duration is determined by
   * the frequency of this alarm, which feeds the watchdog. */
  auto set_alarm = broadcaster.make_setter(APP_EVT_ALARM);
  clock::alarm alarm{[&set_alarm](auto& alarm) {
      instr_aux.assert();
      set_alarm();
      instr_aux.deassert();
      return true;
    }};
  alarm
    .set_interval(APP_INTERVAL_s * clock::uptime::Frequency_Hz)
    .set_deadline(0)
    .schedule();

  unsigned int nsde = 0;
  while (true) {
    uptime::text_type buf;
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
      auto const ble_evt_buffer = ble_evt_buffer_union.u8;
      unsigned int err;

      /* A broadcast-only device, especially one that doesn't support
       * scan responses, will probably never see a soft-device
       * event. */
      cputs("SD event\n");
      do {
        uint32_t evt_id = 0;
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
    if (pending.test_and_clear(APP_EVT_TX_BEACON)) {
      cprintf("%s tx beacon %u\n", uptime::as_text(buf, uptime::now()),
              test_bcn.counter);
    }
    if (pending.test_and_clear(APP_EVT_ALARM)) {
      cs.watchdogFeed(WATCHDOG_CHANNEL_ALARM);
      uint64_t sleep;
      uint64_t radio;
      unsigned int scale = 1000000;
      auto total = cs.operationalModeBreakdown(sleep, radio);
      cprintf("%s main loop, %u SD events, hu %u / %u, su %u / %u\n", uptime::as_text(buf, uptime::now()), nsde,
             cs.heap_used(), cs.heap_reserved(),
             cs.stack_used(), cs.stack_reserved());
      if (0 < total) {
        static unsigned int last_wfe;
        unsigned int wfe = cs.wfe_count();
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
