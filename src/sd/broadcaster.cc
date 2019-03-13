// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/sd/broadcaster.hpp>

#include "ble.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include <nrfcxx/console/null.hpp>

/* Optional override to prevent sleeping between events. */
#define MAX_POWER 0

#define INSTR_PSEL_ACTIVE NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_RADIO NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn

static inline
uint32_t
data_start ()
{
  extern unsigned int __data_start__;
  return reinterpret_cast<uintptr_t>(&__data_start__);
}

namespace {

constexpr unsigned int sd_stack_add_words =
#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
  /* S130 version 2.0 maximum usage is 1536 bytes (384 words).  We
   * want to keep stack sizes to multiples of 1 KiBy to match
   * ApplicationIdBeacon, so round up to 2 KiBy. */
  512
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
  /* S132 version 6.1 maximum usage with FPU disabled would be 1536
   * bytes (384 words).  FPU adds 16 words per interrupt priority.
   * We'll round up to 512 words. */
  512
#else /* NRFCXX_SOFTDEVICE_IS_ */
#endif /* NRFCXX_SOFTDEVICE_IS_ */
  ;

/* Extend stack space to account for SD use */
__attribute__((__section__(".stack.extension")))
volatile uint32_t sd_stack_addition[sd_stack_add_words];

using namespace nrfcxx;

/** Internal event indicating that a telemetry beacon prepare backoff
 * has been reached. */
constexpr event_set::event_type IEVT_TLM_PREPARE = 0x0001;

/** Internal event indicating that beacon transmission requires
 * service. */
constexpr event_set::event_type IEVT_BEACON =      0x0002;

/** Internal event indicating that radio has been turned off. */
constexpr event_set::event_type IEVT_RADIO_OFF   = 0x0004;

gpio::instr_psel<INSTR_PSEL_ACTIVE> instr_active;
gpio::instr_psel<INSTR_PSEL_RADIO> instr_radio;
gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;

event_set events;

sd::Broadcaster * broadcaster;

} // anonymous

extern "C" {

void SD_EVT_IRQHandler()
{
  events.set(nrfcxx::sd::Broadcaster::EVT_SD);
}

void RADIO_NOTIFICATION_IRQHandler()
{
  using nrfcxx::systemState;
  using nrfcxx::sd::Broadcaster;

  static bool radio_on;
  radio_on = !radio_on;
  instr_radio.set(radio_on);

  if (radio_on) {
    systemState::updateOperationalMode(0, systemState::OM_RADIO);
    events.set(Broadcaster::EVT_RADIO_ON);
  } else {
    systemState::updateOperationalMode(systemState::OM_RADIO, 0);
    events.set(IEVT_RADIO_OFF | Broadcaster::EVT_RADIO_OFF);
  }
}
} // extern "C"

namespace nrfcxx {
namespace sd {

namespace {

} // anonymous

void
Broadcaster::state_setup (const systemState::state_type& ss,
                          bool is_reset,
                          bool retained)
{
  nrfcxx::sd::Beacon::telemetry_state_setup(ss, is_reset, retained);
}

Broadcaster::Broadcaster (const systemState& system_state) :
  sys_beacon{system_state},
  tlm_beacon{system_state},
  vdd_sensor{vdd_callback}
{
  if (broadcaster) {
    failsafe(FailSafeCode::RESOURCE_VIOLATION);
  }
  broadcaster = this;

  Beacon::set_notify(events.make_setter(IEVT_BEACON));
  tlm_beacon.set_prepare_backoff(events.make_setter(IEVT_TLM_PREPARE), tlm_beacon.min_interval_utt() / 2);

  instr_active.enable();
  instr_active.assert();
  instr_radio.enable();
  instr_aux.enable();
}

Broadcaster::~Broadcaster ()
{
  stop();
}

notifier_type
Broadcaster::make_setter (event_set::event_type bits) const
{
  if (bits & (EVT_APP_BASE - 1)) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  return events.make_setter(bits);
}

int
Broadcaster::start ()
{
  unsigned int err = 0;
  int rc = 0;

  uint32_t initial_ram_start = data_start();
  auto ram_start = initial_ram_start;
  nrf_clock_lf_cfg_t lfcfg = {
    .source = board::default_lfclk_src(),
    .rc_ctiv = board::has_lfxt ? 0 : 16,
    .rc_temp_ctiv = board::has_lfxt ? 0 : 2,
#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_30_PPM,
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
    .accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM,
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
  };

  ram_delta_ = 0;
  addr_ = {};

  do {

    /* Initialize the soft device.
     *
     * This tells the device what clocks are available, and transfers
     * control of all reserved and block resources to the soft device.
     * The operation will fail if resources are in use in an
     * incompatible way, including interrupt priority for peripherals
     * not used by the SD. */
    err = sd_softdevice_enable(&lfcfg, systemState::sd_fault_handler);
    if (NRF_SUCCESS != err) {
      break;
    }

    /* Set low power sleep. */
    err = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    if (NRF_SUCCESS != err) {
      break;
    }

    /* Initialize radio notifications.
     *
     * We want these primarily to track the operational mode, and for
     * instrumentation during development.
     */
    err = sd_radio_notification_cfg_set(NRF_RADIO_NOTIFICATION_TYPE_INT_ON_BOTH,
                                        NRF_RADIO_NOTIFICATION_DISTANCE_800US);
    if (NRF_SUCCESS != err) {
      break;
    }

    /* Allow soft device events to be reported to the application main
     * loop. */
    NVIC_EnableIRQ(SD_EVT_IRQn);
    NVIC_EnableIRQ(RADIO_NOTIFICATION_IRQn);

    /* Initialize Bluetooth Low Energy support in the soft device. */
#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
    ble_enable_params_t prm{};
    err = sd_ble_enable(&prm, &ram_start);
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
    err = sd_ble_enable(&ram_start);
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
    ram_delta_ = initial_ram_start - ram_start;
    if (NRF_SUCCESS != err) {
      break;
    }

#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
    err = sd_ble_gap_address_get(&addr_);
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
    err = sd_ble_gap_addr_get(&addr_);
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
    if (NRF_SUCCESS != err) {
      break;
    }

    err = sd_ble_version_get(&fwid_);
    if (NRF_SUCCESS != err) {
      break;
    }

    // Provide the firmware ID to the ApplicationId beacon.
    appid_beacon.prepare(sys_beacon.system_state, fwid_);
  } while (false);

  if ((NRF_SUCCESS != err)
      || (0 != rc)) {
    stop();
  }

  return (NRF_SUCCESS != err) ? err : rc;
}

void
Broadcaster::stop ()
{
  appid_beacon.deactivate();
  tlm_beacon.deactivate();
  sys_beacon.deactivate();
  NVIC_DisableIRQ(RADIO_NOTIFICATION_IRQn);
  NVIC_DisableIRQ(SD_EVT_IRQn);
  sd_radio_notification_cfg_set(NRF_RADIO_NOTIFICATION_TYPE_NONE,
                                NRF_RADIO_NOTIFICATION_DISTANCE_800US);
  sd_softdevice_disable();
}

event_set_copy
Broadcaster::wait_for_event ()
{
#if !(MAX_POWER - 0)
  if (!events) {
    instr_active.deassert();
    while (!events) {
      auto sleeper = systemState::make_scoped_sleeper();
      sd_app_evt_wait();
    }
    instr_active.assert();
  }
#endif // MAX_POWER
  event_set::cev();
  auto pending = events.copy_and_clear();
  if (pending.test_and_clear(IEVT_TLM_PREPARE)) {
    pending.set(EVT_VDD_REQUIRED);
  }
  if (pending.test_and_clear(IEVT_BEACON)) {
    int rc = sd::Beacon::process_event();
    cprintf("BCST BEACON %p event %d\n", sd::Beacon::active(), rc);
  }
  if (pending.test_and_clear(IEVT_RADIO_OFF)) {
    int rc = sd::Beacon::process_completion();
    cprintf("BCST radio off, bcn %d\n", rc);
  }
  cprintf("BCST return %x\n", pending.events());
  return pending;
}

void
Broadcaster::vdd_callback (uint16_t vdd_mV)
{
  broadcaster->vdd_callback_(vdd_mV);
}

void
Broadcaster::vdd_callback_ (uint16_t vdd_mV)
{
  vdd_mV_ = vdd_mV;
  if (0 != vdd_mV) {
    tlm_beacon.update_pwr_mV(vdd_mV, true);
    events.set(EVT_VDD_UPDATED);
  }
}

} // namespace sd
} // namespace nrfcxx
