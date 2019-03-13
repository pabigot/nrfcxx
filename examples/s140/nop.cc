// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Basic test and validation framework for nRF52840 S132 soft device. */

#include <cinttypes>
#include <cstdio>

#include <nrfcxx/impl.hpp>
#include <nrfcxx/clock.hpp>

#include "nrf_sdm.h"
#include "ble.h"

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

namespace {
/* S140 version 6.1.1 maximum memory usage is 0x1628 bytes (1418
 * words).  This soft device has never specified maximum usage of a
 * stack shared with the application.  The last recorded maximum stack
 * usage was for S132 at 5.0.0-3.alpha at 0x05f4 bytes (381 words).
 * We'll assume 512 words is enough. */
__attribute__((__section__(".stack.extension")))
volatile uint32_t sd_stack_addition[512];
} // ns anonymous

static inline
uint32_t
data_start ()
{
  extern unsigned int __data_start__;
  return reinterpret_cast<uintptr_t>(&__data_start__);
}

namespace {
} // ns anonymous

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

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  uint8_t sde = 0;
  unsigned int err = sd_softdevice_is_enabled(&sde);
  cprintf("check enabled: %u with err %x\n", sde, err);

  ble_version_t ver;
  err = sd_ble_version_get(&ver);
  if (NRF_ERROR_SOFTDEVICE_NOT_ENABLED != err) {
    cprintf("sd_ble_version_get unexpected err %x\n", err);
    return -1;
  }
  cputs("Got expected not-enabled error");

  uint32_t initial_ram_start = data_start();
  uint32_t ram_start = initial_ram_start;

  nrf_clock_lf_cfg_t lfcfg = {
    .source = board::default_lfclk_src(),
    .rc_ctiv = board::has_lfxt ? 0 : 16,
    .rc_temp_ctiv = board::has_lfxt ? 0 : 2,
    .accuracy = NRF_CLOCK_LF_ACCURACY_30_PPM,
  };

  /* Initialize the soft device.
   *
   * This tells the device what clocks are available, and transfers
   * control of all reserved and block resources to the soft device.
   * The operation will fail if resources are in use in an
   * incompatible way, including interrupt priority for peripherals
   * not used by the SD. */
  err = sd_softdevice_enable(&lfcfg, systemState::sd_fault_handler);
  if (NRF_SUCCESS != err) {
    cprintf("sd_enable unexpected %x\n", err);
    return -2;
  }

  {
    ble_cfg_t cfg{};
    cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 1;
    err = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &cfg, ram_start);
    cprintf("uuid count cfg_set got %x\n", err);

    cfg = {};
    cfg.gap_cfg.role_count_cfg.periph_role_count = 1;
    cfg.gap_cfg.role_count_cfg.central_role_count = 4;
    cfg.gap_cfg.role_count_cfg.central_sec_count = 1;
    err = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &cfg, ram_start);
    cprintf("role count cfg_set got %x\n", err);

#if 0
    // Configure total number of connections.
    cfg = {};
    cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_IPSP_MAX_CHANNELS;
    cfg.conn_cfg.params.gap_conn_cfg.event_length = BLE_GAP_EVENT_LENGTH_DEFAULT;
    err = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &cfg, ram_start);
    cprintf("conn cfg_set got %x\n", err);
#endif
  }

  cprintf("Proposed RAM start %lx\n", ram_start);
  err = sd_ble_enable(&ram_start);
  cprintf("ble_enable got %x, want ram %lx, diff %d\n", err, ram_start, static_cast<int>(initial_ram_start - ram_start));
  if (NRF_SUCCESS != err) {
    return -3;
  }

  err = sd_ble_version_get(&ver);
  cprintf("ble_version_get got %x\n", err);
  if (NRF_SUCCESS != err) {
    return -4;
  }
  cprintf("CID %04x, version %u %04x\n", ver.company_id, ver.version_number, ver.subversion_number);

  while (true) {
    auto sleeper = systemState::make_scoped_sleeper();
    err = sd_app_evt_wait();
    if (NRF_SUCCESS != err) {
      cprintf("AEW err %x\n", err);
    }
  }

  return 0;
}
