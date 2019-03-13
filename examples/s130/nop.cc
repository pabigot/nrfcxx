// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Basic test and validation framework for nRF51 S130 soft device. */

#include <cstdio>
#include <cinttypes>

#include <nrfcxx/impl.hpp>
#include <nrfcxx/clock.hpp>

#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "ble.h"

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

namespace {
/* S130 version 2.0 maximum usage is 1536 bytes (384 words).  We
 * want to keep stack sizes to multiples of 1 KiBy to match
 * ApplicationIdBeacon, so round up to 2 KiBy. */
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

int
main (void)
{
  using namespace nrfcxx;

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
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_30_PPM,
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

  cprintf("Proposed RAM start %lx\n", ram_start);
  {
    ble_enable_params_t prm{};

    prm.common_enable_params.vs_uuid_count = 1;
    prm.gap_enable_params = {
      .periph_conn_count = 1,
      .central_conn_count = 4,
      .central_sec_count = 1,
    };
    err = sd_ble_enable(&prm, &ram_start);
  }
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
