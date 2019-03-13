// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Measure time required to complete SAADC calibration with various
 * configurations.
 *
 * There's nothing in the nRF52832 Product Specification that
 * indicates calibration is configuration-specific, but [this
 * page](https://github.com/NordicPlayground/nRF52-ADC-examples/blob/master/saadc_low_power/README.md)
 * provides a table of acquisition-time--specific calibration delays.
 *
 * This sample is used to determine whether SAADC calibration time is
 * dependent on TACQ.  Specifically, it determines whether we should
 * be setting up the ADC in the configuration desired before
 * initiating calibration.
 *
 * Tests with this confirm that the configured acquisition time, and
 * the OVERSAMPLE setting, affect calibration.  Runs with OVERSAMPLE=0
 * show:
 *
 *     0 00020000: 1276 tcks, 79 us
 *     1 01020000: 1276 tcks, 79 us
 *     2 00000000: 604 tcks, 37 us
 *     3 01000000: 602 tcks, 37 us
 *     4 00010000: 794 tcks, 49 us
 *     5 00020000: 1274 tcks, 79 us
 *     6 00030000: 1756 tcks, 109 us
 *     7 00040000: 2236 tcks, 139 us
 *     8 00050000: 4156 tcks, 259 us
 *     9 01050000: 4156 tcks, 259 us
 *
 * This shows BURST does not affect calibration time.  Further testing
 * shows that oversampling does affect calibration time, and that
 * calibration is performed with the configuration for SAADC->CH[0]
 * only.  The results for a 4x oversample single-channel are:
 *
 *     0 00020000: 4351 tcks, 271 us
 *     1 01020000: 4317 tcks, 269 us
 *     2 00000000: 1828 tcks, 114 us
 *     3 01000000: 1741 tcks, 108 us
 *     4 00010000: 2594 tcks, 162 us
 *     5 00020000: 4334 tcks, 270 us
 *     6 00030000: 6174 tcks, 385 us
 *     7 00040000: 8014 tcks, 500 us
 *     8 00050000: 15374 tcks, 960 us
 *     9 01050000: 15357 tcks, 959 us
 *
 * which fairly closely match the example documentation referenced
 * above.
 */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/periph.hpp>

#define CFG_OVERSAMPLE 2
#define CFG_TWOCHANNEL 0
#define CFG_CHBASE 0

#define EVT_RUN       0x0001
#define EVT_CALIBRATE 0x0002
#define EVT_CALDONE   0x0004

namespace {
nrfcxx::gpio::instr_psel<NRFCXX_BOARD_PSEL_SCOPE0> instr0;
nrfcxx::event_set events;
auto& hrt = nrfcxx::periph::TIMER::instance(0);
volatile unsigned int ts1;
} // ns anonymous

extern "C" {
#if !(NRF51 - 0)
void SAADC_IRQHandler ()
{
  using namespace nrfcxx;

  if (nrf5::SAADC->EVENTS_CALIBRATEDONE) {
    ts1 = hrt.counter();
    instr0.deassert();
    events.set(EVT_CALDONE);
    nrf5::SAADC->EVENTS_CALIBRATEDONE = 0;
  }
}
#endif /* NRF51 */

} // extern "C"

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  puts("\n" __FILE__ " " __DATE__ " " __TIME__);
#if (NRF51 - 0)
  puts("Application requires nRF52 SAADC peripheral");
  return -1;
#else
  using SAADC = nrfcxx::nrf5::series::SAADC_Peripheral;
  static int configs[] = {
    -1,
    SAADC::make_config(),
    SAADC_CH_CONFIG_TACQ_3us << SAADC_CH_CONFIG_TACQ_Pos,
    SAADC::make_config(SAADC_CH_CONFIG_REFSEL_Internal,
                       SAADC_CH_CONFIG_GAIN_Gain1_6,
                       SAADC_CH_CONFIG_TACQ_3us),
    SAADC_CH_CONFIG_TACQ_5us << SAADC_CH_CONFIG_TACQ_Pos,
    SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos,
    SAADC_CH_CONFIG_TACQ_15us << SAADC_CH_CONFIG_TACQ_Pos,
    SAADC_CH_CONFIG_TACQ_20us << SAADC_CH_CONFIG_TACQ_Pos,
    SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos,
    SAADC::make_config(SAADC_CH_CONFIG_REFSEL_Internal,
                       SAADC_CH_CONFIG_GAIN_Gain1_6,
                       SAADC_CH_CONFIG_TACQ_40us),
  };
  constexpr auto nconfigs = sizeof(configs)/sizeof(*configs);

  instr0.enable();
  hrt.configure(clock::hfclk::Frequency_Hz, TIMER_BITMODE_BITMODE_32Bit);
  hrt.start();
  printf("Timer at %u Hz\n", hrt.frequency_Hz());

  printf("OVERSAMPLE= %u\n", CFG_OVERSAMPLE);
#if (CFG_TWOCHANNEL - 0)
  printf("Two channels starting at CH%u\n", CFG_CHBASE);
#else
  printf("One channel starting at CH%u\n", CFG_CHBASE);
#endif
  nrf5::SAADC->OVERSAMPLE = CFG_OVERSAMPLE;
  for (auto chi = 0U; chi < nrf5::SAADC.AUX; ++chi) {
    auto chp = nrf5::SAADC->CH + chi;
    chp->CONFIG = {};
    chp->PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
    chp->PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
  }

  nrf5::SAADC->EVENTS_STARTED = 0;
  nrf5::SAADC->EVENTS_END = 0;
  nrf5::SAADC->EVENTS_DONE = 0;
  nrf5::SAADC->EVENTS_STOPPED = 0;
  nrf5::SAADC->EVENTS_CALIBRATEDONE = 0;
  nrf5::SAADC->INTENSET = (SAADC_INTENSET_CALIBRATEDONE_Set << SAADC_INTENSET_CALIBRATEDONE_Pos);
  NVIC_EnableIRQ(SAADC_IRQn);
  nrf5::SAADC->ENABLE = 1;

  events.set(EVT_RUN);
  unsigned int cfi = 0;
  unsigned int ts0 = 0;
  while (cfi < nconfigs) {
    event_set::cev();
    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      if (pending.test_and_clear(EVT_RUN)) {
        auto chp = nrf5::SAADC->CH + CFG_CHBASE;
        auto cfg = configs[cfi];
        if (0 <= cfg) {
          chp->CONFIG = cfg;
          chp->PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0 << SAADC_CH_PSELP_PSELP_Pos;
#if (CFG_TWOCHANNEL - 0)
          ++chp;
          chp->CONFIG = cfg;
          chp->PSELP = SAADC_CH_PSELP_PSELP_AnalogInput1 << SAADC_CH_PSELP_PSELP_Pos;
#endif /* CFG_TWOCHANNEL */
        } else {
          chp->CONFIG = SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos;
          chp->PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
#if (CFG_TWOCHANNEL - 0)
          ++chp;
          chp->CONFIG = SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos;
          chp->PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
#endif /* CFG_TWOCHANNEL */
        }
        pending.set(EVT_CALIBRATE);
      }
      if (pending.test_and_clear(EVT_CALIBRATE)) {
        instr0.assert();
        ts0 = hrt.counter();
        nrf5::SAADC->TASKS_CALIBRATEOFFSET = 1;
      }
      if (pending.test_and_clear(EVT_CALDONE)) {
        auto chp = nrf5::SAADC->CH + CFG_CHBASE;
        unsigned int td = ts1 - ts0;
        unsigned int td_us = (uint64_t{1'000'000} * td) / hrt.frequency_Hz();
        printf("%u %08" PRIx32 ": %d tcks, %u us\n", cfi, chp->CONFIG, td, td_us);
        sleep_ms(10);
        if (++cfi < nconfigs) {
          events.set(EVT_RUN);
        }
      }
    }
    if (!events) {
      systemState::WFE();
    }
  }
  puts("Runs complete\n");

  return 0;
#endif /* NRF51 */
}
