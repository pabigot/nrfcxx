// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Evaluation of SAADC oversampling behavior with one-shot and
 * scan-mode collections.
 *
 * Evidence suggests that CONFIG.BURST is not reliable when
 * interleaving one-shot and scan-mode collections with a non-zero
 * OVERSAMPLE.  This application demonstrates and tests the new API
 * used to configure resolution, oversampling, and handling of
 * required additional SAMPLE triggers when oversampling is used.
 *
 * See: https://devzone.nordicsemi.com/f/nordic-q-a/45339/saadc-burst-problems-in-scan-vs-non-scan-acquisitions
 */

/* Set both to 1 (one-shot), both to 2 (scan), or min 1 max 2.  The
 * latter demonstrates the failure with CONFIG.BURST in the second
 * scan. */
#define MIN_CHANCNT 1
#define MAX_CHANCNT 2

#include <climits>
#include <cmath>
#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/misc/regulator.hpp>
#include <nrfcxx/sensor/adc.hpp>

namespace {

#define EVT_ALARM 0x01
#define EVT_PROCESS 0x02
nrfcxx::event_set events;

auto& SAADC = *nrfcxx::nrf5::SAADC.instance();

void
dump_adc ()
{
  using namespace nrfcxx;

  printf("ADC: ST %ld END %ld DONE %ld RDONE %ld CD %ld STOP %ld\n",
         SAADC.EVENTS_STARTED, SAADC.EVENTS_END,
         SAADC.EVENTS_DONE, SAADC.EVENTS_RESULTDONE,
         SAADC.EVENTS_CALIBRATEDONE, SAADC.EVENTS_STOPPED);
  printf("INTEN %lx STAT %lx ENA %lx\n",
         SAADC.INTEN, SAADC.STATUS, SAADC.ENABLE);
  printf("OVERSAMPLE %lu\n", SAADC.OVERSAMPLE);
  printf("RESULT %lu of %lu\n", SAADC.RESULT.AMOUNT, SAADC.RESULT.MAXCNT);
  for (unsigned int ci = 0; ci < nrf5::SAADC.AUX; ++ci) {
    auto& ch = SAADC.CH[ci];
    printf("%u: %08lx %08lx %08lx\n", ci, ch.PSELP, ch.PSELN, ch.CONFIG);
  }
}

class client_type : public nrfcxx::periph::ADCClient
{
  using super = nrfcxx::periph::ADCClient;

  bool multi_;
  uint8_t nresults_;
  uint16_t volatile results_[8];

  int configure_bi_ () override
  {
    using namespace nrfcxx;

    unsigned int ain = 1;
    nresults_ = 1;
    if (multi_) {
      ain = 4;
      nresults_ = 2;
    }

    auto config = peripheral::make_config(SAADC_CH_CONFIG_REFSEL_VDD1_4,
                                          SAADC_CH_CONFIG_GAIN_Gain1_4,
                                          SAADC_CH_CONFIG_TACQ_40us);
    config |= configure_resolution_bi_();
    for (auto ci = 0U; ci < nrf5::SAADC.AUX; ++ci) {
      results_[ci] = 0;
      nrf5::SAADC->CH[ci].CONFIG = config;
      if (ci < nresults_) {
        nrf5::SAADC->CH[ci].PSELP = (SAADC_CH_PSELP_PSELP_AnalogInput0 + ain + ci) << SAADC_CH_PSELP_PSELP_Pos;
      } else {
        nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
      }
      nrf5::SAADC->CH[ci].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
    }
    peripheral::setup_result_bi(results_, nresults_);
    return 0;
 }

public:
  client_type () = default;

  unsigned int nresults () const
  {
    return nresults_;
  }

  const int16_t* results () const
  {
    return reinterpret_cast<int16_t *>(const_cast<uint16_t *>(results_));
  }

  int sample (nrfcxx::notifier_type notify,
              bool multi)
  {
    multi_ = multi;
    return super::sample(notify);
  }
};

} // anonymous namespace

extern "C" {
void ADCSeriesVariant_IRQHandler (void)
{
  nrfcxx::periph::ADC::irq_handler();
}

void GPIOTE_IRQHandler (void) {
  nrfcxx::periph::GPIOTE::irq_handler();
}
} // extern "C"

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  using clock::uptime;

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto alarm = clock::alarm::for_event<EVT_ALARM, true>(events);
  alarm
    .set_interval(1 * uptime::Frequency_Hz)
    .set_deadline(alarm.interval())
    .schedule();

  auto client = client_type{};
  if (false) {
    // 2x oversampling, 14-bit, CONFIG.BURST.
    // This will fail for the order scan, one-shot, scan.
    // It will work with scan*, and one-shot*
    client.resolution(SAADC_RESOLUTION_VAL_14bit,
                      2,
                      true);
  } else if (false) {
    // 2x oversampling, 12-bit, manual.
    // This will fail because nothing supplies the second SAMPLE.
    client.resolution(SAADC_RESOLUTION_VAL_12bit,
                      2,
                      false);
  } else {
    // no oversampling, no burst, default resolution.
    // This will work.
  }
  printf("Configured for %lu oversample, %lu resolution, %u burst support\n",
         client.oversample(), client.resolution(), client.burst());

  events.set(EVT_ALARM);

  unsigned int ctr = MIN_CHANCNT;
  int rc = client.claim();
  bool ready = true;
  while (0 <= rc) {
    uptime::text_type buf;
    event_set::cev();

    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      if (pending.test_and_clear(EVT_PROCESS)) {
        static unsigned int reps;
        printf("Completed %lu of %lu = %u:", SAADC.RESULT.AMOUNT, SAADC.RESULT.MAXCNT, client.nresults());
        for (auto ci = 0U; ci < 8; ++ci) {
          printf(" %d", client.results()[ci]);
        }
        putchar('\n');
        /* Display the SAADC config for the first two collections. */
        if (2 > reps++) {
          dump_adc();
        }
        ready = true;
      }
      if (pending.test_and_clear(EVT_ALARM)) {
        if (!ready) {
          printf("Sample failed to complete: %lu of %lu\n",
                 SAADC.RESULT.AMOUNT, SAADC.RESULT.MAXCNT);
          for (auto ci = 0U; ci < 8; ++ci) {
            printf(" %d", client.results()[ci]);
          }
          putchar('\n');
          dump_adc();
          alarm.cancel();
        }
        rc = client.sample(events.make_setter(EVT_PROCESS), !(ctr & 1));
        if (0 != rc) {
          printf("%s: sample %u (2^%lu res %lu burst %u) got %d\n", uptime::as_text(buf, uptime::now()),
                 client.nresults(), client.oversample(), client.resolution(), client.burst(),
                 rc);
        }
        if (MAX_CHANCNT < ++ctr) {
          ctr = MIN_CHANCNT;
        }
        ready = false;
      }
    }
    systemState::WFE();
  }
  printf("Exit rc %d\n", rc);
}
