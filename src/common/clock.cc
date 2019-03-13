// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <cstdio>
#include <cstdlib>

#include <nrfcxx/clock.hpp>

namespace nrfcxx {
namespace clock {

const char*
uptime::as_text (text_type buf,
                 uint64_t dur_utt)
{
  unsigned int subsecond = dur_utt % Frequency_Hz;
  auto ldr = ldiv(dur_utt / Frequency_Hz, 60);
  unsigned int s = ldr.rem;
  ldr = ldiv(ldr.quot, 60);
  unsigned int min = ldr.rem;
  unsigned int h = ldr.quot % 100;
  snprintf(buf, sizeof(text_type), "%02u:%02u:%02u.%03u",
           h, min, s, static_cast<unsigned int>(to_ms(subsecond)));
  return buf;
}

const char*
uptime::as_day_text (day_text_type buf,
                     uint64_t dur_utt,
                     bool with_msec)
{
  unsigned int subsecond = dur_utt % Frequency_Hz;
  auto ldr = ldiv(dur_utt / Frequency_Hz, 60);
  unsigned int s = ldr.rem;
  ldr = ldiv(ldr.quot, 60);
  unsigned int min = ldr.rem;
  ldr = ldiv(ldr.quot, 24);
  unsigned int h = ldr.rem;
  unsigned int d = ldr.quot % 10000;
  auto bp = buf;
  auto const bpe = bp + sizeof(day_text_type);
  bp += snprintf(bp, bpe-buf, "%03u %02u:%02u:%02u", d, h, min, s);
  if (with_msec) {
    bp += snprintf(bp, bpe - buf, ".%03u", static_cast<unsigned int>(to_ms(subsecond)));
  }
  return buf;
}

void
clock_shift::update_sync (time_type now_ctt,
                          time_type now_utt) noexcept
{
  if (sync_ctt_) {
    diff_type den_utd = now_utt - sync_utt_;
    rate_ = RATE_DENOM;
    if (0 < den_utd) {
      diff_type den = (static_cast<diff_type>(now_ctt - sync_ctt_) * RATE_DENOM) / den_utd;
      rate_ = den;
      if (rate_ != den) {
        rate_ = RATE_DENOM;
      }
    }
  }

  /** @todo Conversion of the same local time given different
   * synchronzation data will produce different civil times, in part
   * because rate adjustments are currently discontinuous: if the
   * offset is reduced, a conversion after the synchronization point
   * may go backwards in time relative to a conversion before the
   * synchronization point.
   *
   * In theory we could avoid the possibility of non-monotonic time
   * adjustments by instead adjusting the offset as a side effect of
   * invoking convert_ctt().  However, that would only work if the
   * arguments to convert_ctt() remained monotonic.  Although that is
   * the only supported use case, adding the complexity isn't yet
   * warranted since we can force monotonicity in the uptime()
   * module's civiltime method. */
  sync_ctt_ = now_ctt;
  sync_utt_ = now_utt;
}

clock_shift::time_type
clock_shift::convert_ctt (time_type now_utt) noexcept
{
  time_type now_ctt = now_utt;
  if (sync_ctt_) {
    // Set to civil time at last sync
    now_ctt = sync_ctt_;
    // Adjust for the time difference since last sync
    auto adj = (static_cast<diff_type>(now_utt - sync_utt_) * rate_) / RATE_DENOM;
    now_ctt += adj;
  }
  return now_ctt;
}

} // namespace clock
} // namespace nrfcxx
