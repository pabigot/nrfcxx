// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <gtest/gtest.h>

#include <nrfcxx/clock.hpp>

namespace {

TEST(Uptime, Literal_utt)
{
  using namespace nrfcxx::clock;

  ASSERT_EQ(32768_utt, uptime::duration_type{32768});
}

TEST(Uptime, as_text)
{
  using namespace nrfcxx::clock;

  uptime::text_type buf;
  ASSERT_STREQ(uptime::as_text(buf, 0), "00:00:00.000");
  ASSERT_STREQ(uptime::as_text(buf, uptime::Frequency_Hz / 2), "00:00:00.500");
  ASSERT_STREQ(uptime::as_text(buf, (999 + 543 * uptime::Frequency_Hz) / 1000 + uptime::Frequency_Hz * (56 + 60 * (57 + 60 * 98ULL))),
               "98:57:56.543");
  ASSERT_STREQ(uptime::as_text(buf, (999 + 543 * uptime::Frequency_Hz) / 1000 + uptime::Frequency_Hz * (56 + 60 * (57 + 60 * 101ULL))),
               "01:57:56.543");
}

TEST(Uptime, as_day_text)
{
  using nrfcxx::clock::uptime;
  constexpr uint64_t D_utt = 24 * 60 * 60 * uptime::Frequency_Hz;
  constexpr uint64_t db = uptime::from_ms(790 + 1000 * (56 + 60 * (34 + 60 * 12)));

  uptime::day_text_type buf;
  ASSERT_STREQ(uptime::as_day_text(buf, 0), "000 00:00:00");
  ASSERT_STREQ(uptime::as_day_text(buf, db), "000 12:34:56");
  ASSERT_STREQ(uptime::as_day_text(buf, 0, true), "000 00:00:00.000");
  ASSERT_STREQ(uptime::as_day_text(buf, db, true), "000 12:34:56.789");
  ASSERT_STREQ(uptime::as_day_text(buf, 123 * D_utt + db, true), "123 12:34:56.789");
  ASSERT_STREQ(uptime::as_day_text(buf, 1234 * D_utt + db, true), "1234 12:34:56.789");
  ASSERT_STREQ(uptime::as_day_text(buf, 12345 * D_utt + db, true), "2345 12:34:56.789");
}

TEST(Uptime, delta24)
{
  using namespace nrfcxx::clock;

  const unsigned int max24 = (1U << 24) - 1;

  ASSERT_EQ(uptime::delta24(0, 1), 1U);
  ASSERT_EQ(uptime::delta24(1, 0), max24);
  ASSERT_EQ(uptime::delta24(max24, 0), 1U);
  ASSERT_EQ(uptime::delta24(0, max24), max24);
}

TEST(ClockShift, Basics)
{
  using nrfcxx::clock::clock_shift;
  clock_shift::time_type utt;
  clock_shift::time_type ctt;
  unsigned int offset {10000};
  unsigned int step {1U << 20};

  clock_shift ct;

  /* Initial conversion is unit rate, epoch civil time */
  utt = offset + 12345;
  ASSERT_EQ(utt, ct.convert_ctt(utt));
  ASSERT_EQ(ct.RATE_DENOM, ct.rate());
  ASSERT_EQ(0U, ct.offset());

  /* One sample is unit rate, offset civil time */
  ctt = 123456789;
  ct.update_sync(ctt, utt);
  ASSERT_EQ(ct.RATE_DENOM, ct.rate());
  ASSERT_EQ(ctt, ct.convert_ctt(utt));
  ASSERT_EQ(ctt - utt, ct.offset());

  /* Update with both clocks in sync */
  utt += step;
  ctt += step;

  ct.update_sync(ctt, utt);
  ASSERT_EQ(ct.RATE_DENOM, ct.rate());
  ASSERT_EQ(ctt, ct.convert_ctt(utt));
  ASSERT_EQ(ctt + offset, ct.convert_ctt(utt + offset));

  /* Update with civil time faster by 1/16 */
  utt += step;
  ctt += step + (step / 16);
  ct.update_sync(ctt, utt);
  ASSERT_EQ((17 * ct.RATE_DENOM) / 16, ct.rate());
  ASSERT_EQ(ctt + 17 * offset, ct.convert_ctt(utt + 16 * offset));

  /* Update with civil time slower by 1/16 */
  utt += step;
  ctt += step - (step / 16);
  ct.update_sync(ctt, utt);
  ASSERT_EQ((15 * ct.RATE_DENOM) / 16, ct.rate());
  ASSERT_EQ(ctt + 15 * offset, ct.convert_ctt(utt + 16 * offset));
}

} // ns anonymous
