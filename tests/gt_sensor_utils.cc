// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <gtest/gtest.h>
#include <nrfcxx/sensor/utils.hpp>

namespace {

TEST(SensorUtils, TempConv)
{
  using namespace nrfcxx::sensor;

  ASSERT_EQ(0, temperature_cK_cCel(27315));
  ASSERT_EQ(1234, temperature_cK_cCel(27315 + 1234));
  ASSERT_EQ(-1234, temperature_cK_cCel(27315 - 1234));
}

} // ns anonymous
