// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/sensor/utils.hpp>

namespace nrfcxx {
namespace sensor {

unsigned int
battery_level_pptt (unsigned int batt_mV,
                    const battery_level_point_type* curve)
{
  auto pb = curve;
  if (batt_mV >= pb->lvl_mV) {
    // Measured voltage above highest point, cap at maximum.
    return pb->lvl_pptt;
  }
  // Go down to the last point at or below the measured voltage.
  while ((0 < pb->lvl_pptt)
         && (batt_mV < pb->lvl_mV)) {
    ++pb;
  }
  if (batt_mV < pb->lvl_mV) {
    // Below lowest point, cap at minimum
    return pb->lvl_pptt;
  }
  // Linear interpolation between below and above points.
  auto pa = pb - 1;
  return pb->lvl_pptt + (pa->lvl_pptt - pb->lvl_pptt) * (batt_mV - pb->lvl_mV) / (pa->lvl_mV - pb->lvl_mV);
}

namespace dischargeCurve {

/* "Curve" here grossly estimated from in-progress tests on started
 * 2018-11-04T08:49-0600 on ddc4921dde96 contrasted with the last
 * half of long-term tests on def6fab1f697 ending around 2018-11-01,
 * overlapping in the 2.8 V to 2.6 V range to get a translation
 * factor of 144 h equals 55 d.
 *
 * This is for Amazon Basics standard batteries. */
const battery_level_point_type alkaline[] = {
  {10000, 3000}, // 3.0
  {9557, 2800},  // 7 d to 2.8
  {4620, 2500},  // 78 d to 2.5
  {887, 1950},   // 59 d to 1.95
  {0, 1750},     // 14 d to 1.75
};

/* "Curve" here eyeballed from captured e58eaf327317 data between
 * 2018-10-28T07:20-0500 and 2018-10-29T02:30-0500.  This sensor
 * started with a charge of 3.96 V and dropped about linearly to 3.58
 * V over 15 hours.  It then dropped rapidly to 3.10 V over one hour,
 * at which point it stopped transmitting.
 *
 * Based on eyeball comparisons we'll say that 15/16 of life goes
 * between 3.95 and 3.55 V, and 1/16 goes between 3.55 V and 3.1 V. */
const battery_level_point_type lipo[] = {
  {10000, 3950},
  {625, 3550},
  {0, 3100},
};

} // ns dischargeCurve

} // ns sensor
} // ns nrfcxx
