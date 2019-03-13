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

} // ns sensor
} // ns nrfcxx
