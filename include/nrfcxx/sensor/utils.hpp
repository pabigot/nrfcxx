/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Functions and data structures for unit conversion and other
 * sensor-related operations.
 *
 * @file */

#ifndef NRFCXX_SENSOR_UTILS_HPP
#define NRFCXX_SENSOR_UTILS_HPP

#include <array>
#include <cinttypes>
#include <type_traits>

namespace nrfcxx {
namespace sensor {

/** Convert a temperature from cK to cCel
 *
 * The core sensor interfaces return temperatures in scaled Kelvin
 * because we know those values will always be non-negative and so can
 * use a negative value to indicate a sensor error.  Nonetheless few
 * sensor applications are going to want to deal with Kelvin, and
 * applications shouldn't have to remember that 273.15 Cel is 0 K.
 * This does the math to convert the common scaled temperature value.
 *
 * @param t_cK a non-negative temperature measured in hundredths Kelvin.
 *
 * @return the same temperature measured in hundredths Celsius. */
inline int temperature_cK_cCel (int t_cK)
{
  return t_cK - 27315;
}

/** A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence. */
struct battery_level_point_type
{
  /** Remaining life at #lvl_mV. */
  uint16_t lvl_pptt;

  /** Battery voltage at #lvl_pptt remaining life. */
  uint16_t lvl_mV;
};

/** Calculate the estimated battery level based on a measured voltage.
 *
 * @param batt_mV a measured battery voltage level.
 *
 * @param curve the discharge curve for the type of battery installed on the system.
 *
 * @return the estimated remaining capacity in parts per ten thousand. */
unsigned int battery_level_pptt (unsigned int batt_mV,
                                 const battery_level_point_type* curve);

} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_UTILS_HPP */
