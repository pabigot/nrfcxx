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
#include <cstdlib>
#include <type_traits>

namespace nrfcxx {
namespace sensor {

/** A flag value for invalid standard environmental readings.
 *
 * The value is chosen to allow its positive and negative values to be
 * stored in an `int16_t` with magnitudes that are not reasonable for
 * true observed measurements in either cCel (temperature) or pptt
 * (humidity).  Any observation with a magnitude equal to or greater
 * than this is invalid.
 *
 * Distinct values may indicate different types of failure.  For
 * example the base value may indicates a reading that was not
 * performed (perhaps because the sensor is not populated).  Assigned
 * distinct values should remain less than @ref INVALID_TAGGED_stdenv,
 * which can be used to provide an 11-bit code.
 *
 * The function is_stdenv_valid() can be used to test for validity of
 * a standard environmental measurement. */
static constexpr int INVALID_stdenv = 30000;

/** A flag value to carry an 11-bit code as an invalid standard
 * environmental reading.
 *
 * Values between 0x00 and @ref INVALID_MAX_TAG may be added to
 * this base to carry additional identifying information about the
 * reason why a specific environmental reading is not available. */
static constexpr int INVALID_TAGGED_stdenv = 30720;

/** The maximum value that can be added to @ref INVALID_TAGGED_stdenv
 * and still produce a value that is identified as an invalid standard
 * environmental reading. */
static constexpr unsigned INVALID_MAX_TAG = 0x7FF;

/** Function to test whether a standard environmental reading is valid.
 *
 * @param v a value for a standard environmental reading, such as
 * temperature in cCel or humidity in pptt.
 *
 * @return `true` iff the value is a valid reading per @ref
 * INVALID_stdenv. */
inline bool is_stdenv_valid (int v)
{
  return abs(v) < INVALID_stdenv;
}

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
