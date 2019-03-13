// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/periph.hpp>

#define NUM_CHANNELS nrfcxx::nrf5::PPI.AUX
#define NUM_GROUPS nrfcxx::nrf5::PPI_Type::NUM_GROUPS

#define RESERVED_CHANNELS 0
#define AVAILABLE_CHANNELS (((1U << NUM_CHANNELS) - 1) & (~ RESERVED_CHANNELS))
#define RESERVED_GROUPS 0
#define AVAILABLE_GROUPS (((1U << NUM_GROUPS) - 1) & (~ RESERVED_GROUPS))

namespace nrfcxx {
namespace periph {

using channel_set_type = nrf5::PPI_Type::channel_set_type;
using group_set_type = nrf5::PPI_Type::group_set_type;

namespace {

channel_set_type channels{AVAILABLE_CHANNELS};
group_set_type groups{AVAILABLE_GROUPS};

} // anonymous

int
PPI::request ()
{
  int idx = 0;
  channel_set_type bit = 1;
  while (bit && !(channels & bit)) {
    bit <<= 1;
    ++idx;
  }
  if (channels & bit) {
    channels &= ~bit;
    return idx;
  }
  return -1;
}

int
PPI::group_request ()
{
  int idx = 0;
  group_set_type bit = 1;
  while (bit && !(groups & bit)) {
    bit <<= 1;
    ++idx;
  }
  if (groups & bit) {
    groups &= ~bit;
    return idx;
  }
  return -1;
}

int
PPI::release (int ppidx)
{
  int rv = -1;
  if ((0 <= ppidx)
      && (ppidx < 32)) {
    channel_set_type bit = (1U << ppidx);
    if (bit & AVAILABLE_CHANNELS) {
      channels |= bit;
      rv = 0;
    }
  }
  return rv;
}

int
PPI::group_release (int gidx)
{
  int rv = -1;
  if ((0 <= gidx)
      && (gidx < 32)) {
    group_set_type bit = (1U << gidx);
    if (bit & AVAILABLE_GROUPS) {
      groups |= bit;
      rv = 0;
    }
  }
  return rv;
}

} // namespace periph
} // namespace nrfcxx
