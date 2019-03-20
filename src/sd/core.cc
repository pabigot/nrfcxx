// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Peter A. Bigot

#include <nrfcxx/core.hpp>
#include <nrfcxx/periph.hpp>
#include "nrf_soc.h"

namespace nrfcxx {

/* Override weak definition in nrfcxx core. */
int
systemState::die_temperature_ ()
{
  if (!systemState::softdevice_is_enabled()) {
    return periph::TEMP::temperature();
  }
  int32_t temp;
  auto err = sd_temp_get(&temp);
  if (NRF_SUCCESS == err) {
    return temp;
  }
  return 25 * 4;
}

} // namespace nrfcxx
