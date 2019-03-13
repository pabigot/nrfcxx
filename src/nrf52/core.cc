// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/impl.hpp>

/* If you're looking for nrfcxx::series::delay_cycles it's in
 * delay_cycles.S. */

namespace nrfcxx {
namespace nrf5 {
namespace series {

void
enable_pinreset ()
{
  using nrfcxx::nrf5::NVMC;
  using nrfcxx::nrf5::UICR;
  constexpr unsigned int psel =
#if (NRF52832 - 0)
    21
#elif (NRF52840 - 0)
    18
#else
#error Unsupported NRF52 MCU
#endif
    ;

  NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  while (!NVMC->READY);
  UICR->PSELRESET[0] = psel;
  while (!NVMC->READY);
  UICR->PSELRESET[1] = psel;
  while (!NVMC->READY);
  NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  while (!NVMC->READY);
}

bool SAADC_Peripheral::calibrating_bi_;

} // ns series
} // ns nrf5
} // ns nrfcxx
