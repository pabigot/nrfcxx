// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Display memory contents. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/utility.hpp>
#include <cstring>
#include <cstdio>

int
main (void)
{
  using namespace nrfcxx::utility;

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

#if (NRF51 - 0)
  puts("Original NRF_1MBIT:");
  display_data(NRF_FICR->NRF_1MBIT,
               sizeof(NRF_FICR->NRF_1MBIT) / sizeof(*NRF_FICR->NRF_1MBIT),
               reinterpret_cast<uintptr_t>(NRF_FICR->NRF_1MBIT));
  uint8_t buf[sizeof(NRF_FICR->NRF_1MBIT)];
  puts("Unaligned copy:");
  memcpy(buf, const_cast<uint32_t*>(NRF_FICR->NRF_1MBIT), sizeof(buf));
#else
  uint8_t buf[16] = {1, 2, 3, 4, 5};
#endif
  display_data(buf, sizeof(buf), reinterpret_cast<uintptr_t>(buf));
}
