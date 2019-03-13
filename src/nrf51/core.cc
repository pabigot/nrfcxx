// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/core.hpp>

/** Loop to delay for a requested number of cycles. */
void
__attribute__((__naked__))
nrfcxx::nrf5::series::delay_cycles (unsigned int cycles)
{
  // Four cycles per loop iteration
  __ASM volatile ("1:\tSUB %0, %0, #4\n\t"
                  "BHI\t1b\n\t"
                  "BX\tLR\n\t"
                  : "+l"(cycles));
}
