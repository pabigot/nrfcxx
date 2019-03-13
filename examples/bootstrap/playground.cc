// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** A template program you can use for playing with various things.
 *
 * Much like nop but is built with libnosys support, so is a good
 * framework for experiments that require cstdio. */

#include <nrfcxx/core.hpp>

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  while (true) {
    systemState::WFE();
  }
}
