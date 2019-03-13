// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Trivial example of using the events infrastructure.
 *
 * Might also be useful for verifying data structure sizes and
 * generated code. */

#include <nrfcxx/core.hpp>
#include <cstdio>

#define EVT_ONE 0x0001
#define EVT_TWO 0x0002

int
main (void)
{
  using nrfcxx::event_set;

  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  event_set events;

  printf("Events: %08x\n", events.fetch());
  events.set(EVT_ONE);
  printf("Events: %08x\n", events.fetch());
  auto setter = events.make_setter(EVT_TWO);
  printf("Events: %08x\n", events.fetch());
  setter();
  printf("Events: %08x\n", events.fetch());
  events.reset();

  return 0;
}
