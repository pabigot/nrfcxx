// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Basic application to scan the primary I2C bus and identify
 * devices found thereon. */

#include <cstdio>

#include <nrfcxx/periph.hpp>

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto& twi = periph::TWI::instance(0);
  int rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                             NRFCXX_BOARD_PSEL_TWI0_SDA,
                             TWI_FREQUENCY_FREQUENCY_K100,
                             1000);
  printf("TWI0 at SCL %u SDA %u ; configure got %d\n",
         NRFCXX_BOARD_PSEL_TWI0_SCL, NRFCXX_BOARD_PSEL_TWI0_SDA,
         rc);
  if (0 <= rc) {
    for (uint8_t addr = 4; addr <= 0x77; ++addr) {
      rc = twi.check_addr(addr);
      if (0 == rc) {
        printf("%02x: ok\n", addr);
      } else {
        auto ec = twi.error_decoded(rc);
        if (!(twi.ERR_ANACK & ec)) {
          printf("%02x: undiagnosed error %x\n", addr, ec);
        }
      }
    }
  }
  puts("Scan complete");
}
