// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Provide device-specific information.
 *
 * Primarily this provides the Bluetooth address under which the
 * device will communicate.
 *
 * A secondary use is to enable the PSELRESET capability on nRF52
 * devices.  This feature is sticky, but is lost whenever the UICR is
 * erased, e.g. when transitioning from a non-SD to an SD
 * installation. */

#include <cinttypes>
#include <cstdio>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/impl.hpp>

int
main (void)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  printf("Device id at %p: %08" PRIx32 "%08" PRIx32 "\n",
         NRF_FICR->DEVICEID,
         NRF_FICR->DEVICEID[1], // most significant
         NRF_FICR->DEVICEID[0]);
  printf("Device address: %08" PRIx32 "%08" PRIx32 " type %lx\n",
         NRF_FICR->DEVICEADDR[1], // most significant
         NRF_FICR->DEVICEADDR[0],
         NRF_FICR->DEVICEADDRTYPE);
  printf("BLE address: %04" PRIx16 "%08" PRIx32 "\n",
         0xC000 | static_cast<uint16_t>(NRF_FICR->DEVICEADDR[1]),
         NRF_FICR->DEVICEADDR[0]);

#if (NRF51 - 0)
  printf("HWID %04x\n", static_cast<uint16_t>(NRF_FICR->CONFIGID));
#else
  union {
    struct {
      uint32_t u32;
      uint8_t eol;
    };
    char text[5];
  } u{};

  /* nRF52 doesn't have a HWID.  What we're emitting is the value at
   * offset 0x5C from the start of the FICR, which is where HWID was
   * in the nRF51, and what OpenOCD reads as the hardware ID. */
  {
    using nrfcxx::nrf5::UICR;
    auto fp = NRF_FICR->RESERVED1 + 17;
    printf("*HWID %04x", static_cast<uint16_t>(*fp));
    u.u32 = pabigot::byteorder::host_x_be(NRF_FICR->INFO.VARIANT);
    printf(" ; %lx %s ; Pkg %lx\n", NRF_FICR->INFO.PART, u.text, NRF_FICR->INFO.PACKAGE);

    printf("PSELRESET %ld %ld\n", UICR->PSELRESET[0], UICR->PSELRESET[1]);
    if (0 > (int)UICR->PSELRESET[0]) {
      nrfcxx::nrf5::series::enable_pinreset();
      printf("Configured PSELRESET %ld\n", UICR->PSELRESET[0]);
    }
  }
#endif
  return 0;
}
