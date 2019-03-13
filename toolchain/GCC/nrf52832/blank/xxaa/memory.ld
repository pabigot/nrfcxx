/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF52832 variant XXAA
 * 512 kiB flash at 0x0000.0000
 *  64 kiB SRAM  at 0x2000.0000 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512k
  PERSIST (r): ORIGIN = 0x00080000, LENGTH = 0k
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 64k
}
