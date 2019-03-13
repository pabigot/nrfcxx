/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF52840 variant XXAA
 * 1024 kiB flash at 0x0000.0000
 *  256 kiB SRAM  at 0x2000.0000 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 1024k
  PERSIST (r): ORIGIN = 0x00080000, LENGTH = 0k
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 256k
}
