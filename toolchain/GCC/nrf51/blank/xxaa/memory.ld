/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF51 variant XXAA
 * 256 kiB flash at 0x0000.0000
 *  16 kiB SRAM  at 0x2000.0000 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256k
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 16k
}
