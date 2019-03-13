/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF52832 variant XXAB
 * 256 kiB flash at 0x0000.0000
 *  32 kiB SRAM  at 0x2000.0000
 *
 * S132 minimum memory requirements:
 * Version | Flash   | RAM
 * ------- | ------- | ------
 * 6.0.0   | 0x26000 | 0x1628
 * 5.1.0   | 0x23000 | 0x1380
 * 5.0.0   | 0x23000 | 0x14b8
 *
 * SDK-15 doesn't support S132 on this variant, which is reasonable.
 * Since we're here, we'll allow it. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00026000, LENGTH = 0x1A000
  PERSIST (r): ORIGIN = 0x00040000, LENGTH = 0x00000
  RAM (rwx)  : ORIGIN = 0x20001628, LENGTH = 0x069D8
}
