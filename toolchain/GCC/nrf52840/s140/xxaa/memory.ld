/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF52840 variant XXAA
 * 1024 kiB flash at 0x0000.0000
 *  256 kiB SRAM  at 0x2000.0000
 *
 * S140 minimum memory requirements:
 * Version       | Flash   | RAM
 * ------------- | ------- | ------
 * 6.0.0 - 6.1.1 | 0x26000 | 0x1628
 * 6.0.0-6.alpha | 0x25000 | 0x1530
 * 5.0.0-3.alpha | 0x24000 | 0x14B8
 * 5.0.0-2.alpha | 0x21400 | 0x1468
 * 5.0.0-1.alpha | 0x21000 | 0x19C0
 *
 * The minimum is completely unusable.  Configure for one
 * vendor-specific UUID, one peripheral, and four centrals sharing a
 * single security manager with the default GATT size.  Plus padding.
 *
 * S132 6.1.1 at 0x00000 for 0x26000 flash, 0x20000000 for 0x4000 RAM
 * Persisted  at 0xFF000 for 0x01000 FLASH
 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00026000, LENGTH = 0xD9000
  PERSIST (r): ORIGIN = 0x000FF000, LENGTH = 0x01000
  RAM (rwx)  : ORIGIN = 0x20004000, LENGTH = 0x3C000
}
