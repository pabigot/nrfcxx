/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF52832 variant XXAA
 * 512 kiB flash at 0x0000.0000
 *  64 kiB SRAM  at 0x2000.0000
 *
 * S132 minimum memory requirements:
 * Version | Flash   | RAM
 * ------- | ------- | ------
 * 6.1.0   | 0x26000 | 0x1628
 * 6.0.0   | 0x26000 | 0x1628
 * 5.1.0   | 0x23000 | 0x1380
 * 5.0.0   | 0x23000 | 0x14b8
 *
 * SDK-15 uses S132-6.0.0 with 0x26000 for flash but only 0x14b8 for
 * RAM, which seems wrong.
 *
 * The minimum is completely unusable.  Configure for one
 * vendor-specific UUID, one peripheral, and four centrals sharing a
 * single security manager with the default GATT size.  Plus padding.
 *
 * S132 6.1.0 at 0x00000 for 0x26000 flash, 0x20000000 for 0x4000 RAM
 * Persisted  at 0x7F000 for 0x01000 FLASH
 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00026000, LENGTH = 0x5A000
  PERSIST (r): ORIGIN = 0x0007F000, LENGTH = 0x01000
  RAM (rwx)  : ORIGIN = 0x20004000, LENGTH = 0x0C000
}
