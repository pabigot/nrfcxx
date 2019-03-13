/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/* Linker script for Nordic Semiconductor nRF51 variant XXAC
 * 256 kiB flash at 0x0000.0000
 *  32 kiB SRAM  at 0x2000.0000
 *
 * S130 minimum memory requirements:
 * Version | Flash   | RAM
 * ------- | ------- | ------
 * 2.0.1   | 0x1B000 | 0x13C8
 * 2.0.0   | 0x1B000 | 0x13C8
 * 1.0.0   | 0x1C000 | 0x2800
 *
 * The SDK uses 0x13C8 as the minimum RAM size in its default linker
 * map.  This is right for a BLE configuration with one UUID, no
 * peripherals, no connections, no security manager, and the minimum
 * (216 byte) attribute table size.
 *
 * It is wrong for anything useful.  The correct RAM size requirement
 * is returned from sd_ble_enable():
 *
 * #UUID  |  #P  |  #C  | #SMP | AT Size | APP Start | Comment
 * ------ | ---- | ---- | ---- | ------  | --------- | ----------
 *   1    |  0   |  0   |  0   | 0x00d8  | 0x13C8
 *   1    |  1   |  0   |  0   | 0x00d8  | 0x1B40    | 0x778 for peripheral support
 *   1    |  0   |  1   |  0   | 0x00d8  | 0x1838    | allowed, but central w/o SMP is useless(?)
 *   1    |  0   |  1   |  1   | 0x00d8  | 0x1A08
 *   1    |  1   |  1   |  1   | 0x00d8  | 0x2138    | 0x730 for peripheral support
 *   1    |  0   |  2   |  1   | 0x00d8  | 0x1E30    | 0x428 per added central
 *   1    |  0   |  4   |  1   | 0x00d8  | 0x2688    | 0x858 for two more central
 *   1    |  1   |  4   |  1   | 0x00d8  | 0x2DC0    | 0x738 for peripheral support
 *   1    |  1   |  4   |  1   |   0     | 0x3268    | default AT size 0x4A8 not 0x580?
 *
 * Reviewing this it's clear that there's no easy way to calculate the
 * required RAM, as even single-knob changes produce different minimum
 * memory requirements, and any documented default numbers aren't
 * right either.
 *
 * The value below is for the configuration that would be produced
 * using the softdevice handler's
 * softdevice_enable_get_default_config() for one vendor-specific
 * UUID, one peripheral, and four centrals sharing a single security
 * manager with the default GATT size.  Which is enough for basic
 * verification, but any real application needs its own specification.
 *
 * S130 2.0.1 at 0x00000 for 0x1B000 flash, 0x0000 for 0x3268 RAM
 * Custom     at 0x1B000 for 0x24C00 FLASH, 0x3268 for 0x4D98 RAM
 * Persisted  at 0x3FC00 for 0x00400 FLASH
 * ??Secure DFU at 0x3A800 for 0x05000 FLASH, 0x3C00 for 0x5380 RAM
 * ??DFU config at 0x3F800 for 0x00400 FLASH
 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x0001b000, LENGTH = 0x24C00
  PERSIST (r): ORIGIN = 0x0003FC00, LENGTH = 0x00400
  RAM (rwx)  : ORIGIN = 0x20003268, LENGTH = 0x04D98
}
