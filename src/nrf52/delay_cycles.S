/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/* For performance as specified the loop must be cacheable.  The cache
 * involved is distinct from the I-Cache configurable by NVMC; rather
 * it appears to be an internal ARMv7-M cache used for fetching
 * instructions from flash.  Experimentation shows that if the
 * function begins at an address 0xzz0 through 0xzz6 then performance
 * is acceptable; at 0xzz8 or later performance will be poor.
 * Information on ARMv7-M internal instruction cache details has not
 * been located beyond the link below, which says only:
 *
 *   Note: A small caching component is present in the Cortex-M3 and
 *   Cortex-M4 processors to accelerate flash memory accesses during
 *   instruction fetches.
 *
 * So we align it to 16-bytes and it just works.
 *
 * The __align__ attribute isn't supported for function declarations
 * in arm-gcc, and G++ won't allow aliases between data and function
 * objects, so just implement the thing in assembly. */

/* void ::nrfcxx::nrf5::series::delay_cycles (unsigned int cycles) */
	.section	.text._ZN6nrfcxx4nrf56series12delay_cyclesEj,"ax",%progbits
	.align	4  /* 16-byte aligned */
	.global	_ZN6nrfcxx4nrf56series12delay_cyclesEj
	.syntax unified
	.thumb
	.thumb_func
_ZN6nrfcxx4nrf56series12delay_cyclesEj:
1:	SUBS	r0, #3
	BHI	1b
	BX	LR
