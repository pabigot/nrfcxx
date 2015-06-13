/**************************************************************************//**
 * @file     startup_ARMCM0.S
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM0 Device
 * @version  V5.3.1
 * @date     09. July 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/

                .syntax  unified
                .arch    armv6-m


/*
;<h> Stack Configuration
;  <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>
*/
                .equ     Stack_Size, 0x00000400

                .section .stack
                .align   3
	            .globl   __StackTop
	            .globl   __StackLimit
__StackLimit:
                .space   Stack_Size
                .size    __StackLimit, . - __StackLimit
__StackTop:
                .size    __StackTop, . - __StackTop


/*
;<h> Heap Configuration
;  <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>
*/
                .equ     Heap_Size, 0x00000C00

                .if      Heap_Size != 0                     /* Heap is provided */
                .section .heap
                .align   3
	            .globl   __HeapBase
	            .globl   __HeapLimit
__HeapBase:
                .space   Heap_Size
                .size    __HeapBase, . - __HeapBase
__HeapLimit:
                .size    __HeapLimit, . - __HeapLimit
                .endif


                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVCall Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

		/* External interrupts */
#define EXPAND_RESERVED(x_)		.long	0
#define EXPAND_IRQ_REFERENCE(x_)		.long	x_##_IRQHandler
#include <nrfcxx/internal/nrf51/vector_table.inc>
#undef EXPAND_IRQ_REFERENCE
#undef EXPAND_RESERVED

__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
/* PAN #16: RAMON reset value causes problems under certain conditions
 *
 * When this anomaly is present RAMON is zero on power-up, which means
 * the stack that's used during initialization is located in unpowered
 * RAM.  We need to turn that RAM on.
 *
 * In the v2 nRF51 chips the low 4 bits of RAMON controlled four
 * blocks of RAM, and an ONMODE mask of 0x0F was suggested.  In v3
 * this was split into RAMON with two bits and RAMONB with the other
 * two bits.  The startup sequence used in later SDKs provided by
 * Nordic stores both separately.
 *
 * In the v3 chips PAN #16 does not apply.  There are no IC variants
 * prior to v3 that have more than two blocks of RAM (i.e., there is
 * no need to set RAMONB).  However, RAMON and RAMONB are retained
 * registers, so it is not impossible that Reset_Handler could be
 * invoked in a state where some blocks are turned off (e.g., a wakeup
 * from system off.  There is no harm in setting RAMON bits if the
 * variant does not have the corresponding RAM blocks, and if we're
 * running this handler we're invoking initialization code that
 * assumes all RAM is available.  So it's best to enforce consistency
 * and set both RAMON and RAMONB to their nominal power-up values.  */
                .equ    NRF_POWER_RAMON_ADDRESS, 0x40000524
                .equ    NRF_POWER_RAMONB_ADDRESS, 0x40000554
                .equ    NRF_POWER_RAMONx_RAMxON_ONMODE_Msk, 0x3

                movs    r1, #NRF_POWER_RAMONx_RAMxON_ONMODE_Msk
                ldr     r0, =NRF_POWER_RAMON_ADDRESS
                ldr     r2, [r0]
                orrs    r2, r1
                str     r2, [r0]
                ldr     r0, =NRF_POWER_RAMONB_ADDRESS
                ldr     r2, [r0]
                orrs    r2, r1
                str     r2, [r0]

/* Firstly it copies data from read only memory to RAM.
 * There are two schemes to copy. One can copy more than one sections.
 * Another can copy only one section. The former scheme needs more
 * instructions and read-only data to implement than the latter.
 * Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.
 */

#ifdef __STARTUP_COPY_MULTIPLE
/* Multiple sections scheme.
 *
 * Between symbol address __copy_table_start__ and __copy_table_end__,
 * there are array of triplets, each of which specify:
 *   offset 0: LMA of start of a section to copy from
 *   offset 4: VMA of start of a section to copy to
 *   offset 8: size of the section to copy. Must be multiply of 4
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]
                ldr      r2, [r4, #4]
                ldr      r3, [r4, #8]

.L_loop0_0:
                subs     r3, #4
                blt      .L_loop0_0_done
                ldr      r0, [r1, r3]
                str      r0, [r2, r3]
                b        .L_loop0_0

.L_loop0_0_done:
                adds     r4, #12
                b        .L_loop0

.L_loop0_done:
#else
/* Single section scheme.
 *
 * The ranges of copy from/to are specified by following symbols
 *   __etext: LMA of start of the section to copy from. Usually end of text
 *   __data_start__: VMA of start of the section to copy to
 *   __data_end__: VMA of end of the section to copy to
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
                ldr      r1, =__etext
                ldr      r2, =__data_start__
                ldr      r3, =__data_end__

                subs     r3, r2
                ble      .L_loop1_done

.L_loop1:
                subs     r3, #4
                ldr      r0, [r1,r3]
                str      r0, [r2,r3]
                bgt      .L_loop1

.L_loop1_done:
#endif /*__STARTUP_COPY_MULTIPLE */

/* This part of work usually is done in C library startup code.
 * Otherwise, define this macro to enable it in this startup.
 *
 * There are two schemes too.
 * One can clear multiple BSS sections. Another can only clear one section.
 * The former is more size expensive than the latter.
 *
 * Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 * Otherwise define macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/* Multiple sections scheme.
 *
 * Between symbol address __copy_table_start__ and __copy_table_end__,
 * there are array of tuples specifying:
 *   offset 0: Start of a BSS section
 *   offset 4: Size of this BSS section. Must be multiply of 4
 */
                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]
                ldr      r2, [r3, #4]
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4
                blt      .L_loop2_0_done
                str      r0, [r1, r2]
                b        .L_loop2_0
.L_loop2_0_done:

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/* Single BSS section scheme.
 *
 * The BSS section is specified by following symbols
 *   __bss_start__: start of the BSS section.
 *   __bss_end__: end of the BSS section.
 *
 * Both addresses must be aligned to 4 bytes boundary.
 */
                ldr      r1, =__bss_start__
                ldr      r2, =__bss_end__

                movs     r0, 0
                subs     r2, r1
                ble      .L_loop3_done

.L_loop3:
                subs     r2, #4
                str      r0, [r1, r2]
                bgt      .L_loop3
.L_loop3_done:
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

                bl       SystemInit
                bl       _start

                .cantunwind
                .fnend
                .size    Reset_Handler, . - Reset_Handler


                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
#if (__STARTUP_USE_SHARED_DEFAULT_HANDLER - 0)
/* Just reference the default handler */
                .set     \Handler_Name, Default_Handler
#else /* __STARTUP_USE_SHARED_DEFAULT_HANDLER */
/* Provide a unique default handler for each candidate so
 * we can use the debugger to figure out which handler we
 * forgot to provide. */
                .type    \Handler_Name, %function
\Handler_Name:
                b        .
                .size    \Handler_Name, . - \Handler_Name
#endif /* __STARTUP_USE_SHARED_DEFAULT_HANDLER */
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  HardFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

#define EXPAND_IRQ_REFERENCE(x_)		Set_Default_Handler x_##_IRQHandler
#define EXPAND_RESERVED(x_)
#include <nrfcxx/internal/nrf51/vector_table.inc>
#undef EXPAND_IRQ_REFERENCE
#undef EXPAND_RESERVED


                .end
