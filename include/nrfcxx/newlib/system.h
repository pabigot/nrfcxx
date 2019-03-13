/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2014-2019 Peter A. Bigot */

/** NRFCXX system enhancements to newlib.
 *
 * This file declares functions that support the newlib nosys
 * replacement used by default in nrfcxx.
 *
 * @note Several parts of newlib require support for allocation from
 * the heap.  If the stdio infrastructure is referenced normally,
 * roughly 1468 bytes is allocated for state maintenance.  If
 * setvbuf(3) is used to disable buffering on stdio, this is reduced
 * to about 436 bytes.  In any case, selecting _nrfcxx_sbrk_fatal()
 * will cause failures if some newlib functions are invoked, and
 * reducing the default heap allocation in startup_CMx.S is risky.
 *
 * @file */

#ifndef NRFCXX_NEWLIB_SYSTEM_H
#define NRFCXX_NEWLIB_SYSTEM_H
#pragma once

#include <stdbool.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/** An sbrk() implementation that rejects any attempt to allocate
 * memory dynamically.
 *
 * This is not quite equivalent to _nrfcxx_sbrk_heap() with a
 * zero-sized heap, as simply invoking _sbrk() will result in the
 * failure even if the requested increment was zero.
 *
 * @note _nrfcxx_heap_used() invokes _sbrk with a zero increment.  So
 * does newlib prior to allocating for stdio support. */
void* _nrfcxx_sbrk_fatal (ptrdiff_t increment);

/** An sbrk() implementation that depends on a fixed heap allocated
 * within the standard startup infrastructure.
 *
 * An error is indicated if the reserved heap size would be exceeded.
 * There is no check against the current stack pointer. */
void* _nrfcxx_sbrk_heap (ptrdiff_t increment);

/** An sbrk() implementation that allows heap (growing up) to grow to
 * the bottom of a reserved stack region.
 *
 * An error is indicated if the new program break would encroach into
 * the reserved stack space.  There is no check against the current
 * stack pointer.
 *
 * @note This policy is preferred to _nrfcxx_sbrk_unlimitedstack()
 * when code may be executing in tasks where the stack frame is in
 * previously allocated memory. */
void* _nrfcxx_sbrk_fixedstack (ptrdiff_t increment);

/** An sbrk() implementation that allows heap (growing up) and stack
 * (growing down) to share a region of memory.
 *
 * An error is indicated if the new break point would encroach into
 * the current stack space.
 *
 * @note Like _nrfcxx_sbrk_unlimited(), but eliminating the minimum
 * reserved stack.  Not sure why this would be worth doing, but for
 * completeness.... */
void* _nrfcxx_sbrk_dynstack (ptrdiff_t increment);

/** An sbrk() implementation that allows heap (growing up) and stack
 * (growing down) to share a region of memory, with a minimum size
 * reserved for the stack but allowing for the stack to grow below
 * that point.
 *
 * An error is indicated if the new break point would encroach into
 * the reserved stack space or the currently used stack space. */
void* _nrfcxx_sbrk_unlimitedstack (ptrdiff_t increment);

/** The system function used to allocate memory for use by libc heap
 * memory management.
 *
 * By default this symbol is a weak alias to
 * _nrfcxx_sbrk_unlimitedstack().  This matches the nosys behavior of
 * newlib.  To select another policy you must provide a non-weak alias
 * to one of the other policies or your own implementation.
 * Alternative implementations include:
 * * _nrfcxx_sbrk_fatal()
 * * _nrfcxx_sbrk_heap()
 * * _nrfcxx_sbrk_fixedstack()
 * * _nrfcxx_sbrk_dynstack()
 * * _nrfcxx_sbrk_unlimitedstack()
 *
 * You can do this in the application main file with:
 *
 *     extern "C" {
 *     void* _sbrk_for_app (ptrdiff_t increment)
 *     {
 *       return _nrfcxx_sbrk_heap(increment);
 *     }
 *     void* _sbrk (ptrdiff_t increment) __attribute__((__alias__("_sbrk_for_app")));
 *     }
 *
 * @note As of GCC 4 the alias must be defined in the same translation
 * unit, hence the need for a local definition that wraps the provided
 * function.
 *
 * The reserved system stack space is 3 KiBy, which is sufficient for
 * use with standard newlib operations.  The application can increase
 * this by declaring objects that will force that space to be
 * expanded, as with:
 *
 *     __attribute__((__section__(".heap.extension")))
 *     volatile uint32_t extend_by_512_words[512];
 *
 * @note All NRFCXX policies invoke _nrfcxx_sbrk_error() if allocation
 * fails, allowing an application to control response to the failure.
 *
 * @param increment the number of bytes of additional memory that libc
 * needs in order to perform additional allocations.
 *
 * @return a pointer to the new end-of-memory, or <tt>(void*)-1</tt>
 * if no allocation can be performed.
 *
 * @see _nrfcxx_heap_used */
void* _sbrk (intptr_t increment);

/** This function is invoked whenever _sbrk() runs out of memory.
 *
 * @c libnrfcxx.a provides a weak definition that invokes
 * nrfcxx::failsafe with nrfcxx::FailSafeCode::HEAP_OVERRUN.  The
 * application may provide an alternative implementation that is more
 * diagnostic or that returns the responsibility of handling
 * out-of-memory to the application (i.e. requires the application to
 * check allocation return values).
 *
 * @param brk the current program break
 *
 * @param current total number of bytes allocated by previous
 * successful invocations of _sbrk() (i.e., allocated bytes preceding
 * @p brk)
 *
 * @param increment the number of bytes in the request that _sbrk()
 * cannot satisfy
 *
 * @return This function need not return.  An implementation that does
 * return must set @c errno to @c ENOMEM and return
 * <tt>(void*)-1</tt>. */
void* _nrfcxx_sbrk_error (void* brk,
                          ptrdiff_t current,
                          ptrdiff_t increment);

/** Function used to control whether the automatic enabling of UART to
 * support stdio operations is supported.
 *
 * For convenience during development this feature is enabled by
 * default, but having a UART running and active can complicate power
 * and critical timing measurements.  Invoking this function *prior to
 * any cstdio operations* will inhibit starting the UART and cause all
 * primitive I/O operations to return as though they did something.
 *
 * @param allowed if `false` will prevent any subsequent cstdio
 * operations from enabling the UART.
 *
 * @return `true` iff the UART had already been automatically enabled
 * in support of cstdio operations.
 */
bool _nrfcxx_cstdio_allowed (bool allowed);

/** Return the amount of heap memory currently in use within the
 * system, in bytes. */
size_t _nrfcxx_heap_used (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* NRFCXX_NEWLIB_SYSTEM_H */
