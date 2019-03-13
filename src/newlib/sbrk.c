// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2019 Peter A. Bigot

/** Alternative implementations of _sbrk(2) for NRFCXX
 *
 * This file contains implementations of _sbrk(2) that follow a
 * variety of policies.  The desired implementation is selected at
 * link time by defining the _sbrk symbol to refer to one of these.
 * If @c -ffunction-sections is used, exactly one implementation is
 * included.
 *
 * The code assumes the standard CMSIS startup structure and linker
 * script is used.  This organizes RAM from lower addresses to upper,
 * with initialized and uninitialized data at the bottom, the end
 * marked with the symbol @c end, followed by any allocated heap
 * ending at symbol @c __HeapLimit, followed by any allocated stack
 * sections.  End of RAM is indicated by symbol @c __StackTop, with @c
 * __StackLimit a symbol far enough below that to hold all allocated
 * stack sections.
 *
 * @file */

/* We're providing system call implementation here, so ensure we have
 * visible prototypes that match what newlib is expecting. */
#define _COMPILING_NEWLIB

#include <errno.h>
#include <sys/types.h>
#include <sys/unistd.h>

#if (NRF51 - 0)
#include <nrf51.h>
#else /* NRF_SERIES */
#include <nrf52.h>
#endif /* NRF_SERIES */

#include <nrfcxx/newlib/system.h>

/* Implement allocation with a policy-dependent upper bound. */
static inline __attribute__((__gnu_inline__,__always_inline__))
void*
common_sbrk (char* const upper_bound,
             ptrdiff_t increment)
{
  static char* brk;        /* the current program break */
  extern char end;          /* symbol at which heap starts */
  char* nbrk;
  void* rv;

  if (0 == brk) {
    brk = &end;
  }
  nbrk = increment + brk;
  if (upper_bound < nbrk) {
    return _nrfcxx_sbrk_error(brk, brk - &end, increment);
  }
  rv = brk;
  brk = nbrk;
  return rv;
}

void*
_nrfcxx_sbrk_fatal (ptrdiff_t increment)
{
  return _nrfcxx_sbrk_error(0, 0, increment);
}

void*
_nrfcxx_sbrk_heap (ptrdiff_t increment)
{
  extern char __HeapLimit;  /* symbol placed just past end of heap */
  return common_sbrk(&__HeapLimit, increment);
}

void*
_nrfcxx_sbrk_fixedstack (ptrdiff_t increment)
{
  extern char __StackLimit;   /* reserved end of stack */
  return common_sbrk(&__StackLimit, increment);
}

void*
_nrfcxx_sbrk_dynstack (ptrdiff_t increment)
{
  register char* sp __asm__("sp");
  return common_sbrk(sp, increment);
}

void*
_nrfcxx_sbrk_unlimitedstack (ptrdiff_t increment)
{
  extern char __StackLimit;   /* reserved end of stack */
  register char* sp __asm__("sp");
  char* upper_bound = &__StackLimit;
  if (sp < upper_bound) {
    upper_bound = sp;
  }
  return common_sbrk(upper_bound, increment);
}

/* Provide a weak alias that will resolve to the unlimitedstack
 * implementation in the case where _sbrk() is requested.  This
 * matches the nosys behavior of newlib. */
void* _sbrk (ptrdiff_t increment) __attribute__((__weak__,__alias__("_nrfcxx_sbrk_heap")));

size_t
_nrfcxx_heap_used (void)
{
  extern char end;
  extern void* sbrk (ptrdiff_t __incr);
  return (char*)sbrk(0) - &end;
}
