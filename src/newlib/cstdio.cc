// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2018 Peter A. Bigot

#include <cerrno>
#include <cstdio> // needed for ssize_t

#include <nrfcxx/periph.hpp>
#include <nrfcxx/newlib/system.h>

#ifndef ENABLE_NANO
#define ENABLE_NANO 1
#endif /* ENABLE_NANO */

namespace {

uint8_t flags;

/** Bit in `flags` that is set if cstdio operations should return
 * successfully without doing anything. */
#define FLAGS_DISABLED 0x01

/** Bit in `flags` that is set to indicate that the UART was
 * initialized as a side-effect of cstdio operations that were not
 * disabled. */
#define FLAGS_INITIALIZED 0x02

void
cstdio_uart0_initialize ()
{
  using nrfcxx::periph::UART;
  UART::instance().enable(0, false);
}

inline bool
cstdio_allowed ()
{
  if (FLAGS_DISABLED & flags) {
    return false;
  }
  if (!(FLAGS_INITIALIZED & flags)) {
    cstdio_uart0_initialize ();
    flags |= FLAGS_INITIALIZED;
  }
  return true;
}

} // anonymous

extern "C" {

bool
_nrfcxx_cstdio_allowed (bool allowed)
{
  if (allowed) {
    flags &= ~FLAGS_DISABLED;
  } else {
    flags |= FLAGS_DISABLED;
  }
  return (FLAGS_INITIALIZED & flags);
}

ssize_t
_write (int fd,
        const void* buf,
        size_t count)
{
  using nrfcxx::periph::UART;
  UART& uart{UART::instance()};

  ssize_t rc = -1;
  if ((1 == fd) || (2 == fd)) {
    /* newlib likes to be told that things work.  Assume they will; if
     * we follow a path where we try and they don't the value will be
     * corrected. */
    rc = count;
    if (cstdio_allowed()) {
#if (ENABLE_NANO - 0)
      const uint8_t* sp = reinterpret_cast<const uint8_t*>(buf);
      const uint8_t* const spe = sp + count;
      while (sp < spe) {
        sp += uart.write(sp, spe-sp);
      }
#else /* ENABLE_NANO */
      rc = uart.write(reinterpret_cast<const uint8_t*>(buf), count);
      if (0 == rc) {
        errno = EAGAIN;
        rc = -1;
      }
#endif /* ENABLE_NANO */
    }
  } else {
    errno = EINVAL;
  }
  return rc;
}

ssize_t
_read (int fd,
       void* buf,
       size_t count)
{
  using nrfcxx::periph::UART;
  UART& uart{UART::instance()};

  ssize_t rv = -1;
  if (0 == fd) {
    if (cstdio_allowed()) {
      rv = uart.read(reinterpret_cast<uint8_t*>(buf), count);
      if (0 == rv) {
        errno = EAGAIN;
        rv = -1;
      }
    } else {
      errno = EAGAIN;
    }
  } else {
    errno = EINVAL;
  }
  return rv;
}

} // extern "C"
