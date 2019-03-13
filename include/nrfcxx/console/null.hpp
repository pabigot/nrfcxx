/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Functions that discard all console output.
 *
 * This file should be included into application implementation files
 * that use the console output infrastructure of
 * <nrfcxx/console/cstdio.hpp> in a build where output is not desired.
 *
 * @file */

#ifndef NRFCXX_CONSOLE_HPP
#define NRFCXX_CONSOLE_HPP
#pragma once

/** @cond DOXYGEN_EXCLUDE */

#include <cstdio>

namespace {

inline void csetvbuf ()
{ }

template <typename ...Args>
inline void cprintf (const char* format, Args... args)
{ }

inline void cputs (const char* text)
{ }

inline int cputchar (int ch)
{
  return ch;
}

inline bool cisstdio ()
{
  return false;
}

} // ns anonymous

#endif /* NRFCXX_CONSOLE_HPP */
