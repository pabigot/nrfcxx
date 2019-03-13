/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Functions supporting console output using cstdio.
 *
 * This file should be included into application implementation files
 * for which console output using C stdio is desired.
 *
 * @file */

#ifndef NRFCXX_CONSOLE_HPP
#define NRFCXX_CONSOLE_HPP
#pragma once

#include <cstdio>

namespace {

/** Disable buffering on stdout.
 *
 * This reduces heap usage by about 1032 octets. */
inline void csetvbuf ()
{
  setvbuf(stdout, NULL, _IONBF, 0);
}

/** Formatted printf. */
template <typename ...Args>
inline void cprintf (const char* format, Args... args)
{
  printf(format, args...);
}

/** Pure text output with added newline. */
inline void cputs (const char* text)
{
  puts(text);
}

/** Output a single character. */
inline int cputchar (int ch)
{
  return putchar(ch);
}

/** Indicate whether console selection is cstdio or null */
inline bool cisstdio ()
{
  return true;
}

} // ns anonymous

#endif /* NRFCXX_CONSOLE_HPP */
