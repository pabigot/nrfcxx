/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Primary header for nrfcxx implementation dependencies
 *
 * This header is included in implementation files and any headers
 * that require direct access to nRF51 peripheral resources.
 *
 * @file */

#ifndef NRFCXX_IMPL_HPP
#define NRFCXX_IMPL_HPP
#pragma once

#include <nrfcxx/core.hpp>

namespace nrfcxx {
namespace nrf5 {
namespace series {

/** Material common to all ADC peripheral implementations. */
struct ADC_Base
{

  /** Record the destination address and count of ADC samples.
   *
   * Collected samples are copied into destination buffer by the
   * interrupt handler or the peripheral itself. */
  static void setup_result_bi (volatile uint16_t* ptr,
                               uint16_t maxcnt = 1)
  {
    result_ptr_ = ptr;
    result_maxcnt_ = maxcnt;
  }

  static volatile uint16_t* result_ptr_;
  static uint16_t result_maxcnt_;
};

} // ns series
} // ns nrf5
} // ns nrfcxx

#if (51 == NRF_SERIES)
#include <nrfcxx/nrf51/impl.hpp>
#elif (52 == NRF_SERIES)
#include <nrfcxx/nrf52/impl.hpp>
#endif // NRF_SERIES
#include <nrfcxx/board.hpp>

/** Pin selection specification for a disabled scope function.
 *
 * Boards define zero or more macros with names like
 * `NRFCXX_BOARD_PSEL_SCOPE0` which have values from 0 through 30
 * indicating that the corresponding GPIO may be used as an external
 * signal with the nrfcxx::gpio::instr_psel API.
 *
 * This constant is used to note that a particular instr_psel instance
 * is disabled. */
#define NRFCXX_BOARD_PSEL_SCOPEn -1

namespace nrfcxx {
namespace board {

/** @var nrfcxx::board::has_lfxt
 * `true` iff the board has an installed 32 KiHz crystal. */

/** Provide the LFCLKSRC value preferred by the board.
 *
 * If the board has an installed 32 KiHz crystal this will be the
 * crystal source; otherwise it will be the internal RC oscillator for
 * which you probably want nrfcxx::clock to calibrate for you.
 *
 * @return either `CLOCK_LFCLKSRC_SRC_Xtal` or `CLOCK_LFCLKSRC_SRC_RC`
 * depending on board::has_lfxt. */
static constexpr unsigned int default_lfclk_src ()
{
  return (board::has_lfxt ? CLOCK_LFCLKSRC_SRC_Xtal : CLOCK_LFCLKSRC_SRC_RC);
}

} // namespace board

} // namespace nrfcxx

#endif /* NRFCXX_IMPL_HPP */
