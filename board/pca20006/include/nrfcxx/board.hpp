/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Board-specific header for PCA20006
 *
 * Pin | 0        | 1         | 2        | 3        | 4
 * --: | :------- | :-------- | :------- | :------- | :--------
 *   0 | n/c      | n/c       | n/c      | n/c      | n/c
 *   5 | n/c      | n/c       | n/c      | BTN0     | UART.TXD
 *  10 | n/c      | UART.RXD  | LED1     | n/c      | n/c
 *  15 | LED2     | LED0      | n/c      | BTN1     | n/c
 *  20 | SCOPE0   | n/c       | n/c      | n/c      | n/c
 *  25 | n/c      | XL2       | XL1      | n/c      | n/c
 *  30 | n/c      | n/c       | n/c      | n/c      | n/c
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 8
#define NRFCXX_BOARD_PSEL_BUTTON1 18

constexpr bool button_active_low = true;

constexpr bool has_lfxt = true;

constexpr bool led_active_low = true;

#define NRFCXX_BOARD_PSEL_UART0_RXD 11
#define NRFCXX_BOARD_PSEL_UART0_TXD 9

#define NRFCXX_BOARD_PSEL_TWI0_SCL -1
#define NRFCXX_BOARD_PSEL_TWI0_SDA -1

/* Available on test points */
#define NRFCXX_BOARD_PSEL_SCOPE0 20

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
