/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Board-specific header for PCA10031
 *
 * Pin | 0        | 1         | 2        | 3        | 4
 * --: | :------- | :-------- | :------- | :------- | :--------
 *   0 | n/c      | n/c       | n/c      | n/c      | AIN5
 *   5 | n/c      | n/c       | SDA      | UART.RTS | UART.TXD
 *  10 | UART.CTS | UART.RXD  | n/c      | n/c      | n/c
 *  15 | SCOPE0   | SCOPE1    | SCOPE2   | SCOPE3   | SCOPE4
 *  20 | SCOPE5   | LED0      | LED1     | LED2     | n/c
 *  25 | n/c      | XL2       | XL1      | n/c      | n/c
 *  30 | n/c      | n/a       | n/a      | n/a      | n/a
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP

namespace nrfcxx {
namespace board {

/* Scope pins are exposed on edge of board */
#define NRFCXX_BOARD_PSEL_SCOPE0 15
#define NRFCXX_BOARD_PSEL_SCOPE1 16
#define NRFCXX_BOARD_PSEL_SCOPE2 17
#define NRFCXX_BOARD_PSEL_SCOPE3 18
#define NRFCXX_BOARD_PSEL_SCOPE4 19
#define NRFCXX_BOARD_PSEL_SCOPE5 20

#define NRFCXX_BOARD_PSEL_BUTTON0 NRFCXX_BOARD_PSEL_SCOPE5

#define NRFCXX_BOARD_PSEL_UART0_RXD 11
#define NRFCXX_BOARD_PSEL_UART0_TXD 9
#define NRFCXX_BOARD_PSEL_UART0_CTS 10
#define NRFCXX_BOARD_PSEL_UART0_RTS 8

#define NRFCXX_BOARD_PSEL_TWI0_SDA -1
#define NRFCXX_BOARD_PSEL_TWI0_SCL -1

constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;
constexpr bool button_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
