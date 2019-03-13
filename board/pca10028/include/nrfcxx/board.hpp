/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Board-specific header for PCA10028
 *
 * The assignments are based on the nRF51-DK standard assignments,
 * augmented with the Arduino-compatible signal purposes, with unused
 * pins assigned to scope function.
 *
 * Pin | 0        | 1         | 2        | 3        | 4
 * --: | :------- | :-------- | :------- | :------- | :--------
 *   0 | AREF     | AIN2      | AIN3     | AIN4     | AIN5
 *   5 | AIN6     | AIN7      | SCL      | UART.RTS | UART.TXD
 *  10 | UART.CTS | UART.RXD  | SCOPE0   | SCOPE1   | SCOPE2
 *  15 | SCOPE3   | SCOPE4    | BTN0     | BTN1     | BTN2
 *  20 | BTN3     | LED0      | LED1     | LED2     | LED3
 *  25 | SCOPE5   | XL2       | XL1      | SCOPE6   | SCOPE7
 *  30 | SDA      | n/c       | n/c      | n/c      | n/c
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 17
#define NRFCXX_BOARD_PSEL_BUTTON1 18
#define NRFCXX_BOARD_PSEL_BUTTON2 19
#define NRFCXX_BOARD_PSEL_BUTTON3 20

#define NRFCXX_BOARD_PSEL_SCOPE0 12
#define NRFCXX_BOARD_PSEL_SCOPE1 13
#define NRFCXX_BOARD_PSEL_SCOPE2 14
#define NRFCXX_BOARD_PSEL_SCOPE3 15
#define NRFCXX_BOARD_PSEL_SCOPE4 16
#define NRFCXX_BOARD_PSEL_SCOPE5 25
#define NRFCXX_BOARD_PSEL_SCOPE6 28
#define NRFCXX_BOARD_PSEL_SCOPE7 29

#define NRFCXX_BOARD_PSEL_UART0_RXD 11
#define NRFCXX_BOARD_PSEL_UART0_TXD 9
#define NRFCXX_BOARD_PSEL_UART0_CTS 10
#define NRFCXX_BOARD_PSEL_UART0_RTS 8

#define NRFCXX_BOARD_PSEL_TWI0_SCL 7
#define NRFCXX_BOARD_PSEL_TWI0_SDA 30

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
