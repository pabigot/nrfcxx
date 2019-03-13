/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Board-specific header for PCA10040
 *
 * The assignments are based on the nRF52-DK standard assignments,
 * augmented with the Arduino-compatible signal purposes, with unused
 * pins assigned to scope function.
 *
 * Pin | 0        | 1         | 2        | 3        | 4
 * --: | :------- | :-------- | :------- | :------- | :--------
 *   0 | XL1      | XL2       | AREF     | AIN1     | AIN2
 *   5 | UART.RTS | UART.TXD  | UART.CTS | UART.RXD | NFC1
 *  10 | NFC2     | SCOPE0    | SCOPE1   | BTN0     | BTN1
 *  15 | BTN2     | BTN3      | LED0     | LED1     | LED2
 *  20 | LED3     | RESET     | SCOPE2   | SCOPE3   | SCOPE4
 *  25 | SCOPE5   | SDA       | SCL      | AIN4     | AIN5
 *  30 | AIN6     | AIN7      |
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 13
#define NRFCXX_BOARD_PSEL_BUTTON1 14
#define NRFCXX_BOARD_PSEL_BUTTON2 15
#define NRFCXX_BOARD_PSEL_BUTTON3 16

#define NRFCXX_BOARD_PSEL_SCOPE0 11
#define NRFCXX_BOARD_PSEL_SCOPE1 12
#define NRFCXX_BOARD_PSEL_SCOPE2 22
#define NRFCXX_BOARD_PSEL_SCOPE3 23
#define NRFCXX_BOARD_PSEL_SCOPE4 24
#define NRFCXX_BOARD_PSEL_SCOPE5 25

#define NRFCXX_BOARD_PSEL_UART0_RXD 8
#define NRFCXX_BOARD_PSEL_UART0_TXD 6
#define NRFCXX_BOARD_PSEL_UART0_CTS 7
#define NRFCXX_BOARD_PSEL_UART0_RTS 5

#define NRFCXX_BOARD_PSEL_TWI0_SDA 26
#define NRFCXX_BOARD_PSEL_TWI0_SCL 27

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
