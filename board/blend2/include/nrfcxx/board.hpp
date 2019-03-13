/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Board-specific header for RedBear Blend2.
 *
 * Unused pins are assigned to scope.
 *
 * Pin | 0        | 1         | 2        | 3        | 4
 * --: | :------- | :-------- | :------- | :------- | :--------
 *   0 | XL1      | XL2       | SMBA     | AIN1     | AIN2
 *   5 | UART.RTS | UART.RX   | UART.CTS | UART.RXD | NFC1
 *  10 | NFC2     | SCOPE0    | SCOPE1   | SCOPE3   | SCOPE4
 *  15 | SCOPE5   | SCOPE6    | SCOPE7   | SCOPE8   | SCOPE9
 *  20 | SCOPE10  | SCOPE11   | SPI.CSn  | SPI.MOSI | SPI.MISO
 *  25 | SPI.SCK  | SDA       | SCL      | AIN4     | AIN5
 *  30 | AIN6     | AIN7      |
 *
 * @file
 */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_SCOPE0 11
#define NRFCXX_BOARD_PSEL_SCOPE1 12
#define NRFCXX_BOARD_PSEL_SCOPE2 13
#define NRFCXX_BOARD_PSEL_SCOPE3 14
#define NRFCXX_BOARD_PSEL_SCOPE4 15
#define NRFCXX_BOARD_PSEL_SCOPE5 16
#define NRFCXX_BOARD_PSEL_SCOPE6 17
#define NRFCXX_BOARD_PSEL_SCOPE7 18
#define NRFCXX_BOARD_PSEL_SCOPE8 19
#define NRFCXX_BOARD_PSEL_SCOPE9 20
#define NRFCXX_BOARD_PSEL_SCOPE10 21
#define NRFCXX_BOARD_PSEL_SCOPE11 22

#define NRFCXX_BOARD_PSEL_BUTTON0 NRFCXX_BOARD_PSEL_SCOPE11

#define NRFCXX_BOARD_PSEL_UART0_RXD 8
#define NRFCXX_BOARD_PSEL_UART0_TXD 6
#define NRFCXX_BOARD_PSEL_UART0_CTS 7
#define NRFCXX_BOARD_PSEL_UART0_RTS 5

#define NRFCXX_BOARD_PSEL_TWI0_SDA 26
#define NRFCXX_BOARD_PSEL_TWI0_SCL 27
#define NRFCXX_BOARD_PSEL_TWI0_SMBA 2

#define NRFCXX_BOARD_PSEL_SPI1_MISO 24
#define NRFCXX_BOARD_PSEL_SPI1_MOSI 23
#define NRFCXX_BOARD_PSEL_SPI1_SCK 25
#define NRFCXX_BOARD_PSEL_SPI1_CSN 22


constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
