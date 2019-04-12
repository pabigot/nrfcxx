/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Board-specific header for PCA10059
 *
 * @warning PCA10059 does not have an on-board JLink chip.  Normal
 * programming involves nrfutil and the on-board DFU firmware.  nrfcxx
 * does not support this, and using the `program` target on this board
 * will overwrite the DFU firmware.
 *
 * @see https://devzone.nordicsemi.com/f/nordic-q-a/40924/q
 *
 * P0  | 0         | 1         | 2        | 3        | 4
 * --: | :-------  | :-------- | :------- | :------- | :--------
 *   0 | XL1       | XL2       | 0.02     | n/c      | 0.04
 *   5 | n/c       | LD1       | n/c      | LD2.R    | 0.09
 *  10 | 0.10      | 0.11      | LD2.B    | 0.13     | 0.14
 *  15 | 0.15      | n/c       | 0.17     | RESET    | RESET
 *  20 | 0.20      | RESET     | 0.22     | RESET    | 0.24
 *  25 | RESET     | 0.26      | n/c      | n/c      | 0.29
 *  30 | n/c       | 0.31      |
 *
 * NB: Second GPIO instance is assigned ordinals 32 through 47.
 *
 * P1  | 0         | 1         | 2        | 3         | 4
 * --: | :-------  | :-------- | :------- | :-------- | :--------
 *   0 | 1.00      | 1.01      | 1.02     | n/c       | 1.04
 *   5 | n/c       | SW1       | 1.07     | n/c       | LD2.G
 *  10 | 1.10      | 1.11      | n/c      | 1.13      | n/c
 *  15 | 1.15      |
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 (32 + 6)

#define NRFCXX_BOARD_PSEL_SCOPE0 (32 + 1)
#define NRFCXX_BOARD_PSEL_SCOPE1 (32 + 2)
#define NRFCXX_BOARD_PSEL_SCOPE2 (32 + 3)
#define NRFCXX_BOARD_PSEL_SCOPE3 (32 + 4)
#define NRFCXX_BOARD_PSEL_SCOPE4 (32 + 5)
#define NRFCXX_BOARD_PSEL_SCOPE5 (32 + 6)
#define NRFCXX_BOARD_PSEL_SCOPE6 (32 + 7)
#define NRFCXX_BOARD_PSEL_SCOPE7 (32 + 8)

#define NRFCXX_BOARD_PSEL_UART0_RXD (32 + 13)
#define NRFCXX_BOARD_PSEL_UART0_TXD (32 + 10)
#define NRFCXX_BOARD_PSEL_UART0_CTS (32 + 15)
#define NRFCXX_BOARD_PSEL_UART0_RTS (0 + 2)

#define NRFCXX_BOARD_PSEL_TWI0_SDA 9
#define NRFCXX_BOARD_PSEL_TWI0_SCL 10

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
