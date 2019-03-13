/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2019 Peter A. Bigot */

/** Board-specific header for Electronut Papyr
 *
 * Header P1 provides SWD.
 *
 * Header P2 provides Vdd, GND, and P0.05 through P0.08.  Pin
 * assignments are per vendor firmware Zephyr board documentation.
 *
 * P0  | 0         | 1         | 2        | 3        | 4
 * --: | :-------  | :-------- | :------- | :------- | :--------
 *   0 | XL1       | XL2       | EP.RESn  | EP.BUSY  | n/c
 *   5 | P2.1      | P2.2      | P2.3     | P2.4     | NFC1
 *  10 | NFC2      | Eink.EN   | n/c      | Green    | Red
 *  15 | Blue      | n/c       | n/c      | RESETn   | n/c
 *  20 | n/c       | n/c       | n/c      | n/c      | n/c
 *  25 | n/c       | n/c       | n/c      | EP.DCn   | EP.MOSI
 *  30 | EP.CSn    | EP.SCK    |
 *
 * NB: Second GPIO instance is assigned ordinals 32 through 47.
 * No pins from this instance are exposed.
 *
 * Also see: https://www.waveshare.com/wiki/1.54inch_e-Paper_Module
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 18

#define NRFCXX_BOARD_PSEL_SCOPE0 5
#define NRFCXX_BOARD_PSEL_SCOPE1 6
#define NRFCXX_BOARD_PSEL_UART0_RXD 7
#define NRFCXX_BOARD_PSEL_UART0_TXD 8

#define NRFCXX_BOARD_PSEL_EPAPER_PWR_CTRL 11

#define NRFCXX_BOARD_PSEL_EPAPER_BUSY 3
#define NRFCXX_BOARD_PSEL_EPAPER_RESn 2
#define NRFCXX_BOARD_PSEL_EPAPER_DCn 28
#define NRFCXX_BOARD_PSEL_EPAPER_CSn 30
#define NRFCXX_BOARD_PSEL_EPAPER_SCK 31 /* D0 */
#define NRFCXX_BOARD_PSEL_EPAPER_MOSI 29 /* D1 */

#define NRFCXX_BOARD_PSEL_TWI0_SDA NRFCXX_BOARD_PSEL_SCOPE0
#define NRFCXX_BOARD_PSEL_TWI0_SCL NRFCXX_BOARD_PSEL_SCOPE1

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
