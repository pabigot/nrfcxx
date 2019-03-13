/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** Board-specific header for host-based unit tests (based BLE400)
 *
 * Assignments correspond to labeled headers on the BLE400, with I2C
 * supplied by TWI0 and SPI by SPI1 to avoid conflict.  "NSS" is on
 * the SPI header and is assumed to be the default chip-select line.
 *
 * Pins 5, 6, 7, and 12 are exposed on the USART1 header but are
 * ignored favoring the on-board USB UART.
 *
 * Pin | 0        | 1         | 2        | 3        | 4
 * --: | :------- | :-------- | :------- | :------- | :--------
 *   0 | SDA      | SCL       | SCOPE0   | SCOPE1   | SCOPE2
 *   5 | SCOPE3   | SCOPE4    | SCOPE5   | UART.RTS | UART.TXD
 *  10 | UART.CTS | UART.RXD  |          |          |
 *  15 |          | BTN0      | BTN1     | LED0     | LED1
 *  20 | LED2     | LED3      | LED4     | MISO     | MOSI
 *  25 | SCK      | XL2       | XL1      |          |
 *  30 | NSS      | n/a       | n/a      | n/a      | n/a
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 16
#define NRFCXX_BOARD_PSEL_BUTTON1 17

#define NRFCXX_BOARD_PSEL_SCOPE0 2
#define NRFCXX_BOARD_PSEL_SCOPE1 3
#define NRFCXX_BOARD_PSEL_SCOPE2 4
#define NRFCXX_BOARD_PSEL_SCOPE3 5
#define NRFCXX_BOARD_PSEL_SCOPE4 6
#define NRFCXX_BOARD_PSEL_SCOPE5 7

#define NRFCXX_BOARD_PSEL_SPI1_MISO 23
#define NRFCXX_BOARD_PSEL_SPI1_MOSI 24
#define NRFCXX_BOARD_PSEL_SPI1_SCK 25
#define NRFCXX_BOARD_PSEL_SPI1_CSN 30

#define NRFCXX_BOARD_PSEL_TWI0_SDA 0
#define NRFCXX_BOARD_PSEL_TWI0_SCL 1

#define NRFCXX_BOARD_PSEL_UART0_RXD 11
#define NRFCXX_BOARD_PSEL_UART0_TXD 9
#define NRFCXX_BOARD_PSEL_UART0_CTS 10
#define NRFCXX_BOARD_PSEL_UART0_RTS 8

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = false;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
