/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Board-specific header for PCA10056
 *
 * The assignments are based on the nRF52-DK standard assignments,
 * augmented with the Arduino-compatible signal purposes, with unused
 * pins assigned to scope function.
 *
 * SPI.MOSI is QSPI.IO0, SPI.MISO is QSPI.IO1.
 *
 * EFL is extern 64 Mb flash MX25R6435F with SPI/QSPI interfaces.
 * Re-use of EFL and SPI/QSPI pins requires board hardware
 * modification (via solder bridges).  This device is assigned
 * SPI2/SPIM2/QSPI.
 *
 * P0  | 0         | 1         | 2        | 3        | 4
 * --: | :-------  | :-------- | :------- | :------- | :--------
 *   0 | XL1       | XL2       | AIN0     | AIN1     | AIN2
 *   5 | UART.RTS  | UART.TXD  | UART.CTS | UART.RXD | NFC1
 *  10 | NFC2      | BTN0/TD2  | BTN1/TD1 | LED0     | LED1
 *  15 | LED2      | LED3      | EFL.CSn  | RESET    | EFL.SCK
 *  20 | EFL.MOSI  | EFL.MISO  | QSPI.IO2 | QSPI.IO3 | BTN2
 *  25 | BTN3      | SDA       | SCL      | AIN4     | AIN5
 *  30 | AIN6      | AIN7      |
 *
 * NB: Second GPIO instance is assigned ordinals 32 through 47.
 *
 * P1  | 0         | 1         | 2        | 3         | 4
 * --: | :-------  | :-------- | :------- | :-------- | :--------
 *   0 | TD0/SWO   | SCOPE0    | SCOPE1   | SCOPE2    | SCOPE3
 *   5 | SCOPE4    | SCOPE5    | SCOPE6   | SCOPE7    | TD3
 *  10 |           |           | SPI1.CSn | SPI1.MOSI | SPI1.MISO
 *  15 | SPI1.SCK  |
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 11
#define NRFCXX_BOARD_PSEL_BUTTON1 12
#define NRFCXX_BOARD_PSEL_BUTTON2 24
#define NRFCXX_BOARD_PSEL_BUTTON3 25

/* NOTE: The connections from these signals to the P24 header are
 * through solder bridges SB20 through SB25, which are by default
 * open. */
#define NRFCXX_BOARD_PSEL_EFL_CSn 17
#define NRFCXX_BOARD_PSEL_EFL_SCK 19
#define NRFCXX_BOARD_PSEL_EFL_MOSI 20
#define NRFCXX_BOARD_PSEL_EFL_MISO 21
#define NRFCXX_BOARD_PSEL_EFL_WPn 22
#define NRFCXX_BOARD_PSEL_EFL_HOLDn 23

#define NRFCXX_BOARD_PSEL_SCOPE0 (32 + 1)
#define NRFCXX_BOARD_PSEL_SCOPE1 (32 + 2)
#define NRFCXX_BOARD_PSEL_SCOPE2 (32 + 3)
#define NRFCXX_BOARD_PSEL_SCOPE3 (32 + 4)
#define NRFCXX_BOARD_PSEL_SCOPE4 (32 + 5)
#define NRFCXX_BOARD_PSEL_SCOPE5 (32 + 6)
#define NRFCXX_BOARD_PSEL_SCOPE6 (32 + 7)
#define NRFCXX_BOARD_PSEL_SCOPE7 (32 + 8)

#define NRFCXX_BOARD_PSEL_UART0_RXD 8
#define NRFCXX_BOARD_PSEL_UART0_TXD 6
#define NRFCXX_BOARD_PSEL_UART0_CTS 7
#define NRFCXX_BOARD_PSEL_UART0_RTS 5

#define NRFCXX_BOARD_PSEL_SPI1_MISO (32 + 14)
#define NRFCXX_BOARD_PSEL_SPI1_MOSI (32 + 13)
#define NRFCXX_BOARD_PSEL_SPI1_SCK (32 + 15)
#define NRFCXX_BOARD_PSEL_SPI1_CSN 31

#define NRFCXX_BOARD_PSEL_SPI2_MISO NRFCXX_BOARD_PSEL_EFL_MISO
#define NRFCXX_BOARD_PSEL_SPI2_MOSI NRFCXX_BOARD_PSEL_EFL_MOSI
#define NRFCXX_BOARD_PSEL_SPI2_SCK NRFCXX_BOARD_PSEL_EFL_SCK

#define NRFCXX_BOARD_PSEL_QSPI_IO0 NRFCXX_BOARD_PSEL_SPI2_MOSI
#define NRFCXX_BOARD_PSEL_QSPI_IO1 NRFCXX_BOARD_PSEL_SPI2_MISO
#define NRFCXX_BOARD_PSEL_QSPI_IO2 NRFCXX_BOARD_PSEL_EFL_WPn
#define NRFCXX_BOARD_PSEL_QSPI_IO3 NRFCXX_BOARD_PSEL_EFL_HOLDn

#define NRFCXX_BOARD_PSEL_TWI0_SDA 26
#define NRFCXX_BOARD_PSEL_TWI0_SCL 27

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
