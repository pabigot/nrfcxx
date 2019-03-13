/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2019 Peter A. Bigot */

/** Board-specific header for Particle Xenon
 *
 * SPI.MOSI is QSPI.IO0, SPI.MISO is QSPI.IO1.
 *
 * EFL is extern 32 Mb flash GD25Q32C with SPI/QSPI interfaces.  This
 * device is assigned SPI2/SPIM2/QSPI.
 *
 * P0  | 0         | 1         | 2        | 3        | 4
 * --: | :-------  | :-------- | :------- | :------- | :--------
 *   0 | XL1       | XL2       | AIN0     | AIN1     | AIN2
 *   5 | BAT_DET   | UART.TXD  | n/c      | UART.RXD | NFC1
 *  10 | NFC2      | BTN0      | Vbus_DET | LED0     | LED1
 *  15 | LED2      |           | EFL.CSn  | RESETn   | EFL.SCK
 *  20 | EFL.MOSI  | EFL.MISO  | EFL.WPn  | EFL.HOLD | SKY.VC1
 *  25 | SKY.VC2   | SDA       | SCL      | AIN4     | AIN5
 *  30 | AIN6      | AIN7      |
 *
 * NB: Second GPIO instance is assigned ordinals 32 through 47.
 *
 * P1  | 0         | 1         | 2        | 3        | 4
 * --: | :-------  | :-------- | :------- | :------- | :--------
 *   0 | SWO       | SCOPE0    | SCOPE1   | SCOPE6   | n/c
 *   5 | n/c       | n/c       | n/c      | SCOPE2   | BAT_CHGn
 *  10 | SCOPE3    | SCOPE4    | SCOPE5   | SPI1.MOSI| SPI1.MISO
 *  15 | SPI1.SCK
 *
 * Feather headers: There is no standard numbering.  Pins begin with
 * zero at lower right, go up right, then go up left, skipping those
 * that don't connect to the microcontroller.
 *
 *  #  | Connection | Role         | Role   | Connection |  #
 * --: | :--------- | :----------- | :----- | :--------- | :--
 * n/c | P0.18      | RESETn       |        |            | n/c
 * n/c | -          | 3.3V         |        |            |
 *  20 | P0.11      | MODEn        |        |            |
 * n/c | -          | GND          |        |            |
 *  19 | P0.03      | ADC0 (AIN1)  |        | LiPo+      |
 *  18 | P0.04      | ADC1 (AIN2)  |        | LDO Enable |
 *  17 | P0.28      | ADC2 (AIN4)  |        | Vbus       |
 *  16 | P0.29      | ADC3 (AIN5)  |        | P1.03      | 8
 *  15 | P0.30      | ADC4 (AIN6)  |        | P1.12      | 7
 *  14 | P0.31      | ADC5, SPI_SS |        | P1.11      | 6
 *  13 | P1.15      | SPI_SCK      |        | P1.10      | 5
 *  12 | P1.13      | SPI_MOSI     |        | P1.08      | 4
 *  11 | P1.14      | SPI_MISO     | SCL1   | P1.02      | 3
 *  10 | P0.08      | UART1_RX     | SDA1   | P1.01      | 2
 *   9 | P0.06      | UART1_TX     | SCL    | P0.27      | 1
 * n/c |            |              | SDA    | P0.26      | 0
 *
 * @file */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {
namespace board {

#define NRFCXX_BOARD_PSEL_BUTTON0 11

#define NRFCXX_BOARD_PSEL_EFL_CSn 17
#define NRFCXX_BOARD_PSEL_EFL_SCK 19
#define NRFCXX_BOARD_PSEL_EFL_MOSI 20
#define NRFCXX_BOARD_PSEL_EFL_MISO 21
#define NRFCXX_BOARD_PSEL_EFL_WPn 22
#define NRFCXX_BOARD_PSEL_EFL_HOLDn 23

#define NRFCXX_BOARD_PSEL_SCOPE0 (32 + 1)
#define NRFCXX_BOARD_PSEL_SCOPE1 (32 + 2)
#define NRFCXX_BOARD_PSEL_SCOPE2 (32 + 8)
#define NRFCXX_BOARD_PSEL_SCOPE3 (32 + 10)
#define NRFCXX_BOARD_PSEL_SCOPE4 (32 + 11)
#define NRFCXX_BOARD_PSEL_SCOPE5 (32 + 12)
#define NRFCXX_BOARD_PSEL_SCOPE6 (32 + 3)

#define NRFCXX_BOARD_PSEL_UART0_RXD 8
#define NRFCXX_BOARD_PSEL_UART0_TXD 6
#define NRFCXX_BOARD_PSEL_UART0_CTS (32 + 2)
#define NRFCXX_BOARD_PSEL_UART0_RTS (32 + 1)

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

/* Battery is sampled through AIN3 (P0.5) with a voltage divider of
 * 806K over 2M1. */
#define NRFCXX_BOARD_BATTERY_AIN 3
#define NRFCXX_BOARD_BATTERY_R1 806000U
#define NRFCXX_BOARD_BATTERY_R2 2100000U

#define NRFCXX_BOARD_PSEL_VCHG_DETECT 12
#define NRFCXX_BOARD_PSEL_CHGn_STATUS (32 + 9)

/* Antenna source is controlled through a SKY13351 SPDT switch.
 *
 * VC1 connects to the PCB antenna.  VC2 connects to the uFL
 * connector.
 *
 * If neither switch is driven high the Xenon signal strength is very
 * poor.  board::initialize() sets both these GPIOs to WRONLY and
 * enables the PCB antenna.
 *
 * The SKY switch is documented to use 5 uA, provided through the VCx
 * input, but in practice it doesn't appear to increase nop current
 * draw significantly. */
#define NRFCXX_BOARD_PSEL_SKY_PCBEN 24
#define NRFCXX_BOARD_PSEL_SKY_UFLEN 25

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
