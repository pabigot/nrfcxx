/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Board-specific header for Nordic Thingy:52 aka PCA20020
 *
 * GPIO pin assignments:
 *
 * Pin | 0            | 1          | 2            | 3          | 4
 * --: | :----------- | :--------- | :----------- | :--------- | :-----------
 *   0 | XL1          | XL2        | ANA/DIG0     | ANA/DIG1   | ANA/DIG2
 *   5 | IOX_OSCIO    | MPU_INT    | SDA          | SCL        | NFC1
 *  10 | NFC2         | BUTTON     | LIS_INT1     | USB_DETECT | SDA_EXT
 *  15 | SCL_EXT      | IOX_RESETn | BAT_CHG_STAT | MOS_1      | MOS_2
 *  20 | MOS_3        | MOS_4      | CCS_INT      | LPS_INT    | HTS_INT
 *  25 | MIC_DOUT     | MIC_CLK    | SPEAKER      | BATTERY    | SPK_PWR_CTRL
 *  30 | VDD_PWR_CTRL | BH_INT
 *
 * System default PIN_CNF:
 * XL1, XL2: OUT CLR
 * DIG0, DIG1, DIG2, IOX_OSCIO,MPU_INT: DISCON PULLDOWN
 * SDA, SCL, NFC1, NFC2: DISCON NOPULL
 * BUTTON: DISCON PULLUP
 * LIS_INT1, USB_DETECT, SDA_EXT, SCL_EXT, IOX_RESET, BAT_CHG_STATE: DISCON NOPULL
 * MOS_1, MOS_2, MOS_3, MOS_4: OUT CLR
 * CCS_INT, LPS_INT, MIC_DOUT, MIC_CLK: DISCON PULLDOWN
 * SPEAKER: OUT CLR
 * BATTERY: DISCON NOPULL,
 * SPK_PWR_CTRL: OUT CLR
 * VDD_PWR_CTRL: OUT CLR
 * BH_INT: DISCON PULLDOWN
 *
 * Many key functions are provided on a Semtech SX1509B IO extender,
 * an I2C device at address 0x3E.  The device is primarily used for
 * LED PWM and to control power to various peripherals.  The interrupt
 * facility of the extender is not connected to the nRF MCU.
 *
 * The device uses 16-bit registers for anything associated with
 * output pins.  Access them as big-endian values to use a uint16_t
 * representation.
 *
 * IOX pin assignments:
 *
 * Pin | 0            | 1           | 2           | 3            | 4
 * --: | :----------- | :---------- | :---------- | :----------- | :-----------
 *   0 | IOEXT0       | IOEXT1      | IOEXT2      | IOEXT3       | BAT_MON_EN
 *   5 | LIGHTWELL_G  | LIGHTWELL_B | LIGHTWELL_R | MPU_PWR_CTRL | MIC_PWR_CTRL
 *  10 | CCS_PWR_CTRL | CCS_RESETn  | CCS_WAKEn   | SENSE_LED_R  | SENSE_LED_G
 *  15 | SENSE_LED B
 *
 */

#ifndef NRFCXX_BOARD_HPP
#define NRFCXX_BOARD_HPP
#pragma once

namespace nrfcxx {

namespace periph {
// Forward declaration
class TWI;
} // ns periph

namespace board {

#define NRFCXX_BOARD_PSEL_XL1 0
#define NRFCXX_BOARD_PSEL_XL2 1
#define NRFCXX_BOARD_PSEL_ANADIG0 2
#define NRFCXX_BOARD_PSEL_ANADIG1 3
#define NRFCXX_BOARD_PSEL_ANADIG2 4
#define NRFCXX_BOARD_PSEL_IOX_OSCIO 5
#define NRFCXX_BOARD_PSEL_MPU_INT 6
#define NRFCXX_BOARD_PSEL_TWI0_SDA_INT 7
#define NRFCXX_BOARD_PSEL_TWI0_SCL_INT 8
#define NRFCXX_BOARD_PSEL_NFC1 9
#define NRFCXX_BOARD_PSEL_NFC2 10
#define NRFCXX_BOARD_PSEL_BUTTON0 11
#define NRFCXX_BOARD_PSEL_LIS_INT1 12
#define NRFCXX_BOARD_PSEL_USB_DETECT 13
#define NRFCXX_BOARD_PSEL_TWI0_SDA 14 // external
#define NRFCXX_BOARD_PSEL_TWI0_SCL 15 // external
#define NRFCXX_BOARD_PSEL_IOX_RESETn 16
#define NRFCXX_BOARD_PSEL_BAT_CHG_STAT 17
#define NRFCXX_BOARD_PSEL_MOS_1 18
#define NRFCXX_BOARD_PSEL_MOS_2 19
#define NRFCXX_BOARD_PSEL_MOS_3 20
#define NRFCXX_BOARD_PSEL_MOS_4 21
#define NRFCXX_BOARD_PSEL_CCS_INT 22
#define NRFCXX_BOARD_PSEL_LPS_INT 23
#define NRFCXX_BOARD_PSEL_HTS_INT 24
#define NRFCXX_BOARD_PSEL_MIC_DOUT 25
#define NRFCXX_BOARD_PSEL_MIC_CLK 26
#define NRFCXX_BOARD_PSEL_SPEAKER 27
#define NRFCXX_BOARD_PSEL_BATTERY 28
#define NRFCXX_BOARD_PSEL_SPK_PWR_CTRL 29
#define NRFCXX_BOARD_PSEL_VDD_PWR_CTRL 30
#define NRFCXX_BOARD_PSEL_BH_INT 31

/* Battery is sampled through AIN3 (P0.5) with a voltage divider of
 * 806K over 2M1. */
#define NRFCXX_BOARD_BATTERY_AIN 4
#define NRFCXX_BOARD_BATTERY_R1 1500000U
#define NRFCXX_BOARD_BATTERY_R2 180000U

#define NRFCXX_BOARD_PSEL_UART0_RXD NRFCXX_BOARD_PSEL_ANADIG0
#define NRFCXX_BOARD_PSEL_UART0_TXD NRFCXX_BOARD_PSEL_ANADIG1
#define NRFCXX_BOARD_PSEL_SCOPE0 NRFCXX_BOARD_PSEL_ANADIG2

#define NRFCXX_BOARD_IOX_EXT0 0          // OUT CLR
#define NRFCXX_BOARD_IOX_EXT1 1          // OUT CLR
#define NRFCXX_BOARD_IOX_EXT2 2          // OUT CLR
#define NRFCXX_BOARD_IOX_EXT3 3          // OUT CLR
#define NRFCXX_BOARD_IOX_BAT_MON_EN 4    // IN NOPULL
#define NRFCXX_BOARD_IOX_LIGHTWELL_G 5   // OUT SET
#define NRFCXX_BOARD_IOX_LIGHTWELL_B 6   // OUT SET
#define NRFCXX_BOARD_IOX_LIGHTWELL_R 7   // OUT SET
#define NRFCXX_BOARD_IOX_MPU_PWR_CTRL 8  // OUT CLR
#define NRFCXX_BOARD_IOX_MIC_PWR_CTRL 9  // OUT CLR
#define NRFCXX_BOARD_IOX_CCS_PWR_CTRL 10 // OUT CLR
#define NRFCXX_BOARD_IOX_CCS_RESETn 11   // IN PULLDOWN
#define NRFCXX_BOARD_IOX_CCS_WAKEn 12    // IN PULLDOWN
#define NRFCXX_BOARD_IOX_SENSE_LED_R 13  // OUT SET
#define NRFCXX_BOARD_IOX_SENSE_LED_G 14  // OUT SET
#define NRFCXX_BOARD_IOX_SENSE_LED_B 15  // OUT SET

constexpr bool button_active_low = true;
constexpr bool has_lfxt = true;
constexpr bool led_active_low = true;

/** Access the TWI peripheral (TWI0) used for internal devices.
 *
 * @warning This function powers and configures the SX1509B IO
 * extender on its first reference.  Invocation must follow
 * board::initialize(). */
periph::TWI& twi () noexcept;

/** Configure all NRF52832 GPIOs to their nominal default operational
 * state. */
void gpio_startup ();

/** Same as gpio_startup(), but configures the pins in reverse order.
 *
 * This is what the reference implementation does.  No idea why. */
void gpio_shutdown ();

/* Most IOX pins are output, some set and some clear.  The inputs
 * are either no-pull or pulldown.
 *
 * The reference implementation is insanely complex.  Relevant flags
 * and their effects are:
 *
 * * Variation among DIR_INPUT, DIR_OUTPUT.  This is RegDir, which
 *   resets to all set (input).
 * * Everything is INPUT_BUF_ENABLED.  This is RegInputDisable
 *   which resets to zero (enabled).  No action required on reset,
 *   but the value is changed for the LED driver.
 * * Variation among NOPULL, PULLDOWN, PULLUP.  These are
 *   RegPullUp and RegPullDown which reset to zero (disabled).
 * * Everything is DRIVE_PUSHPULL.  This is RegOpenDrain which resets
 *   to zero (pushpull).  No action required on reset, but the value
 *   is changed for the LED driver.
 * * Everything is INCREASED_SLEWRATE_DISABLED.  This is RegLongSlew
 *   which resets to zero (disabled).  No action required on reset,
 *   and AFAICT the value is never changed by reference
 *   implementation.
 * * Variation among NOOUTPUT, SET, CLEAR.  This is RegData, which
 *   resets to all set (high).  Set only the ones that are output
 *   set. */

/** Bit set identifying IO extender pins that initialize to output
 * clear.
 *
 * Pass to misc::sx1509b::multiconfigure() as necessary. */
constexpr uint16_t iox_out_clear = 0
  | (1U << NRFCXX_BOARD_IOX_EXT0)
  | (1U << NRFCXX_BOARD_IOX_EXT1)
  | (1U << NRFCXX_BOARD_IOX_EXT2)
  | (1U << NRFCXX_BOARD_IOX_EXT3)
  | (1U << NRFCXX_BOARD_IOX_MPU_PWR_CTRL)
  | (1U << NRFCXX_BOARD_IOX_MIC_PWR_CTRL)
  | (1U << NRFCXX_BOARD_IOX_CCS_PWR_CTRL)
  | (1U << NRFCXX_BOARD_IOX_MPU_PWR_CTRL)
  ;

/** Bit set identifying IO extender pins that initialize to output
 * set.
 *
 * Pass to misc::sx1509b::multiconfigure() as necessary. */
constexpr uint16_t iox_out_set = 0
  | (1U << NRFCXX_BOARD_IOX_LIGHTWELL_G)
  | (1U << NRFCXX_BOARD_IOX_LIGHTWELL_B)
  | (1U << NRFCXX_BOARD_IOX_LIGHTWELL_R)
  | (1U << NRFCXX_BOARD_IOX_SENSE_LED_G)
  | (1U << NRFCXX_BOARD_IOX_SENSE_LED_B)
  | (1U << NRFCXX_BOARD_IOX_SENSE_LED_R)
  ;

/** Bit set identifying IO extender pins that initialize to pull down.
 *
 * All other bits have no pull.
 *
 * Pass to misc::sx1509b::multiconfigure() as necessary. */
constexpr uint16_t iox_in_pulldown = 0
  | (1U << NRFCXX_BOARD_IOX_CCS_RESETn)
  | (1U << NRFCXX_BOARD_IOX_CCS_WAKEn)
  ;

} // namespace board
} // namespace
#endif /* NRFCXX_BOARD_HPP */
