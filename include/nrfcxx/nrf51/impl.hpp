/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** @file
 * API specific to the nRF51 series supporting <nrfcxx/impl.hpp>.
 */

#ifndef NRFCXX_NRF51_IMPL_HPP
#define NRFCXX_NRF51_IMPL_HPP
#pragma once

#include <nrf51_bitfields.h>

namespace nrfcxx {
namespace nrf5 {
namespace series {

/** Constants and function specific to the nRF51 ADC peripheral. */
struct ADC_Peripheral : public ADC_Base
{
  using mutex_type = mutex_irq<static_cast<IRQn_Type>(nrf5::ADC.IRQn)>;

  static constexpr IRQn_Type IRQn = ADC_IRQn;

  /** nRF51 reference voltage is 1.2 V. */
  static constexpr unsigned int VBG_mV = 1200;

  /** ADC offset error is documented +/- 2%, with a 1.5% error for
   * VBG.  Worst case this would be around bit 5 for an 8-bit
   * acquisition.
   *
   * In practice SAADC has 9 bits of accuracy, and ADC does much
   * better than SAADC (at least for repeatability).  Make this the
   * same as SAADC. */
  static constexpr uint16_t TOLERANCE_adc16 = (1U << 7);

  /** Test whether a value appears to be indistinguishable from zero.
   *
   * This tests whether a normalized ADC result is within
   * #TOLERANCE_adc16 of zero, handling the case where the SAADC
   * result was a small negative that appears as a large unsigned
   * measurement.
   *
   * @param v_adc16 a normalized ADC result.
   *
   * @return true iff the value is within #TOLERANCE_adc16 of zero. */
  static constexpr bool near_zero (uint16_t v_adc16)
  {
    return ((TOLERANCE_adc16 > v_adc16)
            || (v_adc16 >= ((1U << 16) - TOLERANCE_adc16)));
  }

  /** Helper to build up a ADC `CONFIG`.
   *
   * @param ain a negative value to indicate the input is not from an
   * analog pin, otherwise the ordinal of the AIN used as input.
   * Non-negative values range from 0 through 7 inclusive.  When a
   * negative value is used the analog input may subsequently be
   * selected by bitwise-adding the apppriate AIN selection:
   *
   *     ADC_CONFIG_PSEL_AnalogInput0 << (ADC_CONFIG_PSEL_Pos + ain)
   *
   * @param res one of:
   * * `ADC_CONFIG_RES_8bit`
   * * `ADC_CONFIG_RES_9bit`
   * * `ADC_CONFIG_RES_10bit`
   *
   * @param prescale one of:
   * * `ADC_CONFIG_INPSEL_AnalogInputNoPrescaling`
   * * `ADC_CONFIG_INPSEL_AnalogInputTwoThirdsPrescaling`
   * * `ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling`
   * * `ADC_CONFIG_INPSEL_SupplyTwoThirdsPrescaling`
   * * `ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling`
   *
   * @param refsel one of:
   * * `ADC_CONFIG_REFSEL_VBG`
   * * `ADC_CONFIG_REFSEL_External`
   * * `ADC_CONFIG_REFSEL_SupplyOneHalfPrescaling`
   * * `ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling`
   *
   * @param extrefsel one of:
   * * `ADC_CONFIG_EXTREFSEL_None` (default)
   * * `ADC_CONFIG_EXTREFSEL_AnalogReference0`
   * * `ADC_CONFIG_EXTREFSEL_AnalogReference1`
   */
  static constexpr
  unsigned int
  make_config (int ain,
               unsigned int res,
               unsigned int prescale,
               unsigned int refsel,
               unsigned int extrefsel = ADC_CONFIG_EXTREFSEL_None)
  {
    return 0
      | (ADC_CONFIG_RES_Msk & (res << ADC_CONFIG_RES_Pos))
      | (ADC_CONFIG_INPSEL_Msk & (prescale << ADC_CONFIG_INPSEL_Pos))
      | (ADC_CONFIG_REFSEL_Msk & (refsel << ADC_CONFIG_REFSEL_Pos))
      | ((((0 <= ain) && (7 >= ain))
          ? (ADC_CONFIG_PSEL_AnalogInput0 << ain)
          : ADC_CONFIG_PSEL_Disabled) << ADC_CONFIG_PSEL_Pos)
      | (ADC_CONFIG_EXTREFSEL_Msk & (extrefsel << ADC_CONFIG_EXTREFSEL_Pos));
  }

  static constexpr uint32_t inten = 0
    | (ADC_INTENSET_END_Set << ADC_INTENSET_END_Pos);

  /** Return `true` iff the ADC has an in-progress conversion. */
  static bool busy ()
  {
    return nrf5::ADC->BUSY;
  }

  /** Initiate an ADC sample with the current configuration.
   *
   * @warning This function should only be invoked by or on account of
   * the ADC instance that currently @link configure owns@endlink the
   * ADC peripheral. */
  static void trigger ()
  {
    nrf5::ADC->TASKS_START = 1;
  }

  static void enable_bi ()
  {
    nrf5::ADC->EVENTS_END = 0;
    nrf5::ADC->INTENSET = inten;
    nvic_ClearPendingIRQ(IRQn);
    nvic_EnableIRQ(IRQn);
    nrf5::ADC->ENABLE = 1;
  }

  static void disable_bi ()
  {
    nrf5::ADC->ENABLE = 0;
    nvic_DisableIRQ(IRQn);
    nrf5::ADC->INTENCLR = inten;
  }

  static int calibrate_bi ()
  {
    return 0;
  }

  static int start_bi ()
  {
    enable_bi();
    result_idx_ = 0;
    nrf5::ADC->TASKS_START = 1;
    return 0;
  }

  static void stopped_bi ()
  {
    disable_bi();
  }

  /* nRF51 doesn't support DMA so the FLIH needs to know which sample
   * we're on so the converted value can be stored into the result
   * block, and the client can be asked to provide the configuration
   * for the next sample. */
  static uint16_t result_idx_;
};

using ADC_Variant = ADC_Peripheral;

} // ns series
} // ns nrf5

/** NVIC IRQ priority reserved for critical soft-device interrupts. */
static constexpr auto IRQ_PRIORITY_SD_HIGH = 0;

/** NVIC IRQ priority reserved for critical application interrupts. */
static constexpr auto IRQ_PRIORITY_APP_HIGH = 1;

/** NVIC IRQ priority reserved for non-critical soft-device interrupts. */
static constexpr auto IRQ_PRIORITY_SD_LOW = 2;

/** NVIC IRQ priority reserved for non-critical application interrupts. */
static constexpr auto IRQ_PRIORITY_APP_LOW = 3;

} // ns nrfcxx

#endif /* NRFCXX_NRF51_IMPL_HPP */
