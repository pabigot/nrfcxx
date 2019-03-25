/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** @file
 * API specific to the nRF52 series supporting <nrfcxx/impl.hpp>.
 *
 */

#ifndef NRFCXX_NRF52_IMPL_HPP
#define NRFCXX_NRF52_IMPL_HPP
#pragma once

#if (NRF52832 - 0)
#include <nrf52_bitfields.h>
#elif (NRF52840 - 0)
#include <nrf52840_bitfields.h>
#else /* TARGET_PRODUCT */
#error Unsupported NRF52 MCU
#endif /* NRF52 */

namespace nrfcxx {
namespace nrf5 {
namespace series {

/** Constants and function specific to the nRF52 SAADC peripheral.
 *
 * @note The SAADC peripheral has much higher noise than the ADC
 * peripheral of nRF51.  Significant oversampling is required for
 * reasonable results, and it's likely the low bits should simply be
 * discarded.
 *
 * @note SAADC has a +/- 2 LSB (0.19%) error tolerance for offset at
 * 10 bit resolution.  This can cause voltages near ground to be
 * negative, which appears as extremely high voltages.  There is also
 * a +/- 3% error due to gain (+4% for gain >= 0.5).
 *
 * @note So, basically, assume you only get 9 bits out of this thing.
 *
 * @see https://devzone.nordicsemi.com/f/nordic-q-a/12287/nrf52-saadc-noise
 */
struct SAADC_Peripheral : public ADC_Base {
  using mutex_type = mutex_irq<static_cast<IRQn_Type>(nrf5::SAADC.IRQn)>;

  static constexpr IRQn_Type IRQn = SAADC_IRQn;

  /** nRF52 reference voltage is 0.6 V. */
  static constexpr unsigned int VBG_mV = 600;

  /** The value corresponding to the 9th bit of a normalized
   * acquisition.
   *
   * See above for discussion of SAADC error bounds. */
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

  /** Helper to build up a ADC channel-specific `CONFIG` value.
   *
   * @param refsel one of:
   * * `SAADC_CH_CONFIG_REFSEL_Internal` for internal 600 mV reference **default**;
   * * `SAADC_CH_CONFIG_REFSEL_VDD1_4` for VDD/4 as reference;
   *
   * @param gain one of:
   * * `SAADC_CH_CONFIG_GAIN_Gain1_6` for a gain of 1/6  **default**;
   * * `SAADC_CH_CONFIG_GAIN_Gain1_5` for a gain of 1/5;
   * * `SAADC_CH_CONFIG_GAIN_Gain1_4` for a gain of 1/4;
   * * `SAADC_CH_CONFIG_GAIN_Gain1_3` for a gain of 1/3;
   * * `SAADC_CH_CONFIG_GAIN_Gain1_2` for a gain of 1/2;
   * * `SAADC_CH_CONFIG_GAIN_Gain1` for a gain of 1;
   * * `SAADC_CH_CONFIG_GAIN_Gain2` for a gain of 2;
   * * `SAADC_CH_CONFIG_GAIN_Gain4` for a gain of 4.
   *
   * @param tacq one of:
   * * `SAADC_CH_CONFIG_TACQ_3us` for 3 us;
   * * `SAADC_CH_CONFIG_TACQ_5us` for 5 us;
   * * `SAADC_CH_CONFIG_TACQ_10us` for 10 us **default**;
   * * `SAADC_CH_CONFIG_TACQ_15us` for 15 us;
   * * `SAADC_CH_CONFIG_TACQ_20us` for 20 us;
   * * `SAADC_CH_CONFIG_TACQ_40us` for 40 us.
   *
   * @param burst one of:
   * * `false` for normal operation **default**;
   * * `true` for burst mode operation
   *
   * @param resp one of:
   * * `SAADC_CH_CONFIG_RESP_Bypass` to bypass the resistor ladder **default**;
   * * `SAADC_CH_CONFIG_RESP_Pulldown` to pull-down to GND;
   * * `SAADC_CH_CONFIG_RESP_Pullup` to pull-up to VDD;
   * * `SAADC_CH_CONFIG_RESP_VDD1_2` to set input at VDD/2.
   *
   * @param differential one of:
   * * `false` for single ended **default**;
   * * `true` for differential.
   *
   * @param resn one of:
   * * `SAADC_CH_CONFIG_RESN_Bypass` to bypass the resistor ladder **default**;
   * * `SAADC_CH_CONFIG_RESN_Pulldown` to pull-down to GND;
   * * `SAADC_CH_CONFIG_RESN_Pullup` to pull-up to VDD;
   * * `SAADC_CH_CONFIG_RESN_VDD1_2` to set input at VDD/2.
   */
  static constexpr
  unsigned int
  make_config (unsigned int refsel = SAADC_CH_CONFIG_REFSEL_Internal,
               unsigned int gain = SAADC_CH_CONFIG_GAIN_Gain1_6,
               unsigned int tacq = SAADC_CH_CONFIG_TACQ_10us,
               bool burst = false,
               unsigned int resp = SAADC_CH_CONFIG_RESP_Bypass,
               bool differential = false,
               unsigned int resn = SAADC_CH_CONFIG_RESN_Bypass)
  {
    return 0
      | (SAADC_CH_CONFIG_RESP_Msk & (resp << SAADC_CH_CONFIG_RESP_Pos))
      | (SAADC_CH_CONFIG_RESN_Msk & (resn << SAADC_CH_CONFIG_RESN_Pos))
      | (SAADC_CH_CONFIG_GAIN_Msk & (gain << SAADC_CH_CONFIG_GAIN_Pos))
      | (SAADC_CH_CONFIG_REFSEL_Msk & (refsel << SAADC_CH_CONFIG_REFSEL_Pos))
      | (SAADC_CH_CONFIG_TACQ_Msk & (tacq << SAADC_CH_CONFIG_TACQ_Pos))
      | (SAADC_CH_CONFIG_MODE_Msk & (differential << SAADC_CH_CONFIG_MODE_Pos))
      | (SAADC_CH_CONFIG_BURST_Msk & (burst << SAADC_CH_CONFIG_BURST_Pos));
  }

  static constexpr uint32_t inten = 0
    | (SAADC_INTENSET_CALIBRATEDONE_Set << SAADC_INTENSET_CALIBRATEDONE_Pos)
    | (SAADC_INTENSET_STARTED_Set << SAADC_INTENSET_STARTED_Pos)
    | (SAADC_INTENSET_DONE_Set << SAADC_INTENSET_DONE_Pos)
    | (SAADC_INTENSET_RESULTDONE_Set << SAADC_INTENSET_RESULTDONE_Pos)
    | (SAADC_INTENSET_END_Set << SAADC_INTENSET_END_Pos)
    | (SAADC_INTENSET_STOPPED_Set << SAADC_INTENSET_STOPPED_Pos);

  static bool busy ()
  {
    return nrf5::SAADC->STATUS;
  }

  /* PAN 74: SAADC: Started events fires prematurely.
   * If TACQ <= 5 us EVENTS_STARTED may be spontaneously generated after ENABLE.
   * (Workaround does not work)
   *
   * PAN 86: SAADC: Triggering START after calibration may write sample to RAM
   * After CALIBRATEDONE issue STOP and wait for STOPPED before issuing START.
   *
   * PAN 150: SAADC: EVENT_STARTED does not fire
   * Applies to PPI-initiated START with TACQ <= 5 us
   *
   * PAN 178: SAADC: END event firing too early
   * END event occurs before data ready when CALIBRATE with TACQ < 10us before SAMPLE.
   *
   * Observations:
   *
   * With TACQ <= 5 us CALIBRATE without START generates:
   * * DONE which remains asserted for 4.252 us (with TACQ 3 us)
   * * anomalous STARTED 5 us after DONE asserted (PAN 74)
   * * two more DONE events
   * * CALIBRATEDONE 64 ns after the third DONE deasserts
   * * anomalous DONE 4.248 us after CALIBRATEDONE
   *
   * With TACQ = 40 us CALIBRATE without START generates:
   * * DONE which remains asserted for 41.252 us
   * * two more DONE events
   * * CALIBRATEDONE 104 ns after third DONE deasserts
   * * anomalous DONE 41.144 us after CALIBRATEDONE
   *
   * The extra DONE event may be the fault underlying PAN 86.  If STOP
   * is issued quickly enough the extra DONE event does not happen.
   *
   * With TACQ <= 5 us START, [DONE], STARTED, CALIBRATE produces END
   * This is consistent with PAN 178.
   *
   * With TACQ = 40 us START, STARTED, CALIBRATE produces END before CALIBRATEDONE
   * This is consistent with PAN 178 except observed with long TACQ.
   *
   * Without ENABLE issuing START produces STARTED.  Issuing SAMPLE or
   * CALIBRATE has no effect.
   *
   * Workarounds:
   * * Issue CALIBRATE after ENABLE before START.
   * * Issue STOP after CALIBRATE_DONE and wait for STOPPED
   * * Ignore STARTED except after explicit START.
   */

  static void enable_bi ()
  {
    nrf5::SAADC->EVENTS_STARTED = 0;
    nrf5::SAADC->EVENTS_END = 0;
    nrf5::SAADC->EVENTS_DONE = 0;
    nrf5::SAADC->EVENTS_STOPPED = 0;
    nrf5::SAADC->EVENTS_CALIBRATEDONE = 0;
    nrf5::SAADC->RESULT.PTR = reinterpret_cast<uint32_t>(result_ptr_);
    nrf5::SAADC->RESULT.MAXCNT = result_maxcnt_;
    nrf5::SAADC->INTENSET = inten;
    nvic_ClearPendingIRQ(IRQn);
    nvic_EnableIRQ(IRQn);
    nrf5::SAADC->ENABLE = 1;
  }

  static void disable_bi ()
  {
    nrf5::SAADC->ENABLE = 0;
    nvic_DisableIRQ(IRQn);
    nrf5::SAADC->INTENCLR = inten;
  }

  static int calibrate_bi ()
  {
    enable_bi();
    calibrating_bi_ = true;
    nrf5::SAADC->TASKS_CALIBRATEOFFSET = 1;
    return 1;
  }

  static int start_bi ()
  {
    enable_bi();
    calibrating_bi_ = false;
    nrf5::SAADC->TASKS_START = 1;
    return 1;
  }

  static void stopped_bi ()
  {
    disable_bi();
  }

  /** Post-process the ADC results so they're normalized as if from a
   * 16-bit ADC.
   *
   * This is done in-place in the memory to which the results were
   * DMAd. */
  static unsigned int normalize_bi ()
  {
    volatile uint16_t* rp = result_ptr_;
    volatile uint16_t* const rpe = rp + result_maxcnt_;

    // NB: RESOLUTION is a 7-bit value, but only the low two bits
    // are defined to be supported.  If we used
    // SAADC_RESOLUTION_VAL_Msk we could end up with a negative
    // shift.
    unsigned int shift16 = 8 - 2 * ((nrf5::SAADC->RESOLUTION & 0x03) >> SAADC_RESOLUTION_VAL_Pos);
    while (rp < rpe) {
      *rp++ <<= shift16;
    }
    return *result_ptr_;
  }

  /** `true` if the ADC is performing a calibration; `false` if it is
   * performing a conversion. */
  static bool calibrating_bi_;
};

using ADC_Variant = SAADC_Peripheral;

} // ns series
} // ns nrf5


/** NVIC IRQ priority reserved for critical soft-device interrupts. */
static constexpr auto IRQ_PRIORITY_SD_HIGH = 0;

/** NVIC IRQ priority reserved for critical soft-device memory
 * isolation and runtime protection. */
static constexpr auto IRQ_PRIORITY_SD_MEMORY = 1;

/** NVIC IRQ priority reserved for critical application interrupts. */
static constexpr auto IRQ_PRIORITY_APP_HIGH = 2;

/** NVIC IRQ priority reserved for non-critical soft-device interrupts. */
static constexpr auto IRQ_PRIORITY_SD_LOW = 4;

/** NVIC IRQ priority reserved for non-critical application interrupts. */
static constexpr auto IRQ_PRIORITY_APP_LOW = 5;

} // ns nrfcxx

#endif /* NRFCXX_NRF52_IMPL_HPP */
