/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** API specific to the nRF51 series supporting <nrfcxx/core.hpp>.
 *
 * @file */

#ifndef NRFCXX_NRF51_CORE_HPP
#define NRFCXX_NRF51_CORE_HPP
#pragma once

#include <nrf51.h>

/** @cond DOXYGEN_EXCLUDE */
/* Forward declarations for IRQ handlers. */
extern "C" {
void POWER_CLOCK_IRQHandler ();
void RADIO_IRQHandler ();
void UART0_IRQHandler ();
void SPI0_TWI0_IRQHandler ();
void SPI1_TWI1_IRQHandler ();
void GPIOTE_IRQHandler ();
void ADC_IRQHandler ();
void TIMER0_IRQHandler ();
void TIMER1_IRQHandler ();
void TIMER2_IRQHandler ();
void RTC0_IRQHandler ();
void TEMP_IRQHandler ();
void RNG_IRQHandler ();
void ECB_IRQHandler ();
void CCM_AAR_IRQHandler ();
void WDT_IRQHandler ();
void RTC1_IRQHandler ();
void QDEC_IRQHandler ();
void LPCOMP_IRQHandler ();
void SWI0_IRQHandler ();
void SWI1_IRQHandler ();
void SWI2_IRQHandler ();
void SWI3_IRQHandler ();
void SWI4_IRQHandler ();
void SWI5_IRQHandler ();
}
/** @endcond */

namespace nrfcxx {
namespace nrf5 {
namespace series {

/** CPU clock speed in MHz. */
static constexpr unsigned int CLOCK_MHz = 16;

/** Overhead setting up the call to delay_cycles() in the standard
 * delay_us() implementation.
 *
 * The value must be strictly less than CLOCK_MHz.
 *
 * This value has been estimated from biased measurement, and is
 * conservative (smaller than the actual overhead).  Note that the
 * resolution of the used value depends on the number of cycles per
 * delay_cycles() iteration: at the nRF51 value of 4 there is no
 * difference between 8 and 11.  11 is chosen because the measured
 * overhead for single-pass loop is about 820 ns, with the overhead of
 * the scope operation probably around 125 ns.  11 is 687.5 ns which
 * is close to the difference without exceeding it. */
static constexpr unsigned int DELAY_US_OVERHEAD_cyc = 11;

/** Loop to delay for a requested number of cycles. */
void delay_cycles (unsigned int cycles);

} // ns series

using POWER_Type = nrf5::peripheral<NRF_POWER_Type>;
static constexpr POWER_Type POWER{NRF_POWER_BASE, POWER_CLOCK_IRQn};

using CLOCK_Type = peripheral<NRF_CLOCK_Type>;
static constexpr CLOCK_Type CLOCK{NRF_CLOCK_BASE, POWER_CLOCK_IRQn};

using MPU_Type = peripheral<NRF_MPU_Type>;
static constexpr MPU_Type MPU{NRF_MPU_BASE};

using RADIO_Type = peripheral<NRF_RADIO_Type>;
static constexpr RADIO_Type RADIO{NRF_RADIO_BASE, RADIO_IRQn};

using UART_Type = peripheral<NRF_UART_Type>;
static constexpr UART_Type UART0{NRF_UART0_BASE, UART0_IRQn, 0};

using SPI_Type = peripheral<NRF_SPI_Type>;
static constexpr SPI_Type SPI0{NRF_SPI0_BASE, SPI0_TWI0_IRQn, 0};
static constexpr SPI_Type SPI1{NRF_SPI1_BASE, SPI1_TWI1_IRQn, 1};

using TWI_Type = peripheral<NRF_TWI_Type>;
static constexpr TWI_Type TWI0{NRF_TWI0_BASE, SPI0_TWI0_IRQn, 0};
static constexpr TWI_Type TWI1{NRF_TWI1_BASE, SPI1_TWI1_IRQn, 1};

using SPIS_Type = peripheral<NRF_SPIS_Type>;
static constexpr SPI_Type SPIS1{NRF_SPIS1_BASE, SPI1_TWI1_IRQn, 1};

using GPIOTE_Type = peripheral<NRF_GPIOTE_Type>;
static constexpr GPIOTE_Type GPIOTE{NRF_GPIOTE_BASE, GPIOTE_IRQn, GPIOTE_Type::NO_INSTANCE, 4};

using ADC_Type = peripheral<NRF_ADC_Type>;
static constexpr ADC_Type ADC{NRF_ADC_BASE, ADC_IRQn};
static constexpr auto& ADCVariant = ADC;
#define ADCSeriesVariant_IRQHandler ADC_IRQHandler

using TIMER_Type = peripheral<NRF_TIMER_Type>;
static constexpr TIMER_Type TIMER0{NRF_TIMER0_BASE, TIMER0_IRQn, 0, 4};
static constexpr TIMER_Type TIMER1{NRF_TIMER1_BASE, TIMER1_IRQn, 1, 4};
static constexpr TIMER_Type TIMER2{NRF_TIMER2_BASE, TIMER2_IRQn, 2, 4};

using RTC_Type = peripheral<NRF_RTC_Type>;
static constexpr RTC_Type RTC0{NRF_RTC0_BASE, RTC0_IRQn, 0, 3};
static constexpr RTC_Type RTC1{NRF_RTC1_BASE, RTC1_IRQn, 0, 4};
#define UPTIME_RTC_IRQHandler RTC1_IRQHandler
static constexpr const RTC_Type& UPTIME_RTC{RTC1};

using TEMP_Type = peripheral<NRF_TEMP_Type>;
static constexpr TEMP_Type TEMP{NRF_TEMP_BASE, TEMP_IRQn};

using RNG_Type = peripheral<NRF_RNG_Type>;
static constexpr RNG_Type RNG{NRF_RNG_BASE, RNG_IRQn};

using ECB_Type = peripheral<NRF_ECB_Type>;
static constexpr ECB_Type ECB{NRF_ECB_BASE, ECB_IRQn};

using AAR_Type = peripheral<NRF_AAR_Type>;
static constexpr AAR_Type AAR{NRF_AAR_BASE, CCM_AAR_IRQn};

using CCM_Type = peripheral<NRF_CCM_Type>;
static constexpr CCM_Type CCM{NRF_CCM_BASE, CCM_AAR_IRQn};

using WDT_Type = peripheral<NRF_WDT_Type>;
static constexpr WDT_Type WDT{NRF_WDT_BASE, WDT_IRQn};

using QDEC_Type = peripheral<NRF_QDEC_Type>;
static constexpr QDEC_Type QDEC{NRF_QDEC_BASE, QDEC_IRQn};

using LPCOMP_Type = peripheral<NRF_LPCOMP_Type>;
static constexpr LPCOMP_Type LPCOMP{NRF_LPCOMP_BASE, LPCOMP_IRQn};

using SWI_Type = peripheral<NRF_SWI_Type>;
static constexpr SWI_Type SWI{NRF_SWI_BASE};

using NVMC_Type = peripheral<NRF_NVMC_Type>;
static constexpr NVMC_Type NVMC{NRF_NVMC_BASE};

struct PPI_Type : public peripheral<NRF_PPI_Type> {
  /** The number of configurable channels.
   *
   * This value is also available in the instance `AUX` field. */
  static constexpr size_t NUM_CHANNELS = 16;

  /** Bit-mask type to record sets of configurable channels. */
  using channel_set_type = uint16_t;

  /** The number of channel groups. */
  static constexpr size_t NUM_GROUPS = 4;

  /** Bit-mask type to record sets of configurable channels. */
  using group_set_type = uint8_t;

  constexpr PPI_Type () :
    peripheral<NRF_PPI_Type>{NRF_PPI_BASE, PPI_Type::NO_IRQ, PPI_Type::NO_INSTANCE, NUM_CHANNELS}
  { }
};
static constexpr PPI_Type PPI{};

using FICR_Type = peripheral<NRF_FICR_Type>;
static constexpr FICR_Type FICR{NRF_FICR_BASE};

using UICR_Type = peripheral<NRF_UICR_Type>;
static constexpr UICR_Type UICR{NRF_UICR_BASE};

using GPIO_Type = peripheral<NRF_GPIO_Type>;
/** The number of GPIO PSEL ordinals supported by the platform.
 *
 * 32 normally, 48 on NRF52840. */
static constexpr int GPIO_PSEL_COUNT = 32;
static constexpr GPIO_Type GPIO{NRF_GPIO_BASE, GPIO_Type::NO_IRQ, 0, 32};

template <>
struct GPIO_Instance<0> {
  static constexpr const GPIO_Type& peripheral{GPIO};
  static constexpr int begin_psel = 0;
  static constexpr int end_psel = begin_psel + GPIO.AUX;
};

} // ns nrf5
} // ns nrfcxx

#endif /* NRFCXX_NRF51_CORE_HPP */
