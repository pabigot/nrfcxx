/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** API specific to the nRF52832 product supporting <nrfcxx/core.hpp>.
 * @file */

#ifndef NRFCXX_NRF52832_CORE_HPP
#define NRFCXX_NRF52832_CORE_HPP
#pragma once

#include <nrf52.h>

/** @cond DOXYGEN_EXCLUDE */
/* Forward declarations for IRQ handlers. */
extern "C" {
void POWER_CLOCK_IRQHandler ();
void RADIO_IRQHandler ();
void UARTE0_UART0_IRQHandler ();
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler ();
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler ();
void NFCT_IRQHandler ();
void GPIOTE_IRQHandler ();
void SAADC_IRQHandler ();
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
void COMP_LPCOMP_IRQHandler ();
void SWI0_EGU0_IRQHandler ();
void SWI1_EGU1_IRQHandler ();
void SWI2_EGU2_IRQHandler ();
void SWI3_EGU3_IRQHandler ();
void SWI4_EGU4_IRQHandler ();
void SWI5_EGU5_IRQHandler ();
void TIMER3_IRQHandler ();
void TIMER4_IRQHandler ();
void PWM0_IRQHandler ();
void PDM_IRQHandler ();
void MWU_IRQHandler ();
void PWM1_IRQHandler ();
void PWM2_IRQHandler ();
void SPIM2_SPIS2_SPI2_IRQHandler ();
void RTC2_IRQHandler ();
void I2S_IRQHandler ();
void FPU_IRQHandler ();
}
/** @endcond */

namespace nrfcxx {
namespace nrf5 {

using FICR_Type = peripheral<NRF_FICR_Type>;
static constexpr FICR_Type FICR{NRF_FICR_BASE};

using UICR_Type = peripheral<NRF_UICR_Type>;
static constexpr UICR_Type UICR{NRF_UICR_BASE};

using BPROT_Type = peripheral<NRF_BPROT_Type>;
static constexpr BPROT_Type BPROT{NRF_BPROT_BASE};

using POWER_Type = peripheral<NRF_POWER_Type>;
static constexpr POWER_Type POWER{NRF_POWER_BASE, POWER_CLOCK_IRQn};

using CLOCK_Type = peripheral<NRF_CLOCK_Type>;
static constexpr CLOCK_Type CLOCK{NRF_CLOCK_BASE, POWER_CLOCK_IRQn};

using RADIO_Type = peripheral<NRF_RADIO_Type>;
static constexpr RADIO_Type RADIO{NRF_RADIO_BASE, RADIO_IRQn};

using UARTE_Type = peripheral<NRF_UARTE_Type>;
static constexpr UARTE_Type UARTE0{NRF_UARTE0_BASE, UARTE0_UART0_IRQn};

using UART_Type = peripheral<NRF_UART_Type>;
static constexpr UART_Type UART0{NRF_UART0_BASE, UARTE0_UART0_IRQn};

using SPIM_Type = peripheral<NRF_SPIM_Type>;
static constexpr SPIM_Type SPIM0{NRF_SPIM0_BASE, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 0};
static constexpr SPIM_Type SPIM1{NRF_SPIM1_BASE, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 1};
static constexpr SPIM_Type SPIM2{NRF_SPIM2_BASE, SPIM2_SPIS2_SPI2_IRQn, 2};

using SPIS_Type = peripheral<NRF_SPIS_Type>;
static constexpr SPIS_Type SPIS0{NRF_SPIS0_BASE, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 0};
static constexpr SPIS_Type SPIS1{NRF_SPIS1_BASE, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 1};
static constexpr SPIS_Type SPIS2{NRF_SPIS2_BASE, SPIM2_SPIS2_SPI2_IRQn, 2};

using SPI_Type = peripheral<NRF_SPI_Type>;
static constexpr SPI_Type SPI0{NRF_SPI0_BASE, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 0};
static constexpr SPI_Type SPI1{NRF_SPI1_BASE, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 1};
static constexpr SPI_Type SPI2{NRF_SPI2_BASE, SPIM2_SPIS2_SPI2_IRQn, 2};

using TWIM_Type = peripheral<NRF_TWIM_Type>;
static constexpr TWIM_Type TWIM0{NRF_TWIM0_BASE, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 0};
static constexpr TWIM_Type TWIM1{NRF_TWIM1_BASE, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 1};

using TWIS_Type = peripheral<NRF_TWIS_Type>;
static constexpr TWIS_Type TWIS0{NRF_TWIS0_BASE, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 0};
static constexpr TWIS_Type TWIS1{NRF_TWIS1_BASE, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 1};

using TWI_Type = peripheral<NRF_TWI_Type>;
static constexpr TWI_Type TWI0{NRF_TWI0_BASE, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 0};
static constexpr TWI_Type TWI1{NRF_TWI1_BASE, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 1};

using NFCT_Type = peripheral<NRF_NFCT_Type>;
static constexpr NFCT_Type NFCT{NRF_NFCT_BASE, NFCT_IRQn};

using GPIOTE_Type = peripheral<NRF_GPIOTE_Type>;
static constexpr GPIOTE_Type GPIOTE{NRF_GPIOTE_BASE, GPIOTE_IRQn, GPIOTE_Type::NO_INSTANCE, 8};

using SAADC_Type = peripheral<NRF_SAADC_Type>;
static constexpr SAADC_Type SAADC{NRF_SAADC_BASE, SAADC_IRQn, SAADC_Type::NO_INSTANCE, 8};
static constexpr auto& ADCVariant = SAADC;
#define ADCSeriesVariant_IRQHandler SAADC_IRQHandler

using TIMER_Type = peripheral<NRF_TIMER_Type>;
static constexpr TIMER_Type TIMER0{NRF_TIMER0_BASE, TIMER0_IRQn, 0, 4};
static constexpr TIMER_Type TIMER1{NRF_TIMER1_BASE, TIMER1_IRQn, 1, 4};
static constexpr TIMER_Type TIMER2{NRF_TIMER2_BASE, TIMER2_IRQn, 2, 4};
static constexpr TIMER_Type TIMER3{NRF_TIMER3_BASE, TIMER3_IRQn, 3, 6};
static constexpr TIMER_Type TIMER4{NRF_TIMER4_BASE, TIMER4_IRQn, 4, 6};

using RTC_Type = peripheral<NRF_RTC_Type>;
static constexpr RTC_Type RTC0{NRF_RTC0_BASE, RTC0_IRQn, 0, 3};
static constexpr RTC_Type RTC1{NRF_RTC1_BASE, RTC1_IRQn, 1, 4};
static constexpr RTC_Type RTC2{NRF_RTC2_BASE, RTC2_IRQn, 2, 4};
#define UPTIME_RTC_IRQHandler RTC1_IRQHandler
static constexpr const RTC_Type& UPTIME_RTC{RTC1};

using TEMP_Type = peripheral<NRF_TEMP_Type>;
static constexpr TEMP_Type TEMP{NRF_TEMP_BASE, TEMP_IRQn};

using RNG_Type = peripheral<NRF_RNG_Type>;
static constexpr RNG_Type RNG{NRF_RNG_BASE, RNG_IRQn};

using ECB_Type = peripheral<NRF_ECB_Type>;
static constexpr ECB_Type ECB{NRF_ECB_BASE, ECB_IRQn};

using CCM_Type = peripheral<NRF_CCM_Type>;
static constexpr CCM_Type CCM{NRF_CCM_BASE, CCM_AAR_IRQn};

using AAR_Type = peripheral<NRF_AAR_Type>;
static constexpr AAR_Type AAR{NRF_AAR_BASE, CCM_AAR_IRQn};

using WDT_Type = peripheral<NRF_WDT_Type>;
static constexpr WDT_Type WDT{NRF_WDT_BASE, WDT_IRQn};

using QDEC_Type = peripheral<NRF_QDEC_Type>;
static constexpr QDEC_Type QDEC{NRF_QDEC_BASE, QDEC_IRQn};

using COMP_Type = peripheral<NRF_COMP_Type>;
static constexpr COMP_Type COMP{NRF_COMP_BASE, COMP_LPCOMP_IRQn};

using LPCOMP_Type = peripheral<NRF_LPCOMP_Type>;
static constexpr LPCOMP_Type LPCOMP{NRF_LPCOMP_BASE, COMP_LPCOMP_IRQn};

using SWI_Type = peripheral<NRF_SWI_Type>;
static constexpr SWI_Type SWI0{NRF_SWI0_BASE, SWI0_EGU0_IRQn, 0};
static constexpr SWI_Type SWI1{NRF_SWI1_BASE, SWI1_EGU1_IRQn, 1};
static constexpr SWI_Type SWI2{NRF_SWI2_BASE, SWI2_EGU2_IRQn, 2};
static constexpr SWI_Type SWI3{NRF_SWI3_BASE, SWI3_EGU3_IRQn, 3};
static constexpr SWI_Type SWI4{NRF_SWI4_BASE, SWI4_EGU4_IRQn, 4};
static constexpr SWI_Type SWI5{NRF_SWI5_BASE, SWI5_EGU5_IRQn, 5};

using EGU_Type = peripheral<NRF_EGU_Type>;
static constexpr EGU_Type EGU0{NRF_EGU0_BASE, SWI0_EGU0_IRQn, 0};
static constexpr EGU_Type EGU1{NRF_EGU1_BASE, SWI1_EGU1_IRQn, 1};
static constexpr EGU_Type EGU2{NRF_EGU2_BASE, SWI2_EGU2_IRQn, 2};
static constexpr EGU_Type EGU3{NRF_EGU3_BASE, SWI3_EGU3_IRQn, 3};
static constexpr EGU_Type EGU4{NRF_EGU4_BASE, SWI4_EGU4_IRQn, 4};
static constexpr EGU_Type EGU5{NRF_EGU5_BASE, SWI5_EGU5_IRQn, 5};

using PWM_Type = peripheral<NRF_PWM_Type>;
static constexpr PWM_Type PWM0{NRF_PWM0_BASE, PWM0_IRQn, 0};
static constexpr PWM_Type PWM1{NRF_PWM1_BASE, PWM1_IRQn, 1};
static constexpr PWM_Type PWM2{NRF_PWM2_BASE, PWM2_IRQn, 2};

using PDM_Type = peripheral<NRF_PDM_Type>;
static constexpr PDM_Type PDM0{NRF_PDM_BASE, PDM_IRQn};

using NVMC_Type = peripheral<NRF_NVMC_Type>;
static constexpr NVMC_Type NVMC{NRF_NVMC_BASE};

struct PPI_Type : public peripheral<NRF_PPI_Type>
{
  /** The number of configurable channels.
   *
   * This value is also available in the instance `AUX` field. */
  static constexpr size_t NUM_CHANNELS = 20;

  /** Bit-mask type to record sets of configurable channels. */
  using channel_set_type = uint32_t;

  /** The number of channel groups. */
  static constexpr size_t NUM_GROUPS = 4;

  /** Bit-mask type to record sets of configurable channels. */
  using group_set_type = uint8_t;

  constexpr PPI_Type () :
    peripheral<NRF_PPI_Type>{NRF_PPI_BASE, PPI_Type::NO_IRQ, PPI_Type::NO_INSTANCE, NUM_CHANNELS}
  { }
};
static constexpr PPI_Type PPI{};

using MWU_Type = peripheral<NRF_MWU_Type>;
static constexpr MWU_Type MWU{NRF_MWU_BASE, MWU_IRQn};

using I2S_Type = peripheral<NRF_I2S_Type>;
static constexpr I2S_Type I2S{NRF_I2S_BASE, I2S_IRQn};

#if 0
/* Nordic defines their macro as FPU instead of NRF_FPU */
using FPU_Type = peripheral<NRF_FPU_Type>;
static constexpr FPU_Type FPU{NRF_FPU_BASE, FPU_IRQn};
#endif

using GPIO_Type = peripheral<NRF_GPIO_Type>;
static constexpr GPIO_Type P0{NRF_P0_BASE, GPIO_Type::NO_IRQ, 0, 32};
static constexpr int GPIO_PSEL_COUNT = 32;
static constexpr const GPIO_Type& GPIO{P0};

template <>
struct GPIO_Instance<0>
{
  static constexpr const GPIO_Type& peripheral{P0};
  static constexpr int begin_psel = 0;
  static constexpr int end_psel = begin_psel + P0.AUX;
};

} // ns nrf5
} // ns nrfcxx

#endif /* NRFCXX_NRF52832_HPP */
