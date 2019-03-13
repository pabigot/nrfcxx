// SPDX-License-Identifier: Apache-2.0
// Copyright 2012-2019 Peter A. Bigot

#include <climits>
#include <cmath>

#include <nrfcxx/sensor/adc.hpp>

namespace nrfcxx {
namespace sensor {
namespace adc {

int
voltage_divider::sample_mV (size_t ci,
                            bool truncate) const
{
  int rv = -1;
  auto m16 = sample_adc16(ci);
  if (0 <= m16) {
    /* Round to zero (avoids really high voltages near SAADC zero */
    if (truncate && peripheral::near_zero(m16)) {
      m16 = 0;
    }
    rv = measurement_mV(m16);
  }
  return rv;
}

int
voltage_divider::input_mV (size_t ci) const
{
  int rv = -1;
  if ((ci < channel_count_)
      && !(FL_MEASURE_RESISTANCE & flags_)) {
    uint16_t mV = this->sample_mV(ci, true);
    rv = uint64_t{mV} * (r1_Ohm + r2_Ohm) / r2_Ohm;
  }
  return rv;
}

int
voltage_divider::sample_Ohm (size_t ci,
                             bool filter_high_negative) const
{
  int rv = -1;
  auto m16 = sample_adc16(ci);
  if ((0 <= m16)
      && (FL_MEASURE_RESISTANCE & flags_)) {
    /* Large values indistinguishable from zero shall be zero. */
    if (filter_high_negative
        && ((1 << 15) < m16)
        && peripheral::near_zero(m16)) {
      m16 = 0;
    }
    rv = measurement_Ohm(m16);
  }
  return rv;
}

int
voltage_divider::nrf51_next_bi_ (size_t ci)
{
  int rv = -1;
#if (NRF51 - 0)
  int ain = channel_ain(ci);
  if (0 <= ain) {
    rv = config_;
    rv |= (ADC_CONFIG_PSEL_AnalogInput0 << (ADC_CONFIG_PSEL_Pos + ain));
  }
#endif /* NRF51 */
  return rv;
}

void
voltage_divider::configure_instance_ (int8_t channel_count,
                                      volatile uint16_t* raw)
{
  using namespace nrfcxx;

  if (0 >= channel_count) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  channel_count_ = channel_count;

  if (raw) {
    flags_ |= FL_RAW_EXTERNAL;
    raw_.ptr = raw;
  } else if (channel_count_ > MAX_INTERNAL) {
    failsafe(FailSafeCode::API_VIOLATION);
  }

  if ((0 == r1_Ohm) && (0 == r2_Ohm)) {
    failsafe(FailSafeCode::API_VIOLATION);
  } else if ((0 == r1_Ohm) || (0 == r2_Ohm)) {
    flags_ |= FL_MEASURE_RESISTANCE;
    /* For resistance we want the reference to be the full-scale
     * supply.  This means scaling the input and the supply. */
#if (NRF51 - 0)
    config_ = peripheral::make_config(-1,
                                      ADC_CONFIG_RES_10bit,
                                      ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling,
                                      ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling);
#else
    config_ = peripheral::make_config(SAADC_CH_CONFIG_REFSEL_VDD1_4,
                                      SAADC_CH_CONFIG_GAIN_Gain1_4,
                                      SAADC_CH_CONFIG_TACQ_40us);
#endif
  } else {
    /* For voltage we use the internal reference voltage. */
#if (NRF51 - 0)
    config_ = peripheral::make_config(-1,
                                      ADC_CONFIG_RES_10bit,
                                      ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling,
                                      ADC_CONFIG_REFSEL_VBG);
#else
    config_ = peripheral::make_config(SAADC_CH_CONFIG_REFSEL_Internal,
                                      SAADC_CH_CONFIG_GAIN_Gain1_6,
                                      SAADC_CH_CONFIG_TACQ_40us);
#endif
  }
}

int
voltage_divider::configure_bi_ ()
{
  using namespace nrfcxx;

#if (NRF51 - 0)
  nrf5::ADC->CONFIG = nrf51_next_bi_(0);
#else
  nrf5::SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos;
  nrf5::SAADC->OVERSAMPLE = 4;
  size_t ci{};
  while (ci < nrf5::SAADC.AUX) {
    int ain = channel_ain(ci);
    nrf5::SAADC->CH[ci].CONFIG = config_;
    if (0 <= ain) {
      nrf5::SAADC->CH[ci].PSELP = (SAADC_CH_PSELP_PSELP_AnalogInput0 + ain) << SAADC_CH_PSELP_PSELP_Pos;
    } else {
      nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
    }
    nrf5::SAADC->CH[ci].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
    ++ci;
  }
#endif
  peripheral::setup_result_bi(raw_adc16_(), channel_count_);
  return 0;
}

namespace steinhartHart {
const SteinhartHart adafruit372fullScale{
  1.19438315714902408e-03,
  2.19852114398042317e-04,
  1.72427510143621963e-07,
  2281,                         // -40.01 Cel
  65133};                       // 200.02 Cel

const SteinhartHart adafruit372hvac{
  1.27327813409593713e-03,
  2.08479354351912715e-04,
  2.04755386136710727e-07,
  2281,                         // -40.01 Cel
  57162};                       //  75.00 Cel

const SteinhartHart adafruit372refrigerator{
  1.25041898643159623e-03,
  2.12452973989279737e-04,
  1.86925137513452905e-07,
  3919,                         // -30 Cel
  22081};                       //  10 Cel

} // ns steinhartHart

int
SteinhartHart::temperature_cK (unsigned int therm_Ohm) const
{
  double lnR = log(therm_Ohm);
  return 100 / (a + lnR * (b + c * lnR * lnR));
}

int
ntcThermistor::temperature_cCel (size_t ci)
{
  int rv = INVALID_cCel;
  auto adc16 = sample_adc16(ci);
  if (0 <= adc16) {
    rv = convert_adc16_cCel(adc16);
  }
  return rv;
}

int
ntcThermistor::convert_adc16_cCel (unsigned int therm_adc16)
{
  if (therm_adc16 <= steinhartHart->open_adc16) {
    return OPEN_cCel;
  }
  if (therm_adc16 >= steinhartHart->short_adc16) {
    return SHORT_cCel;
  }
  unsigned int therm_Ohm = measurement_Ohm(therm_adc16);
  return steinhartHart->temperature_cK(therm_Ohm) + ABSOLUTE_ZERO_cCel;
}

int
vdd::configure_bi_ ()
{
  vdd_adc16_ = 0;
#if (NRF51 - 0)
  static constexpr uint32_t config =
    peripheral::make_config(-1,
                            ADC_CONFIG_RES_10bit,
                            ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling,
                            ADC_CONFIG_REFSEL_VBG);
  nrf5::ADC->CONFIG = config;
#else
  nrf5::SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit;
  nrf5::SAADC->OVERSAMPLE = 4;
  static constexpr unsigned int config =
    peripheral::make_config(SAADC_CH_CONFIG_REFSEL_Internal,
                            SAADC_CH_CONFIG_GAIN_Gain1_6);
  for (unsigned int ci = 0; ci < nrf5::SAADC.AUX; ++ci) {
    if (0 == ci) {
      nrf5::SAADC->CH[ci].CONFIG = config;
      nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_VDD << SAADC_CH_PSELP_PSELP_Pos;
    } else {
      nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
    }
    nrf5::SAADC->CH[ci].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
  }
#endif
  peripheral::setup_result_bi(&vdd_adc16_, 1);
  return 0;
}

int
vdd::convert_adc16_mV (unsigned int vdd_adc16) const
{
  int rv = -1;
#if (NRF51 - 0)
  rv = (3 * peripheral::VBG_mV * vdd_adc16) >> 16;
#else
  rv = (6 * peripheral::VBG_mV * vdd_adc16) >> 16;
#endif
  return rv;
}

void
vdd::complete_bi_ ()
{
  if (vdd_callback_) {
    vdd_callback_(vdd_mV());
  }
}

int
lpsm_wrapper::lpsm_calibrate ()
{
  using lpm::state_machine;
  if (state_machine::MS_IDLE != machine_.state()) {
    return -1;
  }
  machine_.set_state(MS_ENTRY_CALIBRATE, true);
  return 0;
}

int
lpsm_wrapper::lpsm_bypass_calibration ()
{
  using lpm::state_machine;
  int rv = -1;
  if ((state_machine::MS_ENTRY_START == machine_.state())
      || (state_machine::MS_IDLE == machine_.state())) {
    mutex_type mutex;
    flags_bi_ |= FL_CALIBRATED;
    rv = 0;
  }
  return rv;
}

int
lpsm_wrapper::lpsm_process_ (int& delay,
                             process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc{};

  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    case state_machine::MS_IDLE:
      break;
    case state_machine::MS_ENTRY_START:
      pf |= state_machine::PF_STARTED;
      machine_.set_state(state_machine::MS_IDLE);
      break;
    case state_machine::MS_ENTRY_SAMPLE:
    case MS_ENTRY_CALIBRATE:
      rc = client_.sample_setup();
      if (0 > rc) {
        break;
      }
      if (state_machine::MS_ENTRY_SAMPLE == machine_.state()) {
        machine_.set_state(state_machine::MS_SAMPLE);
      } else {
        machine_.set_state(MS_CALIBRATE);
      }
      if (0 != rc) {
        delay = rc;
        break;
      }
      if (MS_CALIBRATE == machine_.state()) {
        goto lbl_calibrate;
      }
      [[fallthrough]]
    case state_machine::MS_SAMPLE:
    lbl_sample:
      {
        bool calibrate_first{};
        {
          mutex_type mutex;
          if (flags_bi_ & FL_CALIBRATED) {
            flags_bi_ |= FL_PENDING | FL_SAMPLING;
          } else {
            calibrate_first = true;
            flags_bi_ |= FL_THEN_SAMPLE;
          }
        }
        if (calibrate_first) {
          machine_.set_state(MS_CALIBRATE);
          goto lbl_calibrate;
        }
      }
      queue_rc_ = -1;
      rc = client_.queue([this](){
          machine_.set_state(state_machine::MS_EXIT_SAMPLE, true);
        }, [this](auto rc) {
          mutex_type mutex;
          flags_bi_ &= ~FL_PENDING;
          queue_rc_ = rc;
          if (0 > rc) {
            machine_.set_state(state_machine::MS_EXIT_SAMPLE, true);
          }
        });
      break;
    case state_machine::MS_EXIT_SAMPLE:
      {
        mutex_type mutex;
        flags_bi_ &= ~FL_SAMPLING;
      }
      if (0 > queue_rc_) {
        rc = queue_rc_;
      } else {
        client_.sample_teardown();
        pf |= state_machine::PF_OBSERVATION;
        machine_.set_state(state_machine::MS_IDLE);
      }
      break;
    case MS_CALIBRATE:
    lbl_calibrate:
      {
        mutex_type mutex;
        flags_bi_ &= ~FL_CALIBRATED;
        flags_bi_ |= FL_PENDING | FL_CALIBRATING;
      }
      queue_rc_ = -1;
      rc = client_.queue([this](){
          flags_bi_ |= FL_CALIBRATED;
          machine_.set_state(MS_EXIT_CALIBRATE, true);
        }, [this](auto rc) {
          mutex_type mutex;
          flags_bi_ &= ~FL_PENDING;
          queue_rc_ = rc;
          if (0 > rc) {
            machine_.set_state(MS_EXIT_CALIBRATE, true);
          }
        }, true);
      break;
    case MS_EXIT_CALIBRATE: {
      bool to_sample{};
      {
        mutex_type mutex;
        to_sample = flags_bi_ & FL_THEN_SAMPLE;
        flags_bi_ &= ~(FL_CALIBRATING | FL_THEN_SAMPLE);
      }
      pf |= PF_CALIBRATED;
      if (0 > queue_rc_) {
        rc = queue_rc_;
      } else if (to_sample) {
        machine_.set_state(state_machine::MS_SAMPLE);
        goto lbl_sample;
      } else {
        client_.sample_teardown();
        machine_.set_state(state_machine::MS_IDLE);
      }
      break;
    }
  }
  return rc;
}

} // ns adc
} // ns sensor
} // ns nrfcxx
