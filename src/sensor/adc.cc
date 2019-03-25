// SPDX-License-Identifier: Apache-2.0
// Copyright 2012-2019 Peter A. Bigot

#include <climits>
#include <cmath>

#include <nrfcxx/sensor/adc.hpp>

namespace nrfcxx {
namespace sensor {
namespace adc {

int
calibrator::configure_bi_ ()
{
  int rv = 0;
#if !(NRF51 - 0)
  auto config = config_ | configure_resolution_bi_();
  for (auto ci = 0U; ci < nrf5::SAADC.AUX; ++ci) {
    nrf5::SAADC->CH[ci].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
    if (0 == ci) {
      nrf5::SAADC->CH[ci].CONFIG = config;
      nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_VDD << SAADC_CH_PSELP_PSELP_Pos;
    } else {
      nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
    }
  }
  peripheral::setup_result_bi(&result_, 1);
#endif
  return rv;
}

int
calibrator::alarm_cb_ (clock::alarm& alarm)
{
  auto self = reinterpret_cast<calibrator *>(alarm.metadata);
  (void)self->lpsm_sample();
  return true;
}

calibrator::calibrator (notifier_type notify,
                        unsigned int interval_s,
                        uint16_t delta_cCel,
                        uint32_t config) :
    super{notify},
    alarm_{alarm_cb_, this},
    config_{config},
    delta_cCel_{delta_cCel}
{
#if (NRF51 - 0)
  /* nRF51 doesn't support calibration so do nothing. */
  config_ = 0;
#else
  if (0 == interval_s) {
    interval_s = DEFAULT_INTERVAL_s;
  }
  if (!config_) {
    config_ = peripheral::make_config();
  }
  alarm_.set_interval(interval_s * clock::uptime::Frequency_Hz);
#endif
  if (config_) {
    latest_cCel_ = systemState::die_cCel();
    queue_calibration_();
  }
}

int
calibrator::queue_calibration_ ()
{
  {
    mutex_type mutex;
    calibrated_cCel_ = INVALID_cCel;
  }
  return queue([this]()
               {
                 complete_calibration_();
               }, [this](auto rc)
               {
                 queue_rc_ = rc;
                 if (0 > rc) {
                   calibrated_cCel_ = FAILED_cCel;
                   machine_.post_event();
                 }
               }, true);
}

void
calibrator::complete_calibration_ ()
{
  calibrated_cCel_ = latest_cCel_;
  machine_.post_event();
}

int
calibrator::recalibrate ()
{
  int rc = lpsm_sample();
  if (0 != rc) {
    pending_calibrate_ = true;
  }
  return rc;
}

int
calibrator::lpsm_process_ (int& delay,
                           process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;

  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    ENTRY_IDLE:
    case MS_ENTRY_IDLE:
      if (pending_calibrate_) {
        pending_calibrate_ = false;
        goto ENTRY_SAMPLE;
      }
      machine_.set_state(state_machine::MS_IDLE);
      [[fallthrough]]
    case state_machine::MS_IDLE:
      break;
    ENTRY_SAMPLE:
    case state_machine::MS_ENTRY_SAMPLE:
      latest_cCel_ = systemState::die_cCel();
      pf |= state_machine::PF_OBSERVATION;
      if (calibrated()
          && ((abs(calibrated_cCel_ - latest_cCel_) < delta_cCel_)
              || (!config_))) {
        goto ENTRY_IDLE;
        break;
      }
      pf |= PF_CALIBRATING;
      rc = queue_calibration_();
      if (0 > rc) {
        break;
      }
      machine_.set_state(state_machine::MS_EXIT_SAMPLE);
      [[fallthrough]]
    case state_machine::MS_EXIT_SAMPLE:
    case state_machine::MS_ENTRY_START:
      {
        mutex_type mutex;
        if (INVALID_cCel == calibrated_cCel_) {
          /* Calibration is running, event will signal completion */
          break;
        }
        if (FAILED_cCel == calibrated_cCel_) {
          rc = -EIO;
        }
      }
      if (0 > rc) {
        break;
      }
      pf |= PF_CALIBRATED;
      if (state_machine::MS_ENTRY_START == machine_.state()) {
        pf |= state_machine::PF_STARTED | state_machine::PF_OBSERVATION;
        alarm_.schedule_offset(static_cast<int>(alarm_.interval()));
      }
      goto ENTRY_IDLE;
      break;
  }
  return rc;
}

int
voltage_divider::regulator_delay (int delay_utt)
{
  int rv = -1;
  if (regulator_) {
    if (0 <= delay_utt) {
      regdelay_utt_ = delay_utt;
    }
    rv = regdelay_utt_;
  }
  return rv;
}

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
voltage_divider::sample_setup ()
{
  auto rv = super::sample_setup();
  if ((0 <= rv) && regulator_) {
    regulator_->request();
    auto delay = regulator_->delay_utt + regdelay_utt_;
    if (0 < (delay - rv)) {
      rv = delay;
    }
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

  /* Pre-calculate the series-specific CONFIG register.
   *
   * The bits corresponding to resolution should be zeroed so they can
   * be set correctly based on parameters passed to resolution().
   *
   * For nRF51 the bits corresponding to AIN selection should be zero
   * so the desired input can be set based on the channel
   * configuration. */
  if ((0 == r1_Ohm) && (0 == r2_Ohm)) {
    failsafe(FailSafeCode::API_VIOLATION);
  } else if ((0 == r1_Ohm) || (0 == r2_Ohm)) {
    flags_ |= FL_MEASURE_RESISTANCE;
    /* For resistance we want the reference to be the full-scale
     * supply.  This means scaling the input and the supply. */
#if (NRF51 - 0)
    config_ = peripheral::make_config(-1,
                                      0,
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
                                      0,
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

  auto config = configure_resolution_bi_();
#if (NRF51 - 0)
  nrf5::ADC->CONFIG = config | nrf51_next_bi_(0);
#else
  config |= config_;
  for (auto ci = 0U; ci < nrf5::SAADC.AUX; ++ci) {
    int ain = channel_ain(ci);
    nrf5::SAADC->CH[ci].CONFIG = config;
    if (0 <= ain) {
      nrf5::SAADC->CH[ci].PSELP = (SAADC_CH_PSELP_PSELP_AnalogInput0 + ain) << SAADC_CH_PSELP_PSELP_Pos;
    } else {
      nrf5::SAADC->CH[ci].PSELP = SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
    }
    nrf5::SAADC->CH[ci].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
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

uint8_t
light_intensity::intensity () const
{
  /* At 10 kOhm reference resistor the maximum measured resistance
   * would be about 650 MOhm ignoring tolerance.
   *
   * The lowest non-zero resistance is about 20 Ohm; 100 Ohm is
   * maximum brightness.
   *
   * The no-light condition is 4 GOhm, but the lowest non-zero
   * voltage produces about 100 MOhm (maximum darkness).
   *
   * We want a log representation of the potential range where low
   * is dark and high is bright; 8 bits of resolution are
   * sufficient.  So:
   *
   * SHORTED_INTENSITY for values less than 100 Ohm
   * OPEN_INTENSITY for values exceeding 100 MOhm.
   * The range [ln(10^2), ln(10^9)] maps reversed to [1, 251). */
  static constexpr auto BRIGHT_Ohm = 100U;
  static constexpr auto BRIGHT_lg = 4.60517018598809136803;
  static constexpr auto DARK_Ohm = 100'000'000U;
  static constexpr auto DARK_lg = 18.42068074395236547214;
  unsigned int light_Ohm = sample_Ohm(0, true);
  if (BRIGHT_Ohm > light_Ohm) {
    return SHORTED_INTENSITY;
  }
  if (DARK_Ohm < light_Ohm) {
    /* @todo Correction of the resistance through
     * SAADC_Peripheral::near_zero might land us here instead of
     * SHORTED for very high measured voltages.  That could probably
     * be avoided by using GAIN_Gain1_6 along with REFSEL_VDD1_4. */
    return OPEN_INTENSITY;
  }
  auto light_lg = log(light_Ohm);
  return 1U + 250U * (DARK_lg - light_lg) / (DARK_lg - BRIGHT_lg);
}

int
vdd::configure_bi_ ()
{
  vdd_adc16_ = 0;
  auto config = configure_resolution_bi_();
#if (NRF51 - 0)
  config |= peripheral::make_config(-1,
                                    0,
                                    ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling,
                                    ADC_CONFIG_REFSEL_VBG);
  nrf5::ADC->CONFIG = config;
#else
  config |= peripheral::make_config(SAADC_CH_CONFIG_REFSEL_Internal,
                                    SAADC_CH_CONFIG_GAIN_Gain1_6);
  for (auto ci = 0U; ci < nrf5::SAADC.AUX; ++ci) {
    nrf5::SAADC->CH[ci].CONFIG = config;
    if (0 == ci) {
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

bool
lpsm_wrapper::lpsm_calibrated (int v)
{
  mutex_type mutex;
  if (0 < v) {
    flags_bi_ |= FL_CALIBRATED_FOREVER;
  } else if (0 == v) {
    flags_bi_ &= ~FL_CALIBRATED_FOREVER;
  }
  return (FL_CALIBRATED_FOREVER & flags_bi_);
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
          if (flags_bi_ & (FL_CALIBRATED_FOREVER | FL_CALIBRATED_ONCE)) {
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
      machine_.set_state(MS_QUEUED);
      rc = client_.queue([this](){
          machine_.set_state(state_machine::MS_EXIT_SAMPLE, true);
        }, [this](auto rc) {
          mutex_type mutex;
          flags_bi_ &= ~(FL_PENDING | FL_CALIBRATED_ONCE);
          queue_rc_ = rc;
          if (0 > rc) {
            machine_.set_state(state_machine::MS_EXIT_SAMPLE, true);
          }
        });
      [[fallthrough]]
    case MS_QUEUED:
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
        flags_bi_ &= ~FL_CALIBRATED_ONCE;
        flags_bi_ |= FL_PENDING | FL_CALIBRATING;
      }
      queue_rc_ = -1;
      machine_.set_state(MS_QUEUED);
      rc = client_.queue([this](){
          flags_bi_ |= FL_CALIBRATED_ONCE;
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
