// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/crc.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/sd/beacon.hpp>
#include <nrfcxx/sensor/utils.hpp>

// CRC-16/DNP
using crc_type = nrfcxx::crc::crc16dnp_type;

#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_AUX2 NRFCXX_BOARD_PSEL_SCOPEn

namespace nrfcxx {
namespace sd {

namespace {

gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;
gpio::instr_psel<INSTR_PSEL_AUX2> instr_aux2;

static constexpr auto& crc_tabler = crc::crc16dnp;

template <typename T>
T extract_le (const uint8_t* &bp,
              size_t span = sizeof(T))
{
  using namespace pabigot::byteorder;
  T rv{};
  // If we're copying out the low bits and have a non-trivial byte
  // swap there'll be trouble.
  static_assert((host_byte_order() == byte_order_enum::little_endian),
                "Byte-order swap on non-native spans is incorrect.");
  memmove(&rv, bp, span);
  bp += span;
  return host_x_le(rv);
}

int
temp_changed (int16_t ta,
              int16_t tb,
              unsigned int thr)
{
  using nrfcxx::sensor::is_stdenv_valid;

  bool inva = !is_stdenv_valid(ta);
  bool invb = !is_stdenv_valid(tb);
  if (inva != invb) {
    /* One valid, one not valid. */
    return -100;
  }
  if (inva) {
    /* Both invalid, maybe in different ways */
    return (ta != tb) ? -200 : 0;
  }
  /* Both valid, check threshold. */
  return (thr <= abs(ta - tb)) ? -300 : 0;
}

} // anonymous

notifier_type Beacon::notify_;
Beacon* Beacon::active_;
Beacon::queue_type Beacon::readyq_;

#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
uint8_t Beacon::handle_ = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_

__attribute__((__section__(".noinit.nrfcxx.sd.Beacon.telemetry_state")))
Beacon::telemetry_state_type Beacon::telemetry_state_;

void
Beacon::telemetry_state_setup (const systemState::state_type& ss,
                               bool is_reset,
                               bool retained)
{
  instr_aux.enable();
  instr_aux2.enable();
  if (is_reset) {
  } else if (retained) {
    // Update from valid retained state.
    telemetry_state_.total_uptime += ss.last_uptime;
    telemetry_state_.previous_adv += telemetry_state_.current_adv;
    telemetry_state_.current_adv = 0;
  } else {
    memset(&telemetry_state_, 0, sizeof(telemetry_state_));
  }
}

Beacon::crc_type
Beacon::update_crc_ (uint8_t* sp, size_t span)
{
  auto crc = crc_tabler.finalize(crc_tabler.append(sp, sp + span));
  crc_tabler.store(crc, sp + span);
  return crc;
}

int
Beacon::validate () const
{
  if (0 == min_interval_) {
    return -1;
  }
  if (min_interval_ > max_interval_) {
    return -2;
  }
  return 0;
}

int
Beacon::process_event ()
{
  if (active_) {
    return -EBUSY;
  }
  {
    clock::alarm::mutex_type block;
    active_ = readyq_.unlink_front();
  }
  if (!active_) {
    return 0;
  }

  unsigned int err = 0;
  bool skip = false;
  using namespace pabigot::ble::gap;
  adv_data ad{active_->pdu_};
  do {
    ad.reset();
    ad.set_Flags(active_->dt_flags_);
#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
    err = sd_ble_gap_tx_power_set(active_->dt_tx_power_);
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
    if (BLE_GAP_ADV_SET_HANDLE_NOT_SET != handle_) {
      err = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
                                    handle_,
                                    active_->dt_tx_power_);
    }
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
    if (err) {
      break;
    }
    if (0 != active_->dt_tx_power_) {
      ad.set_TXPowerLevel(active_->dt_tx_power_);
    }
    int rc = active_->populate_(ad);
    if (0 > rc) {
      // cancelled by implementation
      err = -rc;
    } else if (0 < rc) {
      // beacon thinks it's ready, but might have screwed up.
      if (!ad.valid()) {
        err = EINVAL;
      }
    } else { // rc == 0
      // no transmission this time
      skip = true;
    }
    if (err || skip) {
      break;
    }

    ble_gap_adv_params_t prm{};
#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
    prm.type = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
    prm.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
    if (FDT_LE_NON_DISCOVERABLE != active_->dt_tx_power_) {
      // set connectable?
    }
    prm.interval = 0x4000; // 625 us ticks = 10.24 s

#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
    err = sd_ble_gap_adv_data_set(ad.data(), ad.size(), NULL, 0);
    if (!err) {
      err = sd_ble_gap_adv_start(&prm);
    }
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
    {
      ble_gap_adv_data_t gad{};
      gad.adv_data.p_data = active_->pdu_.data();
      gad.adv_data.len = ad.size();

      err = sd_ble_gap_adv_set_configure(&handle_, &gad, &prm);
    }
    if (!err) {
      err = sd_ble_gap_adv_start(handle_, BLE_CONN_CFG_TAG_DEFAULT);
    }
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
    if (!err) {
      if (active_->tx_notify_) {
        active_->tx_notify_();
      }
      telemetry_state_.current_adv += 1;
    }
  } while (false);
  if (err) {
    //printf("** Beacon %p failed: %x\n", active_, err);
    active_->state_ = FAILED;
    active_ = nullptr;
    if (!readyq_.empty()) {
      notify_();
    }
  } else {
    active_->state_ = ACTIVE;
    // If we were told not to transmit this time, just put active_
    // back on the queue.  Otherwise it'll signal when there's
    // something left to do.
    if (skip) {
      requeue_active_();
    } else {
      active_->tx_count_ += 1;
    }
  }
  return active_ ? 1 : 0;
}

Beacon*
Beacon::reschedule_ ()
{
  if (CANCELLED == state_) {
    state_ = INACTIVE;
  } else if (ACTIVE == state_) {
    if (interval_ < min_interval_) {
      interval_ = min_interval_;
    } else if (interval_ < max_interval_) {
      interval_ += interval_;
    }
    if (interval_ > max_interval_) {
      interval_ = max_interval_;
    }
    last_due_ = alarm_.deadline();
    auto delay = interval_;
    if (prepare_notify_) {
      state_ = SCHEDULED_PREPARE;
      delay -= prepare_backoff_;
    } else {
      state_ = SCHEDULED;
    }
    alarm_
      .set_deadline(last_due_ + delay)
      .schedule();
  }
  return nullptr;
}

int
Beacon::reset_interval ()
{
  int rv = -1;

  if (SCHEDULED_PREPARE <= state_) {
    /* SCHEDULED_PREPARE, SCHEDULED, READY, or ACTIVE.  It's
     * SCHEDULED_PREPARE or SCHEDULED only if cancelling the alarm
     * indicates the alarm was scheduled.
     *
     * If the alarm was scheduled, we need to reschedule it.  If the
     * time since the last fire is less than the minimum interval
     * schedule at the minimum interval after that last fire,
     * otherwise schedule it now.  The rescheduled transmission will
     * occur without a prepare notification. */
    rv = (clock::alarm::state_type::ST_scheduled == alarm_.cancel());
    if (rv) {
      unsigned int now = clock::uptime::now();
      unsigned int since = (now - last_due_);
      last_due_ = now;
      if (min_interval_ > since) {
        last_due_ += since;
      }
      state_ = SCHEDULED;
      interval_ = 0;
      alarm_
        .set_deadline(last_due_)
        .schedule();
    }
    interval_ = 0;
  }
  return rv;
}

int
Beacon::set_interval (unsigned int min_utt,
                      unsigned int max_utt)
{
  auto ins = state_;
  if ((INVALID != ins)
      && (INACTIVE != ins)) {
    return -1;
  }
  if (0 == min_utt) {
    return -2;
  }
  if ((0 < prepare_backoff_)
      && (prepare_backoff_ >= min_utt)) {
    return -3;
  }
  if (min_utt > max_utt) {
    return -4;
  }
  min_interval_ = min_utt;
  max_interval_ = max_utt;
  if ((INVALID == ins)
      && (0 == validate())) {
    state_ = INACTIVE;
  }
  return 0;
}

int
Beacon::set_prepare_backoff (notifier_type notify,
                             unsigned int backoff_utt)
{
  auto ins = state_;
  if ((INVALID != ins)
      && (INACTIVE != ins)) {
    return -1;
  }
  if (!notify) {
    prepare_notify_ = {};
    prepare_backoff_ = 0;
    return 0;
  }
  if (backoff_utt >= min_interval_) {
    return -2;
  }
  prepare_notify_ = notify;
  prepare_backoff_ = backoff_utt;
  return 1;
}

void
Beacon::requeue_active_ ()
{
  /* Reschedule and clear active (systemd idiom).  The beacon may be
   * moved to the ready queue, but nothing moves to active_() until
   * process() is re-invoked. */
  active_ = active_->reschedule_();
  // Emit another notify event if there's something already waiting.
  if (!readyq_.empty()) {
    notify_();
  }
}

int
Beacon::process_completion ()
{
  if (!active_) {
    return -1;
  }
  unsigned int err = 0;
#if (NRFCXX_SOFTDEVICE_IS_S130 - 0)
  err = sd_ble_gap_adv_stop();
#elif ((NRFCXX_SOFTDEVICE_IS_S132 - 0) || (NRFCXX_SOFTDEVICE_IS_S140 - 0))
  err = sd_ble_gap_adv_stop(handle_);
#else // NRFCXX_SOFTDEVICE_IS_
#error softdevice not supported
#endif // NRFCXX_SOFTDEVICE_IS_
  // @todo something with error?

  requeue_active_();
  return -err;
}

bool
Beacon::alarm_callback (clock::alarm& alarm)
{
  auto bp = static_cast<Beacon*>(alarm.metadata);
  auto state = bp->state_;
  if (SCHEDULED_PREPARE == state) {
    bp->prepare_notify_();
    alarm.set_deadline(bp->last_due_ + bp->interval_);
    bp->state_ = SCHEDULED;
    return true;
  }
  if (SCHEDULED == state) {
    readyq_.link_back(*bp);
    bp->state_ = READY;
    if (!active_) {
      notify_();
    }
  }
  return false;
}

void
Beacon::checked_notify_ ()
{
  if (!notify_) {
    failsafe(FailSafeCode::INCOMPLETE_SETUP);
  }
  notify_();
}

Beacon::Beacon () :
  alarm_{alarm_callback, this}
{ }

int
Beacon::activate ()
{
  if (INACTIVE != state_) {
    return -1;
  }
  int rc = pre_activate_();
  if (0 != rc) {
    return rc;
  }
  interval_ = 0;
  tx_count_ = 0;
  state_ = prepare_notify_ ? SCHEDULED_PREPARE : SCHEDULED;
  last_due_ = clock::uptime::now();
  alarm_
    .set_deadline(last_due_)
    .schedule();
  return 0;
}

int
Beacon::deactivate ()
{
  auto state = state_;
  if ((SCHEDULED == state)
      || (SCHEDULED_PREPARE == state)) {
    // If we can cancel it before it fires, mark it inactive and we're
    // done.  Otherwise the only states we need to do something about
    // are READY or ACTIVE.
    if (clock::alarm::state_type::ST_scheduled == alarm_.cancel()) {
      state_ = INACTIVE;
    }
  }
  state = state_;
  if (READY == state) {
    readyq_.unlink(*this);
    state_ = INACTIVE;
  } else if (ACTIVE == state) {
    state_ = CANCELLED;
  }
  return state_;
}

Beacon::~Beacon ()
{
  deactivate();
}

void
Beacon::set_notify (notifier_type notify)
{
  notify_ = notify;
}

TelemetryBeacon::TelemetryBeacon (const systemState& system_state) :
  system_state{system_state}
{
  constexpr auto UTT_Hz = clock::uptime::Frequency_Hz;
  set_interval(1 * UTT_Hz, 5 * 60 * UTT_Hz);
}

int
TelemetryBeacon::populate_ (pabigot::ble::gap::adv_data& ad)
{
  auto beacon_span = sizeof(frame_s);
  auto flags = flags_;
  if (power_level_
      && pwr_mV_
      && !(FT_FLAG_MAINS_POWER & flags)) {
    flags |= FT_FLAG_HAS_POWER_LEVEL;
    beacon_span += sizeof(uint8_t);
  }
  auto bp = static_cast<frame_s *>(ad.set_ManufacturerSpecificData(COMPANY_ID, beacon_span));
  if (bp) {
    bp->prefix = {FRAME_TYPE, flags};
    /* Store the power source voltage then discard the value so if we
     * don't get an update before the next populate_() we don't claim
     * to be providing a valid voltage. */
    bp->pwr_mV = pwr_mV_;
    pwr_mV_ = 0;
    bp->adv_cnt = Beacon::telemetry_state().adv_cnt();
    bp->sec_cnt = Beacon::telemetry_state().sec_cnt();

    // @todo Remove the flag setting when all firmware using this
    // offset for wfe_cnt has been updated.
    bp->prefix.flags |= FT_FLAG_HAS_RESET_COUNT;
    bp->reset_cnt = system_state.state().reset_count;

    uint64_t sleep;
    uint64_t radio;
    auto total = system_state.operationalModeBreakdown(sleep, radio);
    /* First transmission may not have any operational mode times
     * accrued, in which case the resulting values will be out of
     * range.  If we don't have any data leave the fields zeroed. */
    if (0 < total) {
      static constexpr unsigned int scale = 1U << 16;
      bp->sleep_n16 = (total - 1U + scale * sleep) / total;
      bp->radio_n16 = (total - 1U + scale * radio) / total;
    }
    auto op(bp->optional);
    if (FT_FLAG_HAS_POWER_LEVEL & flags) {
      *op++ = (255 * power_level_(bp->pwr_mV)) / 10000;
    }
  }
  return 1;
}

SystemStateBeacon::SystemStateBeacon (const systemState& system_state) :
  system_state{system_state}
{
  constexpr auto UTT_Hz = clock::uptime::Frequency_Hz;
  set_interval(1 * UTT_Hz, 60 * 60 * UTT_Hz);
}

int
SystemStateBeacon::populate_ (pabigot::ble::gap::adv_data& ad)
{
  auto beacon_span = sizeof(frame_s) + sizeof(crc_type);
  auto bp = static_cast<frame_s *>(ad.set_ManufacturerSpecificData(COMPANY_ID, beacon_span));
  if (bp) {
    const auto& ss = system_state.state();
    bp->prefix = {FRAME_TYPE, 0};
    bp->last_pc = ss.last_pc;
    bp->code = ss.code;
    bp->sdfault_id = ss.sdfault_id;
    bp->reset_count = ss.reset_count;
    bp->reset_reas = ss.reset_reas;
    bp->wdt_status = ss.wdt_status;
    bp->heap_use = (128 * system_state.heap_used()) / system_state.heap_reserved();
    bp->stack_use = (128 * system_state.stack_used()) / system_state.stack_reserved();
    update_crc(bp);
  }
  return 1;
}

EnvSensorBeacon::EnvSensorBeacon ()
{
  constexpr auto UTT_Hz = nrfcxx::clock::uptime::Frequency_Hz;
  instance_.span = 0;
  set_interval(1 * UTT_Hz, 60 * UTT_Hz);
}

EnvSensorBeacon&
EnvSensorBeacon::resetInstance ()
{
  auto fp = reinterpret_cast<frame_s *>(instance_.buffer.data());
  *fp = {FRAME_TYPE};
  bi_ = sizeof(*fp);
  instance_.span = 0;
  return *this;
}

bool
EnvSensorBeacon::append_ (const void* vp, size_t len)
{
  if (0 == bi_) {
    return false;
  }
  if ((bi_ + len) > (instance_.buffer.size() - sizeof(crc_type))) {
    bi_ = 0;
    return false;
  }
  memmove(instance_.buffer.data() + bi_, vp, len);
  bi_ += len;
  return true;
}

EnvSensorBeacon&
EnvSensorBeacon::addTemperature (int16_t value)
{
  value = pabigot::byteorder::host_x_le(value);
  if (append_(&value, sizeof(value))) {
    setFlag_(FT_FLAG_TEMPERATURE);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addHumidity (uint16_t value)
{
  value = pabigot::byteorder::host_x_le(value);
  if (append_(&value, sizeof(value))) {
    setFlag_(FT_FLAG_HUMIDITY);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addAbsPressure (unsigned int value)
{
  value = pabigot::byteorder::host_x_le(value);
  if (append_(&value, 3)) {     // low 3 bytes = uint24_t
    setFlag_(FT_FLAG_ABS_PRESSURE);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addDiffPressure (int16_t value)
{
  value = pabigot::byteorder::host_x_le(value);
  if (append_(&value, sizeof(value))) {
    setFlag_(FT_FLAG_DIFF_PRESSURE);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addECO2 (uint16_t value)
{
  value = pabigot::byteorder::host_x_le(value);
  if (append_(&value, sizeof(value))) {
    setFlag_(FT_FLAG_IAQ_ECO2);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addETVOC (uint16_t value)
{
  value = pabigot::byteorder::host_x_le(value);
  if (append_(&value, sizeof(value))) {
    setFlag_(FT_FLAG_IAQ_ETVOC);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addLightIntensity (uint8_t value)
{
  if (append_(&value, sizeof(value))) {
    setFlag_(FT_FLAG_LIGHT_INTENSITY);
  }
  return *this;
}

EnvSensorBeacon&
EnvSensorBeacon::addThermistor (int16_t value)
{
  value = pabigot::byteorder::host_x_le(value);
  append_(&value, sizeof(value));
  return *this;
}

int
EnvSensorBeacon::finalizeInstance ()
{
  if (0 == bi_) {
    return -1;
  }
  update_crc_(instance_.buffer.data(), bi_);
  instance_.span = bi_ + 2;
  bi_ = 0;
  return 0;
}

int
EnvSensorBeacon::populate_ (pabigot::ble::gap::adv_data& ad)
{
  if (!instance_.span) {
    // Not ready.
    return 0;
  }
  auto bp = static_cast<frame_s *>(ad.set_ManufacturerSpecificData(COMPANY_ID, instance_.span));
  if (bp) {
    auto fp = reinterpret_cast<frame_s *>(instance_.buffer.data());
    memcpy(bp, fp, instance_.span);
  }
  return 1;
}

int
EnvSensorBeacon::significantChange (const instance_s& from,
                                    const threshold_s& thr) const
{
  int rv = 0;
  if (from.span != instance_.span) {
    return -1;
  }
  auto ffp = reinterpret_cast<const frame_s *>(from.buffer.data());
  auto tfp = reinterpret_cast<const frame_s *>(instance_.buffer.data());
  if (ffp->frame_type != tfp->frame_type) {
    return -2;
  }
  if (ffp->flags != tfp->flags) {
    return -3;
  }
  uint8_t flags = ffp->flags;
  auto fp = from.buffer.data() + sizeof(*ffp);
  auto tp = instance_.buffer.data() + sizeof(*tfp);
  if (FT_FLAG_TEMPERATURE & flags) {
    auto f_cCel = extract_le<int16_t>(fp);
    auto t_cCel = extract_le<int16_t>(tp);
    rv = temp_changed(f_cCel, t_cCel, thr.temperature_cCel);
    //printf("compare %d %d cCel : %d\n", f_cCel, t_cCel, rv);
    if (rv) {
      return rv - 4;
    }
  }
  if (FT_FLAG_HUMIDITY & flags) {
    auto f_pptt = extract_le<uint16_t>(fp);
    auto t_pptt = extract_le<uint16_t>(tp);
    rv = (thr.humidity_pptt <= abs(f_pptt - t_pptt));
    //printf("compare %u %u pptt : %d\n", f_pptt, t_pptt, rv);
    if (rv) {
      return -5;
    }
  }
  if (FT_FLAG_ABS_PRESSURE & flags) {
    auto f_cPa = extract_le<unsigned int>(fp, 3);
    auto t_cPa = extract_le<unsigned int>(tp, 3);
    rv = (thr.abs_cPa <= abs(f_cPa - t_cPa));
    //printf("compare %u %u cPa : %d ; %d\n", f_cPa, t_cPa, rv, static_cast<int>(t_cPa - f_cPa));
    if (rv) {
      return -6;
    }
  }
  if (FT_FLAG_IAQ_ECO2 & flags) {
    auto f_ppm = extract_le<uint16_t>(fp);
    auto t_ppm = extract_le<uint16_t>(tp);
    rv = (thr.eCO2_ppm <= abs(f_ppm - t_ppm));
    //printf("compare %u %u ppm : %d ; %d\n", f_ppm, t_ppm, rv, static_cast<int16_t>(t_ppm - f_ppm));
    if (rv) {
      return -7;
    }
  }
  if (FT_FLAG_IAQ_ETVOC & flags) {
    auto f_ppb = extract_le<uint16_t>(fp);
    auto t_ppb = extract_le<uint16_t>(tp);
    rv = (thr.eTVOC_ppb <= abs(f_ppb - t_ppb));
    //printf("compare %u %u ppb : %d ; %d\n", f_ppb, t_ppb, rv, static_cast<int16_t>(t_ppb - f_ppb));
    if (rv) {
      return -8;
    }
  }
  if (FT_FLAG_LIGHT_INTENSITY & flags) {
    auto f_1 = extract_le<uint8_t>(fp);
    auto t_1 = extract_le<uint8_t>(tp);
    rv = (thr.light_1 <= abs(f_1 - t_1));
    //printf("compare %u %u 1 : %d ; %d\n", f_1, t_1, rv, static_cast<int8_t>(t_ppb - f_ppb));
    if (rv) {
      return -9;
    }
  }
  return 0;
}

ApplicationIdBeacon::ApplicationIdBeacon ()
{
  constexpr auto UTT_Hz = clock::uptime::Frequency_Hz;
  set_interval(10 * UTT_Hz, 6 * 60 * 60 * UTT_Hz);
}

void
ApplicationIdBeacon::prepare (const systemState& system_state,
                              const ble_version_t& fwid)
{
  frame_ = {
    .prefix = {FRAME_TYPE},
    .app_crc = application_crc32(),
    .sd_fwid = fwid.subversion_number,
    .llver = fwid.version_number,
    .stack_KiBy = static_cast<uint8_t>(system_state.stack_reserved() / 1024),
    .heap_KiBy = static_cast<uint8_t>(system_state.heap_reserved() / 1024),
  };
}

int
ApplicationIdBeacon::pre_activate_ ()
{
  if (!frame_.llver) {
    return -1;
  }
  return 0;
}

int
ApplicationIdBeacon::populate_ (pabigot::ble::gap::adv_data& ad)
{
  auto beacon_span = sizeof(frame_);
  auto bp = static_cast<frame_s *>(ad.set_ManufacturerSpecificData(COMPANY_ID, beacon_span));
  if (bp && frame_.stack_KiBy) {
    memcpy(bp, &frame_, sizeof(frame_));
    return 1;
  }
  return -1;
}

} // namespace sd
} // namespace nrfcxx
