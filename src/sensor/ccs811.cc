// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/sensor/ccs811.hpp>

#if 0
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

namespace nrfcxx {
namespace sensor {

namespace {

/** Number of seconds between observations in each drive mode. */
const uint8_t dm_interval_s[] = { 0, 1, 10, 60};

constexpr uint8_t R_STATUS = 0x00;
constexpr uint8_t R_MEAS_MODE = 0x01;
constexpr uint8_t R_ALG_RESULT_DATA = 0x02;
constexpr uint8_t R_RAW_DATA = 0x03;
constexpr uint8_t R_ENV_DATA = 0x05;
constexpr uint8_t R_THRESHOLDS = 0x10;
constexpr uint8_t R_BASELINE = 0x11;
constexpr uint8_t R_HW_ID = 0x20;
constexpr uint8_t R_HW_Version = 0x21;
constexpr uint8_t R_FW_Boot_Version = 0x23;
constexpr uint8_t R_FW_App_Version = 0x24;
constexpr uint8_t R_ERROR_ID = 0xE0;

constexpr unsigned int RESET_DELAY_utt = clock::uptime::from_ms(2U);
constexpr unsigned int APP_START_DELAY_us = 1000U;

constexpr uint8_t MM_INTERRUPT = 0x08;
constexpr uint8_t MM_DRIVE_MODE_Msk = 0x70;
constexpr uint8_t MM_DRIVE_MODE_Pos = 4;

/* Get the MEAS_MODE value to use for a given drive mode.
 *
 * DM_IDLE returns zero; absense of MM_INTERRUPT shows that this is
 * not a supported active mode. */
uint8_t dm_to_meas_mode (uint8_t dm)
{
  uint8_t rv = 0;
  if (ccs811::DM_IDLE != dm) {
    rv = MM_INTERRUPT | (MM_DRIVE_MODE_Msk & (dm << MM_DRIVE_MODE_Pos));
  }
  return rv;
}

/* State where the CCS811 is known to be present and if necessary a
 * reset to clear observed problems has been executed.  If the status
 * indicates a fatal error, transitions to FAILED.  If the status
 * indicates the device is in BOOT mode, entry to APP mode is
 * initiated and transitions to ENTRY_MEASMODE after
 * APP_START_DELAY_utt.  If status indicates in APP mode transitions
 * directly to MEASMODE. */
constexpr auto MS_INITIALIZE = 0 + lpm::state_machine::MS_ENTRY_SETUP;

/* State waiting for confirmed entry to application mode.  Status is
 * read, and if not in APP mode transitions to FAILED.  Otherwise sets
 * MEAS_MODE.  On confirmation transitions to MEASMODE, otherwise
 * transitions to FAILED. */
constexpr auto MS_ENTRY_MEASMODE = 1 + lpm::state_machine::MS_ENTRY_SETUP;

/* State where MEAS_MODE setting is confirmed.  The current mode is
 * read.  If it is correct, the machine transitions to SAMPLE.  If the
 * current mode is idle, the correct setting is written, the system
 * transitions to ACTIVE.  If the current mode is not idle and does
 * not match the desired mode, transitions to RESET. */
constexpr auto MS_MEASMODE = 2 + lpm::state_machine::MS_ENTRY_SETUP;

} // ns anonymous

__attribute__((__section__(".noinit.nrfcxx.sensor.ccs811.retained_state")))
ccs811::retained_state_type ccs811::retained_state_;

void
ccs811::state_setup (const systemState::state_type& ss,
                     bool is_reset,
                     bool retained)
{
  if (is_reset) {
    // Nothing to update
  } else if (retained) {
    // Nothing to update.  If the reset was uncontrolled we didn't
    // lose anything, because total_now() survives retained but
    // uncontrolled resets.
  } else {
    memset(&retained_state_, 0, sizeof(retained_state_));
  }
}

int
ccs811::active_drive_mode () const
{
  if (lpm::state_machine::MS_SAMPLE != machine_.state()) {
    return -1;
  }
  return active_drive_mode_();
}

int
ccs811::drive_mode (unsigned int dm)
{
  if (lpm::state_machine::MS_OFF != machine_.state()) {
    return -1;
  }
  if (3 < dm) {
    return -2;
  }
  int rv = drive_mode_;
  drive_mode_ = dm;
  return rv;
}

int
ccs811::update_persisted (persisted_state_type& pst) const
{
  const auto rsp = &retained_state_;
  if (!(retained_state_type::FL_BASELINED & rsp->flags)) {
    return -1;
  }
  int dm = active_drive_mode();
  if (0 > dm) {
    return -2;
  }
  pst.baseline[dm] = rsp->baseline;
  return 0;
}

int
ccs811::restore_from_persisted (const persisted_state_type& pst)
{
  auto rsp = &retained_state_;
  if (retained_state_type::FL_BASELINED & rsp->flags) {
    return -1;
  }
  int dm = active_drive_mode();
  if (0 > dm) {
    return -2;
  }
  if (0 == pst.baseline[dm]) {
    return -3;
  }
  rsp->baseline = pst.baseline[dm];
  rsp->flags |= retained_state_type::FL_BASELINED;
  return 0;
}

int
ccs811::fill_system_beacon (system_beacon_type& fr) const
{
  using clock::uptime;
  const auto& rst = retained_state_;
  const auto& obs = observations_;
  fr = {};
  auto tn = systemState::total_now();
  if (0 != rst.reset_utt) {
    fr.active_ds = (10 * (tn - rst.reset_utt)) / uptime::Frequency_Hz;
  }
  if (0 != rst.baselined_utt) {
    fr.bl_age_ds = (10 * (tn - rst.baselined_utt)) / uptime::Frequency_Hz;
  }
  if (retained_state_type::FL_BASELINED & rst.flags) {
    fr.bl_retained = rst.baseline;
  }
  if (ST_DATA_READY & obs.status) {
    fr.bl_latest = obs.baseline;
  }
  fr.app_version = version_.fw_app_version;
  fr.hw_version = version_.hw_version;
  fr.flags = rst.flags;
  return 0;
}

ccs811::observation_beacon_type
ccs811::observation_beacon () const
{
  observation_beacon_type rv{};
  if (!observations_.is_ready()) {
    rv.status = -1;
  } else {
    rv.status = retained_state_.flags;
  }
  rv.env_data = env_data_;
  rv.baseline = observations_.baseline;
  rv.eCO2 = observations_.eCO2;
  rv.eTVOC = observations_.eTVOC;
  return rv;
}

/** Type for holding encoded environmental data as stored in the
 * CCS811 ENV_DATA register. */
union envdata_u {
  /* Read back in native byte order; when stored in the I2C buffer
   * the result will be in the required byte order. */
  uint32_t u32;

  /* HH, LH, HT, LT representing big-endian humidity followed by
   * temperature in 2^-9 units.  The sub-unit bytes (1, 3) of each
   * half-word are always zero.  So we set indexes 0 for humidity, and
   * 2 for temperature. */
  uint8_t u8[4];
  uint16_t u16[2];
};

uint32_t
ccs811::encode_env (int16_t temp_cCel,
                    uint16_t rh_pptt)
{
  envdata_u u{};

  /* Values are documented as being as 16-bit unsigned integer where
   * the upper 7 bits are the whole value and the lower 9 bits are the
   * fractional value scaled by 1000.  Of course fractions above 0.511
   * can't be represented in 9 bits, so the programming guide notes
   * that the device only pays attention to the upper bit of the
   * fraction part (which is the low bit of the first octet),
   * interpreting it as indicating 0.5.
   *
   * Use significantly simpler code than the data sheet to convert to
   * the half-unit. */

  /* Required humidity is simple (can't be negative, could be too
   * high), but still gets rounded. */
  int v = (25 + rh_pptt) / 50;
  if (200 < v) {
    v = 200;
  }
  u.u8[0] = v;

  /* Required temperature value is offset from -25 Cel.  We add 0.25
   * Cel to round to nearest half Cel.  The resulting value is clamped
   * to uint8_t representation. */
  v = (25 + 2500 + temp_cCel) / 50;
  if (0 > v) {
    v = 0;
  } else if (255 < v) {
    v = 255;
  }
  u.u8[2] = v;

  return u.u32;
}

int
ccs811::env_data (uint32_t enc)
{
  size_t len = 1 + sizeof(enc);
  uint8_t buf[len];
  buf[0] = R_ENV_DATA;
  memcpy(buf + 1, &enc, sizeof(enc));

  int rc;
  if (auto enabler = scoped_enable()) {
    rc = twi().write(addr_, buf, len);
    if (0 <= rc) {
      env_data_ = enc;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
ccs811::threshold_s::env_data_changed (uint32_t ed1,
                                       uint32_t ed2) const
{
  envdata_u u1{ed1};
  envdata_u u2{ed2};
  // Scale thresholds to handle half-unit values
  if ((2 * temperature_Cel) <= abs(u1.u8[0] - u2.u8[0])) {
    return -1;
  }
  if ((2 * humidity_pph) <= abs(u1.u8[2] - u2.u8[2])) {
    return -2;
  }
  return 0;
}

int
ccs811::threshold_s::observation_beacon_changed (const observation_beacon_type& ob1,
                                                 const observation_beacon_type& ob2) const
{
  if (ob1.status != ob2.status) {
    return -1;
  }
  if (ob1.baseline != ob2.baseline) {
    return -2;
  }
  if (eCO2_ppm <= abs(ob1.eCO2 - ob2.eCO2)) {
    return -3;
  }
  if (eTVOC_ppb <= abs(ob1.eTVOC - ob2.eTVOC)) {
    return -4;
  }
  if (ob1.env_data != ob2.env_data) {
    return -5;
  }
  return 0;
}

void
ccs811::intn_callback_ (const periph::GPIOTE::sense_status_type* sp)
{
  while ((0 <= sp->psel)
         && (sp->psel < iface_config_.intn_psel)) {
    ++sp;
  }
  if ((sp->psel == iface_config_.intn_psel)
      && (!(1 & sp->counter_state))) { // active low
    machine_.post_event();
  }
}

ccs811::ccs811 (notifier_type setter,
                iface_config_type& ifc,
                bool addr_sec) :
  super{setter},
  iface_config_{ifc},
  waken_{ifc.waken},
  resetn_{ifc.resetn},
  intn_listener_{[this](const periph::GPIOTE::sense_status_type* sp){
      intn_callback_(sp);
    }},
  intn_psel_{static_cast<int8_t>(((0 <= ifc.intn_psel) && (ifc.intn_psel < 32)) ? ifc.intn_psel : -1)},
  addr_(0x5A + addr_sec)
{
  if ((!ifc.waken.valid())
      || (!ifc.resetn.valid())
      || (0 > intn_psel_)
      || (nrf5::GPIO_PSEL_COUNT <= intn_psel_)) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  waken_.enable();
  resetn_.enable();
  nrf5::GPIO->PIN_CNF[intn_psel_] = gpio::PIN_CNF_ACTIVE_LOW_NOPULL;
  intn_listener_.enable();
}

int
ccs811::read_u8_ (uint8_t reg) const
{
  uint8_t v;
  int rc = twi().write_read(addr_, &reg, 1, &v, sizeof(v));
  return (sizeof(v) == rc) ? v : rc;
}

int
ccs811::status_ () const
{
  int rv = read_u8_(R_STATUS);
  if ((0 <= rv) && (ST_ERROR & rv)) {
    int rc = read_u8_(R_ERROR_ID);
    if (0 <= rc) {
      return (rc << 8) | rv;
    }
    return rc;
  }
  return rv;
}

int
ccs811::status () const
{
  int rc;
  if (auto enabler = scoped_enable()) {
    rc = status_();
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
ccs811::reset () const
{
  resetn_.assert();
  /* Mark when we did the reset and that the device is no longer past
   * its conditioning period. */
  retained_state_.reset_utt = systemState::total_now();
  retained_state_.flags &= ~(retained_state_type::FL_DRIVE_MODE_Msk | retained_state_type::FL_CONDITIONED);

  delay_us(15); // t_RESET
  resetn_.deassert();
  // implicit t_DRESET 20 us
  return RESET_DELAY_utt;
}

int
ccs811::id_version (version_s& vid) const
{
  int rc;
  do {
    using namespace pabigot::byteorder;
    auto enabler = scoped_enable();

    rc = enabler.result();
    if (0 > rc) {
      break;
    }
    rc = twi().write_read(addr_, &R_HW_ID, 1, &vid.hw_id, sizeof(vid.hw_id));
    if (1 != rc) {
      break;
    }
    rc = twi().write_read(addr_, &R_HW_Version, 1, &vid.hw_version, sizeof(vid.hw_version));
    if (1 != rc) {
      break;
    }
    rc = twi().write_read(addr_, &R_FW_Boot_Version, 1,
                         reinterpret_cast<uint8_t*>(&vid.fw_boot_version),
                         sizeof(vid.fw_boot_version));
    if (sizeof(vid.fw_boot_version) != rc) {
      break;
    }
    vid.fw_boot_version = host_x_be(vid.fw_boot_version);
    rc = twi().write_read(addr_, &R_FW_App_Version, 1,
                         reinterpret_cast<uint8_t*>(&vid.fw_app_version),
                         sizeof(vid.fw_app_version));
    if (sizeof(vid.fw_app_version) != rc) {
      break;
    }
    vid.fw_app_version = host_x_be(vid.fw_app_version);
    rc = 0;
  } while (false);
  return rc;
}

int
ccs811::baseline_ () const
{
  uint16_t v = 0;
  int rc = twi().write_read(addr_, &R_BASELINE, 1,
                           reinterpret_cast<uint8_t*>(&v),
                           sizeof(v));
  if (sizeof(v) != rc) {
    return -1;
  }
  // NB: BASELINE register is not converted to host byte order
  return v;
}

int
ccs811::baseline () const
{
  int rc;
  if (auto enabler = scoped_enable()) {
    rc = baseline_();
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
ccs811::baseline_ (uint16_t v) const
{
  size_t len = 1 + sizeof(v);
  uint8_t buf[len];
  buf[0] = R_BASELINE;
  memcpy(buf + 1, &v, sizeof(v));
  return twi().write(addr_, buf, len);
}

int
ccs811::baseline (uint16_t v) const
{
  int rc;
  if (auto enabler = scoped_enable()) {
    rc = baseline_(v);
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
ccs811::retain_baseline ()
{
  /* Can't save unless we've completed the conditioning period. */
  if (!(retained_state_type::FL_CONDITIONED & retained_state_.flags)) {
    return -1;
  }
  /* Can't save unless we have an observation with a valid
   * baseline. */
  if (!(ST_DATA_READY & observations_.status)) {
    return -2;
  }
  retained_state_.baseline = observations_.baseline;
  retained_state_.flags |= retained_state_type::FL_BASELINED;
  retained_state_.baselined_utt = systemState::total_now();
  return observations_.baseline;
}

int
ccs811::fetch_ (observations_type& obs)
{
  obs = {};
  int rc;
  do {
    using namespace pabigot::byteorder;

    /* Read the BASELINE first, so the STATUS we get from
     * ALG_RESULT_DATA includes any errors resulting from that
     * read.
     *
     * The baseline value is not converted to host-byte order.  (Based
     * on its values, it's probably little-endian anyway.) */
    rc = twi().write_read(addr_, &R_BASELINE, 1,
                         reinterpret_cast<uint8_t*>(&obs.baseline),
                         sizeof(obs.baseline));
    if (sizeof(obs.baseline) != rc) {
      // If this fails the CCS811 may be stretching the clock beyond
      // what the configured TWI timeout allows.
      rc = -1;
      break;
    }


    // Make this 8 if you want the raw data too. */
    constexpr auto len = 6U;
    rc = twi().write_read(addr_, &R_ALG_RESULT_DATA, 1,
                         reinterpret_cast<uint8_t*>(&obs.eCO2), len);
    if (len != rc) {
      rc = -2;
      break;
    }
    obs.eCO2 = host_x_be(obs.eCO2);
    obs.eTVOC = host_x_be(obs.eTVOC);
    rc = 0;
  } while (false);
  return rc;
}

int
ccs811::lpsm_process_ (int& delay,
                       process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;
  bool do_reset = false;
  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    case state_machine::MS_FAILED:
      break;
    case state_machine::MS_ENTRY_START:
      rc = id_version(version_);
      if (0 > rc) {
        break;
      }
      rc = 0;
      if (HARDWARE_ID != version_.hw_id) {
        // @todo notification?
        machine_.set_state(state_machine::MS_FAILED);
        break;
      }
      rc = status();
      if (0 > rc) {
        break;
      }
      cprintf("* CCS811 STARTUP %x rs %u\n", rc, static_cast<unsigned int>(retained_state_.reset_utt));
      if ((ST_ERROR & rc)
          || (0U == retained_state_.reset_utt)) {
        cprintf("** CCS811 STARTUP state %x %s reset_utt, going reset\n", rc,
                (0U == retained_state_.reset_utt) ? "lost" : "retained");
        do_reset = true;
        break;
      }
      machine_.set_state(MS_INITIALIZE);
      // FALLTHRU
    case MS_INITIALIZE:
      rc = status();
      if (0 > rc) {
        break;
      }
      if (ST_APP_VALID != ((ST_APP_VALID | ST_ERROR) & rc)) {
        cprintf("*** CCS811 INITIALIZE state %x, going failed\n", rc);
        machine_.set_state(state_machine::MS_ENTRY_FAILED);
        rc = 0;
        break;
      }
      cprintf("* CCS811 INITIALIZE state %x\n", rc);
      if (!(ST_FW_MODE & rc)) {
        static uint8_t cmd = 0xF4;
        if (auto enabler = scoped_enable()) {
          rc = twi().write(addr_, &cmd, sizeof(cmd));
        } else {
          rc = enabler.result();
        }
        if (0 > rc) {
          cprintf("*** CCS811 going APP_MODE got %d\n", rc);
          break;
        }
        delay = uptime::from_us(APP_START_DELAY_us);
        machine_.set_state(MS_ENTRY_MEASMODE);
        rc = 0;
        break;
      }
      machine_.set_state(MS_MEASMODE);
      [[fallthrough]]
    case MS_MEASMODE: {
      uint8_t mm = dm_to_meas_mode(drive_mode_);
      if (!(MM_INTERRUPT & mm)) {
        cprintf("*** CCS811 cannot operate with drive mode %u\n", drive_mode_);
        rc = -1;
        break;
      }
      uint8_t const exp_mm = dm_to_meas_mode(active_drive_mode_());
      uint8_t old_mm = 0;
      if (auto enabler = scoped_enable()) {
        rc = twi().write_read(addr_, &R_MEAS_MODE, 1, &old_mm, sizeof(old_mm));
      } else {
        rc = enabler.result();
      }
      if (0 > rc) {
        break;
      }
      cprintf("* CCS811 MEASMODE mm was %02x, expect %02x, want %02x\n", old_mm, exp_mm, mm);
      if (exp_mm != old_mm) {
        /* We think the CCS811 is observing in a particular mode, but
         * it denies it.  Probably something cycled power.  Reset. */
        cprintf("** CCS811 MEASMODE %02x expected %02x requires reset\n", old_mm, exp_mm);
        do_reset = true;
        break;
      }
      if (mm != old_mm) {
        if (MM_DRIVE_MODE_Msk & old_mm) {
          /* Transition from a non-idle operating mode to different
           * operating mode.  Make this clean: reset the device. */
          cprintf("** CCS811 MEASMODE %02x to %02x requires reset\n", old_mm, mm);
          do_reset = true;
          break;
        }
        uint8_t buf[]{R_MEAS_MODE, mm};
        if (auto enabler = scoped_enable()) {
          rc = twi().write(addr_, buf, sizeof(buf));
        } else {
          rc = enabler.result();
        }
        if (0 > rc) {
          cprintf("*** CCS811 MEASMODE set %02x got %d\n", mm, rc);
          break;
        }
      }

      auto& rst = retained_state_;
      auto rsdm = (retained_state_type::FL_DRIVE_MODE_Msk & rst.flags) >> retained_state_type::FL_DRIVE_MODE_Pos;
      if (rsdm != drive_mode_) {
        cputs("* CCS811 Clearing basedlined state");
        /* We're not running in the mode for which we've retained
         * baseline state.  Update the mode, clear the BASELINED flag
         * and value, and reset the baselined timestamp.  The caller
         * may react to state_machine::PF_STARTED by using
         * restore_from_persisted() to provide a baseline value for
         * the current mode. */
        rst.flags &= ~(retained_state_type::FL_DRIVE_MODE_Msk | retained_state_type::FL_BASELINED);
        rst.flags |= (drive_mode_ << retained_state_type::FL_DRIVE_MODE_Pos);
        rst.baseline = 0;
        rst.baselined_utt = systemState::total_now();
      }
      pf |= state_machine::PF_STARTED;
      machine_.set_state(state_machine::MS_SAMPLE);
      /* Fall through to MS_SAMPLE because the CCS811 might have
       * ridden through a system reset and be sitting there with an
       * observation ready to collect. */
    }
      [[fallthrough]]
    case state_machine::MS_SAMPLE:
      if (!intn_asserted()) {
#if (WITH_FALLBACK_DELAY - 0)
        if (!delay) {
          /* Not ready, and not fallen through MS_MEAS_MODE Must have
           * gotten here due to application calling unnecessarily, or
           * a fallback timer that fired too early.  Try again in a
           * short time, earlier if INTn is asserted. */
          delay = - uptime::from_ms(100);
        }
#endif // WITH_FALLBACK_DELAY
        break;
      }
      if (auto enabler = scoped_enable()) {
        rc = fetch_(observations_);
      } else {
        rc = enabler.result();
      }
      if (0 > rc) {
        break;
      }
#if (WITH_FALLBACK_DELAY - 0)
      {
        /* Notification of INTn asserted is not absolutely robust; the
         * DETECT event might be missed.  Add a fallback wakeup a half
         * second after we expect the next observation. */
        unsigned int interval = dm_interval_s[drive_mode_] * uptime::Frequency_Hz;
        if (0 < interval) {
          delay = - (interval + uptime::Frequency_Hz / 2);
        }
      }
#endif // WITH_FALLBACK_DELAY
      pf |= state_machine::PF_OBSERVATION;
      // Simulate ST_DATA_READY on firmware 1.1 which doesn't provide it.
      if ((0x1100 == version_.fw_app_version)
          && (0 < observations_.eCO2)) {
        observations_.status |= ccs811::ST_DATA_READY;
      }
      if (ccs811::ST_DATA_READY & observations_.status) {
        uint64_t tnow = systemState::total_now();
        auto& obs = observations_;
        auto& rst = retained_state_;
        if (!(retained_state_type::FL_REPORTING & rst.flags)) {
          rst.flags |= retained_state_type::FL_REPORTING;
          pf |= PF_REPORTING;
        }
        if (!(retained_state_type::FL_CONDITIONED & rst.flags)) {
          if (CONDITIONING_DELAY_utt < (tnow - rst.reset_utt)) {
            rst.flags |= retained_state_type::FL_CONDITIONED;
            pf |= PF_CONDITIONED;

            /* Update BASELINE if we have a retained BASELINE reading. */
            if (retained_state_type::FL_BASELINED & rst.flags) {
              // Restore the existing baseline
              rc = baseline(rst.baseline);
              if (0 > rc) {
                break;
              }
              // Mark the reading invalid, as it was collected with
              // a baseline that's now been replaced.
              obs.status &= ~ccs811::ST_DATA_READY;
              pf |= PF_RESTORED;
            }
            // If we didn't restore, we could try capturing the
            // current value as an initial baseline, but there's
            // no reason to believe it's any good.
          }
        } else if ((tnow > rst.baselined_utt) // should always be true, but...
                   && (BASELINE_CAPTURE_INTERVAL_utt < (tnow - rst.baselined_utt))) {
          rc = retain_baseline();
          if (0 > rc) {
            break;
          }
          pf |= PF_BASELINED;
        }
      }
      if (ccs811::ST_ERROR & observations_.status) {
        pf |= ccs811::PF_CCS811_ERROR;
      }
      break;
    case state_machine::MS_ENTRY_RESET:
      rc = reset();
      cprintf("* CCS811 RESET got %d\n", rc);
      if (0 > rc) {
        break;
      }
      machine_.set_state(state_machine::MS_EXIT_RESET);
      delay = rc;
      break;
    case state_machine::MS_EXIT_RESET:
      if (auto enabler = scoped_enable()) {
        rc = status();
      }
      if (0 > rc) {
        cprintf("*** CCS811 WAIT_RESET got %d\n", rc);
        break;
      }
      /* Check for a value that's been observed to show up soon after
       * an app firmware update.  In these observations it was up to 8
       * ms before the device responded properly. */
      if (0xFDFD == rc) {
        // @todo timeout to failed?
        delay = uptime::from_ms(1U);
        rc = 0;
        break;
      }
      machine_.set_state(MS_INITIALIZE);
      machine_.post_event();
      rc = 0;
      break;
    case MS_ENTRY_MEASMODE:
      {
        auto enable = scoped_enable();
        rc = status();
      }
      cprintf("* CCS811 ENTRY_MEASMODE got %d %x\n", rc, rc);
      if (0 > rc) {
        break;
      }
      if (ST_FW_MODE != ((ST_FW_MODE | ST_ERROR) & rc)) {
        cprintf("*** CSS811 WAIT_RESET state %x, going failed\n", rc);
        machine_.set_state(state_machine::MS_FAILED);
        rc = 0;
        break;
      }
      machine_.set_state(MS_MEASMODE, true);
      rc = 0;
      break;
  }
  if (do_reset) {
    machine_.set_state(state_machine::MS_ENTRY_RESET, true);
    pf |= state_machine::PF_RESET;
  }
  return rc;
}

} // ns sensor

} // ns nrfcxx
