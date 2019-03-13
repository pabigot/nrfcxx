// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/sensor/hts221.hpp>
#include <nrfcxx/utility.hpp>

#if 0
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn

namespace nrfcxx {
namespace sensor {

namespace {

gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;

constexpr uint8_t I2C_ADDRESS = 0x5f;
constexpr auto DRDY_IDLE_PIN_CNF = gpio::PIN_CNF_PWRUP | gpio::PIN_CNF_PULLDOWN;

constexpr uint8_t XR_AUTOINC = 0x80;
constexpr uint8_t R_WHO_AM_I = 0x0F;
constexpr uint8_t R_AV_CONF = 0x10;
constexpr uint8_t R_CTRL_REG1 = 0x20;
constexpr uint8_t R_CTRL_REG2 = 1 + R_CTRL_REG1;
constexpr uint8_t R_CTRL_REG3 = 2 + R_CTRL_REG1;
constexpr uint8_t R_STATUS_REG = 0x27;
/* uint16_t h_out_le ; uint16_t t_out_le. */
constexpr uint8_t R_MEAS = 0x28;
constexpr uint8_t R_CALIB = 0x30;

/** Measured time for a single sample at default oversample is 1547
 * utt or about 47 ms. */
constexpr unsigned int ObsDelay_utt = 1547;

/* Layout for R_CALIB */
struct calib_raw_s {
  uint8_t h0_pph_x2;
  uint8_t h1_pph_x2;
  uint8_t t0_Cel_x8_;
  uint8_t t1_Cel_x8_;
  uint8_t reserved_34;
  uint8_t tx_msb;
  int16_t h0_t0_out;
  uint16_t reserved_38;
  int16_t h1_t0_out;
  int16_t t0_out;
  int16_t t1_out;
} __attribute__((__packed__));

void
set_calibration (const calib_raw_s& raw,
                 hts221::calibration_type& calib)
{
  memset(&calib, 0, sizeof(calib));
  calib.cal_cCel[0] = 100 * (((0x03 & raw.tx_msb) << 8) | raw.t0_Cel_x8_) / 8;
  calib.cal_cCel[1] = 100 * (((0x0C & raw.tx_msb) << 6) | raw.t1_Cel_x8_) / 8;
  calib.cal_pptt[0] = 50 * raw.h0_pph_x2;
  calib.cal_pptt[1] = 50 * raw.h1_pph_x2;
  calib.td_cCel = calib.cal_cCel[1] - calib.cal_cCel[0];
  calib.hd_pptt = calib.cal_pptt[1] - calib.cal_pptt[0];
  calib.td_out = raw.t1_out - raw.t0_out;
  calib.hd_out = raw.h1_t0_out - raw.h0_t0_out;
  calib.t0_out = raw.t0_out;
  calib.h0_out = raw.h0_t0_out;
}

/* Layout for R_MEAS */
struct meas_raw_s {
  int16_t h_out_le;
  int16_t t_out_le;
} __attribute__((__packed__));

} // ns anonymous

void
hts221::drdy_callback_ (const periph::GPIOTE::sense_status_type* sp)
{
  while ((0 <= sp->psel)
         && (sp->psel < iface_config_.drdy_psel)) {
    ++sp;
  }
  if ((sp->psel == iface_config_.drdy_psel)
      && (1 & sp->counter_state)) {
    machine_.post_event();
  }
}

hts221::hts221 (notifier_type setter,
                iface_config_type& ifc) :
  super{setter},
  iface_config_{ifc},
  drdy_listener_{[this](const periph::GPIOTE::sense_status_type* sp){
      drdy_callback_(sp);
    }}
{
  if ((0 > ifc.drdy_psel)
      || (nrf5::GPIO_PSEL_COUNT <= ifc.drdy_psel)) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  nrf5::GPIO->PIN_CNF[ifc.drdy_psel] = DRDY_IDLE_PIN_CNF;
  iface_config_.address = I2C_ADDRESS;
  instr_aux.enable();
}

int
hts221::lpsm_sample ()
{
  using lpm::state_machine;
  auto rc = super::lpsm_sample();
  /* On the Thingy:52 with CCS811 running we're observing situations
   * where the DRDY signal DETECT event was not observed.  If the
   * attempt to initiate a sample was rejected because we never picked
   * up a completed result from the previous sample, wake up the state
   * machine. */
  if ((0 > rc)
      && (state_machine::MS_SAMPLE == machine_.state())
      && drdy_asserted()) {
    ++lost_drdy_;
    cprintf("*** HTS221 DRDY lost: %u\n", lost_drdy_);
    machine_.post_event();
  }
  return rc;
}

int
hts221::odr (uint8_t dr)
{
  if (lpm::state_machine::MS_OFF != machine_.state()) {
    return -1;
  }
  if (3 < dr) {
    return -2;
  }
  odr_ = dr;
  return odr_;
}

int
hts221::status () const
{
  auto& twi = iface_config_.twi;
  const uint8_t reg = R_STATUS_REG;
  uint8_t sv;
  int rc;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(I2C_ADDRESS, &reg, sizeof(reg), &sv, sizeof(sv));
  } else {
    rc = enabler.result();
  }
  if (sizeof(sv) == rc) {
    rc = sv;
  }
  return rc;
}

int
hts221::lpsm_process_ (int& delay,
                       process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;

  auto& twi = iface_config_.twi;
  switch (machine_.state()) {
    default:
      cprintf("*** HTS221 error state: %x\n", machine_.state());
      machine_.set_lost();
      break;
    case state_machine::MS_ENTRY_ERRORED:
    case state_machine::MS_ENTRY_FAILED:
    case state_machine::MS_ENTRY_STOPPED:
      cputs("disable path\n");
      drdy_listener_.disable();
      nrf5::GPIO->PIN_CNF[iface_config_.drdy_psel] = DRDY_IDLE_PIN_CNF;
      if (state_machine::MS_ENTRY_STOPPED == machine_.state()) {
        machine_.set_state(state_machine::MS_OFF);
        pf |= state_machine::PF_STOPPED;
      } else if (state_machine::MS_ENTRY_ERRORED == machine_.state()) {
        machine_.set_state(state_machine::MS_ERRORED);
      } else {
        machine_.set_state(state_machine::MS_FAILED);
      }
      break;
    case state_machine::MS_ENTRY_START:
      nrf5::GPIO->PIN_CNF[iface_config_.drdy_psel] = gpio::PIN_CNF_ACTIVE_HIGH_NOPULL;
      drdy_listener_.enable();
      {
        calib_raw_s raw;
        uint8_t cmd = (R_CALIB | XR_AUTOINC);
        if (auto enabler = twi.scoped_enable()) {
          rc = twi.write_read(I2C_ADDRESS, &cmd, sizeof(cmd),
                              reinterpret_cast<uint8_t*>(&raw), sizeof(raw));
        } else {
          rc = enabler.result();
        }
        if (sizeof(raw) != rc) {
          break;
        }

        set_calibration(raw, calibration_);
        uint8_t buf[4];
        auto bp = buf;
        *bp++ = R_CTRL_REG1 | XR_AUTOINC;
        *bp++ = 0x80 | odr_;    // active mode in odr
        *bp++ = 0x00;           // defaults
        *bp++ = 0x04;           // DRDY active high
        if (auto enabler = twi.scoped_enable()) {
          rc = twi.write(I2C_ADDRESS, buf, bp - buf);
        } else {
          rc = enabler.result();
        }
        if ((bp - buf) != rc) {
          break;
        }
        pf |= state_machine::PF_STARTED;
        if (odr_ || drdy_asserted()) {
          /* We either expect or have an observation-ready signal.
           * Read the data when the signal is present. */
          machine_.set_state(state_machine::MS_SAMPLE);
        } else {
          machine_.set_state(state_machine::MS_IDLE);
        }
      }
      break;
    case state_machine::MS_ENTRY_SAMPLE:
      if (drdy_asserted()) {
        // Something left a sample: go read it
        goto MS_SAMPLE_lbl;
      }
      if (auto enabler = twi.scoped_enable()) {
        static const uint8_t buf[] = {R_CTRL_REG2, 0x01};
        rc = twi.write(I2C_ADDRESS, buf, sizeof(buf));
        if (sizeof(buf) == rc) {
          machine_.set_state(state_machine::MS_SAMPLE);
#if (WITH_FALLBACK_DELAY - 0)
          // In case DRDY edge isn't caught.
          delay = - ObsDelay_utt;
#endif // WITH_FALLBACK_DELAY
        }
      } else {
        rc = enabler.result();
      }
      break;
    case state_machine::MS_SAMPLE:
    MS_SAMPLE_lbl:
      rc = status();
      if (0 > rc) {
        break;
      }
      if (SR_READY != rc) {
        // Not entirely ready (maybe only partial result triggered
        // DRDY). Sleep a little longer.
#if (WITH_FALLBACK_DELAY - 0)
        delay = - uptime::from_ms(10);
#endif // WITH_FALLBACK_DELAY
        break;
      }
      {
        meas_raw_s raw;
        uint8_t cmd = (R_MEAS | XR_AUTOINC);
        if (auto enabler = twi.scoped_enable()) {
          rc = twi.write_read(I2C_ADDRESS, &cmd, sizeof(cmd), reinterpret_cast<uint8_t*>(&raw), sizeof(meas_raw_s));
        } else {
          rc = enabler.result();
        }
        if (sizeof(meas_raw_s) != rc) {
          cprintf("** HTS221 meas read failed: %d\n", rc);
          break;
        }
        using namespace pabigot::byteorder;
        observations_.temperature_cCel = calibration_.conv_cCel(host_x_le(raw.t_out_le));
        observations_.humidity_pptt = calibration_.conv_pptt(host_x_le(raw.h_out_le));
      }
      pf |= state_machine::PF_OBSERVATION;
      if (ODR_OneShot == odr()) {
        machine_.set_state(state_machine::MS_IDLE);
      }
      break;
    case state_machine::MS_IDLE:
      // On startup in one-shot mode we seem to sometimes get a
      // spurious data collection shortly after the calibration data
      // is read.  If it isn't cleared requesting another reading
      // won't result in a sense interrupt and we won't move out of
      // MS_SAMPLE.
      if (drdy_asserted()) {
        cputs("HTS221 shorted sample");
        goto MS_SAMPLE_lbl;
      }
      break;
  }
  return rc;
}

} // ns sensor

} // ns nrfcxx
