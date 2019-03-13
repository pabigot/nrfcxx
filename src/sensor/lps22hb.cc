// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/sensor/lps22hb.hpp>
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

constexpr uint8_t I2C_ADDRESS = 0x5C;
constexpr auto DRDY_IDLE_PIN_CNF = gpio::PIN_CNF_PWRUP | gpio::PIN_CNF_PULLDOWN;

constexpr uint8_t R_WHO_AM_I = 0x0F;
constexpr uint8_t V_WHO_AM_I = 0xB1;
constexpr uint8_t R_CTRL_REG1 = 0x10;
constexpr uint8_t V_CTRL_REG1 = 0x00; // other flags defaulted
constexpr uint8_t XP_REG1_ODR = 4; // shift to place ODR
constexpr uint8_t R_CTRL_REG2 = 1 + R_CTRL_REG1;
constexpr uint8_t V_CTRL_REG2 = 0x10;            // IF_ADD_INC
constexpr uint8_t XB_REG2_ONESHOT = 0x01;        // ONESHOT
constexpr uint8_t XB_REG2_SWRESET = 0x04;        // SWRESET
constexpr uint8_t R_CTRL_REG3 = 2 + R_CTRL_REG1;
constexpr uint8_t V_CTRL_REG3 = 0x04;    // DRDY
constexpr uint8_t R_STATUS_REG = 0x27;
constexpr uint8_t R_MEAS = 0x28;

/** Measured time for a single sample at default oversample is 441
 * utt or about 13 ms. */
constexpr unsigned int ObsDelay_utt = 441;

} // ns anonymous

void
lps22hb::drdy_callback_ (const periph::GPIOTE::sense_status_type* sp)
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

lps22hb::lps22hb (notifier_type notify,
                  iface_config_type& ifc,
                  unsigned int addr) :
  super{notify},
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
  if (1 < addr) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  iface_config_.address = I2C_ADDRESS + addr;
  instr_aux.enable();
}

int
lps22hb::lpsm_sample ()
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
    cprintf("*** LPS22HB DRDY lost: %u\n", lost_drdy_);
    machine_.post_event();
  }
  return rc;
}

int
lps22hb::odr (uint8_t dr)
{
  if (lpm::state_machine::MS_OFF != machine_.state()) {
    return -1;
  }
  if (5 < dr) {
    return -2;
  }
  odr_ = dr;
  return odr_;
}

int
lps22hb::status () const
{
  auto& twi = iface_config_.twi;
  const uint8_t reg = R_STATUS_REG;
  uint8_t sv;
  int rc;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(iface_config_.address, &reg, sizeof(reg), &sv, sizeof(sv));
  } else {
    rc = enabler.result();
  }
  if (sizeof(sv) == rc) {
    rc = sv;
  }
  return rc;
}

int
lps22hb::lpsm_process_ (int& delay,
                        process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;

  auto& twi = iface_config_.twi;
  switch (machine_.state()) {
    default:
      cprintf("*** LPS22HB error state: %x\n", machine_.state());
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
      cprintf("* LPS22HB drdy psel %u: %d\n", iface_config_.drdy_psel, drdy_asserted());
      [[fallthrough]]
    case state_machine::MS_ENTRY_RESET:
      // Initiate a SWRESET
      if (auto enabler = twi.scoped_enable()) {
        static const uint8_t reset[] = {
          R_CTRL_REG2,
          V_CTRL_REG2 | XB_REG2_SWRESET,
        };
        rc = twi.write(iface_config_.address, reset, sizeof(reset));
      } else {
        rc = enabler.result();
      }
      if (0 > rc) {
        cprintf("*** LPS22HB SWRESET got %d\n",rc);
        break;
      }
      machine_.set_state(state_machine::MS_EXIT_RESET);
      [[fallthrough]]
    case state_machine::MS_EXIT_RESET:
      {
        uint8_t reg2;

        /* Determine whether SWRESET complete.  This may be expected
         * to pass on the first check. */
        if (auto enabler = twi.scoped_enable()) {
          rc = twi.write_read(iface_config_.address, &R_CTRL_REG2, 1, &reg2, 1);
        } else {
          rc = enabler.result();
        }
        if (0 > rc) {
          cprintf("*** LPS22HB SWRESET read got %d\n",rc);
          break;
        }
        if (XB_REG2_SWRESET & reg2) {
          cputs("** LPS22HB waiting reset");
          delay = uptime::from_ms(1);
          break;
        }

        /* Configure everything.  Don't assume the registers are
         * defaulted. */
        if (auto enabler = twi.scoped_enable()) {
          const uint8_t init[] = {
            R_CTRL_REG1,
            static_cast<uint8_t>(V_CTRL_REG1 | (odr_ << XP_REG1_ODR)),
            V_CTRL_REG2,
            V_CTRL_REG3,
          };
          rc = twi.write(iface_config_.address, init, sizeof(init));
        } else {
          rc = enabler.result();
        }
        if (0 > rc) {
          cprintf("*** LPS22HB ctrl set got %d\n",rc);
          break;
        }
#if 0
        {
          uint8_t raw[6];
          if (auto enabler = twi.scoped_enable()) {
            rc = twi.write_read(iface_config_.address, &R_CTRL_REG1, 1,
                                raw, 3);
          } else {
            rc = enabler.result();
          }
          cprintf("* LPS CFG got %d : %02x %02x %02x\n", rc,
                  raw[0], raw[1], raw[2]);
          if (auto enabler = twi.scoped_enable()) {
            rc = twi.write_read(iface_config_.address, &R_STATUS_REG, 1,
                                raw, 6);
          } else {
            rc = enabler.result();
          }
          cprintf("* LPS STAT got %d : %02x %02x %02x %02x %02x %02x\n", rc,
                  raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
        }
#endif

        pf |= state_machine::PF_STARTED;
        if (odr_
            || drdy_asserted()) {
          /* We either expect or have an observation-ready signal.
           * Read the data when the signal is present. */
          machine_.set_state(state_machine::MS_SAMPLE);
          cputs("** LPS22HB starting SAMPLE");
        } else {
          machine_.set_state(state_machine::MS_IDLE);
          cputs("** LPS22HB starting IDLE");
        }
      }
      /* No fallthru: we need to wait for a result. */
      break;
    case state_machine::MS_ENTRY_SAMPLE:
      if (drdy_asserted()) {
        // Something left a sample: go read it
        goto MS_SAMPLE_lbl;
      }
      if (auto enabler = twi.scoped_enable()) {
        static const uint8_t trigger[] = {
          R_CTRL_REG2,
          V_CTRL_REG2 | XB_REG2_ONESHOT,
        };
        rc = twi.write(iface_config_.address, trigger, sizeof(trigger));
        if (0 < rc) {
          machine_.set_state(state_machine::MS_SAMPLE);
#if (WITH_FALLBACK_DELAY - 0)
          // In case DRDY edge isn't caught
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
      cprintf("* LPS22HB status %02x ; drdy %d\n",
              rc, drdy_asserted());
      if (0 > rc) {
        break;
      }
      if (SR_READY != (SR_READY & rc)) {
        // Not entirely ready (maybe only partial result triggered
        // DRDY). Sleep a little longer.
#if (WITH_FALLBACK_DELAY - 0)
        delay = - uptime::from_ms(10);
#endif // WITH_FALLBACK_DELAY
        break;
      }
      {
        uint8_t buf[5];
        if (auto enabler = twi.scoped_enable()) {
          rc = twi.write_read(iface_config_.address, &R_MEAS, 1,
                              buf, sizeof(buf));
        } else {
          rc = enabler.result();
        }
        cprintf("* LPS22HB meas read got %d, drdy %d\n", rc, drdy_asserted());
        if (0 > rc) {
          cprintf("*** LPS22HB meas read failed: %d\n", rc);
          break;
        }
        unsigned int pres_raw = (buf[2] << 16) | (buf[1] << 8) | (buf[0] << 0);
        // Scale by 10000 / 4096 = 2^4*5^4 / 2^12 = 5^4 / 2^8 = 625 / 256
        observations_.pressure_cPa = (128U + 625U * pres_raw) >> 8;
        observations_.temperature_cCel = (buf[4] << 8) | (buf[3] << 0);
      }
      pf |= state_machine::PF_OBSERVATION;
      if (ODR_OneShot == odr()) {
        machine_.set_state(state_machine::MS_IDLE);
      }
      break;
    case state_machine::MS_IDLE:
      // HTS221 sometimes gets a spurious data collection shortly
      // after the calibration data is read.  If it isn't cleared
      // requesting another reading won't result in a sense interrupt
      // and we won't move out of MS_SAMPLE.
      if (drdy_asserted()) {
        cputs("** LPS22HB shorted sample");
        goto MS_SAMPLE_lbl;
      }
      break;
  }
  return rc;
}

} // ns sensor

} // ns nrfcxx
