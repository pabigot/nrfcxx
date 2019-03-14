// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Peter A. Bigot

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/sensor/sht31.hpp>

namespace nrfcxx {
namespace sensor {

namespace {

using crc_type = uint8_t;
constexpr crc_type CRC_OK = 0;

/** The 7-bit default I2C slave address for the device.  The secondary
 * address adds 1. */
constexpr uint8_t I2C_ADDRESS = 0x44;

/* Value returned from TWI infrastructure when a pending measurement
 * has not yet completed. */
constexpr static int NOT_READY = periph::TWI::error_encoded(periph::TWI::ERR_ANACK);

crc_type
crc (const uint8_t* dp,
     size_t count)
{
  constexpr uint16_t poly = 0x131;
  constexpr crc_type sobit = 0x80;
  crc_type crc = -1;
  while (0 != count--) {
    crc ^= *dp++;
    for (unsigned int bi = 0; bi < 8; ++bi) {
      crc = (crc << 1) ^ ((sobit & crc) ? poly : 0U);
    }
  }
  return crc;
}

int
write_wait (sht31::iface_config_type& ifc,
            uint16_t cmd,
            unsigned int delay_ms)
{
  union {
    uint16_t cmd;
    uint8_t raw[2];
  } u = {pabigot::byteorder::host_x_be(cmd)};
  int rc;
  auto& twi = ifc.twi;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(ifc.address, u.raw, sizeof(u.cmd));
    if (sizeof(u.cmd) == rc) {
      rc = delay_ms;
    } else {
      rc = -EIO;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

} // ns anonymous

sht31::sht31 (notifier_type setter,
              iface_config_type& ifc,
              bool secondary_addr) :
  super{setter},
  iface_config_{ifc}
{
  iface_config_.address = I2C_ADDRESS;
  if (secondary_addr) {
    iface_config_.address += 1;
  }
}

int
sht31::reset ()
{
  constexpr uint16_t CMD_SOFT_RESET = 0x30A2;

  /* NB: Contrary to the status register documentation it does not
   * appear that this command results in the System Reset Detected bit
   * being set. */
  return write_wait(iface_config_, CMD_SOFT_RESET, 2);
}

int
sht31::trigger ()
{
  /* High repeatability single-shot sample without clock stretching. */
  constexpr uint16_t CMD_SAMPLE = 0x2400;
  observations_ = {};
  return write_wait(iface_config_, CMD_SAMPLE, 15);
}

int
sht31::fetch ()
{
  int rc;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    uint8_t buf[6];

    rc = twi.read(iface_config_.address, buf, sizeof(buf));
    if (NOT_READY == rc) {
      rc = -EBUSY;
    } else if (sizeof(buf) == rc) {
      const uint8_t* rp = buf;
      rc = 0;
      if (CRC_OK == crc(rp, 3)) {
        unsigned int v = rp[0] << 8U | rp[1];
        /* T_Cel = -45 + 175 * S / (2^16 - 1) */
        observations_.temperature_cCel = -4500 + 17500 * v / ((1U << 16) - 1);
      } else {
        rc = -EIO;
      }
      rp += 3;
      if (CRC_OK == crc(rp, 3)) {
        unsigned int v = rp[0] << 8U | rp[1];
        /* RH = 100 * S / (2^16 - 1) */
        observations_.humidity_pptt = 10000U * v / ((1U << 16) - 1);
      } else {
        rc = -EIO;
      }
    } else {
      rc = -EIO;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
sht31::status ()
{
  constexpr uint16_t CMD_SR_READ = 0xF32D;
  constexpr uint16_t CMD_SR_CLEAR = 0x3041;
  int rc;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    union {
      uint16_t cmd;
      uint8_t raw[2];
    } u = {pabigot::byteorder::host_x_be(CMD_SR_READ)};
    uint8_t rsp[3];

    rc = twi.write_read(iface_config_.address,
                        u.raw, sizeof(u.cmd),
                        rsp, sizeof(rsp));
    if (sizeof(rsp) == rc) {
      rc = 0;
      if (CRC_OK == crc(rsp, sizeof(rsp))) {
        rc = rsp[0] << 8U | rsp[1];
        u.cmd = pabigot::byteorder::host_x_be(CMD_SR_CLEAR);
        (void)twi.write(iface_config_.address, u.raw, sizeof(u.raw));
      } else {
        rc = -EIO;
      }
    } else {
      rc = -EIO;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
sht31::lpsm_process_ (int& delay,
                      process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;
  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    case state_machine::MS_ENTRY_START:
      rc = reset();
      if (0 <= rc) {
        delay = uptime::from_ms(rc);
        machine_.set_state(state_machine::MS_EXIT_RESET);
        observations_ = {};
      }
      break;
    case state_machine::MS_EXIT_RESET:
      machine_.set_state(state_machine::MS_IDLE);
      pf |= state_machine::PF_STARTED;
      [[fallthrough]]
    case state_machine::MS_IDLE:
      break;
    case state_machine::MS_ENTRY_SAMPLE:
      rc = trigger();
      if (0 < rc) {
        delay = uptime::from_ms(rc);
        machine_.set_state(state_machine::MS_EXIT_SAMPLE);
      }
      break;
    case state_machine::MS_EXIT_SAMPLE:
      rc = fetch();
      if (0 == rc) {
        machine_.set_state(state_machine::MS_IDLE);
        pf |= state_machine::PF_OBSERVATION;
      } else if (-EBUSY == rc) {
        rc = 0;
        delay = uptime::from_ms(1);
      }
      break;
  }
  return rc;
}

} // ns sensor
} // ns nrfcxx
