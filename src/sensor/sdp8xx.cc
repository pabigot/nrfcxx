// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/sensor/sdp8xx.hpp>

namespace nrfcxx {
namespace sensor {

namespace {

/** The 7-bit I2C slave address for the device.  This is not
 * configurable. */
constexpr uint8_t I2C_ADDRESS = 0x25;

using crc_type = uint8_t;
constexpr crc_type CRC_OK = 0;

crc_type
crc (const uint8_t* dp,
     size_t count)
{
  constexpr uint16_t poly = 0x131;
  constexpr crc_type sobit = 0x80;
  crc_type crc = 0xFF;
  while (0 != count--) {
    crc ^= *dp++;
    for (unsigned int bi = 0; bi < 8; ++bi) {
      crc = (crc << 1) ^ ((sobit & crc) ? poly : 0U);
    }
  }
  return crc;
}

using cuint8_pointer = const uint8_t*;

/** Utility to pull out 16-bit unsigned chunks with checksum variation.
 *
 * @param[inout] sp pointer into the I2C response buffer at the start
 * of a 16-bit value.  On exit sp is updated to the next half-word to
 * extract, if @p error was false on input and was not set by the
 * call.
 *
 * @param[inout] error set when the extraction identifies a checksum error.
 *
 * @return zero if @p error is set on input or output, otherwise the
 * extracted half-word. */
uint16_t
extract_hword (const uint8_t* &sp,
               bool &error)
{
  uint16_t rv = 0;
  if (!error) {
    if (sp[2] != crc(sp, 2)) {
      error = true;
    } else {
      rv = (sp[0] << 8) | sp[1];
      sp += 3;
    }
  }

  return rv;
}

} // ns anonymous

sdp8xx::sdp8xx (notifier_type notify,
                iface_config_type& ifc) :
  super{notify},
  iface_config_{ifc},
  observations_{}
{
  iface_config_.address = I2C_ADDRESS;
}

int
sdp8xx::reset ()
{
  int rv = 0;
  /* Reset is implemented as a broadcast operation. */
  uint8_t cmd = 0x06;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    rv = twi.write(0, &cmd, sizeof(cmd));
  } else {
    rv = enabler.result();
  }
  return rv;
}


int
sdp8xx::read_device_info (uint32_t& product,
                          uint64_t& serial)
{
  static constexpr uint16_t CMD_READ_PI1 = 0x367C;
  static constexpr uint16_t CMD_READ_PI2 = 0xE102;

  using nrfcxx::periph::TWI;

  //constexpr auto rv_error = TWI::error_encoded(TWI::ERR_UNKNOWN);
  uint8_t buf[18];
  auto dp = buf;

  *dp++ = CMD_READ_PI1 >> 8;
  *dp++ = CMD_READ_PI1 & 0xFF;
  int rc;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(I2C_ADDRESS, buf, dp - buf);
    if ((dp - buf) == rc) {
      dp = buf;
      *dp++ = CMD_READ_PI2 >> 8;
      *dp++ = CMD_READ_PI2 & 0xFF;
      rc = twi.write_read(I2C_ADDRESS, buf, dp - buf, buf, 18);
    }
    if (18 == rc) {
      const uint8_t* sp = buf;
      bool error = false;
      product = static_cast<uint32_t>(extract_hword(sp, error)) << 16;
      product |= static_cast<uint32_t>(extract_hword(sp, error));
      serial = static_cast<uint64_t>(extract_hword(sp, error)) << 48;
      serial |= static_cast<uint64_t>(extract_hword(sp, error)) << 32;
      serial |= static_cast<uint64_t>(extract_hword(sp, error)) << 16;
      serial |= static_cast<uint64_t>(extract_hword(sp, error));
      rc = -error;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
sdp8xx::lpsm_process_ (int& delay,
                       process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;
  auto& twi = iface_config_.twi;
  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    case state_machine::MS_ENTRY_START:
      rc = reset();
      if (0 <= rc) {
        delay = uptime::from_ms(sdp8xx::RESET_DELAY_ms);
        machine_.set_state(state_machine::MS_ENTRY_SAMPLE);
        observations_ = {};
        scaleFactor_ = 0;
      }
      break;
    case state_machine::MS_ENTRY_SAMPLE: {
      static constexpr uint16_t CMD_TRIG_DP_POLL = 0x362F;
      uint8_t buf[] = { CMD_TRIG_DP_POLL >> 8,
                        CMD_TRIG_DP_POLL & 0xFF };
      if (auto enabler = twi.scoped_enable()) {
        rc = twi.write(I2C_ADDRESS, buf, sizeof(buf));
      } else {
        rc = enabler.result();
      }
      if (sizeof(buf) == rc) {
        delay = uptime::from_ms(sdp8xx::SAMPLE_DELAY_ms);
        machine_.set_state(state_machine::MS_EXIT_SAMPLE);
      }
      break;
    }
    case state_machine::MS_EXIT_SAMPLE: {
      uint8_t buf[9];
      const size_t count = (0 == scaleFactor_) ? 9 : 6;
      if (auto enabler = twi.scoped_enable()) {
        rc = twi.read(I2C_ADDRESS, buf, count);
      } else {
        rc = enabler.result();
      }
      constexpr int NOT_READY = periph::TWI::error_encoded(periph::TWI::ERR_ANACK);
      if (NOT_READY == rc) {
        delay = uptime::from_ms(1);
        rc = 0;
        break;
      }
      if (0 > rc) {
        break;
      }
      const uint8_t* sp = buf;
      bool error = false;
      uint16_t dp_raw = extract_hword(sp, error);
      uint16_t temp_raw = extract_hword(sp, error);
      if ((0 == scaleFactor_) && (!error)) {
        scaleFactor_ = extract_hword(sp, error);
        if (0 == scaleFactor_) {
          machine_.set_error(0x3000);
        }
      }
      if (error) {
        machine_.set_error(0x4000);
      } else if (0 != scaleFactor_) {
        observations_.temperature_cCel = (1 + static_cast<int16_t>(temp_raw)) / 2;
        observations_.diffpres_cPa = ((scaleFactor_ / 2) + 100 * static_cast<int16_t>(dp_raw)) / scaleFactor_;
        machine_.set_state(state_machine::MS_IDLE);
        pf |= state_machine::PF_OBSERVATION;
      }
      break;
    }
  }
  return rc;
}

} // ns sensor
} // ns nrfcxx
