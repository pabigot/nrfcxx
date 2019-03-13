// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/sensor/sht21.hpp>

namespace nrfcxx {
namespace sensor {

namespace {

using crc_type = uint8_t;
constexpr crc_type CRC_OK = 0;

/** The 7-bit I2C slave address for the device.  This is not
 * configurable. */
constexpr uint8_t I2C_ADDRESS = 0x40;

/* These constants define the bits for componentized basic 8-bit
 * commands described in the data sheet. */
constexpr uint8_t CMDBIT_BASE = 0xE0;
constexpr uint8_t CMDBIT_READ = 0x01;
constexpr uint8_t CMDBIT_TEMP = 0x02;
constexpr uint8_t CMDBIT_RH = 0x04;
constexpr uint8_t CMDBIT_UR = 0x06;
constexpr uint8_t CMDBIT_NOHOLD = 0x10;

/* This is a basic 8-bit command that does not conform to the
 * componentized structure used for most commands. */
constexpr uint8_t SOFT_RESET = 0xFE;

constexpr unsigned int SAMPLE_TYPE_Pos = 1;
constexpr uint16_t SAMPLE_TYPE_Temperature = 0;
constexpr uint16_t SAMPLE_TYPE_Humidity = 1;
constexpr uint16_t SAMPLE_TYPE_Msk = (1U << SAMPLE_TYPE_Pos);

constexpr uint16_t STATUS_Msk = 0x0003;

int
convert_pptt (uint16_t raw)
{
  if ((SAMPLE_TYPE_Humidity << SAMPLE_TYPE_Pos)
      != (SAMPLE_TYPE_Msk & raw)) {
    using nrfcxx::periph::TWI;
    return TWI::error_encoded(TWI::ERR_INVALID);
  }
  /* RH_pph = -6 + 125 * S / 2^16 */
  raw &= ~STATUS_Msk;
  return ((12500U * raw) >> 16) - 600;
}

int
convert_cK (uint16_t raw)
{
  if ((SAMPLE_TYPE_Temperature << SAMPLE_TYPE_Pos)
      != (SAMPLE_TYPE_Msk & raw)) {
    using nrfcxx::periph::TWI;
    return TWI::error_encoded(TWI::ERR_INVALID);
  }
  /* T_Cel = -46.85 + 175.72 * S / 2^16
   * T_cK = 27315 - 4685 + 17572 * S / 2^16
   *      = 22630 + 17572 * S / 2^16
   */
  raw &= ~STATUS_Msk;
  return 22630U + ((17572U * raw) >> 16);
}

crc_type
crc (const uint8_t* dp,
     size_t count)
{
  constexpr uint16_t poly = 0x131;
  constexpr crc_type sobit = 0x80;
  crc_type crc = 0;
  while (0 != count--) {
    crc ^= *dp++;
    for (unsigned int bi = 0; bi < 8; ++bi) {
      crc = (crc << 1) ^ ((sobit & crc) ? poly : 0U);
    }
  }
  return crc;
}

} // ns anonymous

sht21::sht21 (notifier_type setter,
              iface_config_type& ifc) :
  super{setter},
  iface_config_{ifc},
  observations_{}
{
  iface_config_.address = I2C_ADDRESS;
}

int
sht21::reset ()
{
  uint8_t cmd = SOFT_RESET;
  int rc;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(I2C_ADDRESS, &cmd, sizeof(cmd));
    if (sizeof(cmd) == rc) {
      rc = 0;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
sht21::configure (int config)
{
  int rv;
  uint8_t data[2];
  uint8_t* dp = data;
  const uint8_t* dpe = dp+1;
  *dp++ = CMDBIT_BASE | CMDBIT_UR | CMDBIT_READ;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    rv = twi.write_read(I2C_ADDRESS, data, dp - data, data, dpe - data);
    if ((dpe - data) == rv) {
      rv = data[0];
      if (0 <= config) {
        dp = data;
        *dp++ = CMDBIT_BASE | CMDBIT_UR;
        *dp++ = (config & (CONFIG_RES_Msk | CONFIG_HEATER)) | CONFIG_OTPRn;
        /* If the write fails, the error supersedes the succesful read. */
        int rc = twi.write(I2C_ADDRESS, data, dp - data);
        if ((dp - data) != rc) {
          rv = rc;
        }
      }
    }
  } else {
    rv = enabler.result();
  }
  return rv;
}

int
sht21::read_eic (eic_type& eic)
{
  using nrfcxx::periph::TWI;

  auto& twi = iface_config_.twi;
  auto enabler = twi.scoped_enable();
  if (!enabler) {
    return enabler.result();
  }

  uint8_t data[16];
  auto dp = data;
  const uint8_t* dpe = dp + 8;
  *dp++ = 0xFA;
  *dp++ = 0x0F;
  int rc = twi.write_read(I2C_ADDRESS, data, dp - data, data, dpe - data);
  if ((dpe - data) != rc) {
    return rc;
  }
  if ((CRC_OK != crc(data, 2))
      || (CRC_OK != crc(data+2, 2))
      || (CRC_OK != crc(data+4, 2))
      || (CRC_OK != crc(data+6, 2))) {
    return TWI::error_encoded(TWI::ERR_CHECKSUM);
  }
  eic[2] = data[0];
  eic[3] = data[2];
  eic[4] = data[4];
  eic[5] = data[6];

  dp = data;
  dpe = dp+6;
  *dp++ = 0xFC;
  *dp++ = 0xC9;
  rc = twi.write_read(I2C_ADDRESS, data, dp - data, data, dpe - data);
  if ((dpe - data) != rc) {
    return rc;
  }
  if ((CRC_OK != crc(data, 3))
      || (CRC_OK != crc(data + 3, 3))) {
    return TWI::error_encoded(TWI::ERR_CHECKSUM);
  }
  eic[6] = data[0];
  eic[7] = data[1];
  eic[0] = data[3];
  eic[1] = data[4];

  return 0;
}

int
sht21::trigger_ (bool humidity)
{
  uint8_t cmd = CMDBIT_BASE | CMDBIT_READ | CMDBIT_NOHOLD;
  cmd |= (humidity ? CMDBIT_RH : CMDBIT_TEMP);
  int rc = -1;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(I2C_ADDRESS, &cmd, sizeof(cmd));
    if (sizeof(cmd) == rc) {
      rc = 0;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
sht21::fetch_ (bool humidity)
{
  using nrfcxx::periph::TWI;

  uint8_t data[3];
  int rc = -1;
  auto& twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    // If result is ready this will return NOT_READY
    rc = twi.read(I2C_ADDRESS, data, sizeof(data));
    if (sizeof(data) == rc) {
      uint16_t raw = (data[0] << 8) | data[1];
      if (CRC_OK != crc(data, sizeof(data))) {
        rc = TWI::error_encoded(TWI::ERR_CHECKSUM);
      } else if (humidity) {
        rc = convert_pptt(raw);
      } else {
        rc = convert_cK(raw);
      }
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

namespace {
constexpr lpm::state_machine::state_type MS_SAMPLE_TEMP = lpm::state_machine::MS_ENTRY_SAMPLE + 0;
constexpr lpm::state_machine::state_type MS_EXIT_TEMP = lpm::state_machine::MS_EXIT_SAMPLE + 0;
constexpr lpm::state_machine::state_type MS_SAMPLE_HUMIDITY = lpm::state_machine::MS_ENTRY_SAMPLE + 1;
constexpr lpm::state_machine::state_type MS_EXIT_HUMIDITY = lpm::state_machine::MS_EXIT_SAMPLE + 1;
} // ns anonymous

int
sht21::lpsm_process_ (int& delay,
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
        delay = uptime::from_ms(sht21::RESET_DELAY_ms);
        machine_.set_state(state_machine::MS_EXIT_RESET);
        observations_ = {};
      }
      break;
    case state_machine::MS_EXIT_RESET:
      rc = configure(sht21::CONFIG_RES_H12T14);
      if (0 > rc) {
        break;
      }
      [[fallthrough]]
    case MS_SAMPLE_TEMP:
      rc = trigger_temperature();
      if (0 <= rc) {
        delay = uptime::from_ms(sht21::SAMPLE14_DELAY_ms);
        machine_.set_state(MS_EXIT_TEMP);
      }
      break;
    case MS_EXIT_TEMP:
      rc = temperature_cK();
      if (NOT_READY == rc) {
        delay = uptime::from_ms(1);
        rc = 0;
        break;
      }
      if (0 > rc) {
        break;
      }
      observations_.temperature_cK = rc;
      [[fallthrough]]
    case MS_SAMPLE_HUMIDITY:
      rc = trigger_humidity();
      if (0 <= rc) {
        delay = uptime::from_ms(sht21::SAMPLE12_DELAY_ms);
        machine_.set_state(MS_EXIT_HUMIDITY);
      }
      break;
    case MS_EXIT_HUMIDITY:
      rc = humidity_pptt();
      if (NOT_READY == rc) {
        delay = uptime::from_ms(1);
        break;
      }
      if (0 > rc) {
        break;
      }
      observations_.humidity_pptt = rc;
      machine_.set_state(state_machine::MS_IDLE);
      pf |= state_machine::PF_OBSERVATION;
      break;
  }
  return rc;
}

} // ns sensor
} // ns nrfcxx
