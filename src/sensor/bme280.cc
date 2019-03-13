// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2019 Peter A. Bigot

#include <cstring>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/sensor/bme280.hpp>

namespace nrfcxx {
namespace sensor {

namespace {

/* This function only gets invoked by the vendor library in these circumstances:
 * * with a 1 ms delay up to 5 times when bme280_init() fails
 *   internally (bme280_soft_reset invoked each time);
 * * with a 2 ms delay on each invocation of bme280_init(). */
void
delay_ms (uint32_t period)
{
  using nrfcxx::clock::uptime;
  uptime::sleep(uptime::from_ms(period));
}

} // anonymous

void
bme280::init_dev_ (bool spi,
                   bool addr_sec)
{
  using namespace nrfcxx;

  if ((spi && spip_)
      || ((!spi) && twip_)) {
    failsafe(FailSafeCode::RESOURCE_VIOLATION);
  }
  memset(&dev, 0, sizeof(dev));
  dev.delay_ms = delay_ms;
  if (spi) {
    dev.intf = BME280_SPI_INTF;
    dev.read = spi_read;
    dev.write = spi_write;
  } else {
    dev.dev_id = addr_sec ? BME280_I2C_ADDR_SEC : BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = twi_read;
    dev.write = twi_write;
  }
#if 0
  /* Configuration for indoor navigation.
   * T = 1.25
   *     + (2.3 * 16 + 0.575)
   *     + (2.3 * 2)
   *     + (2.3 * 1 + 0.575)
   *   = 46.1 ms */
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
#else
  /* Configuration for weather monitoring.
   * T = 1.25
   *     + (2.3 * 1 + 0.575)
   *     + (2.3 * 1)
   *     + (2.3 * 1 + 0.575)
   *   = 9.3 ms */
  dev.settings.osr_p = BME280_OVERSAMPLING_1X;
  dev.settings.osr_t = BME280_OVERSAMPLING_1X;
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.filter = BME280_FILTER_COEFF_OFF;
#endif
}

bme280::bme280 (notifier_type notify,
                nrfcxx::periph::SPI& spi,
                int csn_psel) :
  super{notify}
{
  using namespace nrfcxx;

  init_dev_(true);
  spip_ = &spi;
  if ((0 > csn_psel) || (31 < csn_psel)) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  csn_psel_ = csn_psel;
  nrf5::GPIO->OUTSET = (1U << csn_psel_);
  nrf5::GPIO->PIN_CNF[csn_psel_] = gpio::PIN_CNF_WRONLY;
}

bme280::bme280 (notifier_type notify,
                nrfcxx::periph::TWI& twi,
                bool addr_sec) :
  super{notify}
{
  init_dev_(false, addr_sec);
  twip_ = &twi;
}

nrfcxx::periph::SPI * bme280::spip_;
nrfcxx::periph::TWI * bme280::twip_;
uint8_t bme280::csn_psel_;

int8_t
bme280::spi_read (uint8_t dev_id,
                  uint8_t reg_addr,
                  uint8_t* data,
                  uint16_t len)
{
  using namespace nrfcxx;

  if (!spip_) {
    return -1;
  }
  auto& spi = *spip_;

  int rc;
  nrf5::GPIO->OUTCLR = (1U << csn_psel_);
  do {
    rc = spi.tx_rx(&reg_addr, sizeof(reg_addr), 0, nullptr);
    if (sizeof(reg_addr) == rc) {
      rc = spi.tx_rx(nullptr, 0, len, data);
    }
    if (len == rc) {
      rc = 0;
    } else {
      rc = -1;
    }
  } while (false);
  nrf5::GPIO->OUTSET = (1U << csn_psel_);
  return rc;
}

int8_t
bme280::spi_write (uint8_t dev_id,
                   uint8_t reg_addr,
                   uint8_t* data,
                   uint16_t len)
{
  using namespace nrfcxx;

  if (!spip_) {
    return -1;
  }
  auto& spi = *spip_;

  int rc;
  nrf5::GPIO->OUTCLR = (1U << csn_psel_);
  do {
    rc = spi.tx_rx(&reg_addr, sizeof(reg_addr), 0, nullptr);
    if (sizeof(reg_addr) == rc) {
      rc = spi.tx_rx(data, len, 0, nullptr);
    }
    if (len == rc) {
      rc = 0;
    } else {
      rc = -1;
    }
  } while (false);
  nrf5::GPIO->OUTSET = (1U << csn_psel_);

  return rc;
}


int8_t
bme280::twi_read (uint8_t dev_id,
                  uint8_t reg_addr,
                  uint8_t* data,
                  uint16_t len)
{
  if (!twip_) {
    return -1;
  }
  auto& twi = *twip_;

  int rc;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(dev_id, &reg_addr, sizeof(reg_addr), data, len);
    if (len == rc) {
      rc = 0;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int8_t
bme280::twi_write (uint8_t dev_id,
                   uint8_t reg_addr,
                   uint8_t* data,
                   uint16_t len)
{
  if (!twip_) {
    return -1;
  }
  auto& twi = *twip_;

  uint8_t buf[1 + len];
  auto bp = buf;
  *bp++ = reg_addr;
  memmove(bp, data, len);

  int rc;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(dev_id, buf, 1 + len);
    if ((1 + len) == rc) {
      rc = 0;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
bme280::initialize ()
{
  static auto init_rc = ::bme280_init(&dev);
  return init_rc;
}

int
bme280::fetch (observations_type& obs)
{
  ::bme280_data comp_data{};

  int rc = ::bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
  if (0 == rc) {
    // 100 * C means cCel, which we want
    obs.temperature_cCel = comp_data.temperature;
    // 100 * Pa means cPa, which we want
    obs.pressure_cPa = comp_data.pressure;
    // 1024 * % means %/1024 , scale and round from pph to pptt
    obs.humidity_pptt = (50 + 100 * comp_data.humidity) / 1024;
  }
  return rc;
}

int
bme280::lpsm_process_ (int& delay,
                       process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;
  bool enabled = false;
  if (BME280_SPI_INTF == dev.intf) {
    enabled = spip_->enabled();
    if (!enabled) {
      spip_->enable();
    }
  }
  using nrfcxx::clock::uptime;
  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    case state_machine::MS_ENTRY_START:
      rc = initialize();
      if (0 == rc) {
        /* Transfer settings from the device structure to the device. */
        uint8_t settings_sel = BME280_OSR_PRESS_SEL;
        settings_sel |= BME280_OSR_TEMP_SEL;
        settings_sel |= BME280_OSR_HUM_SEL;
        settings_sel |= BME280_FILTER_SEL;
        rc = bme280_set_sensor_settings(settings_sel, &dev);
        if (0 == rc) {
          machine_.set_state(state_machine::MS_ENTRY_SAMPLE, true);
          observations_ = {};
        }
      }
      break;
    case state_machine::MS_ENTRY_SAMPLE:
      rc = sample();
      if (0 == rc) {
        delay = SAMPLE_DELAY_utt;
        machine_.set_state(state_machine::MS_EXIT_SAMPLE);
      } else if (0 < rc) {
        rc = -rc;
      }
      break;
    case state_machine::MS_EXIT_SAMPLE:
      rc = fetch(observations_);
      if (0 == rc) {
        machine_.set_state(state_machine::MS_IDLE);
        pf |= state_machine::PF_OBSERVATION;
      }
      break;
  }
  if ((!enabled) && (BME280_SPI_INTF == dev.intf)) {
    spip_->disable();
  }
  return rc;
}

} // ns sensor
} // ns nrfcxx
