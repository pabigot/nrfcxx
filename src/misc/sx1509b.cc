// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/impl.hpp>
#include <nrfcxx/misc/sx1509b.hpp>
#include <pabigot/byteorder.hpp>

namespace {

struct reg16_rw_s
{
  uint8_t reg;
  uint16_t value_be;
} __attribute__((__packed__));

constexpr uint8_t I2C_ADDR = 0x3E;
constexpr uint8_t RegInputDisable = 0x00;
constexpr uint8_t RegPullUp = 0x06;
constexpr uint8_t RegDir = 0x0E;
constexpr uint8_t RegData = 0x10;
constexpr uint8_t RegClock = 0x1E;
constexpr uint8_t RegMisc = 0x1F;
constexpr uint8_t RegLEDDriverEnable = 0x20;
constexpr uint8_t RegReset = 0x7D;

constexpr auto t_RESET_ms = 3U;   // 2.5 ms, rounded up

int
led_register (unsigned int li,
              bool& pwm_ok)
{
  using led_type = nrfcxx::misc::sx1509b::led_type;
  using led_pwm_type = nrfcxx::misc::sx1509b::led_pwm_type;

  uint8_t addr = 0x29;
  while (addr < 0x69) {
    if (4 > li) {
      pwm_ok = false;
      return addr + sizeof(led_type) * li;
    }
    addr += 4 * sizeof(led_type);
    li -= 4;
    if (4 > li) {
      pwm_ok = true;
      return addr + sizeof(led_pwm_type) * li;
    }
    addr += 4 * sizeof(led_pwm_type);
    li -= 4;
  }
  return -1;
}

int
read (nrfcxx::periph::TWI& twi,
      uint8_t i2c_addr,
      uint8_t reg,
      void* dp,
      size_t span)
{
  int rc;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(i2c_addr, &reg, 1,
                        reinterpret_cast<uint8_t*>(dp), span);
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
write (nrfcxx::periph::TWI& twi,
       uint8_t i2c_addr,
       uint8_t reg,
       const void* sp,
       size_t span)
{
  int rc;
  uint8_t buf[1 + span];
  buf[0] = reg;
  memcpy(buf + 1, sp, span);
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(i2c_addr, buf, 1 + span);
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
read8 (nrfcxx::periph::TWI& twi,
       uint8_t i2c_addr,
       uint8_t reg)
{
  int rc;
  uint8_t rv;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(i2c_addr, &reg, 1, &rv, sizeof(rv));
  } else {
    rc = enabler.result();
  }
  if (sizeof(rv) == rc) {
    rc = rv;
  }
  return rc;
}

int
read16 (nrfcxx::periph::TWI& twi,
       uint8_t i2c_addr,
       uint8_t reg)
{
  int rc;
  uint16_t rv;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(i2c_addr, &reg, 1,
                        reinterpret_cast<uint8_t*>(&rv), sizeof(rv));
    if (sizeof(rv) == rc) {
      rc = pabigot::byteorder::host_x_be(rv);
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
write8 (nrfcxx::periph::TWI& twi,
        uint8_t i2c_addr,
        uint8_t reg,
        uint8_t value)
{
  int rc;
  if (auto enabler = twi.scoped_enable()) {
    uint8_t buf[] = {reg, value};
    rc = twi.write(i2c_addr, buf, sizeof(buf));
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
write_helper (nrfcxx::periph::TWI& twi,
              uint8_t i2c_addr,
              const pabigot::byteorder::octets_helper &oh)
{
  int rc;
  if (!oh.valid()) {
    return -EINVAL;
  }
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(i2c_addr, static_cast<const uint8_t*>(oh.begin()), oh.size());
  } else {
    rc = enabler.result();
  }
  return rc;
}

} // ns anonymous

namespace nrfcxx {
namespace misc {

sx1509b::sx1509b (iface_config_type& ifc,
                  unsigned int addr) :
  iface_config_{ifc},
  resetn_{gpio::pin_reference::create(iface_config_.resetn_psel)}
{
  if (3 < addr) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  ifc.address = I2C_ADDR + addr;
  resetn_.set();
  resetn_.configure(gpio::PIN_CNF_WRONLY);
}

int
sx1509b::hw_reset ()
{
  resetn_.clear();
  delay_us(1);                  // t_PULSE = 200 ns
  resetn_.set();
  reset_cache_();
  return t_RESET_ms;            // t_RESET = 2.5 ms
}

int
sx1509b::sw_reset ()
{
  uint8_t buf[] = {RegReset, 0x12};
  int rc;
  auto twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(iface_config_.address, buf, sizeof(buf));
    if (sizeof(buf) == rc) {
      buf[1] = 0x34;
      rc = twi.write(iface_config_.address, buf, sizeof(buf));
    }
    if (sizeof(buf) == rc) {
      reset_cache_();
      rc = t_RESET_ms;
    }
  } else {
    rc = enabler.result();
  }
  return rc;
}

void
sx1509b::reset_cache_ ()
{
  gpio_cache_ = {};
  led_driver_cache_ = {};
  clock_cache_ = {};
  misc_cache_ = {};
}

int
sx1509b::reload_cache ()
{
  int rc;
  auto twi = iface_config_.twi;
  if (auto enabler = twi.scoped_enable()) {
    do {
      auto addr = iface_config_.address;
      rc = read(twi, addr, RegInputDisable, &gpio_cache_, sizeof(gpio_cache_));
      if (sizeof(gpio_cache_) != rc) {
        break;
      }
      auto sp = reinterpret_cast<uint16_t*>(&gpio_cache_);
      auto const spe = sp + sizeof(gpio_cache_) / sizeof(*sp);
      while (sp < spe) {
        *sp = pabigot::byteorder::host_x_be(*sp);
        ++sp;
      }
      rc = read16(twi, addr, RegLEDDriverEnable);
      if (0 > rc) {
        break;
      }
      led_driver_cache_ = rc;
      uint8_t buf[2];
      rc = read(twi, addr, RegClock, buf, sizeof(buf));
      if (0 > rc) {
        break;
      }
      clock_cache_ = buf[0];
      misc_cache_ = buf[1];
    } while (false);
  } else {
    rc = enabler.result();
  }
  return rc;
}

int
sx1509b::clock () const
{
  return read8(iface_config_.twi, iface_config_.address, RegClock);
}

int
sx1509b::clock (uint8_t value)
{
  auto rc = write8(iface_config_.twi, iface_config_.address, RegClock, value);
  if (0 <= rc) {
    clock_cache_ = value;
  }
  return rc;
}

int
sx1509b::misc () const
{
  return read8(iface_config_.twi, iface_config_.address, RegMisc);
}

int
sx1509b::misc (uint8_t value)
{
  auto rc = write8(iface_config_.twi, iface_config_.address, RegMisc, value);
  if (0 <= rc) {
    misc_cache_ = value;
  }
  return rc;
}

int
sx1509b::configure_as_leds (uint16_t leds,
                            bool source_current)
{
  uint8_t buf[1 + sizeof(gpio_cache_)];
  pabigot::byteorder::octets_helper oh{buf, sizeof(buf)};
  int rc;
  auto twi = iface_config_.twi;

  do {
    auto data = gpio_cache_.data;
    data |= leds;                 // Default active low, start with LEDs off
    if (gpio_cache_.data != data) {
      gpio_cache_.data = data;
      oh.reset();
      oh.append<uint8_t>(RegData);
      oh.append_be(gpio_cache_.data);
      rc = write_helper(twi, iface_config_.address, oh);
      if (0 > rc) {
        break;
      }
    }

    gpio_cache_.input_disable |= leds; // Disable input buffer
    gpio_cache_.pull_up &= ~leds;      // Disable pull-up
    gpio_cache_.pull_down &= ~leds;    // Disable pull-down
    if (source_current) {
      // For LED source current
      gpio_cache_.open_drain &= ~leds; // Disable open drain
    } else {
      // For LED sink current
      gpio_cache_.open_drain |= leds;  // Enable open drain
    }
    gpio_cache_.dir &= ~leds;          // Set output
    led_driver_cache_ |= leds;         // Enable LED driver

    oh.reset();
    oh.append<uint8_t>(RegInputDisable);
    oh.append_be(gpio_cache_.input_disable);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }

    oh.reset();
    oh.append<uint8_t>(RegPullUp);
    oh.append_be(gpio_cache_.pull_up);
    oh.append_be(gpio_cache_.pull_down);
    oh.append_be(gpio_cache_.open_drain);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }

    oh.reset();
    oh.append<uint8_t>(RegDir);
    oh.append_be(gpio_cache_.dir);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }

    oh.reset();
    oh.append<uint8_t>(RegLEDDriverEnable);
    oh.append_be(led_driver_cache_);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }
  } while (false);
  return rc;
}

unsigned int
sx1509b::configuration (unsigned int psel) const
{
  if (psel != (0x0F & psel)) {
    return -1;
  }
  uint16_t bit = 1U << psel;

  unsigned int rv = 0;

  // InputDisable low corresponds to INPUT.Connect (INPUT.Disconnect is high)
  if (gpio_cache_.input_disable & bit) {
    rv |= (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
  }
  // Slew not supported
  // Dir low corresponds to DIR.Output (DIR.Input is zero)
  if (!(gpio_cache_.dir & bit)) {
    rv |= (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  }
  // Pull: we assume both are not set (PULL.Disabled is zero)
  if (gpio_cache_.pull_up & bit) {
    rv |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
  } else if (gpio_cache_.pull_down & bit) {
    rv |= (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
  }
  // Drive currently not supported
  // Sense not supported
  return rv;
}

int
sx1509b::multiconfigure (uint16_t out_set,
                         uint16_t out_clear,
                         uint16_t pull_up,
                         uint16_t pull_down) noexcept
{
  int rc;

  /* Device defaults to inputs; we're defaulting to output, clear, no
   * pull. */
  gpio_cache_.dir = 0;
  gpio_cache_.data = 0;
  gpio_cache_.pull_up = 0;
  gpio_cache_.pull_down = 0;
  uint16_t bit = 1;
  while (bit) {
    if (((out_set | out_clear) & bit)) {
      // output: dir clear, data value
      if (out_set & bit) {
        gpio_cache_.data |= bit;
      }
    } else {
      // input: dir set
      gpio_cache_.dir |= bit;
    }
    if (pull_up & bit) {
      gpio_cache_.pull_up |= bit;
    } else if (pull_down & bit) {
      gpio_cache_.pull_down |= bit;
    }
    bit <<= 1;
  }

  uint8_t buf[1 + 4 * sizeof(uint16_t)];
  pabigot::byteorder::octets_helper oh{buf, sizeof(buf)};

  auto twi = iface_config_.twi;
  auto addr = iface_config_.address;

  do {
    // Data first.
    oh.reset();
    oh.append<uint8_t>(RegData);
    oh.append_be(gpio_cache_.data);
    rc = write_helper(twi, addr, oh);
    if (0 > rc) {
      break;
    }

    // Pull next.
    oh.reset();
    oh.append<uint8_t>(RegPullUp);
    oh.append_be(gpio_cache_.pull_up);
    oh.append_be(gpio_cache_.pull_down);
    rc = write_helper(twi, addr, oh);
    if (0 > rc) {
      break;
    }

    // Direction last.
    oh.reset();
    oh.append<uint8_t>(RegDir);
    oh.append_be(gpio_cache_.dir);
    rc = write_helper(twi, addr, oh);
  } while (false);
  return rc;
}

int
sx1509b::configure (unsigned int psel,
                    unsigned int pin_cnf,
                    int initial)
{
  if (psel != (0x0F & psel)) {
    return -1;
  }
  uint16_t bit = 1U << psel;

  uint8_t buf[1 + sizeof(gpio_cache_)];
  pabigot::byteorder::octets_helper oh{buf, sizeof(buf)};
  int rc;
  auto twi = iface_config_.twi;

  do {
    /* Disable LED driver, if enabled. */
    if (led_driver_cache_ & bit) {
      led_driver_cache_ &= ~bit;
      oh.reset();
      oh.append<uint8_t>(RegLEDDriverEnable);
      oh.append_be(led_driver_cache_);
      rc = write_helper(twi, iface_config_.address, oh);
      if (0 > rc) {
        break;
      }
    }

    /* Set the initial output value, if requested and different. */
    if (0 <= initial) {
      auto data = gpio_cache_.data;
      if (0 == initial) {
        data &= ~bit;
      } else {
        data |= bit;
      }
      if (gpio_cache_.data != data) {
        gpio_cache_.data = data;
        oh.reset();
        oh.append<uint8_t>(RegData);
        oh.append_be(gpio_cache_.data);
        rc = write_helper(twi, iface_config_.address, oh);
        if (0 > rc) {
          break;
        }
      }
    }

    if ((GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) & pin_cnf) {
      gpio_cache_.input_disable |= bit;
    } else { // connected
      gpio_cache_.input_disable &= ~bit;
    }
    if ((GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) & pin_cnf) {
      gpio_cache_.dir &= ~bit;
    } else { // input
      gpio_cache_.dir |= bit;
    }
    unsigned int pull = (GPIO_PIN_CNF_PULL_Msk & pin_cnf) >> GPIO_PIN_CNF_PULL_Pos;
    gpio_cache_.pull_up &= ~bit;
    gpio_cache_.pull_down &= ~bit;
    if (GPIO_PIN_CNF_PULL_Pulldown == pull) {
      gpio_cache_.pull_down |= bit;
    } else if (GPIO_PIN_CNF_PULL_Pullup == pull) {
      gpio_cache_.pull_up |= bit;
    }
    gpio_cache_.open_drain &= ~bit; // clear possibly set as LED

    oh.reset();
    oh.append<uint8_t>(RegInputDisable);
    oh.append_be(gpio_cache_.input_disable);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }

    oh.reset();
    oh.append<uint8_t>(RegPullUp);
    oh.append_be(gpio_cache_.pull_up);
    oh.append_be(gpio_cache_.pull_down);
    oh.append_be(gpio_cache_.open_drain);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }

    oh.reset();
    oh.append<uint8_t>(RegDir);
    oh.append_be(gpio_cache_.dir);
    rc = write_helper(twi, iface_config_.address, oh);
    if (0 > rc) {
      break;
    }

  } while (false);
  if (0 > rc) {
    // @todo failed, cache out of sync
  }
  return rc;
}

int
sx1509b::output_sct (uint16_t set,
                     uint16_t clear)
{
  using namespace pabigot::byteorder;

  auto& data = gpio_cache_.data;
  if (!(set || clear)) {
    return data;
  }
  data ^= (set & clear);
  data &= ~(clear & ~set);
  data |= set & ~clear;

  reg16_rw_s s{
    .reg = 0x10,
    .value_be = pabigot::byteorder::host_x_be(data),
  };
  auto& twi = iface_config_.twi;
  int rc;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write(iface_config_.address, &s.reg, sizeof(s));
  } else {
    rc = enabler.result();
  }
  if (sizeof(s) == rc) {
    rc = data;
  } else {
    // @todo cache and device are inconsistent.  Failsafe?
  }
  return rc;
}

int
sx1509b::led_configuration (unsigned int psel,
                            led_type& cfg) const
{
  bool pwm_ok;
  int rc = led_register(psel, pwm_ok);
  if (0 <= rc) {
    rc = read(iface_config_.twi, iface_config_.address, rc, &cfg, sizeof(cfg));
  }
  return rc;
}

int
sx1509b::led_configuration (unsigned int psel,
                            led_pwm_type& cfg) const
{
  bool pwm_ok;
  int rc = led_register(psel, pwm_ok);
  if (0 <= rc) {
    if (pwm_ok) {
      rc = read(iface_config_.twi, iface_config_.address, rc, &cfg, sizeof(cfg));
    } else {
      rc = -1;
    }
  }
  return rc;
}

int
sx1509b::led_configure (unsigned int psel,
                        const led_type& cfg) const
{
  bool pwm_ok;
  int rc = led_register(psel, pwm_ok);
  if (0 <= rc) {
    rc = write(iface_config_.twi, iface_config_.address, rc, &cfg, sizeof(cfg));
  }
  return rc;
}

int
sx1509b::led_configure (unsigned int psel,
                        const led_pwm_type& cfg) const
{
  bool pwm_ok;
  int rc = led_register(psel, pwm_ok);
  if (0 <= rc) {
    if (pwm_ok) {
      rc = write(iface_config_.twi, iface_config_.address, rc, &cfg, sizeof(cfg));
    } else {
      rc = -1;
    }
  }
  return rc;
}

} // ns misc
} // ns nrfcxx
