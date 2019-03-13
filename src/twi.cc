// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

/* Implementation for nRF51 device series TWI (I2C) peripheral interface.
 */

#include <nrfcxx/periph.hpp>
#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>

/** PAN#36 TWI: Shortcuts described in nRF51 SRM are not functional.
 *
 * If enabled a PPI channel is permanently allocated to the TWI device
 * the first time its bus is configured.  If disabled TWI shorts are
 * used for the necessary functionality.
 *
 * Workaround: Use a PPI channel.
 *
 * @note This PAN was resolved in Rev 3 ICs. */
#ifndef NRFCXX_ENABLE_NRF51_PAN_56
#define NRFCXX_ENABLE_NRF51_PAN_36 0
#endif // NRFCXX_ENABLE_NRF51_PAN_56

/** PAN#56 TWI: TWI module lock-up
 *
 * Various conditions require a full power-cycle of the TWI module to
 * restore functionality.  The likelihood of occurrence can be
 * lessened by certain behaviors, but in the end we need to alarm if
 * an expected RXDREADY or TXDREADY signal is not received in a timely
 * manner.
 *
 * If enabled the recovery from I2C bus timeouts will include
 * power-cycling the TWI peripheral.
 *
 * Workaround: Use recovery code from PAN description.
 *
 * @note This PAN was resolved in Rev 3 ICs. */
#ifndef NRFCXX_ENABLE_NRF51_PAN_56
#define NRFCXX_ENABLE_NRF51_PAN_56 0
#endif // NRFCXX_ENABLE_NRF51_PAN_56

namespace nrfcxx {
namespace periph {

namespace {

/** The GPIO configuration used for TWI pins when clearing the bus.
 *
 * Internal pull-up so we can detect external pull-down.  Disconnect
 * when writing 1 to support multimaster (?).  Enable input and output
 * as we need to toggle SCL and read both SCL and SDA. */
constexpr uint32_t PIN_CNF_RESET{gpio::PIN_CNF_RDWR
    | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
    | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)};

/** The GPIO configuration used for TWI pins when TWI is disabled. */
constexpr uint32_t PIN_CNF_DISABLED{gpio::PIN_CNF_RDONLY
    | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
    | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)};

} // anonymous

TWI::error_type
TWI::clear_bus_ ()
{
  unsigned int ec{};
  uint8_t psel_scl = configuration_.psel_scl;
  uint8_t psel_sda = configuration_.psel_sda;
  uint32_t const scl_bit = (1U << psel_scl);
  uint32_t const sda_bit = (1U << psel_sda);
  uint32_t const bits = scl_bit | sda_bit;
  unsigned int const half_cycle_us = 5; /* Half cycle at 100 kHz */
  bool cleared;

  /* Pull up SCL and SDA then turn off TWI and wait a cycle to
   * settle before sampling the signals. */
  nrf5::GPIO->OUTSET = bits;
  nrf5::GPIO->PIN_CNF[psel_scl] = PIN_CNF_RESET;
  nrf5::GPIO->PIN_CNF[psel_sda] = PIN_CNF_RESET;
  twi_->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);

  delay_us(2 * half_cycle_us);
  cleared = (bits == (bits & nrf5::GPIO->IN));

  if (!cleared) {
    /* At least one of SCL or SDA is being held low by a follower
     * device.  Toggle the clock enough to flush both leader and
     * follower bytes, then see if it's let go. */
    int cycles = 18;
    while (0 < cycles--) {
      nrf5::GPIO->OUTCLR = scl_bit;
      delay_us(half_cycle_us);
      nrf5::GPIO->OUTSET = scl_bit;
      delay_us(half_cycle_us);
    }
    cleared = (bits == (bits & nrf5::GPIO->IN));
  }

  if (cleared) {
    twi_->ENABLE = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);
  } else {
    ec |= TWI::ERR_CLEAR;
  }

  /* Put GPIOs back to disabled state */
  nrf5::GPIO->PIN_CNF[psel_sda] = PIN_CNF_DISABLED;
  nrf5::GPIO->PIN_CNF[psel_scl] = PIN_CNF_DISABLED;

  return ec;
}

TWI::error_type
TWI::clear_error_ ()
{
  TWI::error_type ec{twi_->ERRORSRC};
  if (!ec) {
    ec |= TWI::ERR_UNKNOWN;
  }
  twi_->ERRORSRC = 0;
  return (ec | clear_bus_());
}


TWI::ssize_type
TWI::bus_configure (int psel_scl,
                    int psel_sda,
                    uint32_t frequency,
                    unsigned int timeout_us)
{
  error_type ec{};

  do {
    /* Save the previous PPI index then clear the configuration. */
    int ppidx = configuration_.ppidx;
    configuration_ = {};

#if (NRFCXX_ENABLE_NRF51_PAN_36 - 0)
    /* Attempt to allocate a PPI to work around the anomaly */
    if (0 > ppidx) {
      ppidx = PPI::request();
      if (0 > ppidx) {
        ec = ERR_UNKNOWN;
        break;
      }
    }
#endif

    if (timeout_us < minimum_timeout_us) {
      timeout_us = minimum_timeout_us;
    }

    if ((0 > psel_sda) || (32 <= psel_sda)
        || (0 > psel_scl) || (32 <= psel_scl)) {
      ec = ERR_INVALID;
      break;
    }

    configuration_.timeout = clock::uptime::from_us(timeout_us);
    configuration_.frequency = frequency;
    configuration_.psel_sda = psel_sda;
    configuration_.psel_scl = psel_scl;
    configuration_.ppidx = ppidx;

    nrf5::GPIO->PIN_CNF[psel_scl] = PIN_CNF_DISABLED;
    nrf5::GPIO->PIN_CNF[psel_sda] = PIN_CNF_DISABLED;
  } while (0);

  return error_encoded(ec);
}

TWI::error_type
TWI::set_enabled_ (bool enabled)
{
  error_type rv{};
  twi_->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);
  if (enabled) {
    if (0 > configuration_.psel_scl) {
      rv = ERR_INVALID;
    } else {
      twi_->PSEL.SCL = configuration_.psel_scl;
      twi_->PSEL.SDA = configuration_.psel_sda;
      twi_->FREQUENCY = configuration_.frequency;

#if (NRFCXX_ENABLE_NRF51_PAN_36 - 0)
      PPI::CHENCLR(PPI_CHENCLR_CH0_Clear << (configuration_.ppidx + PPI_CHENCLR_CH0_Pos));
#endif /* NRFCXX_ENABLE_NRF51_PAN_36 */
      rv = clear_bus_();
    }
  }
  return rv;
}

TWI::error_type
TWI::reset_periph_ ()
{
  auto address = twi_->ADDRESS;

#if 0
  printf("PAN56 Workaround!\n");
  /* Use this to reset the loop limit to something that works when
   * verifying the behavior of a too-short limit */
  configuration_.timeout = 100;
#endif
  twi_->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);
#if (NRF51 - 0)
  twi_->POWER = 0;
  delay_us(5);
  twi_->POWER = 1;
#endif /* NRF51 */

  auto ec = set_enabled_(true);
  if (!ec) {
    twi_->ADDRESS = address;
  }
  return ec;
}

TWI::ssize_type
TWI::read (unsigned int addr,
           uint8_t* buf,
           size_type count)
{
  uint8_t* dp = buf;
  uint8_t* const dpe = buf + count;
  error_type ec{};
  ssize_type rv = count;

  if (!enabled()) {
    return error_encoded(ERR_INVALID);
  }

  /* The nRF51 SRM demonstrates BB->SUSPEND and BB->STOP connections
   * during TWI reads, but fails to motivate this except for the
   * BB->STOP required for the last byte read.  A compelling discusion
   * of why the non-final byte connections are also needed is at:
   * https://devzone.nordicsemi.com/f/nordic-q-a/3984/is-it-necessary-to-suspend-twi-for-each-byte-read
   *
   * Select the appropriate mechanism to provide this coordination. */
#if (NRFCXX_ENABLE_NRF51_PAN_36 - 0)
  if (1 == count) {
    PPI::configure(configuration_.ppidx, twi_->EVENTS_BB, twi_->TASKS_STOP);
  } else {
    PPI::configure(configuration_.ppidx, twi_->EVENTS_BB, twi_->TASKS_SUSPEND);
  }
  PPI::CHENSET(PPI_CHENSET_CH0_Set << (configuration_.ppidx + PPI_CHENSET_CH0_Pos));
#else /* NRFCXX_ENABLE_NRF51_PAN_36 */
  if (1 == count) {
    twi_->SHORTS = (TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos);
  } else {
    twi_->SHORTS = (TWI_SHORTS_BB_SUSPEND_Enabled << TWI_SHORTS_BB_SUSPEND_Pos);
  }
#endif /* NRFCXX_ENABLE_NRF51_PAN_36 */

  twi_->ADDRESS = addr;
  twi_->EVENTS_ERROR = 0;
  twi_->EVENTS_STOPPED = 0;
  twi_->EVENTS_RXDREADY = 0;
  twi_->TASKS_STARTRX = 1;

  do {
    clock::uptime::timestamp24 ts{};
    while ((dp < dpe) && (0 <= rv)) {
      /* Spin until ready, error, or timeout */
      while (!(twi_->EVENTS_RXDREADY
                || twi_->EVENTS_ERROR)) {
        if (configuration_.timeout <= ts.delta()) {
          break;
        }
      }
      if (twi_->EVENTS_ERROR) {
        twi_->EVENTS_STOPPED = 0;
        twi_->TASKS_STOP = 1;
        rv = -1;
      } else if (twi_->EVENTS_RXDREADY) {
        *dp++ = twi_->RXD;
        if ((dp+1) >= dpe) {
          /* Switch from SUSPEND to STOP when next input byte is received */
#if (NRFCXX_ENABLE_NRF51_PAN_36 - 0)
          PPI::configure(configuration_.ppidx, twi_->EVENTS_BB, twi_->TASKS_STOP);
#else /* NRFCXX_ENABLE_NRF51_PAN_36 */
          twi_->SHORTS = (TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos);
#endif /* NRFCXX_ENABLE_NRF51_PAN_36 */
        }
        twi_->EVENTS_RXDREADY = 0;
        ts.reset();
      } else {
        ec = ERR_TIMEOUT;
#if (NRFCXX_ENABLE_NRF51_PAN_56 - 0)
        ec |= reset_periph_();
#endif /* NRFCXX_ENABLE_NRF51_PAN_56 */
        rv = -1;
      }
      if (0 <= rv) {
#if (NRFCXX_ENABLE_NRF51_PAN_56 - 0)
        /* PAN-56: module lock-up
         *
         * Delay RESUME for a least two TWI clock periods after RXD read
         * to ensure clock-stretched ACK completes.
         *
         * 100 kHz = 20us, 400 kHz = 5us. */
        delay_us(20);
#endif /* NRFCXX_ENABLE_NRF51_PAN_56 */
        twi_->TASKS_RESUME = 1;
      }
    }

  } while (0);

#if (NRFCXX_ENABLE_NRF51_PAN_36 - 0)
  PPI::CHENCLR(PPI_CHENCLR_CH0_Clear << (configuration_.ppidx + PPI_CHENCLR_CH0_Pos));
#else /* NRFCXX_ENABLE_NRF51_PAN_36 */
  twi_->SHORTS = 0;
#endif /* NRFCXX_ENABLE_NRF51_PAN_36 */

  if (0 <= rv) {
    while (!twi_->EVENTS_STOPPED) {
    }
    twi_->EVENTS_STOPPED = 0;
  } else {
    /* Obtain details on error cause */
    rv = error_encoded(ec | clear_error_());
  }
  return rv;
}

TWI::ssize_type
TWI::write (unsigned int addr,
            const uint8_t* buf,
            size_type count)
{
  const uint8_t* sp = buf;
  const uint8_t* const spe = buf + count;
  error_type ec{};
  ssize_type rv = count;

  if (!enabled()) {
    return error_encoded(ERR_INVALID);
  }

  twi_->ADDRESS = addr;
  twi_->EVENTS_ERROR = 0;
  twi_->EVENTS_TXDSENT = 0;
  twi_->TASKS_STARTTX = 1;

  bool do_stop = true;
  do {
    clock::uptime::timestamp24 ts{};
    if (sp < spe) {
      twi_->TXD = *sp++;
    }
    while (!(twi_->EVENTS_TXDSENT
              || twi_->EVENTS_ERROR)) {
      if (configuration_.timeout <= ts.delta()) {
        break;
      }
    }
    if (twi_->EVENTS_ERROR) {
      rv = -1;
    } else if (twi_->EVENTS_TXDSENT) {
      twi_->EVENTS_TXDSENT = 0;
    } else {
      ec = ERR_TIMEOUT;
#if (NRFCXX_ENABLE_NRF51_PAN_56 - 0)
      ec |= reset_periph_();
      do_stop = false;
#endif /* NRFCXX_ENABLE_NRF51_PAN_56 */
      rv = -1;
    }
  } while ((sp < spe) && (0 <= rv));

  if (do_stop) {
    twi_->EVENTS_STOPPED = 0;
    twi_->TASKS_STOP = 1;
    while (!twi_->EVENTS_STOPPED) {
    }
  }

  if (0 > rv) {
    /* Obtain details on error cause */
    rv = error_encoded(ec | clear_error_());
  }
  return rv;
}

} // namespace periph
} // namespace nrfcxx
