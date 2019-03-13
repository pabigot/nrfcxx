// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Peter A. Bigot

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>

namespace nrfcxx {
namespace periph {

namespace {

constexpr uint32_t CINSTR_RDSR = 0
  | (0x05 << QSPI_CINSTRCONF_OPCODE_Pos)
  | (QSPI_CINSTRCONF_LENGTH_2B << QSPI_CINSTRCONF_LENGTH_Pos)
  | (1U << QSPI_CINSTRCONF_LIO2_Pos)
  | (1U << QSPI_CINSTRCONF_LIO3_Pos)
  ;

/** Indicate whether the provided QSPI IFCONFIG0 value requires that
 * the device Quad Enable bit be set.
 *
 * This is the case when either a read or write operation requires
 * four I/O signals. */
inline constexpr bool
ifconfig0_requires_quad (uint32_t ifconfig0)
{
  if ((QSPI_IFCONFIG0_READOC_Msk & ifconfig0)
      >= (QSPI_IFCONFIG0_READOC_READ4O << QSPI_IFCONFIG0_READOC_Pos)) {
    return true;
  }
  if ((QSPI_IFCONFIG0_WRITEOC_Msk & ifconfig0)
      >= (QSPI_IFCONFIG0_WRITEOC_PP4O << QSPI_IFCONFIG0_WRITEOC_Pos)) {
    return true;
  }
  return false;
}

/** Issue an RDSR.
 *
 * As a side-effect this updates the QSPI STATUS.SREG field. */
uint8_t
fetch_status_bi ()
{
  nrf5::QSPI->EVENTS_READY = 0;
  nrf5::QSPI->CINSTRCONF = 0
    | (0x05 << QSPI_CINSTRCONF_OPCODE_Pos)
    | (QSPI_CINSTRCONF_LENGTH_2B << QSPI_CINSTRCONF_LENGTH_Pos)
    | (1U << QSPI_CINSTRCONF_LIO2_Pos)
    | (1U << QSPI_CINSTRCONF_LIO3_Pos)
    ;
  while (!nrf5::QSPI->EVENTS_READY) {
  }
  nrf5::QSPI->EVENTS_READY = 0;
  return 0xFF & nrf5::QSPI->CINSTRDAT0;
}

void
configure_gpios (const QSPI::configuration_type& cfg,
                 bool enable)
{
  auto sck = nrfcxx::gpio::pin_reference::create(cfg.psel_sck);
  auto io0 = nrfcxx::gpio::pin_reference::create(cfg.psel_io[0]);
  auto io1 = nrfcxx::gpio::pin_reference::create(cfg.psel_io[1]);
  auto io2 = nrfcxx::gpio::pin_reference::create(cfg.psel_io[2]);
  auto io3 = nrfcxx::gpio::pin_reference::create(cfg.psel_io[3]);
  auto csn = nrfcxx::gpio::pin_reference::create(cfg.psel_csn);

  if (enable) {
    /* Configure the GPIOs.  The nRF52840 product spec (6.19.1.2)
     * suggests that the IO# signals should be used with high drive, but
     * in practice neither the mdk hal nor driver appear to do this.
     *
     * We do need to ensure that both io2 and io3 are asserted (high),
     * because they tend to be WPn and HOLDn (in some order), and if
     * HOLDn is asserted (low) then the device won't respond if the QE
     * bit in the status register is cleared.  (Because the device will
     * be in HOLD, duh.) */
    uint32_t drive{};
    if (cfg.drive_high) {
      drive = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    }
    csn.set();
    csn.configure(gpio::PIN_CNF_WRONLY | drive);
    sck.configure(gpio::PIN_CNF_WRONLY | drive);
    io0.configure(gpio::PIN_CNF_RDWR | drive);
    io1.configure(gpio::PIN_CNF_RDWR | drive);
    io2.set();
    io2.configure(gpio::PIN_CNF_RDWR | drive);
    io3.set();
    io3.configure(gpio::PIN_CNF_RDWR | drive);
  } else {
    /* Return all GPIOs to power-up state.  If CSn, WPn, or HOLDn do
     * not have physical pull-ups another configuration may be
     * required. */
    csn.configure(gpio::PIN_CNF_PWRUP);
    sck.configure(gpio::PIN_CNF_PWRUP);
    io0.configure(gpio::PIN_CNF_PWRUP);
    io1.configure(gpio::PIN_CNF_PWRUP);
    io2.configure(gpio::PIN_CNF_PWRUP);
    io3.configure(gpio::PIN_CNF_PWRUP);
  }
}

} // anonymous

QSPI* QSPI::owner_ = nullptr;

QSPI::QSPI (const configuration_type& configuration) :
  configuration_{configuration}
{
  if (owner_) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
  int rc = claim();
  if (0 != rc) {
    failsafe(FailSafeCode::API_VIOLATION);
  }
}

int
QSPI::claim ()
{
  mutex_type mutex;
  if (owner_) {
    return -EBUSY;
  }

  /* Take ownership and configure GPIOs for communication with flash
   * device. */
  owner_ = this;
  configure_gpios(configuration_, true);

  /* 6.19.1#1: Set the I/O pins */
  nrf5::QSPI->PSEL.SCK = configuration_.psel_sck;
  nrf5::QSPI->PSEL.CSN = configuration_.psel_csn;
  nrf5::QSPI->PSEL.IO0 = configuration_.psel_io[0];
  nrf5::QSPI->PSEL.IO1 = configuration_.psel_io[1];
  nrf5::QSPI->PSEL.IO2 = configuration_.psel_io[2];
  nrf5::QSPI->PSEL.IO3 = configuration_.psel_io[3];

  /* 6.19.1#2: set GPIO drive strength to high done by
   * configure_gpios(). */

  /* 6.19.1#3: Configure IFCONFIGx, ADDRCONF, and DPMDUR */
  nrf5::QSPI->IFCONFIG0 = configuration_.ifconfig[0];
  nrf5::QSPI->IFCONFIG1 = configuration_.ifconfig[1];
  nrf5::QSPI->ADDRCONF = configuration_.addrconf;
  nrf5::QSPI->DPMDUR = 0
    | (configuration_.enter_dpmdur << QSPI_DPMDUR_ENTER_Pos)
    | (configuration_.exit_dpmdur << QSPI_DPMDUR_EXIT_Pos)
    ;

  nrf5::QSPI->ENABLE = (QSPI_ENABLE_ENABLE_Enabled << QSPI_ENABLE_ENABLE_Pos);

  return 0;
}

int
QSPI::release ()
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }
  if (activated_) {
    return -EBUSY;
  }

  /* PAN-122 workaround: need this before disabling */
  *(volatile uint32_t*)0x40029054 = 1;
  nrf5::QSPI->ENABLE = (QSPI_ENABLE_ENABLE_Disabled << QSPI_ENABLE_ENABLE_Pos);

  /* Put GPIOs back into idle mode and release ownership. */
  configure_gpios(configuration_, false);
  owner_ = {};

  return 0;
}

int
QSPI::read_sr () const
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }
  if (!is_qspi_available()) {
    return -EBUSY;
  }
  return fetch_status_bi();
}

int
QSPI::activate ()
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }
  if (activated_) {
    return 0;
  }

  /* Reset the core configuration values.  IFCONFIG0 should not be
   * mutated, but IFCONFIG1 might have changed due to a previous
   * powerdown() command. */
  nrf5::QSPI->IFCONFIG0 = configuration_.ifconfig[0];
  nrf5::QSPI->IFCONFIG1 = configuration_.ifconfig[1];

  // @todo put in a timeout
  nrf5::QSPI->EVENTS_READY = 0;
  nrf5::QSPI->TASKS_ACTIVATE = 1;
  while (!nrf5::QSPI->EVENTS_READY) {
  }
  nrf5::QSPI->EVENTS_READY = 0;

  if ((QSPI_IFCONFIG0_DPMENABLE_Msk & configuration_.ifconfig[0])
      == (QSPI_IFCONFIG0_DPMENABLE_Enable << QSPI_IFCONFIG0_DPMENABLE_Pos)) {
    /* We can't guarantee the device has been woken from DPM: if the
     * incoming STATUS.DPM was cleared but from a different device, the
     * required RDP would not have been issued by QSPI.  Do an explicit
     * wakeup.
     *
     * Note that we drive IO2 and IO3 high since they might be
     * configured for WPn and HOLDn roles, and the command won't work if
     * those signals are asserted (low).
     *
     * If the RES API is used this command will provide the electronic
     * signature in the upper byte of CINSTRDAT0 even if the device is
     * woken by the operation (at least for some devices).  We don't
     * currently look at it, but a non-zero value would be an indication
     * that the wakeup was successful. */
    nrf5::QSPI->CINSTRCONF = 0
      | (0xAB << QSPI_CINSTRCONF_OPCODE_Pos)
      | (QSPI_CINSTRCONF_LENGTH_5B << QSPI_CINSTRCONF_LENGTH_Pos)
      | (1U << QSPI_CINSTRCONF_LIO2_Pos)
      | (1U << QSPI_CINSTRCONF_LIO3_Pos)
      ;
    while (!nrf5::QSPI->EVENTS_READY) {
    }
    nrf5::QSPI->EVENTS_READY = 0;
  }

  /* If we woke from DPM we might not have the current device SR
   * cached.  Issue RDSR; this will reload the QSPI STATUS
   * register. */
  auto sr = fetch_status_bi();

  /* If the QE field of SR is clear but the QSPI peripheral is
   * configured for quad I/O, we need to update the status
   * register. */
  if ((!(SR_QE & sr))
      && ifconfig0_requires_quad(nrf5::QSPI->IFCONFIG0)) {
    /* Set QE.  Just in case something went horribly wrong with the
     * read operation make sure we don't accidentally set SWRD, which
     * could be irrecoverable. */
    nrf5::QSPI->CINSTRDAT0 = SR_QE | (sr & ~SR_SWRD);
    nrf5::QSPI->CINSTRCONF = 0
      | (0x01 << QSPI_CINSTRCONF_OPCODE_Pos)
      | (QSPI_CINSTRCONF_LENGTH_2B << QSPI_CINSTRCONF_LENGTH_Pos)
      | (QSPI_CINSTRCONF_WREN_Enable << QSPI_CINSTRCONF_WREN_Pos)
      | (1U << QSPI_CINSTRCONF_LIO2_Pos)
      | (1U << QSPI_CINSTRCONF_LIO3_Pos)
      ;
    while (!nrf5::QSPI->EVENTS_READY) {
    }
    nrf5::QSPI->EVENTS_READY = 0;

    /* RDSR does update the status, but we don't want the status with
     * the WIP bit set, so loop until it's cleared. */
    while (SR_WIP & fetch_status_bi()) {
      // spin
    }
  }

  if (0 == jedec_id_) {
    /* Issue RDID.  Note that we drive IO2 and IO3 high since they might
     * be configured for WPn and HOLDn roles, and the command won't work
     * if those signals are asserted (low). */
    nrf5::QSPI->CINSTRDAT0 = 0;
    nrf5::QSPI->CINSTRCONF = 0
      | (0x9F << QSPI_CINSTRCONF_OPCODE_Pos)
      | (QSPI_CINSTRCONF_LENGTH_4B << QSPI_CINSTRCONF_LENGTH_Pos)
      | (1U << QSPI_CINSTRCONF_LIO2_Pos)
      | (1U << QSPI_CINSTRCONF_LIO3_Pos)
      ;
    while (!nrf5::QSPI->EVENTS_READY) {
    }
    nrf5::QSPI->EVENTS_READY = 0;
    auto raw = nrf5::QSPI->CINSTRDAT0;
    /* Reverse the order of the three bytes so the manufacturer ID is
     * in bits 16..23. */
    jedec_id_ = ((0xFF & raw) << 16) | (0xFF00 & raw) | (0xFF & (raw >> 16));
  }

  activated_ = true;
  return 0;
}

int
QSPI::deactivate ()
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }

  if (activated_) {
    if ((QSPI_IFCONFIG0_DPMENABLE_Msk & configuration_.ifconfig[0])
        == (QSPI_IFCONFIG0_DPMENABLE_Enable << QSPI_IFCONFIG0_DPMENABLE_Pos)) {
      /* Entering DPM reads the status then transmits DP.  There is no
       * READY event.  The STATUS register immediately indicates that it's
       * in DPM. */

      nrf5::QSPI->IFCONFIG1 |= (QSPI_IFCONFIG1_DPMEN_Enter << QSPI_IFCONFIG1_DPMEN_Pos);
    }

    nrf5::QSPI->TASKS_DEACTIVATE = 1;
    activated_ = false;
  }

  return 0;
}

ssize_t
QSPI::read (offset_type addr,
            void* dest,
            size_type count)
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }
  if (!activated_) {
    return -EBADF;
  }
  if (!is_qspi_available()) {
    return -EBUSY;
  }
  nrf5::QSPI->READ.SRC = addr;
  nrf5::QSPI->READ.DST = reinterpret_cast<uintptr_t>(dest);
  nrf5::QSPI->READ.CNT = count;
  nrf5::QSPI->EVENTS_READY = 0;
  nrf5::QSPI->TASKS_READSTART = 1;
  while (!nrf5::QSPI->EVENTS_READY) {
  };
  return count;
}

ssize_t
QSPI::write (offset_type addr,
             const void* src,
             size_type count)
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }
  if (!activated_) {
    return -EBADF;
  }
  if (!is_qspi_available()) {
    return -EBUSY;
  }

  nrf5::QSPI->WRITE.DST = addr;
  nrf5::QSPI->WRITE.SRC = reinterpret_cast<uintptr_t>(src);
  nrf5::QSPI->WRITE.CNT = count;
  nrf5::QSPI->EVENTS_READY = 0;
  nrf5::QSPI->TASKS_WRITESTART = 1;
  while (!nrf5::QSPI->EVENTS_READY) {
  };

  return count;
}

int
QSPI::erase (uint8_t type,
             offset_type addr)
{
  mutex_type mutex;
  if (this != owner_) {
    return -EINVAL;
  }
  if (!activated_) {
    return -EBADF;
  }
  if (!is_qspi_available()) {
    return -EBUSY;
  }
  switch (type) {
    case ERASE_4_KB:
      if (addr % (1U << 12)) {
        return -EINVAL;
      }
      break;
    case ERASE_64_KB:
      if (addr % (1U << 16)) {
        return -EINVAL;
      }
      break;
    case ERASE_CHIP:
      if (addr) {
        return -EINVAL;
      }
      break;
    default:
      return -EINVAL;
  }

  nrf5::QSPI->ERASE.PTR = addr;
  nrf5::QSPI->ERASE.LEN = type;
  nrf5::QSPI->EVENTS_READY = 0;
  nrf5::QSPI->TASKS_ERASESTART = 1;
  while (!nrf5::QSPI->EVENTS_READY) {
  };
  return 0;
}

} // namespace periph
} // namespace nrfcxx
