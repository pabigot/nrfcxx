// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>

#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn    // whatever's of interest
#define INSTR_PSEL_TXTASK NRFCXX_BOARD_PSEL_SCOPEn // tracks FLAGS_TXTASK
#define INSTR_PSEL_IRQ NRFCXX_BOARD_PSEL_SCOPEn    // when IRQHandler is active

namespace {

nrfcxx::gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;
nrfcxx::gpio::instr_psel<INSTR_PSEL_TXTASK> instr_txtask;
nrfcxx::gpio::instr_psel<INSTR_PSEL_IRQ> instr_irq;

} // anonymous

/* Set when enable() has been invoked, cleared when disable() has been
 * invoked. */
#define FLAGS_ENABLED 0x01

/* Set when there's a transmission in progress. */
#define FLAGS_TXTASK 0x02

/* Set when autoenable() has been configured on. */
#define FLAGS_AUTOENABLE 0x04

/* Set when the UART has been disabled due to autoenable(). */
#define FLAGS_AUTODISABLED 0x08

namespace nrfcxx {
namespace periph {

void
UART::enable (uint32_t cfg_baudrate,
              bool hwfc)
{
  instr_aux.enable();
  instr_aux.deassert();
  instr_txtask.enable();
  instr_txtask.deassert();
  instr_irq.enable();
  instr_irq.deassert();

  disable();
  uart_->PSEL.RXD = rxd_pin_;
  uart_->PSEL.TXD = txd_pin_;
  hwfc &= (0 <= rts_pin_) && (0 <= cts_pin_);
  if (hwfc) {
    uart_->PSEL.CTS = cts_pin_;
    uart_->PSEL.RTS = rts_pin_;
    uart_->CONFIG = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
  } else {
    uart_->PSEL.CTS = -1;
    uart_->PSEL.RTS = -1;
    uart_->CONFIG = 0;
  }
  events_.reset();
  rxb_.clear();
  txb_.clear();
  statistics_ = {};
  flags_ = FLAGS_ENABLED;
  if (0 == cfg_baudrate) {
    cfg_baudrate = UART_BAUDRATE_BAUDRATE_Baud115200;
  }
  uart_->BAUDRATE = cfg_baudrate;

  uart_->EVENTS_RXDRDY = 0;
  uart_->EVENTS_TXDRDY = 0;

  uart_->INTENCLR = -1;
  uart_->INTENSET = ((UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos)
                     | (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos)
                     | (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos));
  nvic_ClearPendingIRQ(uart_.IRQn);
  nvic_SetPriority(uart_.IRQn, IRQ_PRIORITY_APP_LOW);

  {
    nrfcxx::nvic_BlockIRQ block(uart_.IRQn);
    set_enabled_bi_(true, false);
  }
}

void
UART::set_enabled_bi_ (bool on,
                       bool from_autoenable)
{
  if (on) {
    instr_aux.assert();
    uart_->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    uart_->TASKS_STARTRX = 1;
    nvic_EnableIRQ(uart_.IRQn);
    if (from_autoenable) {
      flags_ &= ~FLAGS_AUTODISABLED;
    }
  } else {
    uart_->TASKS_STOPTX = 1;
    uart_->TASKS_STOPRX = 1;
    nvic_DisableIRQ(uart_.IRQn);
    uart_->ENABLE = 0;
    if (from_autoenable) {
      flags_ |= FLAGS_AUTODISABLED;
    }
    instr_aux.deassert();
  }
}

void
UART::disable ()
{
  nrfcxx::nvic_BlockIRQ block(uart_.IRQn);

  set_enabled_bi_(false, false);
  uart_->INTENCLR = -1;
  flags_ = 0;
}

inline bool
UART::enabled (bool live)
  const
{
  bool rv = uart_->ENABLE;
  if (!live) {
    rv |= (FLAGS_AUTOENABLE & flags_);
  }
  return rv;
}

bool
UART::autoenable (int on)
{
  nrfcxx::nvic_BlockIRQ block(uart_.IRQn);
  bool rv = (FLAGS_AUTOENABLE & flags_);

  if (FLAGS_ENABLED & flags_) {
    if (0 < on) {
      constexpr auto check = FLAGS_ENABLED | FLAGS_TXTASK;
      flags_ |= FLAGS_AUTOENABLE;
      if (FLAGS_ENABLED == (check & flags_)) {
        set_enabled_bi_(false, true);
      }
      rv = true;
    } else if (0 == on) {
      constexpr auto check = FLAGS_ENABLED | FLAGS_AUTODISABLED;
      flags_ &= ~FLAGS_AUTOENABLE;
      if (check == (check & flags_)) {
        set_enabled_bi_(true, false);
      }
    }
  }
  return rv;
}

UART::ssize_type
UART::write (const uint8_t* sp,
             size_type count)
{
  //auto scope = gpio::make_scoped_instr(instr_aux);
  nrfcxx::nvic_BlockIRQ block(uart_.IRQn);

  /* If the UART isn't enabled pretend we transmitted without actually
   * doing anything.  We don't queue the data, and certainly don't
   * submit it. */
  if (!(FLAGS_ENABLED & flags_)) {
    return count;
  }

  size_type rv = 0;
  while ((rv < count) && !txb_.full()) {
    txb_.push(*sp++);
    ++rv;
  }
  if ((0 < rv) && !(FLAGS_TXTASK & flags_)) {
    if (FLAGS_AUTODISABLED & flags_) {
      set_enabled_bi_(true, true);
    }
    flags_ |= FLAGS_TXTASK;
    instr_txtask.assert();
    uart_->TASKS_STARTTX = 1;
    uart_->TXD = txb_.pop();
    statistics_.tx_count += 1;
  }
  return rv;
}

UART::ssize_type
UART::read (uint8_t* dp,
            size_type count)
{
  nrfcxx::nvic_BlockIRQ block(uart_.IRQn);
  size_type rv = 0;
  while ((rv < count) && !rxb_.empty()) {
    *dp++ = rxb_.pop();
    ++rv;
  }
  return rv;
}

UART::statistics_type
UART::statistics () const
{
  nrfcxx::nvic_BlockIRQ block(uart_.IRQn);
  return statistics_;
}

void
UART::irq_handler ()
{
  auto scope = gpio::make_scoped_instr(instr_irq);

  event_set::event_type evt = 0;
  if (uart_->EVENTS_ERROR) {
    uint32_t errorsrc = 0;
    do {
      uart_->EVENTS_ERROR = 0;
      errorsrc |= uart_->ERRORSRC;
      uart_->ERRORSRC = errorsrc;
    } while (uart_->EVENTS_ERROR);

    if (UART_ERRORSRC_BREAK_Present & errorsrc) {
      statistics_.rx_break_errors += 1;
    }
    if (UART_ERRORSRC_FRAMING_Present & errorsrc) {
      statistics_.rx_frame_errors += 1;
    }
    if (UART_ERRORSRC_PARITY_Present & errorsrc) {
      statistics_.rx_parity_errors += 1;
    }
    if (UART_ERRORSRC_OVERRUN_Present & errorsrc) {
      statistics_.rx_overrun_errors += 1;
    }
    if (errorsrc) {
      evt |= EVT_ERROR;
    }
  }
  if (uart_->EVENTS_RXDRDY) {
    uart_->EVENTS_RXDRDY = 0;
    uint8_t rxd = uart_->RXD;
    if (rxb_.push(rxd)) {
      statistics_.rx_dropped += 1;
    }
    statistics_.rx_count += 1;
    evt |= EVT_RXAVAIL;
  }
  if (uart_->EVENTS_TXDRDY) {
    uart_->EVENTS_TXDRDY = 0;
    if (!txb_.empty()) {
      uart_->TXD = txb_.pop();
      statistics_.tx_count += 1;
      evt |= EVT_TXAVAIL;
    } else {
      uart_->TASKS_STOPTX = 1;
      instr_txtask.deassert();
      flags_ &= ~FLAGS_TXTASK;
      if (flags_ & FLAGS_AUTOENABLE) {
        set_enabled_bi_(false, true);
      }
      evt |= EVT_TXDONE;
    }
  }
  if (evt) {
    events_.set(evt);
  }
}

} // namespace periph
} // namespace nrfcxx
