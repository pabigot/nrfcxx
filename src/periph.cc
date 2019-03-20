// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>

/** PAN73: TIMER: Use of an EVENT from any TIMER module to trigger a
 * TASK in GPIOTE or RTC using the PPI could fail under certain
 * conditions.
 *
 * This is one of those ugly ones where you have to set an
 * undocumented register.  I'm also not placing the workaround at the
 * exact recommended location (before START, after STOP) because that
 * would require putting it into the header.  There's [more
 * information](https://devzone.nordicsemi.com/f/nordic-q-a/8481/clarification-of-pan-73-v3-timer-ppi-sleep-issues-workaround)
 * as to what effect this might have.
 *
 * In particular this workaround is applied whenever a timer is
 * enabled, even if it is not being used for GPIOTE/RTC tasks, which
 * causes an additional 260 uA current in sleep.
 *
 * @todo consider adding API to control application of this
 * workaround.
 *
 * @note This PAN was introduced in Rev 3 ICs.  It does not appear to be listed for nRF52 ICs. */
#ifndef NRFCXX_ENABLE_NRF51_PAN_73
#define NRFCXX_ENABLE_NRF51_PAN_73 (NRF51 - 0)
#endif // NRFCXX_ENABLE_NRF51_PAN_73

#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_AUX2 NRFCXX_BOARD_PSEL_SCOPEn

namespace nrfcxx {
namespace periph {

namespace {

std::array<uint8_t, NRFCXX_PERIPH_UART0_RXB_SIZE> rx_storage;
std::array<uint8_t, NRFCXX_PERIPH_UART0_TXB_SIZE> tx_storage;
UART::fifo_type uart0_rx_fifo{&rx_storage[0], rx_storage.max_size()};
UART::fifo_type uart0_tx_fifo{&tx_storage[0], tx_storage.max_size()};

gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;
gpio::instr_psel<INSTR_PSEL_AUX2> instr_aux2;

} // anonymous

#ifndef NRFCXX_BOARD_PSEL_UART0_RXD
#define NRFCXX_BOARD_PSEL_UART0_RXD -1
#endif /* NRFCXX_BOARD_PSEL_UART0_RXD */
#ifndef NRFCXX_BOARD_PSEL_UART0_TXD
#define NRFCXX_BOARD_PSEL_UART0_TXD -1
#endif /* NRFCXX_BOARD_PSEL_UART0_TXD */
#ifndef NRFCXX_BOARD_PSEL_UART0_RTS
#define NRFCXX_BOARD_PSEL_UART0_RTS -1
#endif /* NRFCXX_BOARD_PSEL_UART0_RTS */
#ifndef NRFCXX_BOARD_PSEL_UART0_CTS
#define NRFCXX_BOARD_PSEL_UART0_CTS -1
#endif /* NRFCXX_BOARD_PSEL_UART0_CTS */

UART&
UART::instance ()
{
  static UART uart0{nrf5::UART0,
      uart0_rx_fifo, uart0_tx_fifo,
      NRFCXX_BOARD_PSEL_UART0_RXD,
      NRFCXX_BOARD_PSEL_UART0_TXD,
      NRFCXX_BOARD_PSEL_UART0_CTS,
      NRFCXX_BOARD_PSEL_UART0_RTS};
  return uart0;
}

RTC&
RTC::instance (int idx)
{
  static RTC instances[] = {
    {nrf5::RTC0},
    {nrf5::RTC1},
#if (NRF52832 - 0) || (NRF52840 - 0)
    {nrf5::RTC2},
#endif /* NRF52 */
  };
  if ((0 <= idx)
      && ((size_t)idx < (sizeof(instances)/sizeof(*instances)))) {
    return instances[idx];
  }
  failsafe(FailSafeCode::NO_SUCH_PERIPHERAL);
}

TIMER&
TIMER::instance (int idx)
{
  static TIMER instances[] = {
    {nrf5::TIMER0},
    {nrf5::TIMER1},
    {nrf5::TIMER2},
#if (NRF52832 - 0) || (NRF52840 - 0)
    {nrf5::TIMER3},
    {nrf5::TIMER4},
#endif /* NRF52 */
  };
  if ((0 <= idx)
      && ((size_t)idx < (sizeof(instances)/sizeof(*instances)))) {
    return instances[idx];
  }
  failsafe(FailSafeCode::NO_SUCH_PERIPHERAL);
}

SPI&
SPI::instance (int idx)
{
  static SPI instances[] = {
    {nrf5::SPI0},
    {nrf5::SPI1},
#if (NRF52832 - 0) || (NRF52840 - 0)
    {nrf5::SPI2},
#endif /* NRF52 */
  };
  if ((0 <= idx)
      && ((size_t)idx < (sizeof(instances)/sizeof(*instances)))) {
    return instances[idx];
  }
  failsafe(FailSafeCode::NO_SUCH_PERIPHERAL);
}

TWI&
TWI::instance (int idx)
{
  static TWI instances[] = {
    {nrf5::TWI0},
    {nrf5::TWI1},
  };
  if ((0 <= idx)
      && ((size_t)idx < (sizeof(instances)/sizeof(*instances)))) {
    return instances[idx];
  }
  failsafe(FailSafeCode::NO_SUCH_PERIPHERAL);
}

int
TIMER::configure (unsigned int freq_Hz,
                  unsigned int bitmode,
                  bool use_constlat)
{
  int rv = 0;

  nvic_DisableIRQ(timer_.IRQn);
  nvic_ClearPendingIRQ(timer_.IRQn);

  if (0 < freq_Hz) {
    timer_->TASKS_STOP = 1;
    unsigned int prescaler = 0;
    while ((freq_Hz << prescaler) < nrfcxx::clock::hfclk::Frequency_Hz) {
      ++prescaler;
    }
    timer_->PRESCALER = prescaler;
    clock::hfclk::constlat_request(*this, use_constlat);
#if (NRFCXX_ENABLE_NRF51_PAN_73 - 0)
    *(0x0C0C + (__O uint32_t*)timer_.BASE) = 1;
#endif /* NRFCXX_ENABLE_NRF51_PAN_73 */
  } else {
    timer_->TASKS_SHUTDOWN = 1;
    timer_->PRESCALER = 0;
    clock::hfclk::constlat_request(*this, false);
#if (NRFCXX_ENABLE_NRF51_PAN_73 - 0)
    *(0x0C0C + (__O uint32_t*)timer_.BASE) = 0;
#endif /* NRFCXX_ENABLE_NRF51_PAN_73 */
  }

  timer_->TASKS_CLEAR = 1;
  timer_->SHORTS = 0UL;
  timer_->INTENCLR = -1;
  timer_->EVENTS_COMPARE[0] = 0;
  timer_->EVENTS_COMPARE[1] = 0;
  timer_->EVENTS_COMPARE[2] = 0;
  timer_->EVENTS_COMPARE[3] = 0;
  timer_->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

  bitmode &= (TIMER_BITMODE_BITMODE_Msk >> TIMER_BITMODE_BITMODE_Pos);
  if (!bitmode_supported(timer_, bitmode)) {
    rv = -1;
    bitmode = TIMER_BITMODE_BITMODE_16Bit;
  }
  timer_->BITMODE = TIMER_BITMODE_BITMODE_Msk & (bitmode << TIMER_BITMODE_BITMODE_Pos);
  switch (bitmode) {
    case TIMER_BITMODE_BITMODE_08Bit:
      mask_ = (1U << 8) - 1;
      break;
    case TIMER_BITMODE_BITMODE_16Bit:
      mask_ = (1U << 16) - 1;
      break;
    case TIMER_BITMODE_BITMODE_24Bit:
      mask_ = (1U << 24) - 1;
      break;
    case TIMER_BITMODE_BITMODE_32Bit:
      mask_ = (1ULL << 32) - 1;
      break;
  }

  return rv;
}

unsigned int
TIMER::frequency_Hz ()
  const
{
  return nrfcxx::clock::hfclk::Frequency_Hz >> timer_->PRESCALER;
}

int
ADC::claim_bi_ (ADCClient* client)
{
  int rv = -1;
  if (client
      && (!owner_)) { // implicit state_ == available;
    state_ = state_type::claimed;
    owner_ = client;
    rv = 0;
  }
  return rv;
}

int
ADC::release_bi_ (ADCClient* client)
{
  int rv = -1;
  if (client
      && (owner_ == client)
      && (state_type::claimed == state_)) {
    owner_ = nullptr;
    state_ = state_type::available;
    ADCClient::process_queue_bi_();
    rv = 0;
  }
  return rv;
}

int
ADC::try_calibrate_bi_ (ADCClient* client,
                        const notifier_type& notify)
{
  int rv = -1;
  if ((client == owner_)
      && (state_type::claimed == state_)) {
    state_ = state_type::calibrating;
    notify_callback_ = notify;
    rv = client->configure_bi_();
#if !(NRF51 - 0)
    /* SAADC calibration requires that CH[0].CONFIG be initialized to
     * the desired acquisition time. */
    if (0 == rv) {
      rv = peripheral::calibrate_bi();
    }
#endif /* NRF51 */
    if (0 == rv) {
      complete_notify_bi_(true);
    }
  }
  return rv;
}

int
ADC::try_sample_bi_ (ADCClient* client,
                     const notifier_type& notify)
{
  int rv = -1;
  if (client
      && (client == owner_)
      && (state_type::claimed == state_)) {
    rv = client->configure_bi_();
    if (0 == rv) {
      state_ = state_type::starting;
      notify_callback_ = notify;
      rv = peripheral::start_bi();
    }
    if (0 == rv) {
      state_ = state_type::sampling;
    }
  }
  return rv;
}

void
ADC::complete_notify_bi_ (bool calibrating)
{
  notifier_type notify{};
  notify.swap(notify_callback_);

  peripheral::stopped_bi();
  state_ = state_type::claimed;

  auto owner = owner_;
  if (owner) {
    if (!calibrating) {
      owner->complete_bi_();
    }
    if (state_type::claimed == state_) {
      /* Operation completed, client has not initiated another.  Pass
       * control back to see if this was a queued operation, in which
       * case the client will release the ADC. */
       owner->complete_queue_bi_();
    }
  }
  if (notify) {
    notify();
  }
}

void
ADC::irq_handler ()
{
#if (NRF51 - 0)
  if (nrf5::ADC->EVENTS_END) {
    nrf5::ADC->EVENTS_END = 0;

    unsigned int shift16 = 8 - ((nrf5::ADC->CONFIG & ADC_CONFIG_RES_Msk) >> ADC_CONFIG_RES_Pos);
    unsigned int result = nrf5::ADC->RESULT << shift16;

    auto owner = owner_;
    if (peripheral::result_idx_ < peripheral::result_maxcnt_) {
      peripheral::result_ptr_[peripheral::result_idx_++] = result;
    }
    bool completed = peripheral::result_maxcnt_ <= peripheral::result_idx_;
    if (owner && (!completed)) {
      int cfg = owner->nrf51_next_bi_(peripheral::result_idx_);
      if (0 <= cfg) {
        nrf5::ADC->CONFIG = cfg;
        peripheral::trigger();
      } else {
        completed = true;
      }
    }
    if (completed) {
      complete_notify_bi_(false);
    }
  }
#else /* NRF51 */
  if (nrf5::SAADC->EVENTS_CALIBRATEDONE) {
    /* SAADC generates an extra DONE after CALIBRATEDONE which can
     * cause grief if START occurs too quickly.  Block the event by
     * explicitly STOPing SAADC, and delay notifying the application
     * of calibration completion until STOP has taken effect.
     *
     * Reference PAN 86 and PAN 178. */
    nrf5::SAADC->EVENTS_CALIBRATEDONE = 0;
    nrf5::SAADC->EVENTS_STOPPED = 0;
    nrf5::SAADC->TASKS_STOP = 1;
  }
  if (nrf5::SAADC->EVENTS_STARTED) {
    nrf5::SAADC->EVENTS_STARTED = 0;
    /* PAN 74: SAADC: Started events fires prematurely.  Mitigation:
     * Ignore if not expected. */
    if (state_ == state_type::starting) {
      state_ = state_type::sampling;
      nrf5::SAADC->TASKS_SAMPLE = 1;
    }
  }
  if (nrf5::SAADC->EVENTS_END) {
    nrf5::SAADC->EVENTS_END = 0;

    /* Explicit stop to ensure the ADC isn't consuming power after the
     * sampling sequence has completed, and to ensure the START
     * required for the next acquisition begins in a clean state. */
    nrf5::SAADC->EVENTS_STOPPED = 0;
    nrf5::SAADC->TASKS_STOP = 1;

    (void)peripheral::normalize_bi();
  }
  if (nrf5::SAADC->EVENTS_DONE) {
    nrf5::SAADC->EVENTS_DONE = 0;
  }
  if (nrf5::SAADC->EVENTS_RESULTDONE) {
    nrf5::SAADC->EVENTS_RESULTDONE = 0;
  }
  if (nrf5::SAADC->EVENTS_STOPPED) {
    nrf5::SAADC->EVENTS_STOPPED = 0;
    complete_notify_bi_(peripheral::calibrating_bi_);
  }
#endif /* NRF51 */
}

ADCClient* volatile ADC::owner_;
notifier_type ADC::notify_callback_;
ADC::state_type volatile ADC::state_;
ADCClient::queue_type ADCClient::queue_;

void
ADCClient::complete_queue_bi_ ()
{
  auto qfl = FL_QUEUED_Msk & flags_;
  if (qfl) {
    flags_ &= ~qfl;
    /* The entry conditions in ADC::complete_notify_bi_ ensure that
     * the release operation will succeed. */
    ADC::release_bi_(this);
  }
}

void
ADCClient::process_queue_bi_ ()
{
  if (ADC::state_type::available != ADC::state_) {
    return;
  }
  auto nc = queue_.unlink_front();
  if (!nc) {
    return;
  }
  int rc = nc->claim();
  if (0 == rc) {
    notifier_type notify{};
    notify.swap(nc->notify_callback_);
    if (FL_QUEUED_CALIBRATE & nc->flags_) {
      rc = nc->calibrate(notify);
    } else if (FL_QUEUED_SAMPLE & nc->flags_) {
      rc = nc->sample(notify);
    } else {
      rc = -EINVAL;
    }
  }
  queued_callback_type notify{};
  notify.swap(nc->queued_callback_);
  if (notify) {
    notify(rc);
  }
}

int
ADCClient::queue (const notifier_type& notify,
                  const queued_callback_type& qnotify,
                  bool calibrate)
{
  mutex_type mutex;
  if (FL_QUEUED_Msk & flags_) {
    // Client is already in the queue, or is executing a previously
    // queued operation.
    return -1;
  }
  if (ADC::owner_ == this) {
    return -2;
  }
  queued_callback_ = qnotify;
  notify_callback_ = notify;
  flags_ |= (calibrate ? FL_QUEUED_CALIBRATE : FL_QUEUED_SAMPLE);
  queue_.link_back(*this);
  process_queue_bi_();
  return 0;
}

int
TEMP::temperature ()
{
  primask mutex;

  /* NB: PAN 66 (nRF52) / 78 (nRF52840) workaround is required and
   * should have been supplied by mdk system files. */
  NRF_TEMP->EVENTS_DATARDY = 0;
  NRF_TEMP->TASKS_START = 1;
  while (!NRF_TEMP->EVENTS_DATARDY) {
  }
  /* NB: nRF51 PAN 28 (sign extension only through bit 9) not
   * supported as this was fixed for rev 3 MCUs. */
  int rv = NRF_TEMP->TEMP;
  NRF_TEMP->TASKS_STOP = 1;
  NRF_TEMP->EVENTS_DATARDY = 0;
  return rv;
}

} // namespace periph

volatile uint16_t* nrf5::series::ADC_Base::result_ptr_;
uint16_t nrf5::series::ADC_Base::result_maxcnt_;
#if (NRF51 - 0)
uint16_t nrf5::series::ADC_Peripheral::result_idx_;
#endif

} // namespace nrfcxx

extern "C" {

void
#if 51 == NRF_SERIES
UART0_IRQHandler ()
#else // NRF_SERIES
UARTE0_UART0_IRQHandler ()
#endif // NRF_SERIES
{
  using nrfcxx::periph::UART;
  UART::instance().irq_handler();
}

} // extern "C"
