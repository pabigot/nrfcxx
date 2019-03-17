// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2019 Peter A. Bigot

#include <nrfcxx/periph.hpp>
#include <nrfcxx/gpio.hpp>

#define INSTR_PSEL_GPIOTE_FLIH NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_GPIOTE_EVENT_FLIH NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_GPIOTE_SENSE_FLIH NRFCXX_BOARD_PSEL_SCOPEn

namespace nrfcxx {
namespace periph {

namespace {

gpio::instr_psel<INSTR_PSEL_GPIOTE_FLIH> instr_gpiote_flih;
gpio::instr_psel<INSTR_PSEL_GPIOTE_EVENT_FLIH> instr_gpiote_event_flih;
gpio::instr_psel<INSTR_PSEL_GPIOTE_SENSE_FLIH> instr_gpiote_sense_flih;

struct sense_gpio_type
{
  sense_gpio_type (const nrfcxx::nrf5::GPIO_Type& gpio,
                   unsigned int psel_base = 0) :
    gpio{gpio},
    psel_base{psel_base}
  { }

  /** A GPIO instance used for sense detection. */
  const nrfcxx::nrf5::GPIO_Type& gpio;

  /** Bits identifying pins that have been confirmed to have indicated
   * a sense change.
   *
   * Calculated by sense_clear_and_capture_bi() and used in
   * sense_configure_and_enable_bi(). */
  uint32_t latch = 0;

  /** Bits identifying the captured input status against which sense
   * detection will be compared.
   *
   * Calculated by sense_clear_and_capture_bi() and used in
   * sense_configure_and_enable_bi().  */
  uint32_t in = 0;

  /** The global pin selection index of local psel zero on the associated device.
   *
   * This must be provided by the infrastructure prior to invoking
   * sense_clear_and_capture_bi().  Defaults to zero. */
  unsigned int const psel_base;
};

/* Storage for the status of each potentially monitorable pin plus an
 * end marker. */
GPIOTE::sense_status_type sense_status[nrfcxx::nrf5::GPIO_PSEL_COUNT + 1];

// Single bit that controls whether SENSE detection is enabled.
// (The other bit controls sense level, and is set for Low).
constexpr auto SENSE_Msk = (GPIO_PIN_CNF_SENSE_High & GPIO_PIN_CNF_SENSE_Low) << GPIO_PIN_CNF_SENSE_Pos;
constexpr auto SENSE_LOW_Msk = (GPIO_PIN_CNF_SENSE_High ^ GPIO_PIN_CNF_SENSE_Low) << GPIO_PIN_CNF_SENSE_Pos;

GPIOTE::sense_status_type*
sense_clear_and_capture_bi (sense_gpio_type &sg,
                            GPIOTE::sense_status_type* sp)
{
  uint8_t psel{0};
  auto in0 = sg.gpio->IN;
  while (psel < sg.gpio.AUX) {
    auto pin_cnf = sg.gpio->PIN_CNF[psel];
    if (SENSE_Msk & pin_cnf) {
      sp->psel = sg.psel_base + psel;
      // State contains a boolean indicating whether the pin is
      // configured to sense high.
      sp->counter_state = !(SENSE_LOW_Msk & pin_cnf);
      pin_cnf &= ~SENSE_Msk;
      sg.gpio->PIN_CNF[psel] = pin_cnf;
      ++sp;
    }
    ++psel;
  }
#if (NRF52832 - 0) || (NRF52840 - 0)
  sg.latch = sg.gpio->LATCH;
  sg.gpio->LATCH = -1;
#else
  sg.latch = 0;
#endif
  sg.in = sg.gpio->IN;
  sg.latch |= sg.in ^ in0;
  return sp;
}

GPIOTE::sense_status_type*
sense_configure_and_enable_bi (sense_gpio_type &sg,
                               GPIOTE::sense_status_type* sp)
{
  auto in = sg.gpio->IN;
  int psel_end = sg.psel_base + sg.gpio.AUX;
  while ((0 <= sp->psel)
         && (sp->psel < psel_end)) {
    auto psel = sp->psel - sg.psel_base;
    auto bit = (1U << psel);
    // Assess a change if the pin is configured to sense high and the
    // pin reads high, or is configured to sense low and the pin reads
    // low.
    auto sense_high = sp->counter_state;
    sp->counter_state = (bit & in) ? 1 : 0; // reads high
    auto pin_cnf = sg.gpio->PIN_CNF[psel];
    if (sense_high == sp->counter_state) {
      pin_cnf ^= SENSE_LOW_Msk;
      // Looking for sense that matches current state.  Invert the
      // sense and increment the count (bits 1..7).
      sp->counter_state += (1 << 1);
    } else if (bit & sg.latch) {
      // Sense already correct.  If we have external evidence of a change,
      // it changed and changed back, so count two.
      sp->counter_state += (2 << 1);
    }
    pin_cnf |= SENSE_Msk;
    sg.gpio->PIN_CNF[psel] = pin_cnf;
    ++sp;
  }
  return sp;
}


const periph::GPIOTE::sense_status_type*
sense_update_status_bi ()
{
  using namespace nrfcxx;

  sense_gpio_type p0{nrf5::GPIO};
  auto sp = sense_clear_and_capture_bi(p0, sense_status);
#if (NRF52840 - 0)
  sense_gpio_type p1{nrf5::P1, nrf5::P0.AUX};
  sp = sense_clear_and_capture_bi(p1, sp);
#endif /* NRF2840 */
  sp->psel = -1;
  nrf5::GPIOTE->EVENTS_PORT = 0;
  sp = sense_configure_and_enable_bi(p0, sense_status);
#if (NRF52840 - 0)
  sp = sense_configure_and_enable_bi(p1, sp);
#endif /* NRF2840 */
  return sense_status;
}

void
sense_process_chain_bi (const nrfcxx::periph::GPIOTE::sense_listener::chain_type& chain)
{
  auto sp = sense_update_status_bi();
  for (auto& sl : chain) {
    auto scope = make_scoped_instr(instr_gpiote_sense_flih);
    auto callback_bi = sl.callback_bi;
    if (callback_bi) {
      callback_bi(sp);
    }
  }
}

} // anonymous

void
GPIOTE::event_listener::enable (GPIOTE& instance)
{
  mutex_type mutex;
  if (instance_ != &instance) {
    // Dissociate a different GPIOTE from this listener
    if (instance_ && instance_->listener_) {
      instance_->listener_->disable_bi_();
    }
    // Dissociate a different listener from the new GPIOTE
    if (instance.listener_) {
      instance.listener_->disable_bi_();
    }
    instance.listener_ = this;
    instance_ = &instance;
  }
}

GPIOTE::registry_type GPIOTE::in_use_;
GPIOTE GPIOTE::instances_[GPIOTE::CHANNEL_COUNT] = {
  // 4 on nRF51
  {0}, {1}, {2}, {3},
#if 52 == NRF_SERIES
  // 8 on nRF52
  {4}, {5}, {6}, {7},
#endif // NRF_SERIES
};
GPIOTE::sense_listener::chain_type GPIOTE::sense_chain_;

GPIOTE::event_reference_type
GPIOTE::EVENTS_IN ()
{
  return nrf5::GPIOTE->EVENTS_IN[channel];
}

GPIOTE::task_reference_type
GPIOTE::TASKS_OUT ()
{
  return nrf5::GPIOTE->TASKS_OUT[channel];
}

#if !(NRF51 - 0)
GPIOTE::task_reference_type
GPIOTE::TASKS_SET ()
{
  return nrf5::GPIOTE->TASKS_SET[channel];
}

GPIOTE::task_reference_type
GPIOTE::TASKS_CLR ()
{
  return nrf5::GPIOTE->TASKS_CLR[channel];
}
#endif /* !NRF51 */

void
GPIOTE::sense_listener::enable ()
{
  mutex_type mutex;
  sense_chain_.link_back(*this);
}

void
GPIOTE::sense_listener::disable ()
{
  mutex_type mutex;
  sense_chain_.unlink(*this);
}

void
GPIOTE::enable_sense ()
{
  nrf5::GPIOTE->EVENTS_PORT = 0;
  nrf5::GPIOTE->INTENSET = (GPIOTE_INTENSET_PORT_Set << GPIOTE_INTENSET_PORT_Pos);
  // @todo run through listeners to initialize them
}

GPIOTE*
GPIOTE::allocate ()
{
  mutex_type mutex;

  unsigned int c = 0;
  registry_type bit = 1;
  do {
    if (!(bit & in_use_)) {
      in_use_ |= bit;
      return instances_ + c;
    }
    ++c;
    bit <<= 1;
  } while (c < CHANNEL_COUNT);
  return nullptr;
}

void
GPIOTE::release ()
{
  mutex_type mutex;

  in_use_ &= ~(1U << channel);
  config_disabled_bi_();
  disable_event();
  if (listener_) {
    listener_->disable_bi_();
  }
}

void
GPIOTE::synchronize_sense ()
{
  mutex_type mutex;
  sense_process_chain_bi(sense_chain_);
}

void
GPIOTE::irq_handler (void)
{
  auto scope = make_scoped_instr(instr_gpiote_flih);

  for (auto ri = 0U; ri < CHANNEL_COUNT; ++ri) {
    if (nrf5::GPIOTE->EVENTS_IN[ri]) {
      auto& instance = instances_[ri];
      nrf5::GPIOTE->EVENTS_IN[ri] = 0;
      auto listener = instance.listener_;
      if (listener && listener->callback_bi) {
        auto scope = make_scoped_instr(instr_gpiote_event_flih);
        listener->callback_bi(instance);
      }
    }
  }
  if (nrf5::GPIOTE->EVENTS_PORT) {
    sense_process_chain_bi(sense_chain_);
  }
}

} // namespace gpio
} // namespace nrfcxx
