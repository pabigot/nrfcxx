// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <pabigot/container.hpp>

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/periph.hpp>

#define INSTR_PSEL_TASK_RNG NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_EVENT_VALRDY NRFCXX_BOARD_PSEL_SCOPEn

namespace nrfcxx {
namespace periph {

namespace {

std::array<uint8_t, NRFCXX_PERIPH_RNG_BUFFER_SIZE> rng_storage;
pabigot::container::rr_adaptor<uint8_t> rng_buffer{&rng_storage[0], rng_storage.max_size()};

gpio::instr_psel<INSTR_PSEL_TASK_RNG> instr_task_rng;
gpio::instr_psel<INSTR_PSEL_EVENT_VALRDY> instr_event_valrdy;

} // anonymous

void*
RNG::fill (void* dest,
           size_t count)
{
  static bool initialized;

  if (!initialized) {

    instr_task_rng.enable();
    instr_event_valrdy.enable();

    nrf5::RNG->EVENTS_VALRDY = 0;
    nrf5::RNG->CONFIG = (RNG_CONFIG_DERCEN_Enabled << RNG_CONFIG_DERCEN_Pos);
    nrf5::RNG->INTENCLR = -1;

    rng_buffer.clear();

    nvic_ClearPendingIRQ(nrf5::RNG.IRQn);
    nvic_SetPriority(nrf5::RNG.IRQn, IRQ_PRIORITY_APP_LOW);
    nvic_EnableIRQ(nrf5::RNG.IRQn);

    nrf5::RNG->INTENSET = (RNG_INTENSET_VALRDY_Set << RNG_INTENSET_VALRDY_Pos);

    initialized = true;
  }

  auto dp = static_cast<uint8_t*>(dest);
  const uint8_t* const dpe = dp + count;
  bool started = false;

  /* Prevent the IRQ from manipulating the fifo while we're reading
   * from it. */
  nrfcxx::nvic_BlockIRQ block{nrf5::RNG.IRQn};

  while (dp < dpe) {
    bool need_start = false;
    if (rng_buffer.empty()) {
      /* IRQ is blocked, so we have to (a) start up the generator and
       * (b) read the next value directly as soon as it's ready. */
      need_start = true;
      if (started) {
        /* Spin until the next value is ready, then store it into the
         * output */
        while (!nrf5::RNG->EVENTS_VALRDY) {
        }
        instr_event_valrdy.assert();
        *dp++ = nrf5::RNG->VALUE;
        nrf5::RNG->EVENTS_VALRDY = 0;
        instr_event_valrdy.deassert();
      }
    } else {
      /* Read the next value out of the fifo.  If we're done, start
       * the generator to replenish what we removed. */
      *dp++ = rng_buffer.pop();
      need_start = (dp == dpe);
    }
    if (need_start && (!started)) {
      nrf5::RNG->TASKS_START = 1;
      started = true;
      instr_task_rng.assert();
    }
  }

  return dest;
}

extern "C" {

void RNG_IRQHandler ()
{
  if (nrf5::RNG->EVENTS_VALRDY) {
    instr_event_valrdy.assert();

    /* PAN#21 RNG: Generated random value is reset when VALRDY event
     * is cleared.
     *
     * Workaround: read before clearing event */
    rng_buffer.push(nrf5::RNG->VALUE);

    /* If the buffer is full, reduce power consumption by turning off
     * the generator. */
    if (rng_buffer.full()) {
      /* PAN#23 RNG: STOP stasks clears the VALUE register.
       *
       * Workaround: read before stopping */
      nrf5::RNG->TASKS_STOP = 1;
      instr_task_rng.deassert();
    }

    /* PAN#22 RNG: RNG does not generate a new number after the
     * current number generated.
     *
     * Workaround: Tell it we've read the value. */
    nrf5::RNG->EVENTS_VALRDY = 0;
    instr_event_valrdy.deassert();
  }
}

} // extern C

} // namespace periph
} // namespace nrfcxx
