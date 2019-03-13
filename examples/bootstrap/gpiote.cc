/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2018-2019 Peter A. Bigot */

/** Demonstration of interrupt monitoring using edge or level
 * triggered signals.
 *
 * By default button 0 is monitored for edge events, and button 1
 * (where available) is monitored for level transitions.  At most one
 * edge GPIO and two level GPIOs may be configured.
 *
 * The primary purpose of tracking two level GPIOs is to wire a button
 * on one GPIO instance to a pin on a second GPIO instance to verify
 * that the sense listener infrastructure tracks changes on distinct
 * peripheral instances synchronously. */

#include <cstdio>

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>

#define EVT_EDGE 0x01
#define EVT_LEVEL 0x02
#define EVT_LEVEL2 0x04

#if 1
// Default edge to BTN0, level to BTN1 where available
#define PSEL_EDGE NRFCXX_BOARD_PSEL_BUTTON0
#ifdef NRFCXX_BOARD_PSEL_BUTTON1
#define PSEL_LEVEL NRFCXX_BOARD_PSEL_BUTTON1
#endif // NRFCXX_BOARD_PSEL_BUTTON1
#else // configuration
// Customized selection here.
#define PSEL_LEVEL NRFCXX_BOARD_PSEL_BUTTON0
#if 1
#ifdef NRFCXX_BOARD_PSEL_BUTTON1
#define PSEL_EDGE NRFCXX_BOARD_PSEL_BUTTON1
#endif // NRFCXX_BOARD_PSEL_BUTTON1
#else
// Use a paired button on another GPIO instance
#define PSEL_LEVEL2 (32 + 1)
#endif
#endif // configuration

#ifndef PSEL_EDGE
#define PSEL_EDGE -1
#endif // PSEL_EDGE
#ifndef PSEL_LEVEL
#define PSEL_LEVEL -1
#endif // PSEL_LEVEL
#ifndef PSEL_LEVEL2
#define PSEL_LEVEL2 -1
#endif // PSEL_LEVEL2

extern "C" {
void GPIOTE_IRQHandler (void)
{
  nrfcxx::periph::GPIOTE::irq_handler();
}
} // extern "C"

int
main (void)
{
  using namespace nrfcxx;

  board::initialize();

  using board::button_active_low;
  auto& led0 = led::led_type::lookup(0);
  auto& led1 = led::led_type::lookup(1);
  led0.enable();
  led1.enable();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto gpiote = periph::GPIOTE::allocate();
  if (!gpiote) {
    puts("no channel");
    return -1;
  }
  printf("Channel %u\n", gpiote->channel);

  const auto pin_cnf = (button_active_low
                        ? gpio::PIN_CNF_ACTIVE_LOW
                        : gpio::PIN_CNF_ACTIVE_HIGH);

  event_set events;

#if (0 <= PSEL_EDGE)
  volatile unsigned int edge_count = 0;
  gpio::gpio_pin edge{PSEL_EDGE};
  auto& edge_pr = edge.implementation();
  edge.configure(pin_cnf & ~GPIO_PIN_CNF_SENSE_Msk);
  gpiote->config_event(edge_pr.global_psel);
  periph::GPIOTE::event_listener edge_btn{[&](auto& gpiote)
      {
        ++edge_count;
        events.set(EVT_EDGE);
      }
  };
  edge_btn.enable(*gpiote);
  gpiote->enable_event();

  printf("Button on pin %u edge configured on GPIOTE %u\n", edge_pr.global_psel, gpiote->channel);
#endif

#if (0 <= PSEL_LEVEL)
  gpio::gpio_pin level{PSEL_LEVEL};
  auto& level_pr = level.implementation();
  level.configure(pin_cnf);
  volatile int level_state = 0;

  printf("Button on pin %u level configured\n", level_pr.global_psel);

#if (0 <= PSEL_LEVEL2)
  auto level2_pr = gpio::pin_reference::create(PSEL_LEVEL2);
  level2_pr.configure(level.configuration());
  volatile int level2_state = 0;
  printf("Button on pin %u alternative level configured\n", level2_pr.global_psel);
#endif // PSEL_LEVEL2

  periph::GPIOTE::sense_listener level_btn{[&](auto sp)
      {
        while ((0 <= sp->psel)
               && ((sp->psel <= level_pr.global_psel)
#if (0 <= PSEL_LEVEL2)
                   || (sp->psel <= level2_pr.global_psel)
#endif // PSEL_LEVEL2
                   )) {
          if (level_pr.global_psel == sp->psel) {
            // Add the count from the previous capture to the new state,
            // discarding the previous pin state.
            level_state = sp->aggregate_state(level_state);
            if (1 < sp->counter_state) {
              events.set(EVT_LEVEL);
            }
          }
#if (0 <= PSEL_LEVEL2)
          if (level2_pr.global_psel == sp->psel) {
            level2_state = sp->aggregate_state(level2_state);
            if (1 < sp->counter_state) {
              events.set(EVT_LEVEL2);
            }
          }
#endif // PSEL_LEVEL2
          ++sp;
        }
      }
  };
  level_btn.enable();
  periph::GPIOTE::enable_sense();
  printf("Level configuration active\n");
  {
    auto in = level_pr.peripheral->IN;

    printf("Level1 on %u.%02u PIN_CNF %lx IN %lx %u\n",
           level_pr.peripheral.INSTANCE, level_pr.local_psel,
           level_pr.peripheral->PIN_CNF[level_pr.local_psel],
           in, !!(in & level_pr.local_bit));
#if (0 <= PSEL_LEVEL2)
    in = level2_pr.peripheral->IN;

    printf("Level2 on %u.%02u PIN_CNF %lx IN %lx %u\n",
           level2_pr.peripheral.INSTANCE, level2_pr.local_psel,
           level2_pr.peripheral->PIN_CNF[level2_pr.local_psel],
           in, level2_pr.read());
#endif // PSEL_LEVEL2
  }

#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */

  NVIC_EnableIRQ(GPIOTE_IRQn);
  periph::GPIOTE::synchronize_sense();

  while (true) {
    event_set::cev();
    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      printf("events: %x\n", static_cast<unsigned int>(pending.events()));
#if (0 <= PSEL_LEVEL)
      if (pending.test_and_clear(EVT_LEVEL)) {
        unsigned int cs;
        sleep_ms(10);           // allow time to aggregate multiple changes
        {
          periph::GPIOTE::mutex_type mutex;
          cs = level_state;
        }
        auto in = level_pr.peripheral->IN;
        printf("Level1 state %x signal %d, changed %u : PIN_CNF %lx IN %lx %u\n",
               cs, (1 & cs), cs >> 1,
               level_pr.peripheral->PIN_CNF[level_pr.local_psel],
               in, level_pr.read());
        led1.set(cs & 1);
      }
#endif // PSEL_LEVEL
#if (0 <= PSEL_LEVEL2)
      if (pending.test_and_clear(EVT_LEVEL2)) {
        unsigned int cs;
        {
          periph::GPIOTE::mutex_type mutex;
          cs = level2_state;
        }
        auto in = level2_pr.peripheral->IN;
        printf("Leval2 state %x signal %d, changed %u : PIN_CNF %lx IN %lx %u\n",
               cs, (1 & cs), cs >> 1,
               level2_pr.configuration(),
               in, level2_pr.read());
      }
#endif // PSEL_LEVEL2
#if (0 <= PSEL_EDGE)
      if (pending.test_and_clear(EVT_EDGE)) {
        const bool active = button_active_low ^ edge_pr.read();
        unsigned int count;
        led0.set(active);
        {
          periph::GPIOTE::mutex_type mutex;
          count = edge_count;
        }
        printf("Edge event %u, now %d\n", count, active);
      }
      printf("Edge button now logical %u\n", button_active_low ^ edge_pr.read());
#endif // PSEL_EDGE
    }
    systemState::WFE();
  }
}
