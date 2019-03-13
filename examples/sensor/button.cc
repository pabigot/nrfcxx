// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Example of using Button infrastructure. */

#include <cstdio>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/sensor/button.hpp>

namespace {

#define EVT_EDGE_BUTTON0 0x01
#define EVT_LEVEL_BUTTON1 0x02

const char* const button_state[] = {
  "released",
  "pressed",
};

auto& led0 = nrfcxx::led::led_type::lookup(0);
auto& led1 = nrfcxx::led::led_type::lookup(1);

void b0_callback (unsigned int evt,
                  uint64_t duration_utt)
{
  using nrfcxx::sensor::Button;

  led0.set(evt ? 1 : 0);
  using nrfcxx::clock::uptime;
  uptime::text_type buf;
  printf("B0 %s ", uptime::as_text(buf, uptime::now()));
  printf("%s for %s\n", Button::eventstr(evt), uptime::as_text(buf, duration_utt));
}

#ifdef NRFCXX_BOARD_PSEL_BUTTON1
void b1_callback (unsigned int evt,
                  uint64_t duration_utt)
{
  using nrfcxx::sensor::Button;
  led1.set(evt ? 1 : 0);
  using nrfcxx::clock::uptime;
  uptime::text_type buf;
  printf("B1 %s ", uptime::as_text(buf, uptime::now()));
  printf("%s for %s\n", Button::eventstr(evt), uptime::as_text(buf, duration_utt));
}
#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */

} // anonymous namespace

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
  using sensor::Button;

  board::initialize();

  led0.enable();
  led1.enable();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  event_set events;

  Button button0{NRFCXX_BOARD_PSEL_BUTTON0, events.make_setter(EVT_EDGE_BUTTON0), b0_callback};
  printf("B0 initial %s\n", Button::eventstr(button0.state()));
  led0.set(button0.state());

#ifdef NRFCXX_BOARD_PSEL_BUTTON1
  Button button1{NRFCXX_BOARD_PSEL_BUTTON1, events.make_setter(EVT_LEVEL_BUTTON1), b1_callback};
  printf("B1 initial %s\n", Button::eventstr(button1.state()));
  led1.set(button1.state());
#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */

  NVIC_EnableIRQ(GPIOTE_IRQn);

  while (true) {
    event_set::cev();
    auto pending = events.copy_and_clear();
    if (pending.test_and_clear(EVT_EDGE_BUTTON0)) {
      button0.process();
    }
#ifdef NRFCXX_BOARD_PSEL_BUTTON1
    if (pending.test_and_clear(EVT_LEVEL_BUTTON1)) {
      button1.process();
    }
#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */
    do {
      systemState::WFE();
    } while (!events);
  }
}
