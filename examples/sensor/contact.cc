// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2017-2019 Peter A. Bigot

/** Use buttons to demonstrate the dry contact sensor API. */

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/clock.hpp>
#include <nrfcxx/sensor/contact.hpp>
#include <cstdio>

#ifndef WITH_EDGE
#define WITH_EDGE 1
#endif

#define EVT_EDGE_BUTTON0 0x01
#define EVT_LEVEL_BUTTON1 0x02

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
  using sensor::contact;

  board::initialize();

  auto& led0 = led::led_type::lookup(0);
  auto& led1 = led::led_type::lookup(1);
  led0.enable();
  led1.enable();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  auto b0ch = periph::GPIOTE::allocate();
  if (!b0ch) {
    puts("no channel");
    return -1;
  }
  printf("Channel %u\n", b0ch->channel);

  const auto pin_cnf = (board::button_active_low
                        ? gpio::PIN_CNF_ACTIVE_LOW
                        : gpio::PIN_CNF_ACTIVE_HIGH);
  const char* const button_state[] = {
    board::button_active_low ? "open/pressed" : "open/released",
    board::button_active_low ? "closed/released" : "closed/pressed",
  };

  event_set events;

  contact::state_type ss0;
#if (WITH_EDGE - 0)
  contact ordinal0{NRFCXX_BOARD_PSEL_BUTTON0, [&events](auto ordinal)
      {
        events.set(EVT_EDGE_BUTTON0);
      }, pin_cnf & ~GPIO_PIN_CNF_SENSE_Msk};
  b0ch->config_event(ordinal0.psel);
  periph::GPIOTE::event_listener button0{ordinal0.event_listener()};
  button0.enable(*b0ch);
  b0ch->enable_event();
  ss0 = ordinal0.snapshot();
  printf("Button 0 pin %u edge configured on GPIOTE %u, %s, ordinal %u\n",
         ordinal0.psel, b0ch->channel,
         button_state[ss0.live_state],
         ss0.ordinal);
#else /* WITH_EDGE */
  contact ordinal0{NRFCXX_BOARD_PSEL_BUTTON0, [&events](auto count)
      {
        events.set(EVT_EDGE_BUTTON0);
      }, pin_cnf};
  periph::GPIOTE::sense_listener button0{ordinal0.sense_listener()};
  button0.enable();
  ss0 = ordinal0.snapshot();
  printf("Button 0 pin %u active level configured, %s, count %u\n",
         ordinal0.psel,
         ss0.live_state ? "closed" : "open",
         ss0.count);
#endif /* WITH_EDGE */

#ifdef NRFCXX_BOARD_PSEL_BUTTON1
  contact ordinal1{NRFCXX_BOARD_PSEL_BUTTON1, [&events](auto ordinal)
      {
        events.set(EVT_LEVEL_BUTTON1);
      }, pin_cnf};
  periph::GPIOTE::sense_listener button1{ordinal1.sense_listener()};
  periph::GPIOTE::enable_sense();
  button1.enable();
  auto ss1 = ordinal1.snapshot();
  printf("Button 1 pin %u active level configured, %s, ordinal %u\n",
         ordinal1.psel,
         button_state[ss1.live_state],
         ss1.ordinal);
#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */

  {
    auto pinref = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_BUTTON0);

    printf("PIN_CNF %08lx\n", pinref.configuration());
    printf("us b0 = %u\n", gpio::update_sense_bi(pinref.global_psel));
    printf("us b0 = %u\n", gpio::update_sense_bi(pinref.global_psel));
    printf("PIN_CNF %08lx\n", pinref.configuration());
  }

  NVIC_EnableIRQ(GPIOTE_IRQn);
  periph::GPIOTE::synchronize_sense();

  while (true) {
    using clock::uptime;

    event_set::cev();
    auto pending = events.copy_and_clear();
    if (!pending.empty()) {
      printf("events: %x\n", static_cast<unsigned int>(pending.events()));
      if (pending.test_and_clear(EVT_EDGE_BUTTON0)) {
        auto ss = ordinal0.snapshot();
        if (ss.ordinal == ss0.ordinal) {
          puts("BTN0 redundant event");
        } else {
          led0.set(ss.live_state);
          printf("BTN0 edge event, live %s processed %s, ordinal %u\n",
                 button_state[ss.live_state],
                 button_state[ss.ordinal & 1],
                 ss.ordinal);
          auto lts = ss0.ordinal_ts();
          auto ecn = [&](auto eo, auto ts)
            {
              auto dts = static_cast<unsigned int>(ts - lts);
              lts = ts;
              printf("  B0 %u %s dur %u ticks = %u ms\n", eo, button_state[!contact::state_from_ordinal(eo)],
                     dts, 1000 * dts / contact::TIMESTAMP_Hz);
            };
          auto dts = ss.process_changes(ss0, ecn, false);
          printf("B0 span [ %u, %u ] in %u ms, end %s\n", ss0.ordinal + 1, ss.ordinal,
                 static_cast<unsigned int>(uptime::to_ms(uptime::from_duration(contact::to_duration(dts)))),
                 button_state[ss.state_from_ordinal()]);
          ss0 = ss;
        }
      }
#ifdef NRFCXX_BOARD_PSEL_BUTTON1
      if (pending.test_and_clear(EVT_LEVEL_BUTTON1)) {
        auto ss = ordinal1.snapshot();
        if (ss.ordinal == ss1.ordinal) {
          puts("BTN1 redundant event");
        } else {
          led1.set(ss.live_state);
          auto lts = ss1.ordinal_ts();
          auto ecn = [&](auto eo, auto ts) {
            auto dts = static_cast<unsigned int>(ts - lts);
            lts = ts;
            printf("  B1 %u %s dur %u ticks = %u ms\n", eo, button_state[!contact::state_from_ordinal(eo)],
                   dts, 1000 * dts / contact::TIMESTAMP_Hz);
          };
          auto dts = ss.process_changes(ss1, ecn);
          printf("B1 span [%u , %u] in %u ms, end %s\n", ss1.ordinal + 1, ss.ordinal,
                 static_cast<unsigned int>(uptime::to_ms(uptime::from_duration(contact::to_duration(dts)))),
                 button_state[ss.state_from_ordinal()]);
          ss1 = ss;
        }
      }
#endif /* NRFCXX_BOARD_PSEL_BUTTON1 */
    }
    systemState::WFE();
  }

}
