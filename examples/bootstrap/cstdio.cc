// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2015-2019 Peter A. Bigot

/** Demonstrate and test use of C stdio API.
 *
 * Configures the UART and displays the pins that it uses.
 *
 * If `WRITE_ONLY` is not enabled (default) then the application
 * bus-waits for 1 s then displays at most one character of input and
 * toggles the LED.  The LED toggle indicates that the application is
 * running.
 *
 * If `WRITE_ONLY` is enabled then the application configures the UART
 * for auto-enable and sleeps for 1 s between counter outputs.  This
 * can be used with an ammeter to confirm that that the UART (and
 * hence the high-frequency clock) is being disabled while
 * sleeping. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/clock.hpp>
#include <cstdio>

#define WRITE_ONLY 0

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  auto& led = nrfcxx::led::lookup(0);
  led.enable();
  led.on();

  puts("\n" __FILE__ " " __DATE__ " " __TIME__);
#if (WRITE_ONLY - 0)
  nrfcxx::periph::UART::instance().autoenable(1);
#endif

  auto& uart = nrfcxx::periph::UART::instance().peripheral();
  bool hwfc = ((UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos) & uart->CONFIG);
  printf("RXD %d pin %lu\n",
         NRFCXX_BOARD_PSEL_UART0_RXD, uart->PSEL.RXD);
  printf("TXD %d pin %lu\n",
         NRFCXX_BOARD_PSEL_UART0_TXD, uart->PSEL.TXD);
#ifdef NRFCXX_BOARD_PSEL_UART0_RTS
  printf("RTS %d pin %lu\n",
         NRFCXX_BOARD_PSEL_UART0_RTS, uart->PSEL.RTS);
#endif /* NRFCXX_BOARD_PSEL_UART0_RTS */
#ifdef NRFCXX_BOARD_PSEL_UART0_CTS
  printf("CTS %d pin %lu\n",
         NRFCXX_BOARD_PSEL_UART0_CTS, uart->PSEL.CTS);
#endif /* NRFCXX_BOARD_PSEL_UART0_CTS */
  printf("HWFC %s\n", hwfc ? "ON" : "off");

  unsigned int ctr = 0;
  while (1) {
    int rc = 0;
#if !(WRITE_ONLY - 0)
    rc = getchar();
#endif // WRITE_ONLY
    printf("ctr %u gc %d\n", ++ctr, rc);
#if (WRITE_ONLY - 0)
    /* Current should reduce to minimum when transmission complete. */
    sleep_ms(1000);
#else // WRITE_ONLY
    delay_us(1000000);
#endif // WRITE_ONLY
    led.toggle();
  }

  return 0;
}
