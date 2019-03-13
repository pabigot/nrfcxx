// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Play with SX1509B on SparkFun dev board.  Or Thingy:52.
 *
 * This can be used for arbitrary commands to the device, or to verify
 * the helper APIs like LED support.
 *
 * NB: Saleae Logic I2C decoding is shakey, possibly due to clock
 * stretching or to poor electrical connections.  This can be resolved
 * by enabling the glitch filter at 2 us for the channels being
 * monitored (particularly the clock channel). */

#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/impl.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/utility.hpp>
#include <nrfcxx/misc/sx1509b.hpp>

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
#include <nrfcxx/board/thingy52.hpp>
#define LED0_PSEL NRFCXX_BOARD_IOX_EXT0
#define LED1_PSEL NRFCXX_BOARD_IOX_SENSE_LED_G
#define LED2_PSEL NRFCXX_BOARD_IOX_LIGHTWELL_B
#elif (NRFCXX_BOARD_IS_BLE400 - 0)
#define RESETn_PSEL NRFCXX_BOARD_PSEL_TWI0_SMBA
#define LED0_PSEL 0
#define LED1_PSEL 1
#define LED2_PSEL 4
#else
#define RESETn_PSEL NRFCXX_BOARD_PSEL_SCOPEn
#define LED0_PSEL 0
#define LED1_PSEL 1
#define LED2_PSEL 2
#endif

static constexpr uint8_t I2C_ADDR = 0x3e;

void
dump_regs (const char* tag,
           const nrfcxx::misc::sx1509b& sx1509b)
{
  auto& regs = sx1509b.gpio_cache();
  puts(tag);
  printf("%04x InputDisable\n", regs.input_disable);
  printf("%04x LongSlew\n", regs.long_slew);
  printf("%04x LowDrive\n", regs.low_drive);
  printf("%04x PullUp\n", regs.pull_up);
  printf("%04x PullDown\n", regs.pull_down);
  printf("%04x OpenDrain\n", regs.open_drain);
  printf("%04x Polarity\n", regs.polarity);
  printf("%04x Dir\n", regs.dir);
  printf("%04x Data\n", regs.data);
  printf("%02x Clock\n", sx1509b.clock_cache());
  printf("%02x Misc\n", sx1509b.misc_cache());
  printf("%04x LEDDriverEnable\n", sx1509b.led_driver_cache());
}

void
dump_all (const uint8_t* sp)
{
  using nrfcxx::misc::sx1509b;

  static const char* const gpio[] = {
    "2InputDisable",
    "2LongSlew",
    "2LowDrive",
    "2PullUp",
    "2PullDown",
    "2OpenDrain",
    "2Polarity",
    "2Dir",
    "2Data",
    "2InterruptMask",
    "2SenseHigh",
    "2SenseLow",
    "2InterruptSource",
    "2EventStatus",
    "2LevelShifter",
    "1Clock",
    "1Misc",
    "2LEDDriverEnable",
    "1DebounceConfig",
    "2DebounceEnable",
    "1KeyConfig1",
    "1KeyConfig2",
    "1KeyData1",
    "1KeyData2",
    "BALED0",
    "BALED1",
    "BALED2",
    "BALED3",
    "FALED4",
    "FALED5",
    "FALED6",
    "FALED7",
    "BBLED0",
    "BBLED1",
    "BBLED2",
    "BBLED3",
    "FBLED4",
    "FBLED5",
    "FBLED6",
    "FBLED7",
    "2HighInput",
  };
  auto const sps = sp;
  auto rpp = gpio;
  auto const rppe = rpp + sizeof(gpio) / sizeof(*gpio);
  while (rpp < rppe) {
    const char type = **rpp;
    auto const name = 1 + *rpp;
    ++rpp;
    printf("%02x: %20s ", sp - sps, name);
    switch (type) {
      case '1':
        printf("%02x", *sp++);
        break;
      case '2':
        {
          uint16_t v;
          memmove(&v, sp, 2);
          sp += 2;
          printf("%04x", pabigot::byteorder::host_x_be(v));
        }
        break;
      case 'B':
      case 'F':
        printf("%u %u %u:%u", sp[0], sp[1],
               (sx1509b::RegOffX_TOff_Msk & sp[2]) >> sx1509b::RegOffX_TOff_Pos,
               (sx1509b::RegOffX_IOff_Msk & sp[2]) >> sx1509b::RegOffX_IOff_Pos);
        sp += 3;
        if ('F' == type) {
          printf(" %u %u", sp[0], sp[1]);
          sp += 2;
        }
        break;
    }
    putchar('\n');
  }
}

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  puts("\n" __FILE__ " " __DATE__ " " __TIME__);

  int rc;
#if (NRFCXX_BOARD_IS_THINGY52 - 0)
  auto& twi = board::twi();
  auto& sx1509b = board::iox();
#else // NRFCXX_BOARD_IS
  auto& twi = periph::TWI::instance(0);
  rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                         NRFCXX_BOARD_PSEL_TWI0_SDA,
                         (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                         1000);
  printf("TWI init got %d\n", rc);

  printf("SX1509B on I2C addr %02x via SCL %d SDA %d\n",
         I2C_ADDR,
         NRFCXX_BOARD_PSEL_TWI0_SCL,
         NRFCXX_BOARD_PSEL_TWI0_SDA);
  printf("RESETn on %d\n", RESETn_PSEL);

  misc::sx1509b::iface_config_type sx1509b_ifc{twi, RESETn_PSEL};
  misc::sx1509b sx1509b{sx1509b_ifc};
#endif

  const uint8_t I2C_ADDR = sx1509b.iface_config().address;

  rc = sx1509b.sw_reset();
  printf("Hardware reset got %d\n", rc);
  sleep_ms(rc);                  // t_RESET = 2.5 ms

  uint8_t buf[128];
  buf[0] = 0;
  if (auto enabler = twi.scoped_enable()) {
    rc = twi.write_read(I2C_ADDR, buf, 1, buf, sizeof(buf));
  } else {
    rc = enabler.result();
  }
  printf("reg read got %d\n", rc);

  utility::display_data(buf, sizeof(buf), 0);
  dump_all(buf);

  dump_regs("initial config", sx1509b);

  rc = sx1509b.clock();
  printf("Clock: %d %02x\n", rc, rc);
  if (0 <= rc) {
    uint8_t divexp = 0;          // 2^2 should be 0.5 MHz
    uint8_t clock = (0
                     | (sx1509b.RegClock_FREQ_2MHz << sx1509b.RegClock_FREQ_Pos)
                     | (sx1509b.RegClock_OSCIO_Output << sx1509b.RegClock_OSCIO_Pos)
                     | (sx1509b.RegClock_OUTFREQ_Msk & (((1 + divexp) << sx1509b.RegClock_OUTFREQ_Pos)))
                     );
    rc = sx1509b.clock(clock);
    printf("Set OSCIO output at %u Hz value %02x got %d\n",
           (2000000U >> divexp), clock, rc);
    rc = sx1509b.clock();
    printf("Check: %d %02x\n", rc, rc);
  }
  rc = sx1509b.misc();
  printf("Misc: %d %02x\n", rc, rc);
  if (0 <= rc) {
    uint8_t divexp = 3;         // 2^3 should be 250 kHz
    uint8_t misc = (0
                    | (sx1509b.RegMisc_LEDFREQ_Msk & ((1 + divexp) << sx1509b.RegMisc_LEDFREQ_Pos))
                    );
    rc = sx1509b.misc(misc);
    printf("Set LED ClkX to %u Hz value %02x got %d\n",
           (2000000U >> divexp), misc, rc);
    rc = sx1509b.misc();
    printf("Check: %d %02x\n", rc, rc);
  }

  /* f_OSC is nominally 2 MHz, but can range from 1.3 MHs to 2.6 MHz.
   * It is temperature and Vdd dependent.
   *
   * My test device is 2.083 MHz.
   *
   * LClk is configured at nominal 2 MHz / 8 = 250 kHz.
   *
   * On/Off:
   * Duration values from 1 to 15 use: d * 64 * 255 / LClk ~ 65.28 ms
   * Duration values from 16 to 31 use: d * 512 * 255 / LClk ~ 522 ms / delta d
   * Duration 1 is 65.28 ms
   * Duration 15 is 979.2 ms
   * Duration 16 is 8.356 s
   * Duration 31 is 16.19 s
   *
   * Rise/Fall: similar but 64 and 512 are replaced by I and 16 * I
   * where I is IOn - 4*IOff.  With default IOn=255 and IOff=0 the
   * rise/fall duration is four times as long as on/off duration for a
   * given d: A 1 unit rise is 261 ms.
   */

  if (0 <= LED0_PSEL) {
    rc = sx1509b.configure(LED0_PSEL, gpio::PIN_CNF_WRONLY, 1);
    printf("G%d set to output: %d\n", LED0_PSEL, rc);
  }

  printf("%u and %u: %08x %08x\n", LED1_PSEL, LED2_PSEL,
         sx1509b.configuration(LED1_PSEL),
         sx1509b.configuration(LED2_PSEL));
  uint16_t leds = (1U << LED1_PSEL) | (1U << LED2_PSEL);
  rc = sx1509b.configure_as_leds(leds, true);
  printf("LED configure %04x got %d\n", leds, rc);
  printf("%u and %u: %08x %08x\n", LED1_PSEL, LED2_PSEL,
         sx1509b.configuration(LED1_PSEL),
         sx1509b.configuration(LED2_PSEL));

  /* Static mode: ton = 0.
   * Follows edge of Data register.  ion controls duty cycle.
   *
   * One-shot mode: ton != 0, toff = 0.  Triggers on falling edge.  If
   * rising edge occurs too soon the LED does not light.
   *
   * Blink mode: ton != 0, off != 0.
   */

  /* One-shot test. */
  misc::sx1509b::led_type lblk;
  misc::sx1509b::led_pwm_type lpwm;

  //lblk.ton = 1;
  //lblk.off = (15 << sx1509b.RegOffX_TOff_Pos) | (4 << sx1509b.RegOffX_IOff_Pos);
  //lblk.off = 15 << sx1509b.RegOffX_TOff_Pos;
  rc = sx1509b.led_configure(LED1_PSEL, lblk);
  printf("LED1 on %u configure %u %u %u got %d\n", LED1_PSEL,
         lblk.ton, lblk.ion, lblk.off, rc);
  //lpwm.ton = 2;
  lpwm.trise = 1;
  lpwm.tfall = 1;
  rc = sx1509b.led_configure(LED2_PSEL, lpwm);
  printf("LED2 on %u configure got %d\n", LED2_PSEL, rc);

  if (0 <= LED0_PSEL) {
    leds |= (1U << LED0_PSEL);
  }

  if (true) {
    rc = sx1509b.output_sct(0, leds);
    sleep_ms(2000);
    rc = sx1509b.output_sct(leds, 0);
    printf("Pulse done\n");
  } else if (true) {
    auto iter = 4U;
    do {
      rc = sx1509b.output_sct(0, leds);
      printf("LEDS %04x on got %d\n", leds, rc);
      sleep_ms(10);
      rc = sx1509b.output_sct(leds, 0);
      printf("LEDS off got %d\n", rc);
      sleep_ms(1000);
    } while (--iter);
  } else {
    rc = sx1509b.output_sct(0, leds);
    rc = sx1509b.output_sct(leds, 0);
  }

  if (true) {
    buf[0] = 0;
    if (auto enabler = twi.scoped_enable()) {
      rc = twi.write_read(I2C_ADDR, buf, 1, buf, sizeof(buf));
    } else {
      rc = enabler.result();
    }
    dump_all(buf);
  }

  return 0;
}
