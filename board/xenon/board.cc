// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Peter A. Bigot

#include <array>

#include <nrfcxx/led.hpp>
#include <nrfcxx/board/xenon.hpp>
#include <nrfcxx/misc/lipomon.hpp>
#include <nrfcxx/sensor/adc.hpp>

namespace nrfcxx {
namespace board {

gpio::gpio_pin led0pin{13};
gpio::gpio_pin led1pin{14};
gpio::gpio_pin led2pin{15};
/* LED3 on P1.12 is active-high so cannot be controlled along with the
 * others, which are all active-low. */

sensor::adc::voltage_divider meas_vbatt{NRFCXX_BOARD_BATTERY_R1,
    NRFCXX_BOARD_BATTERY_R2,
    NRFCXX_BOARD_BATTERY_AIN};

power_monitor::power_monitor (notifier_type notify) :
  super{notify,
    gpio::gpio_pin{NRFCXX_BOARD_PSEL_VCHG_DETECT},
    gpio::gpio_pin{NRFCXX_BOARD_PSEL_CHGn_STATUS},
    meas_vbatt}
{
  /*
   * On Xenon and Argon LiPo charging is managed with a Torex XC6802A42
   * charger.  R_SEN is a single 2 kOhm resistor, so I_BAT is nominally
   * 500 mA.
   *
   * The XC6802 performs trickle charging at 50 mA until V_BAT > 2.9 V,
   * then switches to main charging at 500 mA until V_BAT reaches 4.05
   * V, then continues with charging at constant voltage until charge
   * current drops below 50 mA at which point it stops.
   */
}

bool
external_antenna (int on)
{
  bool rv = on;
  auto pcben = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_SKY_PCBEN);
  auto uflen = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_SKY_UFLEN);

  if (0 > on) {
    rv = uflen.is_set();
  } else if (rv) {
    pcben.clear();
    uflen.set();
  } else {
    pcben.set();
    uflen.clear();
  }
  return rv;
}

/* Override from core. */
int
initialize (bool enable_hfxt)
{
  int rc = clock::initialize(enable_hfxt);

  if (0 > rc) {
    return rc;
  }

  /* Configure for the PCB antenna. */
  {
    auto pcben = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_SKY_PCBEN);
    auto uflen = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_SKY_UFLEN);

    pcben.configure(gpio::PIN_CNF_WRONLY);
    uflen.configure(gpio::PIN_CNF_WRONLY);
    external_antenna(false);
  }

  /* Place the MX25L32 into deep power-down mode using SPI2. */
  auto& spi = periph::SPI::instance(2);
  rc = spi.bus_configure(NRFCXX_BOARD_PSEL_SPI2_SCK,
                         NRFCXX_BOARD_PSEL_SPI2_MOSI,
                         NRFCXX_BOARD_PSEL_SPI2_MISO,
                         SPI_FREQUENCY_FREQUENCY_M1,
                         spi.config_from_mode(0));
  if (0 > rc) {
    return rc;
  }

  auto csn = gpio::pin_reference::create(NRFCXX_BOARD_PSEL_EFL_CSn);
  csn.set();
  csn.configure(gpio::PIN_CNF_WRONLY);
  if (auto enabler = spi.scoped_enable()) {
    uint8_t cmd = 0xB9;
    csn.clear();
    rc = spi.tx_rx(&cmd, sizeof(cmd), 0, nullptr);
    csn.set();
  } else {
    rc = enabler.result();
  }
  csn.configure(gpio::PIN_CNF_PWRUP);
  csn.clear();
  return rc;
}

periph::QSPI& mx25l32 () noexcept
{
  using periph::QSPI;

  /* Micronix MX25L3233F, JEDEC id C22016 */
  static const QSPI::configuration_type cfg = {
    .ifconfig = {
      ((QSPI_IFCONFIG0_READOC_READ4IO << QSPI_IFCONFIG0_READOC_Pos)
       | (QSPI_IFCONFIG0_WRITEOC_PP4IO << QSPI_IFCONFIG0_WRITEOC_Pos)
       | (QSPI_IFCONFIG0_ADDRMODE_24BIT << QSPI_IFCONFIG0_ADDRMODE_Pos)
       | (QSPI_IFCONFIG0_DPMENABLE_Enable << QSPI_IFCONFIG0_DPMENABLE_Pos)
       | (QSPI_IFCONFIG0_PPSIZE_256Bytes << QSPI_IFCONFIG0_PPSIZE_Pos)),
      ((1 << QSPI_IFCONFIG1_SCKDELAY_Pos) // 62.5 ns
       | (QSPI_IFCONFIG1_DPMEN_Exit << QSPI_IFCONFIG1_DPMEN_Pos)
       | (QSPI_IFCONFIG1_SPIMODE_MODE0 << QSPI_IFCONFIG1_SPIMODE_Pos)
       | (0 << QSPI_IFCONFIG1_SCKFREQ_Pos)), // 32 MHz
    },
    .psel_io = {
      NRFCXX_BOARD_PSEL_QSPI_IO0,
      NRFCXX_BOARD_PSEL_QSPI_IO1,
      NRFCXX_BOARD_PSEL_QSPI_IO2,
      NRFCXX_BOARD_PSEL_QSPI_IO3,
    },
    .psel_sck = NRFCXX_BOARD_PSEL_SPI2_SCK,
    .psel_csn = NRFCXX_BOARD_PSEL_EFL_CSn,
    .enter_dpmdur = nrfcxx::periph::QSPI::convert_us_dpmdur(40),
    .exit_dpmdur = nrfcxx::periph::QSPI::convert_us_dpmdur(100),
    // At this time no evidence we need drive_high
  };
  static QSPI instance{cfg};

  return instance;
}

} // ns board

namespace led {
namespace {

generic_led<board::led_active_low> led0{board::led0pin};
generic_led<board::led_active_low> led1{board::led1pin};
generic_led<board::led_active_low> led2{board::led2pin};

std::array<led_type*, 3> leds = {&led0, &led1, &led2};

} // ns anonymous

led_type** const led_type::ledps_ = &leds[0];
uint8_t const led_type::nleds_ = leds.max_size();

} // ns led

} // ns nrfcxx
