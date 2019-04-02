// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

/** This file contains Thingy:52 board support.
 *
 * It covers material from both <nrfcxx/board.hpp> and from
 * <nrfcxx/board/thingy52.hpp>. */

#include <array>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/board/thingy52.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/adc.hpp>
#include <nrfcxx/sensor/utils.hpp>

#if 0
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn

namespace nrfcxx {

namespace {

/* Define the default configuration of the nRF52 GPIOs.  Everything is
 * DISCON (PWRUP) except some are OUT CLR.  Some of the discon have
 * pulls. */
constexpr uint32_t gpio_out_clear = 0
  | (1U << NRFCXX_BOARD_PSEL_XL1)
  | (1U << NRFCXX_BOARD_PSEL_XL2)
  | (1U << NRFCXX_BOARD_PSEL_MOS_1)
  | (1U << NRFCXX_BOARD_PSEL_MOS_2)
  | (1U << NRFCXX_BOARD_PSEL_MOS_3)
  | (1U << NRFCXX_BOARD_PSEL_MOS_4)
  | (1U << NRFCXX_BOARD_PSEL_SPEAKER)
  | (1U << NRFCXX_BOARD_PSEL_SPK_PWR_CTRL)
  | (1U << NRFCXX_BOARD_PSEL_VDD_PWR_CTRL)
  ;
constexpr uint32_t gpio_pull_up = 0
  | (1U << NRFCXX_BOARD_PSEL_BUTTON0)
  ;
constexpr uint32_t gpio_pull_down = 0
  | (1U << NRFCXX_BOARD_PSEL_ANADIG0)
  | (1U << NRFCXX_BOARD_PSEL_ANADIG1)
  | (1U << NRFCXX_BOARD_PSEL_ANADIG2)
  | (1U << NRFCXX_BOARD_PSEL_IOX_OSCIO)
  | (1U << NRFCXX_BOARD_PSEL_MPU_INT)
  | (1U << NRFCXX_BOARD_PSEL_CCS_INT)
  | (1U << NRFCXX_BOARD_PSEL_LPS_INT)
  | (1U << NRFCXX_BOARD_PSEL_MIC_DOUT)
  | (1U << NRFCXX_BOARD_PSEL_MIC_CLK)
  | (1U << NRFCXX_BOARD_PSEL_BH_INT)
  ;

/* Using constants above determine the appropriate PIN_CNF setting for
 * the given GPIO. */
uint32_t gpio_default_pin_cnf (unsigned int psel)
{
  const auto bit = 1U << psel;
  if (gpio_out_clear & bit) {
    return gpio::PIN_CNF_WRONLY;
  }
  auto cnf = gpio::PIN_CNF_PWRUP;
  if (gpio_pull_up & bit) {
    cnf |= gpio::PIN_CNF_PULLUP;
  } else if (gpio_pull_down & bit) {
    cnf |= gpio::PIN_CNF_PULLDOWN;
  }
  return cnf;
}

/** Allocate and initialize the TWI device.
 *
 * This is called at most once, on the first invocation of
 * board::twi(), to initialize the static reference. */
periph::TWI& configured_twi ()
{
  auto& twi = periph::TWI::instance(0);

  /* SX1509B requires support for clock stretching.  200 us is not
   * long enough.  Run with 1000 us until that proves inadequate
   * too. */
  int rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL_INT,
                             NRFCXX_BOARD_PSEL_TWI0_SDA_INT,
                             (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                             1000);
  if (0 > rc) {
    failsafe(FailSafeCode::BOARD_INIT_FAILURE);
  }

  /* Light up the power management switch.  Without this SX1509B I2C
   * interaction is fragile.  Reference implementation sleeps 5ms
   * after enabling but turn-on delay and V_OUT rise time for the
   * TCK106AF total less than 250 us.  Maybe they're waiting for the
   * SX1509B to power up, which is 2.5 ms.  We'll go with 3 ms which
   * covers both sequentially. */
  nrf5::GPIO->OUTSET = (1U << NRFCXX_BOARD_PSEL_VDD_PWR_CTRL);
  nrf5::GPIO->PIN_CNF[NRFCXX_BOARD_PSEL_VDD_PWR_CTRL] = gpio::PIN_CNF_WRONLY;
  sleep_ms(3);

  return twi;
}

gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;

constexpr uint16_t iox_leds = 0
  | (1U << NRFCXX_BOARD_IOX_LIGHTWELL_G)
  | (1U << NRFCXX_BOARD_IOX_LIGHTWELL_B)
  | (1U << NRFCXX_BOARD_IOX_LIGHTWELL_R)
  | (1U << NRFCXX_BOARD_IOX_SENSE_LED_G)
  | (1U << NRFCXX_BOARD_IOX_SENSE_LED_B)
  | (1U << NRFCXX_BOARD_IOX_SENSE_LED_R)
  ;

/* Customize the voltage divider to turn BAT_MON_EN on through the
 * SX1509B prior to sampling. */
class vbatt_divider : public sensor::adc::voltage_divider
{
  using super = sensor::adc::voltage_divider;

public:
  using super::super;

private:
  int sample_setup () override
  {
    board::iox().configure(NRFCXX_BOARD_IOX_BAT_MON_EN, gpio::PIN_CNF_WRONLY, 1);
    return clock::uptime::from_ms(1);
  }

  void sample_teardown () override
  {
    board::iox().configure(NRFCXX_BOARD_IOX_BAT_MON_EN, gpio::PIN_CNF_PWRUP);
  }
};

/** ADC client instance that samples the voltage divider from the
 * Thingy:52 LIPO battery. */
vbatt_divider meas_vbatt{NRFCXX_BOARD_BATTERY_R1,
    NRFCXX_BOARD_BATTERY_R2,
    NRFCXX_BOARD_BATTERY_AIN};

} // ns anonymous;

namespace board {

int
initialize (bool enable_hfxt)
{
  // Set the nRF52 GPIOs to their default configuration
  gpio_startup();

  instr_aux.enable();
  instr_aux.assert();

  // Standard board initialization for clocks.
  clock::initialize(enable_hfxt);

  instr_aux.deassert();

  /* Set up the IO extender infrastructure, including default pin
   * configurations.
   *
   * Note that retrieving the sx1509b instance causes the TWI instance
   * to be created, which powers up the TCK106AF. */
  auto& iox = board::iox();
  int rc = iox.hw_reset();
  if (0 <= rc) {
    sleep_ms(rc);
    rc = iox.multiconfigure(iox_out_set, iox_out_clear, iox_in_pulldown);
  }
  if (0 > rc) {
    // Not that this will do any good...
    failsafe(FailSafeCode::BOARD_INIT_FAILURE);
  }

  instr_aux.assert();
  instr_aux.deassert();

  return 0;
}

int
enable_led_driver (unsigned int iox_mask,
                   bool linear)
{
  using misc::sx1509b;
  auto& iox = board::iox();
  if (!iox_mask) {
    iox_mask = -1;
  }
  iox_mask &= iox_leds;
  int rc = iox.configure_as_leds(iox_mask, true);
  if (0 <= rc) {
    auto cv = iox.clock_cache();
    cv &= ~sx1509b::RegClock_FREQ_Msk;
    cv |= (sx1509b::RegClock_FREQ_2MHz << sx1509b::RegClock_FREQ_Pos);
    rc = iox.clock(cv);
  }
  if (0 <= rc) {
    constexpr uint8_t divexp = 3; // Divide 2 MHz by 8
    constexpr auto led_msk = 0
      | sx1509b::RegMisc_LEDFREQ_Msk
      | sx1509b::RegMisc_LEDA_Msk
      | sx1509b::RegMisc_LEDB_Msk
      ;
    auto mv = iox.misc_cache() & ~led_msk;
    if (!linear) {
      mv |= 0
        | (sx1509b::RegMisc_LEDA_Logarithmic << sx1509b::RegMisc_LEDA_Pos)
        | (sx1509b::RegMisc_LEDB_Logarithmic << sx1509b::RegMisc_LEDB_Pos)
        ;
    }
    mv |= ((1 + divexp) << sx1509b::RegMisc_LEDFREQ_Pos);
    rc = iox.misc(mv);
  }
  if (0 <= rc) {
    rc = iox_mask;
  }
  return rc;
}

int
disable_led_driver ()
{
  using misc::sx1509b;
  auto& iox = board::iox();

  /* Reference implementation default config for LED pins includes
   * enabling the input buffer.  Preserve that. */
  constexpr unsigned int pin_cnf = gpio::PIN_CNF_WRONLY  & ~GPIO_PIN_CNF_INPUT_Msk;
  unsigned int psel = 0;
  auto mask = iox_leds;
  int rc = 0;
  while (mask && (0 <= rc)) {
    uint16_t bit = (1U << psel);
    if (bit & mask) {
      mask &= ~bit;
      rc = iox.configure(psel, pin_cnf, 1);
    }
    ++psel;
  }
  if (0 <= rc) {
    auto mv = iox.misc_cache();
    mv &= ~sx1509b::RegMisc_LEDFREQ_Msk;
    rc = iox.misc(mv);
  }
  if (0 <= rc) {
    auto cv = iox.clock_cache();
    cv &= ~sx1509b::RegClock_FREQ_Msk;
    cv |= (sx1509b::RegClock_FREQ_Off << sx1509b::RegClock_FREQ_Pos);
    rc = iox.clock(cv);
  }
  return rc;
}

periph::TWI&
twi() noexcept
{
  static auto& instance = configured_twi();
  return instance;
}

misc::sx1509b&
iox () noexcept
{
  static misc::sx1509b::iface_config_type ifc{twi(), NRFCXX_BOARD_PSEL_IOX_RESETn};
  static misc::sx1509b instance{ifc};
  return instance;
}

void
gpio_startup () noexcept
{
  nrf5::GPIO->OUTCLR = -1;
  for (auto psel = 0U; psel < nrf5::GPIO_PSEL_COUNT; ++psel) {
    nrf5::GPIO->PIN_CNF[psel] = gpio_default_pin_cnf(psel);
  }
}

void
gpio_shutdown () noexcept
{
  nrf5::GPIO->OUTCLR = -1;
  for (unsigned int psel = nrf5::GPIO_PSEL_COUNT - 1; psel < nrf5::GPIO_PSEL_COUNT; --psel) {
    nrf5::GPIO->PIN_CNF[psel] = gpio_default_pin_cnf(psel);
  }
}

unsigned int
iox_pin::configuration () const
{
  return iox().configuration(psel);
}

void
iox_pin::configure (unsigned int pin_cnf)
{
  (void)iox().configure(psel, pin_cnf);
}

bool
iox_pin::read () const
{
  return !(iox().gpio_cache().data & bit);
}

power_monitor::power_monitor (notifier_type notify) :
  super{notify,
    gpio::gpio_pin{NRFCXX_BOARD_PSEL_USB_DETECT},
    gpio::gpio_pin{NRFCXX_BOARD_PSEL_BAT_CHG_STAT},
    meas_vbatt}
{
  /*
   * On Thingy:52 R_ISET is a parallel 27 kOhm and 15 kOhm resistor,
   * equivalent to 9.708 kOhm.  The corresponding I_CHG is therefore
   * 735 mA.
   *
   * The XC6804A4E1 performs trickle charging at 73.5 mA until V_BATT
   * reaches 2.9V within 2 h then switches to main charging at 735 mA
   * until V_BATT reaches 4.2 V or charge current drops below 73.5 mA
   * within 10 h.
   *
   * Charging stops when the charge current drops below 73.5 mA.
   * Battery charging resumes when V_BATT <= 3.9 V and stops when
   * V_BATT reaches 4.2 V (or when battery or V_BUS are
   * removed/restored), but if the charge current is too low it will
   * stop. */
  meas_vbatt.resolution(SAADC_RESOLUTION_VAL_14bit);
}

unsigned int
battery_level_pptt (unsigned int batt_mV)
{
  /* "Curve" here eyeballed from captured e58eaf327317 data between
   * 2018-10-28T07:20-0500 and 2018-10-29T02:30-0500.  This sensor
   * started with a charge of 3.96 V and dropped about linearly to 3.58
   * V over 15 hours.  It then dropped rapidly to 3.10 V over one hour,
   * at which point it stopped transmitting.
   *
   * Based on eyeball comparisons we'll say that 15/16 of life goes
   * between 3.95 and 3.55 V, and 1/16 goes between 3.55 V and 3.1 V. */
  const nrfcxx::sensor::battery_level_point_type discharge_curve[] = {
    {10000, 3950},
    {625, 3550},
    {0, 3100},
  };

  return sensor::battery_level_pptt(batt_mV, discharge_curve);
}

int
led_setup_battery_display (unsigned int batt_mV)
{
  using misc::sx1509b;
  auto& iox = board::iox();

  auto lvl_pptt = battery_level_pptt(batt_mV);
  sx1509b::led_pwm_type clr;
  sx1509b::led_pwm_type cfg;
  cfg.ton = 15;
  bool show_green = (5000 <= lvl_pptt);
  // 8 is the lowest intensity capable of illuminating the LEDs.
  if (show_green) {
    cfg.ion = 8 + (4999 + 247 * (lvl_pptt - 5000)) / 5000;
  } else {
    cfg.ion = 8 + (4999 + 247 * lvl_pptt) / 5000;
  }
  int rc;
  uint16_t led_bit;
  if (show_green) {
    led_bit = (1U << NRFCXX_BOARD_IOX_LIGHTWELL_G);
    rc = iox.led_configure(NRFCXX_BOARD_IOX_LIGHTWELL_G, cfg);
    if (0 <= rc) {
      rc = iox.led_configure(NRFCXX_BOARD_IOX_LIGHTWELL_R, clr);
    }
  } else {
    led_bit = (1U << NRFCXX_BOARD_IOX_LIGHTWELL_R);
    rc = iox.led_configure(NRFCXX_BOARD_IOX_LIGHTWELL_R, cfg);
    if (0 <= rc) {
      rc = iox.led_configure(NRFCXX_BOARD_IOX_LIGHTWELL_G, clr);
    }
  }
  if (0 <= rc) {
    rc = led_bit;
  }
  return rc;
}

} // board

namespace led {
namespace {

class iox_led : public led::led_type
{
public:
  const uint16_t bit;

  iox_led (unsigned int psel) :
    bit(1U << psel)
  { }

private:
  bool enabled_ = false;

  bool exists () const override
  {
    return true;
  }

  void enable_ () override
  {
    enabled_ = true;
  }

  void disable_ () override
  {
    if (enabled_) {
      off();
    }
    enabled_ = false;
  }

  void on () override
  {
    if (enabled_) {
      board::iox().output_sct(0, bit);
    }
  }

  void off () override
  {
    if (enabled_) {
      board::iox().output_sct(bit, 0);
    }
  }

  void toggle () override
  {
    if (enabled_) {
      board::iox().output_sct(bit, bit);
    }
  }

  bool is_on () const override
  {
    return bit & board::iox().gpio_cache().data;
  }
};

iox_led led0{NRFCXX_BOARD_IOX_LIGHTWELL_R};
iox_led led1{NRFCXX_BOARD_IOX_LIGHTWELL_G};
iox_led led2{NRFCXX_BOARD_IOX_LIGHTWELL_B};
iox_led led3{NRFCXX_BOARD_IOX_SENSE_LED_R};
iox_led led4{NRFCXX_BOARD_IOX_SENSE_LED_G};
iox_led led5{NRFCXX_BOARD_IOX_SENSE_LED_B};

static std::array<led_type*, 6> leds = {&led0, &led1, &led2, &led3, &led4, &led5};

} // ns anonymous

led_type** const led_type::ledps_ = &leds[0];
uint8_t const led_type::nleds_ = leds.max_size();

} // namespace led
} // namespace nrfcxx
