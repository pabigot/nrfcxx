// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrate basic interaction with the ams CCS811 Indoor Air
 * Quality sensor. */

#include <cstdio>
#include <cstring>

#include <pabigot/byteorder.hpp>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/core.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/led.hpp>
#include <nrfcxx/periph.hpp>
#include <nrfcxx/sensor/ccs811.hpp>
#include <nrfcxx/utility.hpp>

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
#include <nrfcxx/board/thingy52.hpp>
#endif // NRFCXX_BOARD_IS_THINGY52

extern "C" {
void GPIOTE_IRQHandler (void)
{
  nrfcxx::periph::GPIOTE::irq_handler();
}
} // extern "C"

#if ((NRFCXX_BOARD_IS_BLE400 - 0)               \
     || (NRFCXX_BOARD_IS_PCA10028 - 0)          \
     || (NRFCXX_BOARD_IS_PCA10040 - 0))
#define WAKEn_PSEL NRFCXX_BOARD_PSEL_SCOPE3
#define RESETn_PSEL NRFCXX_BOARD_PSEL_SCOPE4
#ifdef NRFCXX_BOARD_PSEL_TWI0_SMBA
#define INTn_PSEL NRFCXX_BOARD_PSEL_TWI0_SMBA
#else // NRFCXX_BOARD_PSEL_TWI0_SMBA
#define INTn_PSEL NRFCXX_BOARD_PSEL_SCOPE2
#endif // NRFCXX_BOARD_PSEL_TWI0_SMBA
#elif (NRFCXX_BOARD_IS_THINGY52 - 0)
#define WAKEn_PSEL NRFCXX_BOARD_IOX_CCS_WAKEn
#define RESETn_PSEL NRFCXX_BOARD_IOX_CCS_RESETn
#define INTn_PSEL NRFCXX_BOARD_PSEL_CCS_INT
#else //
// Need PSEL assignment for WAKEn and RESETn
// Without them this will failsafe
#define WAKEn_PSEL NRFCXX_BOARD_PSEL_SCOPEn
#define RESETn_PSEL NRFCXX_BOARD_PSEL_SCOPEn
#define INTn_PSEL NRFCXX_BOARD_PSEL_SCOPEn
#endif

namespace {

__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

void
app_state_handler (const nrfcxx::systemState::state_type& ss,
                   bool is_reset,
                   bool retained)
{
  nrfcxx::sensor::ccs811::state_setup(ss, is_reset, retained);
}

nrfcxx::systemState cs{core_state, 2018101314U, app_state_handler};

} // ns anonymous

int
main (void)
{
  using namespace nrfcxx;
  using clock::uptime;

  int rc = board::initialize();

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Display information about the cause of the restart. */
  {
    nrfcxx::clock::uptime::text_type buf;
    printf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           nrfcxx::clock::uptime::as_text(buf, cs.state().last_uptime));
    printf("total uptime %s\n",
           nrfcxx::clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    puts("***UNCONTROLLED RESET\n");
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    printf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
    delay_us(10000);
    systemState::systemOff(0, -1);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    if (barked && (cs.state().magic == cs.state().code)) {
      puts("- to put previous watchdog to sleep");
    } else {
      printf("- due to program code %u %s watchdog\n",
             cs.state().code, barked ? "with" : "without");
    }
  } else if (barked) {
    printf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    printf("- due to direct system reset\n");
  }

  printf("CCS811 on I2C at SCL %d SDA %d ; WAKEn %d ; RESETn %d ; INTn %d\n",
         NRFCXX_BOARD_PSEL_TWI0_SCL,
         NRFCXX_BOARD_PSEL_TWI0_SDA,
         WAKEn_PSEL, RESETn_PSEL, INTn_PSEL);

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
  auto& twi = board::twi();
#else // NRFCXX_BOARD_IS
  auto& twi = periph::TWI::instance(0);

  rc = twi.bus_configure(NRFCXX_BOARD_PSEL_TWI0_SCL,
                         NRFCXX_BOARD_PSEL_TWI0_SDA,
                         (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
                         500);
  printf("bus configure: %d\n", rc);
#endif // NRFCXX_BOARD_IS

  event_set events;
  #define EVT_CCS811 0x001

  /* Whether the secondary address should be used.  This is true for
   * Sparkfun Combo, false for Thingy:52. */
  bool sec_addr = true;

#if (NRFCXX_BOARD_IS_THINGY52 - 0)
  /* Clear RESETn and WAKEn, configure them as outputs (defaults to
   * input), apply power, and wait for CCS811 cold startup. */
  board::iox_pin resetn{RESETn_PSEL};
  resetn.set();
  resetn.configure(gpio::PIN_CNF_WRONLY);
  board::iox_pin waken{WAKEn_PSEL};
  waken.set();
  waken.configure(gpio::PIN_CNF_WRONLY);
  board::iox_pin ccs_pwr_ctrl{NRFCXX_BOARD_IOX_CCS_PWR_CTRL};
  ccs_pwr_ctrl.set();
  sleep_ms(sensor::ccs811::COLD_START_DELAY_utt);
  sec_addr = false;
#else // NRFCXX_BOARD_IS
  gpio::gpio_pin waken{WAKEn_PSEL};
  gpio::gpio_pin resetn{RESETn_PSEL};
#endif// NRFCXX_BOARD_IS

  sensor::ccs811::iface_config_type ifc{
    .twi = twi,
    .waken = waken,
    .resetn = resetn,
    .intn_psel = INTn_PSEL,
  };
  sensor::ccs811 ccs811{events.make_setter(EVT_CCS811), ifc, sec_addr};
  ccs811.drive_mode(ccs811.DM_10_s);

  NVIC_EnableIRQ(GPIOTE_IRQn);
  periph::GPIOTE::enable_sense();

  const auto& rst = ccs811.retained_state();
  printf("Ccs811 last reset %u\n", static_cast<unsigned int>(rst.reset_utt));

  rc = ccs811.lpsm_start();
  printf("LPM start got %d\n", rc);
  // const auto& machine = sensor.machine();
  sensor::ccs811::observation_beacon_type bcn;
  sensor::ccs811::threshold_s threshold;
  printf("Thresholds: eCO2 %u ppm ; eTVOC %u ppb ; %u Cel ; %u %%RH\n",
         threshold.eCO2_ppm, threshold.eTVOC_ppb, threshold.temperature_Cel,
         threshold.humidity_pph);
  while (true) {
    nrfcxx::event_set::cev();
    auto pending = events.copy_and_clear();
    if (pending.test_and_clear(EVT_CCS811)) {
      using lpm::state_machine;

      if (auto pf = ccs811.lpsm_process()) {
        const auto& obs = ccs811.observations();
        if (ccs811.PF_REPORTING & pf) {
          puts("*** CCS811 REPORTING");
        }
        if (ccs811.PF_CONDITIONED & pf) {
          puts("*** CCS811 CONDITIONED");
        }
        if (state_machine::PF_OBSERVATION & pf) {
          uptime::text_type sysut;
          uptime::text_type snsut;
          const char* oc = "";
          if (obs.is_ready()) {
            const auto& nbcn = ccs811.observation_beacon();
            rc = threshold.observation_beacon_changed(bcn, nbcn);
            if (rc) {
              printf("Beacon changed significantly: %d\n", rc);
              printf(" %08lx %u %u vs %08lx %u %u\n",
                     bcn.env_data, bcn.eCO2, bcn.eTVOC,
                     nbcn.env_data, nbcn.eCO2, nbcn.eTVOC);
              oc = " CHGD";
              bcn = nbcn;
            }
          }
          printf("%s ; %5u eCO2 ; %5u eTVOC ; st %04x ; B %04x ; %s%s\n",
                 uptime::as_text(sysut, uptime::now()),
                 obs.eCO2, obs.eTVOC, obs.status,
                 obs.baseline,
                 uptime::as_text(snsut, systemState::total_now() - ccs811.retained_state().reset_utt),
                 oc);
        }
        if (state_machine::PF_STARTED & pf) {
          auto& ver = ccs811.id_version();
          printf("CCS811 started: HW %02x, boot %04x, app %04x\n",
                 ver.hw_version, ver.fw_boot_version, ver.fw_app_version);
        }
        if (state_machine::PF_RESET & pf) {
          puts("*** CCS811 RESET");
        }
        if (ccs811.PF_CCS811_ERROR & pf) {
          printf("*** CCS811 ERROR %x\n", obs.status);
        }
        if (ccs811.PF_BASELINED & pf) {
          puts("*** CCS811 BASELINED");
        }
        if (ccs811.PF_RESTORED & pf) {
          puts("*** CCS811 RESTORED");
        }
        // printf("Process got %x, state %d\n", pf, machine.state());
      } else if (ccs811.machine().has_error()) {
        printf("CCS811 failed permanently: %d\n", ccs811.machine().error());
        break;
      }
    }
    do {
      systemState::WFE();
    } while (!events);
  }

  printf("Exit rc %d\n", rc);

  delay_us(100000);
  return 0;
}
