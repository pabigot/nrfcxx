// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/gpio.hpp>
#include <nrfcxx/misc/lipomon.hpp>

#if 0
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

namespace nrfcxx {
namespace misc {

lipo_monitor::lipo_monitor (notifier_type notify,
                            const gpio::gpio_pin& vchg_detect,
                            const gpio::gpio_pin& chgn_state,
                            sensor::adc::voltage_divider& battmeas) :
  super{notify},
  listener_{[this](auto sp)
      {
        callback_bi_(sp);
      }},
  vchg_detect_{vchg_detect},
  chgn_state_{chgn_state},
  battmeas_{battmeas}
{
}

void
lipo_monitor::callback_bi_ (const periph::GPIOTE::sense_status_type* sp)
{
  int ud = -1;
  int cs = -1;
  while ((0 <= sp)
         && ((0 > ud)
             || (0 > cs))) {
    if (vchg_detect_.implementation().global_psel == sp->psel) {
      ud = sp->counter_state;
    } else if (chgn_state_.implementation().global_psel == sp->psel) {
      cs = sp->counter_state;
    }
    ++sp;
  }
  // failsafe if ud or cs are negative?
  if ((0 > ud) || (0 > cs)) {
    failsafe(FailSafeCode::INTERNAL_ERROR);
  }

  auto ps_cap = PS_OnBattery;
  if (0x01 & ud) {
    // V_BUS present, CHG_STAT low if charging
    if (0x01 & cs) {
      ps_cap = PS_Charged;
    } else {
      ps_cap = PS_Charging;
    }
  }

  auto ps_prc = (flags_bi_ & FL_PWRSRCPRC_Msk) >> FL_PWRSRCPRC_Pos;
  flags_bi_ = (flags_bi_ & ~FL_PWRSRCCAP_Msk) | (ps_cap << FL_PWRSRCCAP_Pos);

  /* If we know something changed state, or if the inferred power
   * state changed, tell the machine. */
  if ((1 < ud) || (1 < cs) || (ps_cap != ps_prc)) {
    machine_.post_event();
  }

}

int
lipo_monitor::power_source () const
{
  mutex_type mutex;
  return (FL_PWRSRCPRC_Msk & flags_bi_) >> FL_PWRSRCPRC_Pos;
}

int
lipo_monitor::calibrate ()
{
  using lpm::state_machine;
  if (state_machine::MS_IDLE != machine_.state()) {
    return -1;
  }
  machine_.set_state(MS_ENTRY_CALIBRATE, true);
  return 0;
}

int
lipo_monitor::lpsm_process_ (int& delay,
                             process_flags_type& pf)
{
  using lpm::state_machine;
  using clock::uptime;

  int rc = 0;

  if (state_machine::MS_ENTRY_START == machine_.state()) {
    /* Do startup preparation first so we can process the power source
     * on startup. */

    /* Enable the battery monitor voltage divider.
     *
     * @todo This should probably only be on when we're going to
     * sample the battery voltage. */

    /* Set the GPIOs for a default sense, enable the listener, then
     * invoke the callback to synchronize the sense and capture the
     * initial state. */
    vchg_detect_.configure(gpio::PIN_CNF_ACTIVE_HIGH_NOPULL);
    chgn_state_.configure(gpio::PIN_CNF_ACTIVE_HIGH_NOPULL);
    flags_bi_ = 0;
    listener_.enable();
    periph::GPIOTE::synchronize_sense();
  }

  /* Record the power source and process any change. */
  uint8_t flags;
  uint8_t ps_cap;
  uint8_t ps_old;
  {
    mutex_type mutex;
    flags = flags_bi_;
    ps_old = (flags & FL_PWRSRCPRC_Msk) >> FL_PWRSRCPRC_Pos;
    ps_cap = (flags & FL_PWRSRCCAP_Msk) >> FL_PWRSRCCAP_Pos;

    /* Move any captured power source into the processed field.  If
     * the power source changed and we're sampling, flag the sample as
     * invalid. */
    if (ps_cap) {
      flags &= ~(FL_PWRSRCCAP_Msk | FL_PWRSRCPRC_Msk);
      flags |= ps_cap << FL_PWRSRCPRC_Pos;
      if ((FL_SAMPLING & flags)
          && (ps_cap != ps_old)) {
        flags |= FL_SAMPLE_CORRUPTED;
      }
      flags_bi_ = flags;
    }
  }

  /* Process the power source setting.
   *
   * @todo The charging state signal goes high as soon as USB
   * connected, but goes low again very quickly to indicate that
   * charging is in progress.  We might want to hold the CHARGED
   * signal until it stabilizes.
   *
   * Also, the XC6804A emits a 1 kHz signal on CSO to indicate
   * charging errors.  That should be detected, emitted as a separate
   * signal, and cause monitoring of BAT_CHG_STAT to be turned off
   * until USB is removed the application enables it again.  This
   * would show up here as ps_cap != PS_Unknown and ps_cap ==
   * ps_old. */
  if (ps_cap && (ps_cap != ps_old)) {
    /* Process flags that depend on power source:
     * * L PF_LIPO
     * * M PF_MAINS
     * * + PF_CHARGING
     * * - PF_CHARGED
     *
     * The entry condition disallows transition to Unknown and the
     * dotted diagonal.
     *
     *        to    Battery  Charging  Charged
     *   from
     * Unknown         L        M+        M-
     * Battery         .        M+        M-
     * Charging        L        .         -
     * Charged         L        +         .
     */
    if (PS_OnBattery == ps_cap) {
      pf |= PF_LIPO;
      // It's tempting to initiate a sample here, but don't.
      // In the first few milliseconds after V_USB drops the
      // battery measuremnt circuit is still inaccurate.
    } else {
      if (ps_old <= PS_OnBattery) {
        pf |= PF_MAINS;
      }
      if (PS_Charging == ps_cap) {
        pf |= PF_CHARGING;
      } else {
        pf |= PF_CHARGED;
      }
    }
  } else {
    ps_cap = ps_old;
  }

  switch (machine_.state()) {
    default:
      machine_.set_lost();
      break;
    case state_machine::MS_ENTRY_STOPPED:
      listener_.disable();
      vchg_detect_.configure(gpio::PIN_CNF_PWRUP);
      chgn_state_.configure(gpio::PIN_CNF_PWRUP);
      battmeas_.sample_teardown();
      machine_.set_state(state_machine::MS_OFF);
      break;
    case state_machine::MS_ENTRY_START:
      // Most was done above.  Do the rest.
      pf |= state_machine::PF_STARTED;
      [[fallthrough]]
    case MS_ENTRY_CALIBRATE:
      flags &= ~FL_CALIBRATED;
      {
        mutex_type mutex;
        flags_bi_ &= ~FL_CALIBRATED;
      }
      cputs("* PWRMON calibrating");
      adc_pending_ = true;
      adc_calibrating_ = 1;
      machine_.set_state(MS_EXIT_CALIBRATE);
      rc = battmeas_.queue([this]{
          adc_calibrating_ = 0;
          machine_.post_event();
        }, [this](auto rc)
        {
          adc_pending_ = false;
          if (0 > rc) {
            adc_calibrating_ = -1;
            machine_.post_event();
          }
        }, true);
      // FALLTHRU
    case MS_EXIT_CALIBRATE:
      {
        adc_mutex_type mutex;
        if (adc_pending_
            || (0 < adc_calibrating_)) {
          /* Calibration still in queue or still running. */
          break;
        }
        rc = adc_calibrating_;
      }
      if (0 > rc) {
        cprintf("*** PWRMON calibration failed: %d\n", rc);
        break;
      }
      machine_.set_state(MS_ENTRY_VBATT);
      pf |= PF_CALIBRATED;
      flags |= FL_CALIBRATED;
      cputs("* PWRMON calibrated");
      {
        mutex_type mutex;
        flags_bi_ |= FL_CALIBRATED;
      }
      // FALLTHRU
    case MS_ENTRY_VBATT:
      if (!(FL_CALIBRATED & flags)) {
        /* Can't sample without calibration.
         *
         * @todo Prove this can't get into a loop. */
        machine_.set_state(MS_ENTRY_CALIBRATE, true);
        break;
      }
      machine_.set_state(MS_VBATT);
      delay = battmeas_.sample_setup();
      if (delay) {
        break;
      }
      // FALLTHRU
    case MS_VBATT:
      adc_pending_ = true;
      adc_sampling_ = 1;

      /* Record that we're sampling and the sample isn't corrupted. */
      {
        mutex_type mutex;
        auto mfl = flags_bi_ & ~FL_SAMPLE_CORRUPTED;
        flags_bi_ = FL_SAMPLING | mfl;
      }
      machine_.set_state(MS_EXIT_VBATT);
      rc = battmeas_.queue([this]{
          adc_sampling_ = 0;
          machine_.post_event();
        }, [this](auto rc)
        {
          adc_pending_ = false;
          if (0 > rc) {
            adc_sampling_ = -1;
            machine_.post_event();
          }
        });
      // FALLTHRU
    case MS_EXIT_VBATT:
      {
        adc_mutex_type mutex;
        if (adc_pending_
            || (0 < adc_sampling_)) {
          /* Sample still in queue or still running. */
          break;
        }
        rc = adc_calibrating_;
      }
      battmeas_.sample_teardown();
      if (0 > rc) {
        cprintf("*** PWRMON sample failed: %d\n", rc);
        return rc;
      }
      {
        mutex_type mutex;
        if (!(FL_SAMPLE_CORRUPTED & flags_bi_)) {
          pf |= state_machine::PF_OBSERVATION;
        }
        flags_bi_ &= ~(FL_SAMPLE_CORRUPTED | FL_SAMPLING);
      }
      if (state_machine::PF_OBSERVATION & pf) {
        /* Sample succeeded.  Store the voltage, negative if it isn't
         * clearly pure. */
        batt_mV_ = battmeas_.input_mV();
        if (PS_OnBattery < ps_cap) {
          batt_mV_ = -batt_mV_;
        }
      }
      machine_.set_state(state_machine::MS_IDLE);
      [[fallthrough]]
    case state_machine::MS_IDLE:
      break;
  }
  return rc;
}

} // ns misc
} // ns nrfcxx
