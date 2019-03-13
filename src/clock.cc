// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <cstdio>
#include <cstdlib>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/periph.hpp>

#define INSTR_PSEL_TASK_LFCLK NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_EVENT_LFCLK NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_TASK_HFCLK NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_EVENT_HFCLK  NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_TASK_CAL NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_CONSTLAT NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_ALARM_PROCESS_READY NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_ALARM_PROCESS NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_ALARM_CCSET NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_SLEEP NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_ASLEEP NRFCXX_BOARD_PSEL_SCOPEn
#define INSTR_PSEL_AUX NRFCXX_BOARD_PSEL_SCOPEn

/** PAN65: HFCLK: A HFCLKSTOP task followed shortly by an HFCLKSTART
 * task will disable HFCLK for up to 5 clock cycles.
 *
 * Workaround requires waiting for HFCLKSTAT.SRC to be RC before
 * restarting HFXT.
 *
 * Implemented in reevaluate_hfxt_ni().
 *
 * @note This PAN was fixed in Rev 3 ICs. */
#ifndef NRFCXX_ENABLE_NRF51_PAN_65
#define NRFCXX_ENABLE_NRF51_PAN_65 0
#endif // NRFCXX_ENABLE_NRF51_PAN_65

/** PAN14: LFCLK: Calibration does not request HFCLK.
 *
 * Workaround requires use of constant latency mode before issuing
 * the calibration command.
 *
 * Implemented in POWER_CLOCK_IRQHandler.
 *
 * @note This PAN is documented present in Rev 2 ICs, but is not
 * listed as present nor as fixed in Rev 3 ICs. */
#ifndef NRFCXX_ENABLE_NRF51_PAN_14
#define NRFCXX_ENABLE_NRF51_PAN_14 (NRF51 - 0)
#endif // NRFCXX_ENABLE_NRF51_PAN_14

#define UPTIME_ALARM_CCIDX 3
#define UPTIME_SLEEP_CCIDX 2

namespace nrfcxx {
namespace clock {

namespace {

gpio::instr_psel<INSTR_PSEL_TASK_HFCLK> instr_task_hfclk;
gpio::instr_psel<INSTR_PSEL_EVENT_HFCLK> instr_event_hfclk;
gpio::instr_psel<INSTR_PSEL_TASK_LFCLK> instr_task_lfclk;
gpio::instr_psel<INSTR_PSEL_EVENT_LFCLK> instr_event_lfclk;
gpio::instr_psel<INSTR_PSEL_TASK_CAL> instr_task_cal;
gpio::instr_psel<INSTR_PSEL_CONSTLAT> instr_constlat;
gpio::instr_psel<INSTR_PSEL_ALARM_PROCESS_READY> instr_alarm_process_ready;
gpio::instr_psel<INSTR_PSEL_ALARM_PROCESS> instr_alarm_process;
gpio::instr_psel<INSTR_PSEL_ALARM_CCSET> instr_alarm_ccset;
gpio::instr_psel<INSTR_PSEL_SLEEP> instr_sleep;
gpio::instr_psel<INSTR_PSEL_ASLEEP> instr_asleep;
gpio::instr_psel<INSTR_PSEL_AUX> instr_aux;

#define HFXT_REQ_APP 0x01
#define HFXT_REQ_LFCLK 0x02
#define HFXT_REQ_CAL_PENDING 0x04
#define HFXT_REQ_CAL_ACTIVE 0x08
volatile uint8_t hfxt_req;

#define CONSTLAT_REQ_APP 0x01
#define CONSTLAT_REQ_CAL 0x02
#define CONSTLAT_REQ_TIMER0 0x04
#define CONSTLAT_REQ_TIMER1 (CONSTLAT_REQ_TIMER0 << 1)  /* 0x08 */
#define CONSTLAT_REQ_TIMER2 (CONSTLAT_REQ_TIMER0 << 2)  /* 0x10 */
#define CONSTLAT_REQ_TIMER3 (CONSTLAT_REQ_TIMER0 << 3)  /* 0x20 */
#define CONSTLAT_REQ_TIMER4 (CONSTLAT_REQ_TIMER0 << 4) /* 0x40 */
// 0x80 remains unassigned
volatile uint8_t constlat_req;

#define PENDING_LFCLK 0x01
#define PENDING_HFCLK 0x02
#define PENDING_CAL 0x03
volatile uint8_t pending;

bool lfxt_req; // = false
volatile uint8_t lfclk_src; // = CLOCK_LFCLKSRC_SRC_RC
volatile uint8_t lfrc_cal_interval_qs;

/* RTC COMPARE is guaranteed to trigger if the stored value is at
 * least 2 greater than the COUNTER.  We add 1 to cover a situation
 * where the COUNTER increments after we captured the value we're
 * using to determine deltas. */
static constexpr int rtc_safe_delta = 3;

volatile bool sleep_wakeup;

volatile uint64_t uptime_now_base;

/** Process pending uptime overflow events
 *
 * This may be invoked from the underlying RTC FLIH or from any call
 * to uptime::now().  The latter might be invoked from a FLIH of
 * higher priority than the RTC flih, so this has to be executed with
 * all interrupts disabled, not just the FLIH IRQ */
inline __attribute__((__gnu_inline__,__always_inline__))
bool
process_uptime_overflow_ni ()
{
  using periph::RTC;
  bool rv = nrf5::UPTIME_RTC->EVENTS_OVRFLW;
  if (rv) {
    uptime_now_base += RTC::counter_modulus;
    nrf5::UPTIME_RTC->EVENTS_OVRFLW = 0;
  }
  return rv;
}

/** Safe read of the 64-bit uptime counter taking into account
 * asynchronous overflow events. */
inline __attribute__((__gnu_inline__,__always_inline__))
uint64_t
now_ni ()
{
  unsigned int ctr0;
  unsigned int ctr1 = nrf5::UPTIME_RTC->COUNTER;
  do {
    ctr0 = ctr1;
    process_uptime_overflow_ni();
    ctr1 = nrf5::UPTIME_RTC->COUNTER;
  } while (ctr0 != ctr1);
  /* We know ctr0==ctr1 and that no overflow occurred during that
   * tick, so the combination results in a valid 64-bit counter. */
  return uptime_now_base | ctr0;
}

/* Configure the module to either CONSTLAT or LOWPWR mode, based on
 * whether something is requesting constant latency.
 *
 * @note This must be invoked with CLOCK.IRQn blocked. */
inline
void
reevaluate_constlat ()
{
  if (constlat_req) {
    instr_constlat.assert();
    NRF_POWER->TASKS_CONSTLAT = 1;
  } else {
    instr_constlat.deassert();
    NRF_POWER->TASKS_LOWPWR = 1;
  }
}

inline void
reevaluate_hfxt_ni ()
{
  static bool last_active;
  if (hfxt_req) {
    /* Don't bother restarting if crystal already running */
    if (!last_active) {
#if (NRFCXX_ENABLE_NRF51_PAN_65 - 0)
      /* Don't re-enable until previous disable has completed */
      while (hfclk::hfxt_active()) {
      }
#endif /* NRFCXX_ENABLE_NRF51_PAN_65 */
      systemState::updateOperationalMode(0, systemState::OM_HFCLK);
      nrf5::CLOCK->TASKS_HFCLKSTART = 1;
      instr_task_hfclk.assert();
      last_active = true;
    }
  } else {
    /* Redundant disable does not cause problems like redundant enable
     * does: there's no event and no product anomaly. */
    instr_task_hfclk.deassert();
    nrf5::CLOCK->TASKS_HFCLKSTOP = 1;
    systemState::updateOperationalMode(systemState::OM_HFCLK, 0);
    last_active = false;
  }
}

} // anonymous namespace

int
initialize (bool enable_hfxt,
            int lfclk_src_,
            unsigned int lfrc_cal_interval_qs_)
{
  static bool initialized;
  if (initialized) {
    return -1;
  }
  instr_task_hfclk.enable();
  instr_event_hfclk.enable();
  instr_task_lfclk.enable();
  instr_event_lfclk.enable();
  instr_task_cal.enable();
  instr_constlat.enable();
  instr_alarm_process_ready.enable();
  instr_alarm_process.enable();
  instr_alarm_ccset.enable();
  instr_sleep.enable();
  instr_asleep.enable();
  instr_aux.enable();

  lfrc_cal_interval_qs = lfclk::DefaultRCOCalInterval_qs;
  nrf5::CLOCK->INTENCLR = -1;
  nvic_ClearPendingIRQ(nrf5::CLOCK.IRQn);
  nvic_SetPriority(nrf5::CLOCK.IRQn, IRQ_PRIORITY_APP_HIGH);
  nvic_EnableIRQ(nrf5::CLOCK.IRQn);
  nrf5::CLOCK->INTENSET = ((CLOCK_INTENSET_HFCLKSTARTED_Set << CLOCK_INTENSET_HFCLKSTARTED_Pos)
                         | (CLOCK_INTENSET_LFCLKSTARTED_Set << CLOCK_INTENSET_LFCLKSTARTED_Pos)
                         | (CLOCK_INTENSET_DONE_Set << CLOCK_INTENSET_DONE_Pos)
                         | (CLOCK_INTENSET_CTTO_Set << CLOCK_INTENSET_CTTO_Pos));
  /* Configure and start LFCLK before HFXT to work around inability to
   * set LFCLKSRC while HFXT is being started. */
  lfclk::source_configure(lfclk_src_, lfrc_cal_interval_qs_);
  lfclk::configure(true);
  hfclk::hfxt_configure(enable_hfxt);

  while (!lfclk::active()) {
    __WFE();
  }
  nrf5::UPTIME_RTC->INTENCLR = -1;
  nrf5::UPTIME_RTC->EVTENCLR = -1;
  nrf5::UPTIME_RTC->PRESCALER = 0;

  nvic_ClearPendingIRQ(nrf5::UPTIME_RTC.IRQn);
  nvic_SetPriority(nrf5::UPTIME_RTC.IRQn, IRQ_PRIORITY_APP_HIGH);
  nvic_EnableIRQ(nrf5::UPTIME_RTC.IRQn);

  nrf5::UPTIME_RTC->INTENSET = (RTC_INTENSET_OVRFLW_Set << RTC_INTENSET_OVRFLW_Pos);
  nrf5::UPTIME_RTC->TASKS_CLEAR = 1;
  nrf5::UPTIME_RTC->TASKS_START = 1;

  initialized = true;
  return 0;
}

bool
configure_pcirq (int on)
{
  bool rv = NVIC->ISER[0] & (1U << nrf5::CLOCK.IRQn);
  if (0 <= on) {
    if (on && !rv) {
      nrfcxx::nvic_EnableIRQ(nrf5::CLOCK.IRQn);
    } else if (rv && !on) {
      nrfcxx::nvic_DisableIRQ(nrf5::CLOCK.IRQn);
    }
  }
  return rv;
}

void
hfclk::constlat_request (const periph::TIMER& timer,
                         bool enable)
{
  const uint8_t bit = CONSTLAT_REQ_TIMER0 << timer.peripheral().INSTANCE;
  nvic_BlockIRQ block(nrf5::CLOCK.IRQn);
  if (enable) {
    constlat_req |= bit;
  } else {
    constlat_req &= ~bit;
  }
  reevaluate_constlat();
}

bool
hfclk::hfxt_configure (int on)
{
  bool rv = hfxt_req & HFXT_REQ_APP;
  if (0 <= on) {
    while (rv && (!hfxt_active())) {
    }
    nvic_BlockIRQ block(nrf5::CLOCK.IRQn);
    if (0 < on) {
      hfxt_req |= HFXT_REQ_APP;
    } else {
      hfxt_req &= ~HFXT_REQ_APP;
    }
    reevaluate_hfxt_ni();
  }
  return rv;
}

int
lfclk::configure (int on)
{
  int rv = lfxt_req;
  if (0 <= on) {
    static bool last_active;
    bool onp = (0 < on);
    /* Only reconfigure if there's a change from the last state. */
    if (last_active != onp) {
      nvic_BlockIRQ block(nrf5::CLOCK.IRQn);
      bool reevaluate_hfxt = false;
      if (onp) {
        instr_event_lfclk.assert();
        nrf5::CLOCK->LFCLKSRC = ((uint32_t)lfclk_src << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk;
        instr_event_lfclk.deassert();
        if (CLOCK_LFCLKSRC_SRC_Synth == lfclk_src) {
          hfxt_req |= HFXT_REQ_LFCLK;
          reevaluate_hfxt = true;
        }
        nrf5::CLOCK->TASKS_LFCLKSTART = 1;
        instr_task_lfclk.assert();
        last_active = true;
        lfxt_req = onp;
      } else if ((HFXT_REQ_CAL_PENDING | HFXT_REQ_CAL_ACTIVE) & hfxt_req) {
        /* Cannot disable the clock while it's calibrating. */
        rv = -1;
      } else {
        /* Turning off the LFCLK takes about 232 us before the
         * fact is registered by active(). */
        instr_task_lfclk.deassert();
        nrf5::CLOCK->TASKS_LFCLKSTOP = 1;
        nrf5::CLOCK->TASKS_CTSTOP = 1;
        if (HFXT_REQ_LFCLK & hfxt_req) {
          hfxt_req &= ~HFXT_REQ_LFCLK;
          reevaluate_hfxt = true;
        }
        lfxt_req = onp;
        last_active = false;
      }
      if (reevaluate_hfxt) {
        reevaluate_hfxt_ni();
      }
    }
  }
  return rv;
}

int
lfclk::source_configure (int src,
                         unsigned cal_interval_qs)
{
  int rv = lfclk_src;
  if (0 <= src) {
    if (CLOCK_LFCLKSRC_SRC_Synth < (unsigned int)src) {
      rv = -1;
    } else {
      lfclk_src = src;
      if (CLOCK_LFCLKSRC_SRC_RC == src) {
        lfrc_cal_interval_qs = cal_interval_qs;
      }
    }
  }
  return rv;
}

uint64_t
uptime::now ()
{
  primask mutex;
  return now_ni();
}

void
uptime::wakeup ()
{
  sleep_wakeup = true;
}

int
uptime::sleep (int dur)
{
  unsigned int deadline = now() + dur;
  instr_sleep.assert();
  while ((0 < dur) && (!sleep_wakeup)) {
    /* If the remainder is long enough to use the timer, do so. */
    if (rtc_safe_delta <= dur) {
      nrf5::UPTIME_RTC->EVENTS_COMPARE[UPTIME_SLEEP_CCIDX] = 0;
      nrf5::UPTIME_RTC->INTENSET = (RTC_INTENSET_COMPARE0_Set << (UPTIME_SLEEP_CCIDX + RTC_INTENSET_COMPARE0_Pos));
      if (periph::RTC::counter_modulus <= static_cast<unsigned int>(dur)) {
        /* The deadline is more than one ounter cycle in the future.
         * Wake up after that cycle.  (Subtract 1 because the SRM
         * suggests the event might fire if CC is set to the current
         * counter value). */
        nrf5::UPTIME_RTC->CC[UPTIME_SLEEP_CCIDX] = nrf5::UPTIME_RTC->COUNTER - 1;
      } else {
        /* Deadline will occur within this cycle: wait until it's
         * due. */
        nrf5::UPTIME_RTC->CC[UPTIME_SLEEP_CCIDX] = nrf5::UPTIME_RTC->COUNTER + dur;
      }
      instr_asleep.assert();
      __WFI();
      instr_asleep.deassert();
    }
    dur = deadline - static_cast<unsigned int>(now());
  }
  nrf5::UPTIME_RTC->INTENCLR = (RTC_INTENCLR_COMPARE0_Clear << (UPTIME_SLEEP_CCIDX + RTC_INTENCLR_COMPARE0_Pos));
  sleep_wakeup = false;
  instr_sleep.deassert();
  return dur;
}

/** Helper class used to manage the alarm queue. */
struct alarm::alarm_queue
{
  /** The head of the alarm queue.  If null the queue is empty. */
  alarm* scheduled;

  /** The head of the ready queue.  Non-null only when the RTC FLIH is
   * running. */
  alarm* ready;

  /** The capture/compare index on nrf5::UPTIME_RTC that's used to detect
   * when alarms are due. */
  const int ccidx;

  alarm_queue (int ccidx_) :
    scheduled{},
    ready{},
    ccidx{ccidx_}
  { }

  /** Get a pointer to the link at which any values after @p ord
   * should be inserted, where @p ord is the ordinal of a point of
   * interest calculated relative to @p now. */
  inline __attribute__((__gnu_inline__,__always_inline__))
  alarm**
  insertion_point_ni (unsigned int now,
                      int ord)
  {
    auto pos = &scheduled;
    while (*pos && (ord >= (*pos)->ordinal_(now))) {
      pos = &(*pos)->next_;
    }
    return pos;
  }

  inline __attribute__((__gnu_inline__,__always_inline__))
  alarm**
  search_ni (alarm** pos,
             const alarm& alarm)
  {
    while (*pos && (&alarm != *pos)) {
      pos = &(*pos)->next_;
    }
    return pos;
  }

  /** Insert @p alarm into the queue, using @p now for ordinal
   * calculation.
   *
   * @return `true` iff the insertion caused #scheduled to change. */
  bool
  schedule_ni (unsigned int now,
               alarm& alarm)
  {
    auto front = scheduled;
    auto pos = insertion_point_ni(now, alarm.ordinal_(now));
    alarm.state_ = ST_scheduled;
    alarm.next_ = *pos;
    *pos = &alarm;
    return scheduled != front;
  }

  /** Return the next alarm in the ready queue, or a null pointer if
   * there are no ready alarms. */
  alarm*
  next_ready_ni ()
  {
    alarm* rv = ready;
    if (rv) {
      rv->state_ = ST_in_callback;
      ready = rv->next_;
      rv->next_ = nullptr;
    }
    return rv;
  }

  /** Split off the prefix of alarms that are due at or before @p
   * now.  A null pointer is returned if no alarms are due.
   *
   * This must only be invoked when #ready is null. */
  alarm*
  split_ni (unsigned int now)
  {
    auto pos = insertion_point_ni(now, 0);
    alarm* rv = nullptr;
    if (&scheduled != pos) {
      /* Chop the ready prefix off the front of the schedule. */
      ready = scheduled;
      scheduled = *pos;
      *pos = nullptr;

      /* Change the state of all ready alarms */
      alarm* ap = ready;
      while (ap) {
        ap->state_ = ST_ready;
        ap = ap->next_;
      }

      /* Return the first ready alarm */
      rv = next_ready_ni();
    }
    return rv;
  }

  /** Reset the capture-compare register to match at the next due
   * alarm. */
  void
  reset_cc_ni ()
  {
    if (!scheduled) {
      /* No alarms in queue; disable interrupts and clear any pending
       * event. */
      nrf5::UPTIME_RTC->INTENCLR = (RTC_INTENCLR_COMPARE0_Clear << (ccidx + RTC_INTENCLR_COMPARE0_Pos));
      nrf5::UPTIME_RTC->EVENTS_COMPARE[ccidx] = 0;
      return;
    }
    /* Setting the CC index is guaranteed to fire only if the value
     * stored is at least 2 past the counter value at the time it is
     * stored.  We add a 1 tick fudge in case the counter increments
     * during the update sequence.
     *
     * At the time of initial capability the duration of the
     * following sequence is a little under 6 us. */
    instr_alarm_ccset.assert();
    auto now = now_ni();
    auto delta = scheduled->ordinal_(now);
    if (delta < rtc_safe_delta) {
      /* Increase delta to be sure it fires; leave any current event
       * uncleared so we can potentially process the alarm
       * immediately. */
      delta = rtc_safe_delta;
    } else {
      /* We know there will be an event as a result of updating the
       * CC, so we don't need any existing event as a shortcut */
      nrf5::UPTIME_RTC->EVENTS_COMPARE[ccidx] = 0;
    }
    nrf5::UPTIME_RTC->INTENSET = (RTC_INTENSET_COMPARE0_Set << (ccidx + RTC_INTENSET_COMPARE0_Pos));
    nrf5::UPTIME_RTC->CC[ccidx] = now + delta;
    instr_alarm_ccset.deassert();
  }

  /** Remove an alarm from the queue, returning its prior state.
   *
   * If the removal affected the head of the scheduled queue the
   * wakeup capture/compare register is reset. */
  alarm::state_type
  cancel_ni (alarm& alarm)
  {
    auto old_scheduled = scheduled;
    auto rv = alarm.state_;

    /* Look in scheduled first, then in ready.  If it's in_callback,
     * that's the user's problem: they're informed through the return
     * value. */
    auto pos = search_ni(&scheduled, alarm);
    if (!*pos) {
      pos = search_ni(&ready, alarm);
    }
    if (&alarm == *pos) {
      /* Found it.  Remove it from whichever queue it was in, mark it
       * cancelled, and if necessary update the capture/compare
       * register. */
      *pos = alarm.next_;
      alarm.state_ = ST_cancelled;
      alarm.next_ = nullptr;
      if (old_scheduled != scheduled) {
        reset_cc_ni();
      }
    }
    return rv;
  }
};

alarm::alarm_queue alarm::queue_{UPTIME_ALARM_CCIDX};

void
alarm::schedule ()
{
  primask mutex;
  if (queue_.schedule_ni(now_ni(), *this)) {
    queue_.reset_cc_ni();
  }
}

void
alarm::schedule_offset (int offset)
{
  primask mutex;
  auto now = now_ni();
  deadline_ = now + offset;
  if (queue_.schedule_ni(now, *this)) {
    queue_.reset_cc_ni();
  }
}

alarm::state_type
alarm::cancel ()
{
  primask mutex;
  return queue_.cancel_ni(*this);
}

} // namespace clock
} // namespace nrfcxx

extern "C" {

void
POWER_CLOCK_IRQHandler ()
{
  using namespace nrfcxx;
  using namespace nrfcxx::clock;
  bool reevaluate_hfxt = false;
  bool cal_ready = false;

  if (nrf5::CLOCK->EVENTS_HFCLKSTARTED) {
    instr_event_hfclk.assert();
    nrf5::CLOCK->EVENTS_HFCLKSTARTED = 0;
    if (HFXT_REQ_CAL_PENDING & hfxt_req) {
      cal_ready = true;
    }
    instr_event_hfclk.deassert();
  }

  if (nrf5::CLOCK->EVENTS_LFCLKSTARTED) {
    instr_event_lfclk.assert();
    nrf5::CLOCK->EVENTS_LFCLKSTARTED = 0;

    /* NB: When the clock gets started the source in LFCLKSTAT will
     * probably be RC, even if the configured source is something
     * else.  If the configured source is RC we should calibrate at
     * least once; if it's Xtal, assume that the crystal will
     * stabilize and the source change at that point.
     *
     * Use the copy of LFCLKSRC cached by the infrastructure when
     * TASKS_LFCLKSTART was triggered as the normative clock
     * source. */
    if ((CLOCK_LFCLKSRCCOPY_SRC_RC << CLOCK_LFCLKSRCCOPY_SRC_Pos)
        == (nrf5::CLOCK->LFCLKSTAT & CLOCK_LFCLKSRCCOPY_SRC_Msk)) {
      nrf5::CLOCK->CTIV = lfrc_cal_interval_qs;
      if (hfclk::hfxt_active()) {
        cal_ready = true;
      } else {
        hfxt_req |= HFXT_REQ_CAL_PENDING;
        reevaluate_hfxt = true;
      }
    }
    instr_event_lfclk.deassert();
  }

  if (nrf5::CLOCK->EVENTS_DONE) {
    nrf5::CLOCK->EVENTS_DONE = 0;
    instr_task_cal.deassert();
#if (NRFCXX_ENABLE_NRF51_PAN_14 - 0)
    constlat_req &= ~CONSTLAT_REQ_CAL;
    reevaluate_constlat();
#endif /* NRFCXX_ENABLE_NRF51_PAN_14 */
    hfxt_req &= ~HFXT_REQ_CAL_ACTIVE;
    reevaluate_hfxt = true;
    if (0 < nrf5::CLOCK->CTIV) {
      nrf5::CLOCK->TASKS_CTSTART = 1;
    }
  }

  if (nrf5::CLOCK->EVENTS_CTTO) {
    nrf5::CLOCK->EVENTS_CTTO = 0;
    if (lfxt_req) {
#if (NRFCXX_ENABLE_NRF51_PAN_14 - 0)
      constlat_req |= CONSTLAT_REQ_CAL;
      reevaluate_constlat();
#endif /* NRFCXX_ENABLE_NRF51_PAN_14 */
      if (hfclk::hfxt_active()) {
        cal_ready = true;
      } else {
        hfxt_req |= HFXT_REQ_CAL_PENDING;
        reevaluate_hfxt = true;
      }
    }
  }

  if (cal_ready) {
    /* The event that cued calibration may have occurred while
     * disabling the LFCLK.  Only initiate calibration if it's still
     * necessary. */
    if (lfxt_req) {
      hfxt_req |= HFXT_REQ_CAL_ACTIVE;
      nrf5::CLOCK->TASKS_CAL = 1;
      instr_task_cal.assert();
    }
    hfxt_req &= ~HFXT_REQ_CAL_PENDING;
    reevaluate_hfxt = true;
  }

  if (reevaluate_hfxt) {
    reevaluate_hfxt_ni();
  }
}

void
UPTIME_RTC_IRQHandler ()
{
  using namespace nrfcxx;
  using namespace nrfcxx::clock;

  if (nrf5::UPTIME_RTC->EVENTS_OVRFLW) {
    nrfcxx::primask mutex;
    process_uptime_overflow_ni();
  }
  if (nrf5::UPTIME_RTC->EVENTS_COMPARE[UPTIME_SLEEP_CCIDX]) {
    /* Just clear this: control will return to sleep() where the
     * appropriate steps will be taken. */
    nrf5::UPTIME_RTC->EVENTS_COMPARE[UPTIME_SLEEP_CCIDX] = 0;
  }
  if (nrf5::UPTIME_RTC->EVENTS_COMPARE[UPTIME_ALARM_CCIDX]) {
    instr_alarm_process_ready.assert();

    /* Grab the list of alarms that are ready to execute.  Only alarms
     * that are not yet due are kept on the queue. */
    alarm* ready;
    {
      nrfcxx::primask mutex;
      ready = alarm::queue_.split_ni(now_ni());
    }
    while (ready) {
      /* Update the deadline, and assume default reschedule if the
       * alarm is periodic. */
      bool resched = (0 != ready->interval_);
      ready->deadline_ += ready->interval_;

      /* Process the alarm */
      instr_alarm_process.assert();
      if (ready->callback_) {
        resched = ready->callback_(*ready);
      }
      instr_alarm_process.deassert();

      /* Reschedule the alarm and get the next ready alarm. */
      {
        nrfcxx::primask mutex;
        if (resched) {
          alarm::queue_.schedule_ni(now_ni(), *ready);
        } else {
          ready->state_ = alarm::ST_unscheduled;
        }
        ready = alarm::queue_.next_ready_ni();
      }
    }
    {
      nrfcxx::primask mutex;
      alarm::queue_.reset_cc_ni();
    }
    instr_alarm_process_ready.deassert();
  }
}

} // extern "C"
